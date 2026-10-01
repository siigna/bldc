/*
	Copyright 2025 Benjamin Vedder	benjamin@vedder.se

	This file is part of the VESC firmware.

	The VESC firmware is free software: you can redistribute it and/or modify
	it under the terms of the GNU General Public License as published by
	the Free Software Foundation, either version 3 of the License, or
	(at your option) any later version.

	The VESC firmware is distributed in the hope that it will be useful,
	but WITHOUT ANY WARRANTY; without even the implied warranty of
	MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
	GNU General Public License for more details.

	You should have received a copy of the GNU General Public License
	along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include "luaif.h"
#include "script_lua.h"
#include "script_alloc.h"
#include "script_queue.h"
#include "script_pack.h"
#include "lua_vesc_mc.h"
#include "lua_vesc_conf.h"
#include "lua_vesc_can.h"

#include <string.h>

#include "ch.h"

/*
 * The seam is narrow on purpose: the kernel is always the real one, because
 * the threading and locking are exactly what wants testing. Only the two
 * things that need a board are swapped out -- the flash the script is read
 * from, and the terminal its output goes to.
 */
#include "datatypes.h"
#include "buffer.h"

#ifdef LUAIF_HOST_TEST
#include "luaif_host.h"
#else
#include "commands.h"
#include "flash_helper.h"
#include "mempools.h"
#include "utils.h"
#endif

/*
 * Memory, and where it goes. Measured on 75_100_V2 with USE_LISPBM=0
 * USE_LUA=0, which is the space this engine has to fit into:
 *
 *   ram0   76,040 of 131,072 used   55,032 free
 *   ram4   33,296 of  63,488 used   30,192 free
 *
 * So the engine thread's stack goes in ram4 (CCM) and the arena goes in main
 * RAM, which is the opposite of what one might guess. CCM has the less room
 * of the two, and a stack is what it suits: no DMA engine needs to reach a
 * stack, and CCM cannot be a DMA target. Putting the arena there as well
 * overflowed ram4 by 7,088 bytes.
 *
 * ram4 is not "LispBM's heap and nothing else" -- 33 KB of application thread
 * working areas live there in every build. Dropping LispBM frees room in
 * ram4, but not all of it.
 *
 * The result still costs less RAM than the LispBM build it replaces:
 * ram0 reaches 100,616 against LispBM's 110,288, and ram4 45,584 against
 * 63,208.
 *
 * The two sizes are coupled and must not be changed independently:
 *
 *   LUA_ARENA_SIZE   what a script may allocate, the ceiling it hits first
 *   LUA_WA_SIZE      the engine thread's stack, which bounds LUAI_MAXCCALLS
 *
 * script.mk sets LUAI_MAXCCALLS from LUA_WA_SIZE by the measured cost of a
 * nested call. Shrinking the working area without lowering that limit trades
 * a catchable Lua error for a HardFault; the arithmetic is in script.mk and
 * the measurement is in tests/qemu.
 */
#define LUA_ARENA_SIZE		(24 * 1024)
#define LUA_WA_SIZE			12288

/*
 * Ceiling below the arena, so a runaway script fails its own allocation with
 * a Lua error the script can see, rather than by exhausting the arena and
 * failing whatever allocates next.
 */
#define LUA_MEM_LIMIT		(20 * 1024)

// Instructions between hook runs. Low enough to stop a tight loop promptly,
// high enough that the hook is not most of the run time.
#define LUA_HOOK_COUNT		2000

// Main RAM, not .ram4: see the budget above.
static uint8_t m_arena[LUA_ARENA_SIZE] __attribute__((aligned(8)));
static THD_WORKING_AREA(m_wa, LUA_WA_SIZE) __attribute__((section(".ram4")));

static mutex_t m_mtx;
static script_queue_t m_queue;
static script_lua_t *m_engine;
static script_blob_t m_blob;
static bool m_blob_valid;

static volatile bool m_running;
static volatile bool m_last_run_ok;
static volatile bool m_load_code;
static volatile bool m_print_restart;
static volatile int m_restart_cnt;

/*
 * Request handshake between callers and the engine thread.
 *
 * Three counters rather than a flag, because one flag cannot mean both "a
 * caller wants the running script stopped" and "the caller may now look at
 * the result" -- the first has to be true while the old script unwinds and
 * false before the new one starts, and the second only becomes true after
 * that. A single flag has to be cleared either too early, so the caller reads
 * a torn-down engine and reports failure, or too late, so should_stop aborts
 * the script it was asked to start. Both of those were written here first.
 *
 *   m_req_seq    bumped by a caller to ask for a stop or a restart
 *   m_ack_tear   bumped once the old engine is gone; should_stop keys off the
 *                gap between this and m_req_seq, so it goes false before the
 *                replacement script runs
 *   m_ack_done   bumped when the request is fully served, which is what a
 *                caller waits for
 */
static volatile uint32_t m_req_seq;
static volatile uint32_t m_ack_tear;
static volatile uint32_t m_ack_done;

static thread_t *m_thd;
static volatile bool m_thd_started;

/* Set while the engine thread is inside the interpreter, so luaif_stop knows
 * whether it has to wait for an unwind or can just tear down. */
static volatile bool m_in_script;

static uint32_t m_timer_last_ms;

/*
 * A REPL line or a streamed snippet waiting to be evaluated on the engine
 * thread.
 *
 * Not evaluated on the thread that received the packet, which is what
 * script_event.h's note about SCRIPT_EV_REPL warns against and what this file
 * did first: building and calling a chunk needs about 2 KB of stack before
 * the script does anything, and the comms thread does not have it to spare.
 * The symptom is not a clean overflow -- it is a HardFault with the PC set to
 * the 0x55555555 stack fill pattern, several calls away from the cause.
 *
 * A dedicated slot rather than a queue entry: an expression is up to 512
 * bytes against the queue's 64-byte payload, and sizing the payload for the
 * REPL would cost that much in every queued CAN frame.
 */
#define LUA_EVAL_MAX		512
static char m_eval[LUA_EVAL_MAX];
static volatile int32_t m_eval_len;
static volatile bool m_eval_pending;
static volatile bool m_eval_ok;

/* ------------------------------------------------------------------ output -- */

static void engine_print(const char *msg) {
	commands_printf_lisp("%s", msg);
}

char *luaif_print_prefix(void) {
	/*
	 * A format string, not a label: commands.c consumes it as
	 * sprintf(buf, prefix, "%s"). LispBM's is a user-settable buffer; until
	 * there is a way for a Lua script to set one, this returns what the
	 * no-engine fallback already returns, so script output is formatted
	 * identically and this change is not also a protocol change.
	 */
	return "%s";
}

/* ------------------------------------------------------------ the script -- */

/*
 * Where the script comes from. flash_helper hands back a pointer straight
 * into memory-mapped flash, so there is no copy and no RAM cost for the
 * source -- but it also means the pointer dies when those pages are erased,
 * which is why luaif_stop is synchronous.
 *
 * Behind a seam so the simulated target can hand over a blob without a flash
 * driver. See tests/qemu.
 */
static const uint8_t *code_ptr(int32_t *len_out) {
#ifdef LUAIF_HOST_TEST
	return luaif_host_code(len_out);
#else
	*len_out = (int32_t)flash_helper_code_size(CODE_IND_LISP);
	return (const uint8_t *)flash_helper_code_data(CODE_IND_LISP);
#endif
}

static bool should_stop(void) {
	// True only between a request arriving and the old engine being released.
	return m_req_seq != m_ack_tear;
}

static void on_tick(void) {
	/*
	 * Hand the CPU back. Without this a script holds its priority level for
	 * as long as it runs, and a script is not obliged to ever return.
	 *
	 * chThdYield, not chThdSleepMilliseconds(0): a zero sleep is
	 * TIME_IMMEDIATE, which is not a valid sleep interval. With
	 * CH_DBG_ENABLE_ASSERTS off -- which is how the firmware is built -- it
	 * does not complain, it just never wakes, and the engine thread parks
	 * forever inside the hook. A script that returns promptly never reaches
	 * the hook at all, so this only ever showed up under `while true do end`.
	 *
	 * Note this yield is not covered by tests/qemu and cannot easily be: the
	 * engine runs below NORMALPRIO, so everything that matters preempts it
	 * whether or not it yields. What the yield buys is fairness against
	 * threads at its own priority, and there are none in the test.
	 */
	chThdYield();
}

static void engine_close(void) {
	if (m_engine != NULL) {
		script_lua_close(m_engine);
		m_engine = NULL;
	}
	m_blob_valid = false;
	m_running = false;
}

/*
 * Build an interpreter and run the script's main chunk.
 *
 * Returns false for "nothing to run" as well as for a real failure, and says
 * which in the message, because an empty code slot is the normal state of a
 * board nobody has uploaded to yet.
 */
static bool engine_open_and_run(bool print) {
	int32_t len = 0;
	const uint8_t *blob = code_ptr(&len);

	engine_close();

	if (blob == NULL || len <= 0) {
		if (print) {
			commands_printf_lisp("No script in flash");
		}
		return false;
	}

	if (!script_pack_parse(blob, len, &m_blob)) {
		if (print) {
			commands_printf_lisp("Script container is not valid");
		}
		return false;
	}
	m_blob_valid = true;

	// Reset the arena before the interpreter, not after: init abandons
	// whatever was allocated, which is only correct once the owner is gone.
	script_alloc_init(m_arena, sizeof(m_arena), LUA_MEM_LIMIT);

	script_lua_cfg_t cfg = {0};
	cfg.mem_limit = LUA_MEM_LIMIT;
	cfg.print = engine_print;
	cfg.should_stop = should_stop;
	cfg.on_tick = on_tick;
	cfg.hook_count = LUA_HOOK_COUNT;
	cfg.blob = &m_blob;
	cfg.alloc = script_alloc;
	cfg.alloc_ud = NULL;

	m_engine = script_lua_open(&cfg);
	if (m_engine == NULL) {
		if (print) {
			commands_printf_lisp("Not enough memory for the interpreter");
		}
		m_blob_valid = false;
		return false;
	}

	script_lua_install_events(m_engine);
	script_lua_register(m_engine, lua_vesc_mc_fns());
	script_lua_register(m_engine, lua_vesc_conf_fns());
	script_lua_register(m_engine, lua_vesc_can_fns());

	char err[128];
	m_in_script = true;
	bool ok = script_lua_run(m_engine, m_blob.src, m_blob.src_len,
			"main", err, sizeof(err));
	m_in_script = false;

	if (!ok) {
		// The interpreter is kept. A main chunk that raised may still have
		// registered handlers before it did, and tearing the state down would
		// also throw away the traceback the author needs.
		if (print) {
			commands_printf_lisp("Script error: %s", err);
		}
		m_running = true;
		return false;
	}

	m_running = true;
	if (print) {
		commands_printf_lisp("Script started, %d bytes", (int)len);
	}
	return true;
}

/* ------------------------------------------------------------- dispatch --- */

static void dispatch_pending(void) {
	script_event_t ev;

	while (script_queue_fetch(&m_queue, &ev)) {
		if (m_engine == NULL) {
			continue;
		}

		char err[128];
		m_in_script = true;
		bool ok = script_lua_dispatch(m_engine, &ev, err, sizeof(err));
		m_in_script = false;

		if (!ok) {
			/*
			 * Reported and kept. A handler that raises on one frame should not
			 * silently unsubscribe a vehicle from its own CAN traffic, which
			 * is what removing it would do.
			 */
			commands_printf_lisp("Event handler error: %s", err);
		}

		if (should_stop()) {
			break;
		}
	}
}

static void run_timer(void) {
	if (m_engine == NULL) {
		return;
	}

	uint32_t period = script_lua_timer_period(m_engine);
	if (period == 0) {
		return;
	}

	uint32_t now = (uint32_t)(ST2MS(chVTGetSystemTimeX()));
	if ((now - m_timer_last_ms) < period) {
		return;
	}
	m_timer_last_ms = now;

	script_event_t ev = {0};
	ev.type = SCRIPT_EV_TIMER;

	char err[128];
	m_in_script = true;
	bool ok = script_lua_dispatch(m_engine, &ev, err, sizeof(err));
	m_in_script = false;

	if (!ok) {
		commands_printf_lisp("Timer handler error: %s", err);
	}
}

/* --------------------------------------------------------------- thread --- */

static THD_FUNCTION(lua_thread, arg) {
	(void)arg;
	chRegSetThreadName("lua");

	m_thd_started = true;

	// Load once at boot, quietly: a board with no script should say nothing.
	chMtxLock(&m_mtx);
	m_last_run_ok = engine_open_and_run(false);
	m_restart_cnt++;
	chMtxUnlock(&m_mtx);

	for (;;) {
		uint32_t seq = m_req_seq;

		if (seq != m_ack_done) {
			bool print = m_print_restart;
			bool load = m_load_code;

			chMtxLock(&m_mtx);
			engine_close();
			chMtxUnlock(&m_mtx);

			/*
			 * The old engine is gone, so should_stop goes false here -- before
			 * the replacement runs, and not before the old one has unwound.
			 */
			m_ack_tear = seq;

			if (load) {
				chMtxLock(&m_mtx);
				m_last_run_ok = engine_open_and_run(print);
				chMtxUnlock(&m_mtx);
			} else {
				m_last_run_ok = false;
			}

			m_restart_cnt++;
			m_ack_done = seq;
		}

		if (m_eval_pending) {
			char err[128];

			chMtxLock(&m_mtx);
			if (m_engine != NULL) {
				m_eval_ok = script_lua_run(m_engine, m_eval, m_eval_len,
						"repl", err, sizeof(err));
				if (!m_eval_ok) {
					commands_printf_lisp("%s", err);
				}
			} else {
				m_eval_ok = false;
			}
			chMtxUnlock(&m_mtx);

			m_eval_pending = false;
		}

		if (m_engine != NULL) {
			chMtxLock(&m_mtx);
			dispatch_pending();
			run_timer();
			chMtxUnlock(&m_mtx);
		}

		/*
		 * 1 ms, which is the resolution a script's timer can ask for and far
		 * longer than a dispatch pass takes. Polling rather than waiting on
		 * the queue keeps the producers free of any blocking call -- the CAN
		 * path must never wait on a script.
		 */
		chThdSleepMilliseconds(1);
	}
}

/* ----------------------------------------------------------- entry points -- */

void luaif_init(void) {
	if (m_thd != NULL) {
		return;
	}

	chMtxObjectInit(&m_mtx);
	script_queue_init(&m_queue);

	/*
	 * NORMALPRIO - 1, which is where LispBM's evaluator thread runs and for
	 * the same reason: a script must not be able to outrank the comms and
	 * control threads it shares the CPU with.
	 */
	m_thd = chThdCreateStatic(m_wa, sizeof(m_wa), NORMALPRIO - 1,
			lua_thread, NULL);
}

/*
 * Ask the engine thread for something and wait until it has done it.
 *
 * Bounded, because the caller is usually the comms thread: blocking it
 * forever on an engine that has somehow wedged would be worse than the stale
 * flash pointer the wait exists to prevent. On timeout the engine is dropped
 * here instead, under the lock, so no pointer into flash outlives the call.
 *
 * The timeout is a real tradeoff, not just a safety net. A script whose main
 * chunk legitimately takes longer than the deadline gets force-closed even
 * though it is working -- which is exactly what happened the first time a
 * long-running script was added to tests/qemu: the script printed its result
 * and the restart still reported failure. A script doing heavy work at
 * startup should move it into a handler rather than the main chunk.
 */
static bool request(bool load_code, bool print, int timeout_ms) {
	uint32_t seq;

	m_print_restart = print;
	m_load_code = load_code;

	chSysLock();
	seq = ++m_req_seq;
	chSysUnlock();

	for (int i = 0; (i < timeout_ms) && (m_ack_done != seq); i++) {
		chThdSleepMilliseconds(1);
	}

	if (m_ack_done != seq) {
		chMtxLock(&m_mtx);
		engine_close();
		chMtxUnlock(&m_mtx);
		m_ack_tear = seq;
		m_ack_done = seq;
		return false;
	}

	return m_last_run_ok;
}

void luaif_stop(void) {
	if (m_thd == NULL) {
		return;
	}

	(void)request(false, false, 1000);
}

bool luaif_restart(bool print, bool load_code) {
	if (m_thd == NULL) {
		return false;
	}

	return request(load_code, print, 2000);
}

int luaif_get_restart_cnt(void) {
	return m_restart_cnt;
}

void luaif_process_can(uint32_t can_id, uint8_t *data8, int len, bool is_ext) {
	if ((m_engine == NULL) || !m_running || (data8 == NULL) || (len < 0)) {
		return;
	}

	int type = is_ext ? SCRIPT_EV_CAN_EID : SCRIPT_EV_CAN_SID;

	/*
	 * Asked before copying. Most scripts register no CAN handler at all, and
	 * on a busy bus this runs for every frame -- queueing work nothing will
	 * consume would cost a memcpy per frame and push out events that do have
	 * a handler.
	 */
	if (!script_lua_wants(m_engine, type)) {
		return;
	}

	script_event_t ev = {0};
	ev.type = (uint8_t)type;
	ev.id = can_id;

	if (len > SCRIPT_EVENT_PAYLOAD) {
		ev.len = SCRIPT_EVENT_PAYLOAD;
		ev.truncated = 1;
	} else {
		ev.len = (uint16_t)len;
	}
	memcpy(ev.data, data8, ev.len);

	script_queue_post(&m_queue, &ev);
}

void luaif_process_custom_app_data(unsigned char *data, unsigned int len) {
	if ((m_engine == NULL) || !m_running || (data == NULL)) {
		return;
	}

	if (!script_lua_wants(m_engine, SCRIPT_EV_APP_DATA)) {
		return;
	}

	script_event_t ev = {0};
	ev.type = SCRIPT_EV_APP_DATA;

	if (len > SCRIPT_EVENT_PAYLOAD) {
		ev.len = SCRIPT_EVENT_PAYLOAD;
		ev.truncated = 1;
	} else {
		ev.len = (uint16_t)len;
	}
	memcpy(ev.data, data, ev.len);

	script_queue_post(&m_queue, &ev);
}

/*
 * Bytes of the engine thread's stack that have been touched.
 *
 * chThdCreateStatic puts thread_t at the base of the working area and fills
 * the stack above it with CH_DBG_STACK_FILL_VALUE, growing it downward from
 * the top, so the run of surviving fill bytes is the headroom. LispBM reports
 * zero for this field; there is no reason to, since the information is free.
 */
static float stack_use_percent(void) {
#if CH_DBG_FILL_THREADS == TRUE
	const unsigned char *base = (const unsigned char *)m_wa + sizeof(thread_t);
	size_t total = sizeof(m_wa) - sizeof(thread_t);
	size_t freebytes = 0;

	while ((freebytes < total)
			&& (base[freebytes] == CH_DBG_STACK_FILL_VALUE)) {
		freebytes++;
	}
	return 100.0f * (float)(total - freebytes) / (float)total;
#else
	return 0.0f;
#endif
}

/*
 * Append the script's numeric globals as name/value pairs, which is what
 * drives VESC Tool's variable plots.
 *
 * Only under chMtxTryLock. A script is entitled to spend a long time in a
 * handler, and the engine thread holds the lock while it does; this runs on
 * the comms thread, so waiting for it would hang the connection on a busy
 * script. Failing to get the lock costs the caller the variable list for one
 * poll, which is the right thing to lose.
 */
static void append_globals(uint8_t *buf, int32_t *ind, bool all) {
	if (m_engine == NULL) {
		return;
	}
	if (!chMtxTryLock(&m_mtx)) {
		return;
	}

	lua_State *L = script_lua_state(m_engine);
	if (L == NULL) {
		chMtxUnlock(&m_mtx);
		return;
	}

	lua_pushglobaltable(L);
	lua_pushnil(L);
	while (lua_next(L, -2) != 0) {
		/* name must be a string and value a number, and there has to be room
		 * for the longest pair this could append. */
		if ((lua_type(L, -2) == LUA_TSTRING) && lua_isnumber(L, -1)
				&& (*ind < 300)) {
			const char *name = lua_tostring(L, -2);
			size_t nlen = strlen(name);

			if ((nlen > 0) && (nlen < 32)
					&& (all || ((name[0] == 'v' || name[0] == 'V')
							&& (name[1] == 't' || name[1] == 'T')))) {
				memcpy(buf + *ind, name, nlen + 1);
				*ind += (int32_t)nlen + 1;
				buffer_append_float32_auto(buf,
						(float)lua_tonumber(L, -1), ind);
			}
		}
		lua_pop(L, 1);
	}
	lua_pop(L, 1);

	chMtxUnlock(&m_mtx);
}

/*
 * Source streamed in by the REPL's run-selection, which arrives in chunks and
 * is run once the last one lands. Small on purpose: this is for a snippet, not
 * a program -- a program goes through COMM_LISP_WRITE_CODE into flash, where
 * it costs no RAM at all.
 */
#define LUA_STREAM_MAX		LUA_EVAL_MAX
static char m_stream[LUA_STREAM_MAX];
static int32_t m_stream_len;

/*
 * Hand a chunk to the engine thread and wait for it.
 *
 * Bounded like the other waits, and for the same reason: the caller is the
 * comms thread. A REPL line that loops is stopped by the instruction hook via
 * should_stop, so the deadline here is a backstop, not the mechanism.
 */
static bool eval_on_engine(const char *src, int32_t len, int timeout_ms) {
	if ((m_thd == NULL) || (src == NULL) || (len <= 0)) {
		return false;
	}
	if (len > LUA_EVAL_MAX) {
		commands_printf_lisp("Expression too long (%d > %d bytes)",
				(int)len, LUA_EVAL_MAX);
		return false;
	}
	if (m_eval_pending) {
		commands_printf_lisp("Busy with the previous expression");
		return false;
	}

	memcpy(m_eval, src, (size_t)len);
	m_eval_len = len;
	m_eval_ok = false;
	m_eval_pending = true;

	for (int i = 0; (i < timeout_ms) && m_eval_pending; i++) {
		chThdSleepMilliseconds(1);
	}

	if (m_eval_pending) {
		// Give up waiting but leave the slot owned by the thread, which will
		// finish and clear it; stealing it back would race the interpreter.
		commands_printf_lisp("Expression did not finish in time");
		return false;
	}

	return m_eval_ok;
}

void luaif_process_cmd(unsigned char *data, unsigned int len,
		void (*reply_func)(unsigned char *data, unsigned int len)) {
	if ((data == NULL) || (len < 1) || (reply_func == NULL)) {
		return;
	}

	COMM_PACKET_ID packet_id = (COMM_PACKET_ID)data[0];
	data++;
	len--;

	switch (packet_id) {
	case COMM_LISP_SET_RUNNING: {
		bool ok;

		if ((len >= 1) && (data[0] == 0)) {
			luaif_stop();
			ok = true;			// stopping succeeded, not "a script runs"
		} else {
			ok = luaif_restart(true, true);
		}

		int32_t ind = 0;
		uint8_t send_buffer[8];
		send_buffer[ind++] = (uint8_t)packet_id;
		send_buffer[ind++] = ok ? 1 : 0;
		reply_func(send_buffer, (unsigned int)ind);
	} break;

	case COMM_LISP_GET_STATS: {
		if (m_thd == NULL) {
			break;
		}

		/*
		 * CPU is the engine thread's share since the previous call, because
		 * p_time is zeroed here -- so the first call after a reset covers
		 * everything since boot, including building the interpreter. Reading
		 * it as a steady-state figure is how a 6x regression gets reported
		 * that was never there.
		 */
		static systime_t time_last;
		float cpu_use = 0.0f;

		utils_sys_lock_cnt();
		systime_t now = chVTGetSystemTimeX();
		systime_t span = now - time_last;
		if (span > 0) {
			cpu_use = 100.0f * (float)m_thd->p_time / (float)span;
		}
		time_last = now;
		m_thd->p_time = 0;
		utils_sys_unlock_cnt();

		bool all = true;
		if (len > 0) {
			all = (data[0] != 0);
		}

		float mem_use = 100.0f * (float)script_alloc_used()
				/ (float)sizeof(m_arena);

		uint8_t *buf = mempools_get_packet_buffer();
		int32_t ind = 0;

		buf[ind++] = (uint8_t)packet_id;
		buffer_append_float16(buf, cpu_use, 1e2, &ind);
		// Heap and arena are the same thing here, unlike LispBM's split.
		buffer_append_float16(buf, mem_use, 1e2, &ind);
		buffer_append_float16(buf, mem_use, 1e2, &ind);
		buffer_append_float16(buf, stack_use_percent(), 1e2, &ind);
		buf[ind++] = '\0';			// result string, unused

		append_globals(buf, &ind, all);

		reply_func(buf, (unsigned int)ind);
		mempools_free_packet_buffer(buf);
	} break;

	case COMM_LISP_REPL_CMD: {
		/*
		 * The engine has to exist to evaluate anything, but the script in
		 * flash must not be re-run just because someone opened a REPL -- so
		 * this starts an interpreter without loading code, matching lisp.
		 */
		if (m_engine == NULL) {
			(void)luaif_restart(true, false);
		}

		if (m_engine == NULL) {
			commands_printf_lisp("No interpreter");
			break;
		}

		if (len < 1) {
			commands_printf_lisp(">");
			break;
		}

		// NUL-terminate in place: the packet is not guaranteed to be.
		char line[256];
		size_t n = (len < sizeof(line) - 1u) ? len : sizeof(line) - 1u;
		memcpy(line, data, n);
		line[n] = '\0';

		if (strncmp(line, ":help", 5) == 0) {
			commands_printf_lisp("== Special Commands ==");
			commands_printf_lisp(":help\n  Print this help text");
			commands_printf_lisp(":info\n  Print memory and stack usage");
			commands_printf_lisp(":reset\n  Reload the script from flash");
			commands_printf_lisp(":stop\n  Stop the script");
			commands_printf_lisp(
					"Anything else is evaluated as Lua. Use print() to see a"
					" value; an expression's result is not echoed.");
		} else if (strncmp(line, ":info", 5) == 0) {
			commands_printf_lisp("Arena  %d of %d bytes, peak %d",
					(int)script_alloc_used(), (int)sizeof(m_arena),
					(int)script_alloc_peak());
			commands_printf_lisp("Script %d bytes, peak %d",
					(int)script_lua_mem_used(m_engine),
					(int)script_lua_mem_peak(m_engine));
			commands_printf_lisp("Stack  %d%% of %d bytes",
					(int)stack_use_percent(),
					(int)(sizeof(m_wa) - sizeof(thread_t)));
			commands_printf_lisp("Restarts %d, events dropped %d",
					m_restart_cnt, (int)script_queue_dropped(&m_queue));
		} else if (strncmp(line, ":reset", 6) == 0) {
			(void)luaif_restart(true, true);
		} else if (strncmp(line, ":stop", 5) == 0) {
			luaif_stop();
			commands_printf_lisp("Stopped");
		} else {
			(void)eval_on_engine(line, (int32_t)n, 2000);
		}
	} break;

	case COMM_LISP_STREAM_CODE: {
		int32_t ind = 0;
		int32_t offset = buffer_get_int32(data, &ind);
		int32_t tot_len = buffer_get_int32(data, &ind);
		int8_t restart = (int8_t)data[ind++];
		int16_t result = 0;

		if (offset == 0) {
			m_stream_len = 0;
			if ((m_engine == NULL) || (restart == 1) || (restart == 2)) {
				(void)luaif_restart(true, restart == 2);
			}
		}

		if ((m_engine == NULL) || (tot_len > LUA_STREAM_MAX)
				|| (offset != m_stream_len)) {
			// -1 is what lisp reports for "cannot take this", and VESC Tool
			// already knows how to show it.
			result = -1;
			m_stream_len = 0;
		} else {
			int32_t chunk = (int32_t)len - ind;
			if ((chunk > 0) && ((m_stream_len + chunk) <= LUA_STREAM_MAX)) {
				memcpy(m_stream + m_stream_len, data + ind, (size_t)chunk);
				m_stream_len += chunk;
			}

			if (m_stream_len >= tot_len) {
				if (!eval_on_engine(m_stream, m_stream_len, 2000)) {
					result = -1;
				}
				m_stream_len = 0;
			}
		}

		int32_t send_ind = 0;
		uint8_t send_buffer[16];
		send_buffer[send_ind++] = (uint8_t)packet_id;
		buffer_append_int32(send_buffer, offset, &send_ind);
		buffer_append_int16(send_buffer, result, &send_ind);
		// send_ind, not the read index: lispif passes `ind` here, which sends
		// two bytes of whatever follows in its buffer.
		reply_func(send_buffer, (unsigned int)send_ind);
	} break;

	case COMM_LISP_RMSG:
		/*
		 * Lisp-specific: it dispatches to a recv-rmsg channel, and there is
		 * no Lua binding for one. Answered anyway -- a sender that gets
		 * nothing back cannot tell a missing feature from a dead board.
		 */
		commands_printf_lisp("RMSG is not supported by the Lua engine");
		break;

	default:
		break;
	}
}

void luaif_process_shutdown(void) {
	/*
	 * Nothing yet, and deliberately nothing rather than a stub that waits.
	 * This sits in the power-off path, so until there is a shutdown hook for
	 * a script to register, the right behaviour is to cost nothing at all.
	 */
}
