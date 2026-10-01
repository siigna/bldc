/*
 * The firmware adapter -- luaif.c -- on the simulated STM32F405.
 *
 * This is the piece that was supposed to need a bench: the engine thread, the
 * lifecycle, and the event path from a producer thread into a running script.
 * All of it runs here against the real ChibiOS kernel, the real engine and the
 * real arena, with only the flash and the terminal stubbed (luaif_host.h).
 *
 * Still not covered, and still needing hardware: the COMM_LISP_* protocol,
 * which luaif.c does not implement yet, and whether flash_helper's pointer
 * really does go stale under an erase.
 */

#include "qrt.h"
#include "luaif.h"
#include "luaif_host.h"
#include "script_alloc.h"
#include "mc_interface.h"
#include "timeout.h"
#include "script_pack.h"
#include "datatypes.h"
#include "buffer.h"

#include <stdarg.h>
#include <stdio.h>
#include <string.h>

/* ------------------------------------------------------- the fake flash -- */

static uint8_t m_code[1024];
static int32_t m_code_len;

const uint8_t *luaif_host_code(int32_t *len_out) {
	*len_out = m_code_len;
	return (m_code_len > 0) ? m_code : NULL;
}

/*
 * CRC-16/XMODEM, matching script_pack.c's own pack_crc16 bit for bit.
 * Deliberately a copy rather than a call into util/crc.c: if the two ever
 * disagree the parser is the authority, and a test that borrows the parser's
 * helper could not detect that disagreement at all.
 */
static uint16_t test_crc16(const uint8_t *data, int32_t len) {
	uint16_t crc = 0;

	for (int32_t i = 0; i < len; i++) {
		crc ^= (uint16_t)((uint16_t)data[i] << 8);
		for (int b = 0; b < 8; b++) {
			crc = (crc & 0x8000u) ? (uint16_t)((crc << 1) ^ 0x1021u)
					: (uint16_t)(crc << 1);
		}
	}
	return crc;
}

/*
 * Build a container the real parser will accept, so the test exercises
 * script_pack_parse rather than side-stepping it. Layout per script_pack.h:
 * big-endian uint32 size, uint16 crc, uint16 flags, NUL-terminated source,
 * then uint16 import count.
 *
 * The size field is *two less* than the length of the region it covers --
 * what VESC Tool writes, and what script_pack.c compensates for. Getting that
 * wrong is how this test first failed, with every container refused.
 */
static void flash_write_script(const char *src, bool lua) {
	size_t srclen = strlen(src) + 1u;          /* including the terminator */
	size_t body = 2u + srclen + 2u;            /* flags + source + imports */
	uint8_t *p = m_code;

	size_t stored = body - 2u;                 /* see the note above */

	p[0] = (uint8_t)((stored >> 24) & 0xffu);
	p[1] = (uint8_t)((stored >> 16) & 0xffu);
	p[2] = (uint8_t)((stored >> 8) & 0xffu);
	p[3] = (uint8_t)(stored & 0xffu);

	uint8_t *b = p + 6;                        /* flags onward */
	b[0] = 0x00u;
	b[1] = lua ? 0x01u : 0x00u;                /* SCRIPT_FLAG_LANG_LUA */
	memcpy(b + 2, src, srclen);
	b[2 + srclen] = 0x00u;                     /* no imports */
	b[3 + srclen] = 0x00u;

	uint16_t ck = test_crc16(b, (int32_t)body);
	p[4] = (uint8_t)((ck >> 8) & 0xffu);
	p[5] = (uint8_t)(ck & 0xffu);

	m_code_len = (int32_t)(body + 6u);
}

static void flash_erase_script(void) {
	m_code_len = 0;
}

/* ----------------------------------------------------- captured terminal -- */

static char m_out[1024];
static size_t m_out_len;

int commands_printf_lisp(const char *format, ...) {
	char line[256];
	va_list ap;

	va_start(ap, format);
	int n = vsnprintf(line, sizeof(line), format, ap);
	va_end(ap);

	if (n > 0) {
		size_t add = strlen(line);
		if (m_out_len + add + 1u < sizeof(m_out)) {
			memcpy(m_out + m_out_len, line, add);
			m_out_len += add;
			m_out[m_out_len] = '\0';
		}
		/*
		 * Captured, not echoed. The engine runs on its own thread, so
		 * printing here races main's own output and shreds both lines. Tests
		 * dump the capture at a point where they control the timing.
		 */
	}
	return n;
}

const char *luaif_host_output(void) {
	return m_out;
}

void luaif_host_output_reset(void) {
	m_out_len = 0;
	m_out[0] = '\0';
}

/* Progress markers: a passing check prints nothing, so without these a hang
 * or a fault gives no clue which stage it happened in. */
#define STAGE(s) do { qrt_puts("  .. "); qrt_puts(s); qrt_puts("\r\n"); } while (0)

/* --------------------------------------------- firmware odds and ends -- */

static uint8_t m_packet_buf[512];
static int m_packet_buf_out;

uint8_t *mempools_get_packet_buffer(void) {
	m_packet_buf_out++;
	return m_packet_buf;
}

void mempools_free_packet_buffer(uint8_t *buffer) {
	(void)buffer;
	m_packet_buf_out--;
}

/* The real ones count, so nesting is safe; so must these. */
static int m_lock_depth;

void utils_sys_lock_cnt(void) {
	if (m_lock_depth++ == 0) {
		chSysLock();
	}
}

void utils_sys_unlock_cnt(void) {
	if (--m_lock_depth == 0) {
		chSysUnlock();
	}
}

/* ------------------------------------------------- captured packet reply -- */

static uint8_t m_reply[512];
static unsigned int m_reply_len;
static int m_reply_count;

static void capture_reply(unsigned char *data, unsigned int len) {
	m_reply_len = (len < sizeof(m_reply)) ? len : sizeof(m_reply);
	memcpy(m_reply, data, m_reply_len);
	m_reply_count++;
}

static void reply_reset(void) {
	m_reply_len = 0;
	m_reply_count = 0;
}

/*
 * Drive one COMM_LISP_* packet the way commands.c does: the id is data[0]
 * and the length includes it.
 */
static uint8_t m_pkt[320];

static void send_cmd(uint8_t id, const uint8_t *payload, unsigned int n) {
	uint8_t *pkt = m_pkt;

	pkt[0] = id;
	if (n > 0u) {
		memcpy(pkt + 1, payload, n);
	}
	luaif_process_cmd(pkt, n + 1u, capture_reply);
}

/* ------------------------------------------------------------------ tests -- */

/* Sized like comm_usb's serial_process_thread, which is what really calls
 * commands_process_packet and therefore luaif_process_cmd. */
static THD_WORKING_AREA(wa_proto, 2048);

static THD_FUNCTION(proto_thread, arg) {
	(void)arg;
	chRegSetThreadName("proto");

	/*
	 * --- the COMM_LISP_* protocol -------------------------------------
	 *
	 * Driven exactly as commands.c drives it. Code upload is not tested here
	 * because it is not in this engine: READ/WRITE/ERASE_CODE go straight to
	 * flash_helper in commands.c with no engine involved.
	 */
	STAGE("protocol: set_running");
	luaif_host_output_reset();
	flash_write_script("vtTest = 42\nprint('proto=ok')\n", true);
	{
		uint8_t on[1] = {1};

		reply_reset();
		send_cmd(COMM_LISP_SET_RUNNING, on, 1);
		qrt_expect("set_running replied once", (unsigned)m_reply_count, 1u);
		qrt_expect("set_running reply length", m_reply_len, 2u);
		qrt_expect("set_running echoes the id",
				m_reply[0], (unsigned)COMM_LISP_SET_RUNNING);
		qrt_expect("set_running reports success", m_reply[1], 1u);
		qrt_expect_ok("the script ran",
				strstr(luaif_host_output(), "proto=ok") != NULL);
	}

	STAGE("protocol: get_stats");
	{
		uint8_t all[1] = {1};
		int32_t ind = 0;
		float cpu, heap, mem, stack;

		reply_reset();
		send_cmd(COMM_LISP_GET_STATS, all, 1);
		qrt_expect("get_stats replied once", (unsigned)m_reply_count, 1u);
		qrt_expect("get_stats echoes the id",
				m_reply[0], (unsigned)COMM_LISP_GET_STATS);

		ind = 1;
		cpu   = buffer_get_float16(m_reply, 1e2, &ind);
		heap  = buffer_get_float16(m_reply, 1e2, &ind);
		mem   = buffer_get_float16(m_reply, 1e2, &ind);
		stack = buffer_get_float16(m_reply, 1e2, &ind);

		qrt_puts("  stats cpu=");   qrt_putu((unsigned)cpu);
		qrt_puts("% mem=");         qrt_putu((unsigned)mem);
		qrt_puts("% stack=");       qrt_putu((unsigned)stack);
		qrt_puts("%\r\n");

		qrt_expect_ok("cpu is a percentage", (cpu >= 0.0f) && (cpu <= 100.0f));
		qrt_expect_ok("heap is a percentage",
				(heap >= 0.0f) && (heap <= 100.0f));
		qrt_expect_ok("memory use is reported", mem > 0.0f);
		/* lisp sends 0 here; this engine has the figure for free */
		qrt_expect_ok("stack use is reported, not stubbed", stack > 0.0f);

		qrt_expect("result string is empty", m_reply[ind], 0u);
		ind++;

		qrt_expect("packet buffer was returned",
				(unsigned)m_packet_buf_out, 0u);
	}

	/*
	 * The variable list is best-effort and the test has to say so.
	 *
	 * append_globals only takes the lock with chMtxTryLock, because the
	 * engine thread holds it while dispatching and this runs on the comms
	 * thread -- waiting would hang the connection on a busy script. So a
	 * single GET_STATS can legitimately come back with no variables at all,
	 * and VESC Tool's plots can show a gap. Asserting on one poll passes or
	 * fails by luck, which is exactly what it did first.
	 */
	STAGE("protocol: get_stats variables");
	{
		bool found = false;
		int attempts = 0;

		for (attempts = 0; (attempts < 40) && !found; attempts++) {
			int32_t ind;

			reply_reset();
			send_cmd(COMM_LISP_GET_STATS, (uint8_t[]){1}, 1);

			ind = 1 + 8 + 1;        /* four float16 and the result byte */
			if ((unsigned)ind < m_reply_len) {
				const char *name = (const char *)(m_reply + ind);
				if (strcmp(name, "vtTest") == 0) {
					int32_t vind = ind + (int32_t)strlen(name) + 1;
					float value = buffer_get_float32_auto(m_reply, &vind);

					qrt_puts("  global ");
					qrt_puts(name);
					qrt_puts("=");
					qrt_putu((unsigned)value);
					qrt_puts(" after ");
					qrt_putu((unsigned)attempts + 1u);
					qrt_puts(" poll(s)\r\n");
					qrt_expect_ok("the global carries its value",
							(value > 41.0f) && (value < 43.0f));
					found = true;
				}
			}
			if (!found) {
				chThdSleepMilliseconds(5);
			}
		}

		qrt_expect_ok("a numeric global is reported within a few polls",
				found);
	}

	STAGE("protocol: get_stats filtered");
	{
		uint8_t none[1] = {0};
		unsigned int all_len;

		reply_reset();
		send_cmd(COMM_LISP_GET_STATS, (uint8_t[]){1}, 1);
		all_len = m_reply_len;

		reply_reset();
		send_cmd(COMM_LISP_GET_STATS, none, 1);
		/* vtTest starts with "vt", so the filter keeps it either way: the
		 * point is only that the flag is read and nothing is dropped. */
		qrt_expect("filtered reply still carries vtTest",
				m_reply_len, all_len);
	}

	STAGE("protocol: repl");
	luaif_host_output_reset();
	{
		const char *line = "print('repl=' .. (6 * 7))";

		reply_reset();
		send_cmd(COMM_LISP_REPL_CMD, (const uint8_t *)line,
				(unsigned int)strlen(line));
		qrt_expect_ok("a repl line is evaluated",
				strstr(luaif_host_output(), "repl=42") != NULL);
	}

	STAGE("protocol: repl error");
	luaif_host_output_reset();
	{
		const char *bad = "this is not lua";

		send_cmd(COMM_LISP_REPL_CMD, (const uint8_t *)bad,
				(unsigned int)strlen(bad));
		qrt_expect_ok("a bad repl line reports instead of dying",
				strstr(luaif_host_output(), "repl") != NULL
				|| strstr(luaif_host_output(), "syntax") != NULL
				|| strstr(luaif_host_output(), "unexpected") != NULL);
	}

	STAGE("protocol: repl :info");
	luaif_host_output_reset();
	{
		const char *info = ":info";

		send_cmd(COMM_LISP_REPL_CMD, (const uint8_t *)info,
				(unsigned int)strlen(info));
		qrt_expect_ok(":info reports the arena",
				strstr(luaif_host_output(), "Arena") != NULL);
		qrt_expect_ok(":info reports the stack",
				strstr(luaif_host_output(), "Stack") != NULL);
	}

	STAGE("protocol: stream_code");
	luaif_host_output_reset();
	{
		const char *src = "print('streamed=1')\n";
		int32_t tot = (int32_t)strlen(src);
		uint8_t pay[160];
		int32_t ind;
		int32_t half = tot / 2;

		/* first chunk, restart=0 so the running script is left alone */
		ind = 0;
		buffer_append_int32(pay, 0, &ind);
		buffer_append_int32(pay, tot, &ind);
		pay[ind++] = 0;
		memcpy(pay + ind, src, (size_t)half);
		reply_reset();
		send_cmd(COMM_LISP_STREAM_CODE, pay, (unsigned int)(ind + half));

		qrt_expect("stream reply length", m_reply_len, 7u);
		qrt_expect("stream echoes the id",
				m_reply[0], (unsigned)COMM_LISP_STREAM_CODE);
		{
			int32_t rind = 1;
			int32_t off = buffer_get_int32(m_reply, &rind);
			int16_t res = buffer_get_int16(m_reply, &rind);

			qrt_expect("stream echoes the offset", (unsigned)off, 0u);
			qrt_expect_ok("first chunk accepted", res == 0);
		}

		/* second chunk completes it, and the snippet runs */
		ind = 0;
		buffer_append_int32(pay, half, &ind);
		buffer_append_int32(pay, tot, &ind);
		pay[ind++] = 0;
		memcpy(pay + ind, src + half, (size_t)(tot - half));
		reply_reset();
		send_cmd(COMM_LISP_STREAM_CODE, pay,
				(unsigned int)(ind + (tot - half)));

		qrt_expect_ok("the streamed snippet ran",
				strstr(luaif_host_output(), "streamed=1") != NULL);
	}

	STAGE("protocol: stream_code out of order");
	luaif_host_output_reset();
	{
		const char *src = "print('never-runs')\n";
		int32_t tot = (int32_t)strlen(src);
		uint8_t pay[160];
		int32_t ind = 0;

		/*
		 * A chunk claiming to start past where the buffer actually is must be
		 * refused. Accepting it would splice a hole into the source and then
		 * run whatever that parsed as -- the stream is just bytes, so a gap
		 * does not necessarily fail to compile.
		 */
		buffer_append_int32(pay, 0, &ind);
		buffer_append_int32(pay, tot, &ind);
		pay[ind++] = 0;
		memcpy(pay + ind, src, 4);
		reply_reset();
		send_cmd(COMM_LISP_STREAM_CODE, pay, (unsigned int)(ind + 4));

		ind = 0;
		buffer_append_int32(pay, 99, &ind);      /* nowhere near 4 */
		buffer_append_int32(pay, tot, &ind);
		pay[ind++] = 0;
		memcpy(pay + ind, src + 4, 4);
		reply_reset();
		send_cmd(COMM_LISP_STREAM_CODE, pay, (unsigned int)(ind + 4));
		{
			int32_t rind = 1;
			int32_t off = buffer_get_int32(m_reply, &rind);
			int16_t res = buffer_get_int16(m_reply, &rind);

			qrt_expect("out-of-order chunk echoes its offset",
					(unsigned)off, 99u);
			qrt_expect_ok("out-of-order chunk is refused", res == -1);
		}
		qrt_expect_ok("nothing was run from a spliced stream",
				strstr(luaif_host_output(), "never-runs") == NULL);
	}

	STAGE("protocol: rmsg is refused, not ignored");
	luaif_host_output_reset();
	{
		uint8_t rm[2] = {0, 0};

		send_cmd(COMM_LISP_RMSG, rm, sizeof(rm));
		qrt_expect_ok("rmsg says it is unsupported",
				strstr(luaif_host_output(), "not supported") != NULL);
	}

	STAGE("protocol: set_running off");
	{
		uint8_t off[1] = {0};

		reply_reset();
		send_cmd(COMM_LISP_SET_RUNNING, off, 1);
		qrt_expect("stop replied", (unsigned)m_reply_count, 1u);
		qrt_expect("stop reports success", m_reply[1], 1u);
		qrt_expect("stop released the arena",
				(unsigned)script_alloc_used(), 0u);
	}

}

int main(void) {
	qrt_console_init();
	chSysInit();
	qrt_systick_init();

	qrt_puts("luaif test\r\n");

	/* --- an empty code slot is a normal state, not a failure ----------- */
	STAGE("empty slot");
	flash_erase_script();
	luaif_init();
	chThdSleepMilliseconds(100);
	qrt_expect_ok("survives an empty code slot", luaif_get_restart_cnt() >= 1);

	/* --- a script that runs, and whose output comes back -------------- */
	STAGE("run a script");
	luaif_host_output_reset();
	flash_write_script(
			"local n = 0\n"
			"for i = 1, 10 do n = n + i end\n"
			"print('total=' .. n)\n", true);
	qrt_expect_ok("restart runs the script", luaif_restart(true, true));
	qrt_expect_ok("script output reached the terminal",
			strstr(luaif_host_output(), "total=55") != NULL);
	qrt_expect_ok("arena in use while running", script_alloc_used() > 0u);

	/* --- a CAN frame gets from a producer thread into a handler -------- */
	luaif_host_output_reset();
	STAGE("can handler");
	flash_write_script(
			"vesc.on_can(function(id, data)\n"
			"  print('can id=' .. id .. ' n=' .. #data)\n"
			"end)\n", true);
	qrt_expect_ok("restart with a can handler", luaif_restart(true, true));
	{
		uint8_t frame[4] = {1, 2, 3, 4};
		luaif_process_can(0x123u, frame, sizeof(frame), false);
		chThdSleepMilliseconds(50);
	}
	qrt_expect_ok("can frame reached the handler",
			strstr(luaif_host_output(), "can id=291 n=4") != NULL);

	/* --- app data takes the same path --------------------------------- */
	luaif_host_output_reset();
	STAGE("app data");
	flash_write_script(
			"vesc.on_app_data(function(data)\n"
			"  print('app n=' .. #data)\n"
			"end)\n", true);
	qrt_expect_ok("restart with an app-data handler", luaif_restart(true, true));
	{
		unsigned char payload[6] = {9, 8, 7, 6, 5, 4};
		luaif_process_custom_app_data(payload, sizeof(payload));
		chThdSleepMilliseconds(50);
	}
	qrt_expect_ok("app data reached the handler",
			strstr(luaif_host_output(), "app n=6") != NULL);

	/* --- a script that raises is reported and does not take the thread - */
	luaif_host_output_reset();
	STAGE("raising script");
	flash_write_script("error('deliberate')\n", true);
	qrt_expect_ok("a raising script is not a successful start",
			!luaif_restart(true, true));
	qrt_expect_ok("the error was reported",
			strstr(luaif_host_output(), "deliberate") != NULL);

	/* --- a corrupt container is refused, not run ---------------------- */
	luaif_host_output_reset();
	STAGE("corrupt container");
	flash_write_script("print('never')\n", true);
	m_code[20] ^= 0xffu;                       /* break the crc */
	qrt_expect_ok("corrupt container refused", !luaif_restart(true, true));
	qrt_expect_ok("corruption was reported",
			strstr(luaif_host_output(), "not valid") != NULL);

	/*
	 * --- stop releases the interpreter --------------------------------
	 * This is the one flash_helper depends on: it erases the pages the
	 * source is read from, so the engine must have let go before the call
	 * returns, not merely been asked to.
	 */
	luaif_host_output_reset();
	STAGE("stop releases");
	flash_write_script("print('running')\n", true);
	qrt_expect_ok("restart before stop", luaif_restart(true, true));
	luaif_stop();
	qrt_expect("stop freed the arena", (unsigned)script_alloc_used(), 0u);

	/* --- and the engine comes back after a stop ----------------------- */
	luaif_host_output_reset();
	STAGE("restart after stop");
	flash_write_script("print('again=1')\n", true);
	qrt_expect_ok("restart after stop", luaif_restart(true, true));
	qrt_expect_ok("ran again after a stop",
			strstr(luaif_host_output(), "again=1") != NULL);

	/*
	 * --- a long script runs to completion ------------------------------
	 *
	 * Long enough to cross the instruction hook, which the other scripts
	 * here are not: they finish in fewer instructions than cfg.hook_count, so
	 * they never ask should_stop anything. That left the whole stop handshake
	 * untestable in the direction that matters most -- a stop request that is
	 * never cleared makes should_stop permanently true, which kills every
	 * script after the first restart, and every check still passed.
	 */
	luaif_host_output_reset();
	STAGE("long script");
	flash_write_script(
			"local n = 0\n"
			"for i = 1, 20000 do n = n + 1 end\n"
			"print('big=' .. n)\n", true);
	qrt_expect_ok("a long script is not cut short", luaif_restart(true, true));
	qrt_expect_ok("the long script finished",
			strstr(luaif_host_output(), "big=20000") != NULL);

	/*
	 * --- a handler stuck in a loop can still be stopped ---------------
	 *
	 * This is the one that makes should_stop worth having, and the reason
	 * flash_helper can trust luaif_stop. A script is not obliged to return:
	 * the stop has to reach it through the instruction hook and unwind it.
	 *
	 * Timed, because the failure mode is not a wrong answer but a slow one --
	 * with the hook ignored, the stop still "works" by timing out and tearing
	 * the engine down from under the caller a full second later. Without a
	 * deadline this check passes either way, which is how it was written the
	 * first time.
	 */
	luaif_host_output_reset();
	STAGE("looping handler");
	flash_write_script(
			"vesc.on_can(function(id, data)\n"
			"  while true do end\n"
			"end)\n", true);
	qrt_expect_ok("restart with a looping handler", luaif_restart(true, true));
	{
		uint8_t frame[1] = {0};
		systime_t t0;
		unsigned ms;

		luaif_process_can(0x1u, frame, sizeof(frame), false);
		chThdSleepMilliseconds(50);          /* let it get into the loop */

		t0 = chVTGetSystemTimeX();
		luaif_stop();
		ms = (unsigned)(chVTGetSystemTimeX() - t0)
				/ (CH_CFG_ST_FREQUENCY / 1000u);

		qrt_puts("stop_of_looping_handler_ms=");
		qrt_putu(ms);
		qrt_puts("\r\n");

		qrt_expect_ok("a looping handler was interrupted, not waited out",
				ms < 300u);
		qrt_expect("looping handler released the arena",
				(unsigned)script_alloc_used(), 0u);
	}

	/*
	 * --- the whole chain, from a CAN frame to the motor ----------------
	 *
	 * The point of the adapter is that a frame arriving on one thread ends up
	 * driving the motor from another, through a script. Everything up to here
	 * has tested a link at a time; this is the chain.
	 */
	STAGE("can frame drives the motor");
	luaif_host_output_reset();
	/*
	 * data is a Lua string, not a table -- script_lua_dispatch pushes the
	 * frame with lua_pushlstring -- so bytes come out with string.byte and
	 * data[1] is nil. Worth having a test that depends on it.
	 */
	flash_write_script(
			"vesc.on_can(function(id, data)\n"
			"  if id == 0x200 then\n"
			"    vesc.set_current_rel(data:byte(1) / 100.0)\n"
			"  end\n"
			"end)\n", true);
	qrt_expect_ok("restart with a motor-driving handler",
			luaif_restart(true, true));
	{
		uint8_t frame[1] = {50};

		fake_mc.set_current_rel = 0.0f;
		fake_timeout_resets = 0;

		luaif_process_can(0x200u, frame, sizeof(frame), false);
		chThdSleepMilliseconds(50);

		qrt_expect_ok("the frame reached the motor as a relative current",
				(fake_mc.set_current_rel > 0.49f)
				&& (fake_mc.set_current_rel < 0.51f));
		/* the binding's timeout refresh has to survive the trip too */
		qrt_expect_ok("driving the motor refreshed the timeout",
				fake_timeout_resets >= 1);
	}

	/*
	 * And a frame the script filters out must leave the motor alone. Without
	 * this the check above passes for a handler that ignores its arguments
	 * and drives the motor unconditionally.
	 */
	{
		uint8_t frame[1] = {99};

		fake_mc.set_current_rel = 0.0f;
		luaif_process_can(0x201u, frame, sizeof(frame), false);
		chThdSleepMilliseconds(50);
		qrt_expect_ok("a frame the script ignores does not drive the motor",
				fake_mc.set_current_rel == 0.0f);
	}

	/*
	 * --- stopping the script stops the motor ---------------------------
	 *
	 * luaif_stop releases the interpreter but says nothing about the motor,
	 * which keeps whatever it was last told until the timeout expires. That
	 * is the firmware's safety net doing its job rather than the adapter's,
	 * and it is worth recording which one is responsible: nothing here
	 * commands zero on stop.
	 */
	STAGE("stop leaves the motor to the timeout");
	{
		fake_mc.set_current_rel = 0.0f;
		fake_mc.released = 0;

		luaif_stop();
		qrt_expect("stopping the script does not itself release the motor",
				(unsigned)fake_mc.released, 0u);
	}

	/*
	 * The protocol runs on a thread the size of the one that really calls it.
	 *
	 * commands_process_packet runs on comm_usb's serial_process_thread, whose
	 * working area is 2048 bytes -- so that, not main's, is the stack
	 * luaif_process_cmd has to fit in. Driving it from main here was both
	 * unfaithful and too small: main's process stack is 0x800 and the checks
	 * faulted intermittently with UFSR.INVPC, three runs in five.
	 */
	STAGE("protocol (on a 2048-byte thread)");
	{
		thread_t *pt = chThdCreateStatic(wa_proto, sizeof(wa_proto),
				NORMALPRIO, proto_thread, NULL);
		qrt_expect_ok("protocol thread created", pt != NULL);
		chThdWait(pt);

		unsigned used = qrt_stack_used(wa_proto, sizeof(wa_proto));
		unsigned total = (unsigned)qrt_stack_total(sizeof(wa_proto));

		qrt_puts("protocol_stack_used=");
		qrt_putu(used);
		qrt_puts(" of ");
		qrt_putu(total);
		qrt_puts("\r\n");
		qrt_expect_ok("the protocol fits a comms-sized stack", used < total);
	}

	/* --- the thread is still alive and its stack is sane -------------- */
	qrt_expect_ok("restarts were counted", luaif_get_restart_cnt() >= 8);

	qrt_report();
	return 0;
}
