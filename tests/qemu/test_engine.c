/*
 * The Lua engine on the simulated target: real ChibiOS, real Thumb-2, real
 * CCM arena, running on a thread whose working area is in .ram4 exactly as
 * the firmware adapter will place it.
 *
 * This is what the simulator was built for. Until now the engine had only
 * been run on a host, against the C library's realloc and with no thread at
 * all, so two of its claims were untested: that it works through the
 * script_alloc arena rather than malloc, and that the engine thread's stack
 * fits in the little that .ram4 has.
 *
 * What it still cannot show is anything board-specific -- no flash load, no
 * bindings, no timing. See README.md.
 */

#include "qrt.h"
#include "script_lua.h"
#include "script_alloc.h"

#include <string.h>

/*
 * The interpreter heap. In a USE_LUA build LispBM's heap and evaluator stack
 * are gone, which is what frees the CCM this sits in. 32 KB here rather than
 * the full region so the engine thread's working area still fits beside it.
 */
#ifndef ARENA_SIZE
#define ARENA_SIZE  (32u * 1024u)
#endif
static uint8_t m_arena[ARENA_SIZE] __attribute__((section(".ram4")))
		__attribute__((aligned(8)));

/* The engine thread. Sizing this is the measurement that needed a board. */
#ifndef NEST_DEPTH
#define NEST_DEPTH 100
#endif
/* two levels: one-level #x stringizes the macro name, not its value */
#define STR_(x) #x
#define XSTR(x) STR_(x)
#define NEST_DEPTH_STR XSTR(NEST_DEPTH)
#ifndef META_DEPTH
#define META_DEPTH 60   /* past the C-call limit on purpose */
#endif
#define META_DEPTH_STR XSTR(META_DEPTH)

/*
 * 12 KB, which is what script.mk's LUAI_MAXCCALLS=16 is sized against:
 * 2088 bytes of baseline plus 16 levels at 464 bytes is 9,512, leaving about
 * 2.8 KB spare. Those two numbers have to move together.
 */
#ifndef ENGINE_WA_SIZE
#define ENGINE_WA_SIZE  12288
#endif
static THD_WORKING_AREA(wa_engine, ENGINE_WA_SIZE)
		__attribute__((section(".ram4")));

/* --------------------------------------------------------- print capture -- */

static char m_out[512];
static size_t m_out_len;

static void capture_print(const char *msg) {
	size_t n = strlen(msg);

	if (m_out_len + n + 1u < sizeof(m_out)) {
		memcpy(m_out + m_out_len, msg, n);
		m_out_len += n;
		m_out[m_out_len] = '\0';
	}
	qrt_puts("  [script] ");
	qrt_puts(msg);
	qrt_puts("\r\n");
}

static bool m_stop;
static bool should_stop(void) {
	return m_stop;
}

/* ----------------------------------------------------------- engine thread -- */

static volatile bool m_done;

static THD_FUNCTION(engine, arg) {
	(void)arg;
	chRegSetThreadName("lua");

	script_lua_cfg_t cfg = {0};
	cfg.mem_limit  = 24u * 1024u;      /* below the arena, so this is what bites */
	cfg.print      = capture_print;
	cfg.should_stop = should_stop;
	cfg.hook_count = 1000;
	cfg.alloc      = script_alloc;     /* the whole point: CCM, not malloc */
	cfg.alloc_ud   = NULL;

	script_lua_t *s = script_lua_open(&cfg);
	qrt_expect_ok("engine opened", s != NULL);

	if (s != NULL) {
		char err[128];

		/* the interpreter is really running out of the arena, not malloc */
		qrt_expect_ok("arena in use after open", script_alloc_used() > 0u);
		qrt_expect_ok("arena is the ccm block",
				((unsigned)(size_t)m_arena >> 24) == 0x10u);

		/* arithmetic, strings and tables: enough to know the VM works */
		bool ok = script_lua_run(s,
				"local t = {}\n"
				"for i = 1, 10 do t[i] = i * i end\n"
				"local sum = 0\n"
				"for _, v in ipairs(t) do sum = sum + v end\n"
				"print('sum=' .. sum)\n"
				"print('fmt=' .. string.format('%d/%s', 7, 'x'))\n",
				-1, "t1", err, sizeof(err));
		if (!ok) {
			qrt_puts("  run error: ");
			qrt_puts(err);
			qrt_puts("\r\n");
		}
		qrt_expect_ok("chunk ran", ok);
		qrt_expect_ok("sum correct", strstr(m_out, "sum=385") != NULL);
		qrt_expect_ok("string.format works", strstr(m_out, "fmt=7/x") != NULL);

		/* a syntax error is reported, not fatal */
		qrt_expect_ok("syntax error caught",
				!script_lua_run(s, "this is not lua", -1, "t2",
						err, sizeof(err)));

		/* a runtime error unwinds and leaves the interpreter usable */
		qrt_expect_ok("runtime error caught",
				!script_lua_run(s, "error('boom')", -1, "t3",
						err, sizeof(err)));
		qrt_expect_ok("still usable after error",
				script_lua_run(s, "print('alive')", -1, "t4",
						err, sizeof(err)));
		qrt_expect_ok("alive printed", strstr(m_out, "alive") != NULL);

#ifndef SKIP_RUNAWAY
		/*
		 * The memory ceiling, which is the containment mechanism for a
		 * runaway script. It must fail the script, not the firmware.
		 */
		qrt_expect_ok("runaway refused",
				!script_lua_run(s,
						"local t = {}\n"
						"for i = 1, 1e9 do t[i] = string.rep('x', 64) end\n",
						-1, "t5", err, sizeof(err)));
		qrt_expect_ok("still usable after oom",
				script_lua_run(s, "print('after-oom')", -1, "t6",
						err, sizeof(err)));
#endif

		qrt_puts("mem_used=");  qrt_putu((unsigned)script_lua_mem_used(s));
		qrt_puts(" mem_peak="); qrt_putu((unsigned)script_lua_mem_peak(s));
		qrt_puts("\r\narena_used="); qrt_putu((unsigned)script_alloc_used());
		qrt_puts(" arena_peak="); qrt_putu((unsigned)script_alloc_peak());
		qrt_puts(" of "); qrt_putu(ARENA_SIZE);
		qrt_puts("\r\n");

		qrt_expect_ok("peak within arena", script_alloc_peak() <= ARENA_SIZE);

		/*
		 * Stack sizing. Lua-level calls live on the interpreter's own stack,
		 * but nested pcall and metamethods recurse on the C stack, so the
		 * worst case is bounded by LUAI_MAXCCALLS rather than by the script.
		 * This drives it deliberately deep to find out what that costs.
		 */
		qrt_expect_ok("deep c-nesting survives",
				script_lua_run(s,
						"local function nest(n)\n"
						"  if n == 0 then return 0 end\n"
						"  local ok, v = pcall(nest, n - 1)\n"
						"  if not ok then error(v, 0) end\n"
						"  return v + 1\n"
						"end\n"
						"local ok, v = pcall(nest, " NEST_DEPTH_STR ")\n"
						"print('nest=' .. tostring(ok) .. ':' .. tostring(v))\n",
						-1, "t7", err, sizeof(err)));
		qrt_puts("  nest_err=");
		qrt_puts(err[0] ? err : "(none)");
		qrt_puts("\r\nstack_after_nesting=");
		qrt_putu(qrt_stack_used(wa_engine, sizeof(wa_engine)));
		qrt_puts("\r\n");

#ifndef SKIP_META
		/*
		 * A metamethod chain is the other way C depth grows, and this one is
		 * deliberately deeper than LUAI_MAXCCALLS. The property that matters
		 * is that it is *refused* -- a Lua error the script could catch --
		 * rather than taking the stack out from under the firmware.
		 */
		qrt_expect_ok("deep metamethod chain refused, not fatal",
				!script_lua_run(s,
						"local function mk(inner)\n"
						"  return setmetatable({}, {__index = function(_, k)\n"
						"    return inner[k]\n"
						"  end})\n"
						"end\n"
						"local t = {x = 1}\n"
						"for _ = 1, " META_DEPTH_STR " do t = mk(t) end\n"
						"print('meta=' .. tostring(t.x))\n",
						-1, "t8", err, sizeof(err)));
		qrt_puts("  meta_err=");
		qrt_puts(err[0] ? err : "(none)");
		qrt_puts("\r\n");
		qrt_expect_ok("usable after metamethod refusal",
				script_lua_run(s, "print('after-meta')", -1, "t9",
						err, sizeof(err)));
#endif

		script_lua_close(s);
		qrt_expect("everything freed", (unsigned)script_alloc_used(), 0u);
	}

	m_done = true;

}

int main(void) {
	qrt_console_init();
	chSysInit();
	qrt_systick_init();

	qrt_puts("engine test\r\n");

	/* The arena's own limit is the arena; cfg.mem_limit is the one a script
	 * reaches first, so a runaway gets a Lua error rather than a dry heap. */
	script_alloc_init(m_arena, sizeof(m_arena), sizeof(m_arena));

	thread_t *tp = chThdCreateStatic(wa_engine, sizeof(wa_engine),
			NORMALPRIO, engine, NULL);
	qrt_expect_ok("engine thread created", tp != NULL);
	chThdWait(tp);
	qrt_expect_ok("engine thread finished", m_done);

	{
		unsigned used  = qrt_stack_used(wa_engine, sizeof(wa_engine));
		unsigned total = (unsigned)qrt_stack_total(sizeof(wa_engine));

		qrt_puts("engine_stack_used=");
		qrt_putu(used);
		qrt_puts(" of ");
		qrt_putu(total);
		qrt_puts("\r\n");
		qrt_expect_ok("engine stack measured", used > 0u);
		qrt_expect_ok("engine stack did not overflow", used < total);
	}

	qrt_report();
	return 0;
}
