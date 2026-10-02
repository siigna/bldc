/*
 * A console session, for showing what the Lua engine actually does.
 *
 * This is the simulated STM32F405 (tests/qemu), not a board. Everything it
 * prints is real: the real ChibiOS kernel, the real engine, the real arena,
 * the real script out of script/examples, and measured numbers rather than
 * decoration. The inputs are the stand-in bindings, because a simulator has
 * no throttle.
 *
 *   make TEST=demo run
 */

#include "qrt.h"
#include "script_lua.h"
#include "script_alloc.h"
#include "lua_vesc_mc.h"
#include "lua_vesc_io.h"
#include "fake_io.h"

#include "mc_interface.h"
#include "timeout.h"

#include <string.h>
#include <stdio.h>

static const char m_script[] =
#include "bench_throttle.inc"
;

#define ARENA_SIZE (40u * 1024u)
static uint8_t m_arena[ARENA_SIZE] __attribute__((aligned(8)));
static THD_WORKING_AREA(wa_engine, 12288) __attribute__((section(".ram4")));

int commands_printf_lisp(const char *format, ...) {
	(void)format;
	return 0;
}

/* script print() goes to the console, prefixed the way the firmware would */
static void engine_print(const char *msg) {
	qrt_puts("  lua> ");
	qrt_puts(msg);
	qrt_puts("\r\n");
}

static bool no_stop(void) {
	return false;
}

static script_lua_t *m_s;

static void say(const char *s) {
	qrt_puts(s);
	qrt_puts("\r\n");
}

static void run(const char *src) {
	char err[160];

	if (!script_lua_run(m_s, src, -1, "console", err, sizeof(err))) {
		qrt_puts("  error: ");
		qrt_puts(err);
		qrt_puts("\r\n");
	}
}

static float number(const char *expr) {
	char src[128];
	char err[160];

	snprintf(src, sizeof(src), "_v = %s", expr);
	if (!script_lua_run(m_s, src, -1, "console", err, sizeof(err))) {
		return -1.0f;
	}

	lua_State *L = script_lua_state(m_s);
	lua_getglobal(L, "_v");
	float v = (float)lua_tonumber(L, -1);
	lua_pop(L, 1);
	return v;
}

static void milli(const char *label, float v, const char *unit) {
	qrt_puts(label);
	qrt_putu((unsigned)v);
	qrt_puts(".");
	{
		unsigned frac = (unsigned)((v - (float)(unsigned)v) * 100.0f);
		if (frac < 10u) {
			qrt_puts("0");
		}
		qrt_putu(frac);
	}
	qrt_puts(unit);
}

static THD_FUNCTION(engine, arg) {
	(void)arg;
	chRegSetThreadName("lua");

	say("");
	say("  ESCargot firmware -- script engine: Lua  (USE_LISPBM=0 USE_LUA=1)");
	say("  target: STM32F405, ChibiOS 3.0.5          [QEMU, not a board]");
	say("");

	script_lua_cfg_t cfg = {0};
	cfg.mem_limit = 32u * 1024u;
	cfg.print = engine_print;
	cfg.should_stop = no_stop;
	cfg.hook_count = 2000;
	cfg.alloc = script_alloc;

	m_s = script_lua_open(&cfg);
	if (m_s == NULL) {
		say("  engine: failed to open");
		return;
	}
	script_lua_register(m_s, lua_vesc_mc_fns());
	script_lua_register(m_s, lua_vesc_io_fns());

	say("  [  OK  ] interpreter heap: 40 KB arena, 32 KB script ceiling");
	say("  [  OK  ] engine thread at NORMALPRIO-1, 12 KB stack in CCM");
	say("  [  OK  ] bindings: motor, config, CAN, inputs");
	say("");

	fake_io_reset();
	fake_mc.vin = 16.8f;			/* 4s */
	fake_mc.rpm = 0.0f;

	/* --- load the script ------------------------------------------- */
	{
		static char wrapped[8192];
		char err[256];

		snprintf(wrapped, sizeof(wrapped),
				"BENCH_TEST = true M = (function()\n%s\nend)()", m_script);
		say("  loading script: examples/bench_throttle.lua");

		if (!script_lua_run(m_s, wrapped, -1, "bench_throttle",
				err, sizeof(err))) {
			qrt_puts("  load failed: ");
			say(err);
			return;
		}
	}

	qrt_puts("  [  OK  ] loaded, ");
	qrt_putu((unsigned)script_lua_mem_used(m_s));
	say(" bytes of script memory");
	say("");

	/* --- a REPL-ish look at the board ------------------------------- */
	say("  > vesc.get_vin()");
	milli("    ", number("vesc.get_vin()"), " V\r\n");
	say("  > M.cfg.max_current");
	milli("    ", number("M.cfg.max_current"), " A\r\n");
	say("");

	/* --- the throttle, stepped ------------------------------------- */
	say("  driving the throttle (RC input, 50 Hz steps)");
	say("");
	say("    throttle   state                        current");
	say("    --------   --------------------------   -------");

	struct { float ppm; float age; int steps; const char *note; } seq[] = {
		{0.80f, 0.00f,  1, NULL},		/* held at power-up */
		{0.00f, 0.00f,  1, NULL},		/* returned */
		{1.00f, 0.00f,  1, NULL},		/* first ramp step */
		{1.00f, 0.00f, 10, NULL},
		{1.00f, 0.00f, 60, NULL},
		{0.50f, 0.00f,  1, NULL},
		{1.00f, 1.00f,  1, "transmitter off"},
		{1.00f, 0.00f,  1, "link back, stick still full"},
		{0.00f, 0.00f,  1, NULL},
	};

	for (unsigned i = 0; i < sizeof(seq) / sizeof(seq[0]); i++) {
		char src[96];

		fake_io.ppm = seq[i].ppm;
		fake_io.ppm_age = seq[i].age;

		snprintf(src, sizeof(src),
				"for _ = 1, %d do _a = M.step(0.02) end _r = M.reason",
				seq[i].steps);
		run(src);

		lua_State *L = script_lua_state(m_s);
		lua_getglobal(L, "_a");
		float amps = (float)lua_tonumber(L, -1);
		lua_pop(L, 1);
		lua_getglobal(L, "_r");
		const char *why = lua_tostring(L, -1);
		char reason[40];
		snprintf(reason, sizeof(reason), "%s", (why != NULL) ? why : "");
		lua_pop(L, 1);

		qrt_puts("      ");
		milli("", seq[i].ppm, "    ");
		qrt_puts(reason);
		for (unsigned p = (unsigned)strlen(reason); p < 29u; p++) {
			qrt_puts(" ");
		}
		milli("", amps, " A");
		if (seq[i].note != NULL) {
			qrt_puts("   <- ");
			qrt_puts(seq[i].note);
		}
		say("");
	}

	say("");
	say("  measured, this session:");
	qrt_puts("    script memory peak   ");
	qrt_putu((unsigned)script_lua_mem_peak(m_s));
	say(" bytes");
	qrt_puts("    arena peak           ");
	qrt_putu((unsigned)script_alloc_peak());
	qrt_puts(" of ");
	qrt_putu(ARENA_SIZE);
	say(" bytes");
	qrt_puts("    engine thread stack  ");
	qrt_putu(qrt_stack_used(wa_engine, sizeof(wa_engine)));
	qrt_puts(" of ");
	qrt_putu((unsigned)qrt_stack_total(sizeof(wa_engine)));
	say(" bytes");

	script_lua_close(m_s);
	qrt_puts("    arena after close    ");
	qrt_putu((unsigned)script_alloc_used());
	say(" bytes");
	m_s = NULL;
	say("");
}

int main(void) {
	qrt_console_init();
	chSysInit();
	qrt_systick_init();

	script_alloc_init(m_arena, sizeof(m_arena), sizeof(m_arena));

	thread_t *tp = chThdCreateStatic(wa_engine, sizeof(wa_engine),
			NORMALPRIO - 1, engine, NULL);
	chThdWait(tp);

	qrt_exit(0);
	return 0;
}
