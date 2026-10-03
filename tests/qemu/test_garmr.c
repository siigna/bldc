/*
 * The Garmr prototype's engage logic, on the real kernel with no board.
 *
 * What is under test is the decision about when to ask for the hold: the
 * Schmitt trigger, the validity window that catches a disconnected pin, the
 * debounce, and that the keepalive is actually called every pass. The hold
 * itself is app_pas.c's walk assist and is covered by tests/app_pas.
 *
 * The script is cut from vesc_pkg/garmr/garmr.lua by the Makefile, so this
 * tests the file that ships rather than a copy of it.
 */

#include "ch.h"
#include "qrt.h"

#include "script_lua.h"
#include "script_alloc.h"
#include "lua_vesc_mc.h"
#include "lua_vesc_io.h"
#include "lua_vesc_pas.h"
#include "mc_interface.h"   // fake_mc
#include "fake_io.h"
#include "fake_pas.h"

#include <stdio.h>
#include <string.h>

static const char m_script[] =
#include "garmr.inc"
;

#define ARENA_SIZE  (40u * 1024u)
static uint8_t m_arena[ARENA_SIZE] __attribute__((aligned(8)));
static THD_WORKING_AREA(wa_engine, 12288) __attribute__((section(".ram4")));

static script_lua_t *m_s;

static void capture(const char *msg) {
	(void)msg;
}

static bool no_stop(void) {
	return false;
}

/* One pass of the script's own step(), with dt in seconds. */
static void step(float dt) {
	char src[96];
	char err[192];

	snprintf(src, sizeof(src), "_e = M.step(%d/1000.0) _r = M.reason",
			(int)(dt * 1000.0f));

	if (!script_lua_run(m_s, src, -1, "step", err, sizeof(err))) {
		qrt_puts("  step error: ");
		qrt_puts(err);
		qrt_puts("\r\n");
	}
}

static bool engaged(void) {
	lua_State *L = script_lua_state(m_s);

	lua_getglobal(L, "_e");
	bool v = lua_toboolean(L, -1) != 0;
	lua_pop(L, 1);
	return v;
}

static bool reason_is(const char *want) {
	lua_State *L = script_lua_state(m_s);

	lua_getglobal(L, "_r");
	const char *r = lua_tostring(L, -1);
	bool match = (r != NULL) && (strstr(r, want) != NULL);
	lua_pop(L, 1);
	return match;
}

/* Hold the switch on long enough for the debounce to pass. */
static void settle_on(void) {
	for (int i = 0; i < 5; i++) {
		step(0.1f);
		chThdSleepMilliseconds(30);
	}
}

static THD_FUNCTION(engine, arg) {
	(void)arg;
	chRegSetThreadName("lua");

	script_lua_cfg_t cfg = {0};
	cfg.mem_limit = 32u * 1024u;
	cfg.print = capture;
	cfg.should_stop = no_stop;
	cfg.hook_count = 2000;
	cfg.alloc = script_alloc;

	m_s = script_lua_open(&cfg);
	qrt_expect_ok("engine opened", m_s != NULL);
	if (m_s == NULL) {
		return;
	}

	script_lua_register(m_s, lua_vesc_mc_fns());
	script_lua_register(m_s, lua_vesc_io_fns());
	script_lua_register(m_s, lua_vesc_pas_fns());

	{
		lua_State *L = script_lua_state(m_s);

		lua_pushboolean(L, 1);
		lua_setglobal(L, "GARMR_TEST");
	}

	fake_io_reset();
	fake_pas_reset();

	{
		static char wrapped[8192];
		char err[256];
		int n = snprintf(wrapped, sizeof(wrapped),
				"M = (function()\n%s\nend)()", m_script);

		qrt_expect_ok("the wrapped script fits the buffer",
				(n > 0) && (n < (int)sizeof(wrapped)));
		qrt_expect_ok("the prototype loads",
				script_lua_run(m_s, wrapped, -1, "garmr", err, sizeof(err)));
	}

	/* --- the switch, off ------------------------------------------- */
	fake_io.adc_volts = 0.5f;
	step(0.1f);
	qrt_expect_ok("a switch that is off does not engage", !engaged());
	qrt_expect_ok("and the keepalive is still called, to release",
			fake_pas.walk_calls > 0);
	qrt_expect_ok("with the request clear", !fake_pas.walk_requested);

	/* --- the debounce ---------------------------------------------- */
	fake_io.adc_volts = 2.5f;
	step(0.1f);
	qrt_expect_ok("one pass above the threshold is not enough", !engaged());
	qrt_expect_ok("and it says why", reason_is("debounc"));

	settle_on();
	qrt_expect_ok("holding the switch engages", engaged());
	qrt_expect_ok("the firmware was asked to hold", fake_pas.walk_requested);

	/* --- the Schmitt trigger --------------------------------------- */
	fake_io.adc_volts = 1.8f;		/* below on, above off */
	step(0.1f);
	qrt_expect_ok("between the thresholds it stays engaged", engaged());

	fake_io.adc_volts = 1.2f;		/* below off */
	step(0.1f);
	qrt_expect_ok("below the lower threshold it releases", !engaged());
	qrt_expect_ok("and the request is cleared", !fake_pas.walk_requested);

	/* --- a disconnected or shorted pin ------------------------------ */
	settle_on();			/* back on first, so the release is the point */
	fake_io.adc_volts = 2.5f;
	settle_on();
	qrt_expect_ok("engaged again before the next check", engaged());

	fake_io.adc_volts = 0.02f;		/* below the validity window */
	step(0.1f);
	qrt_expect_ok("a pin reading near zero releases", !engaged());
	qrt_expect_ok("and names the input, not the switch",
			reason_is("out of range"));

	/*
	 * Held, not stepped once. A single pass cannot engage anything anyway
	 * because of the debounce, so a one-step check here passes whether or
	 * not the validity window exists -- which is exactly what the first
	 * version of these two did.
	 */
	fake_io.adc_volts = 3.3f;		/* above the window, and above "on" */
	settle_on();
	qrt_expect_ok("a pin held at the rail never engages", !engaged());
	qrt_expect_ok("and the firmware is not asked to hold",
			!fake_pas.walk_requested);

	/*
	 * A floating pin sitting mid-rail is the failure most likely to look
	 * like a working switch. 1.65 V is below the on threshold, so the
	 * Schmitt alone would also reject it -- push it above the threshold but
	 * still inside a plausible float to make this about the window.
	 */
	fake_io.adc_volts = 3.25f;
	settle_on();
	qrt_expect_ok("a floating pin above the window never engages",
			!engaged());

	/* --- the vetoes ------------------------------------------------- */
	fake_io.adc_volts = 2.5f;
	settle_on();
	qrt_expect_ok("engaged before the vetoes", engaged());

	fake_pas.flags = 1 << 3;		/* PAS_FLAG_BRAKE_ENGAGED */
	step(0.1f);
	qrt_expect_ok("the brake releases it", !engaged());
	qrt_expect_ok("and says so", reason_is("brake"));
	qrt_expect_ok("the firmware is told to release", !fake_pas.walk_requested);

	fake_pas.flags = 0;
	settle_on();
	qrt_expect_ok("and it re-engages once the brake is off", engaged());

	fake_mc.fault = 1;
	step(0.1f);
	qrt_expect_ok("a fault releases it", !engaged());
	qrt_expect_ok("and says so", reason_is("fault"));

	fake_mc.fault = 0;
	fake_pas.flags = 1 << 2;		/* PAS_FLAG_BRAKE_CH_INVALID */
	settle_on();
	qrt_expect_ok("an invalid brake channel keeps it released", !engaged());

	/*
	 * The keepalive is the deadman: the firmware releases half a second
	 * after the last call, so what matters is that every pass makes one.
	 */
	fake_pas.flags = 0;
	fake_mc.fault = 0;
	fake_io.adc_volts = 2.5f;
	settle_on();

	int before = fake_pas.walk_calls;
	step(0.1f);
	step(0.1f);
	step(0.1f);
	qrt_expect("three passes, three keepalives",
			(unsigned)(fake_pas.walk_calls - before), 3u);

	script_lua_close(m_s);
	m_s = NULL;
}

int main(void) {
	qrt_console_init();
	chSysInit();
	qrt_systick_init();

	qrt_puts("garmr engage logic test\r\n");
	script_alloc_init(m_arena, sizeof(m_arena), sizeof(m_arena));

	thread_t *tp = chThdCreateStatic(wa_engine, sizeof(wa_engine),
			NORMALPRIO - 1, engine, NULL);
	qrt_expect_ok("engine thread created", tp != NULL);
	chThdWait(tp);

	qrt_puts("engine_stack_used=");
	qrt_putu(qrt_stack_used(wa_engine, sizeof(wa_engine)));
	qrt_puts("\r\n");

	qrt_report();
	return 0;
}
