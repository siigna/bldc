/*
 * The motor bindings, on the simulated F405.
 *
 * script/lua_vesc_mc.c is compiled against a fake mc_interface that lives
 * earlier on the include path, so the bindings themselves carry no test seam.
 * Every getter is checked against a value the fake holds and every setter
 * against what the fake recorded, which catches a binding wired to the wrong
 * field -- the kind of mistake that reads perfectly well.
 *
 * What this cannot show is whether the real mc_interface returns sensible
 * numbers, or what a motor does when told to. That is bench work.
 */

#include "qrt.h"
#include "script_lua.h"
#include "script_alloc.h"
#include "lua_vesc_mc.h"

#include "mc_interface.h"
#include "timeout.h"

#include <string.h>

#define ARENA_SIZE  (24u * 1024u)
static uint8_t m_arena[ARENA_SIZE] __attribute__((aligned(8)));
static THD_WORKING_AREA(wa_engine, 12288) __attribute__((section(".ram4")));

static char m_out[512];
static size_t m_out_len;

static void capture(const char *msg) {
	size_t n = strlen(msg);

	if (m_out_len + n + 1u < sizeof(m_out)) {
		memcpy(m_out + m_out_len, msg, n);
		m_out_len += n;
		m_out[m_out_len] = '\0';
	}
}

static void out_reset(void) {
	m_out_len = 0;
	m_out[0] = '\0';
}

static bool no_stop(void) {
	return false;
}

static script_lua_t *m_s;

/* Runs a chunk and fails the named check if it raised. */
static bool run(const char *what, const char *src) {
	char err[128];

	if (!script_lua_run(m_s, src, -1, "t", err, sizeof(err))) {
		qrt_puts("  error in ");
		qrt_puts(what);
		qrt_puts(": ");
		qrt_puts(err);
		qrt_puts("\r\n");
		qrt_expect_ok(what, 0);
		return false;
	}
	return true;
}

/*
 * Checks a getter by having Lua print it and comparing the text, which also
 * proves the value survives the C-to-Lua number conversion. Comparing as text
 * keeps the test independent of float formatting in printf on the target.
 */
static void expect_getter(const char *name, const char *call,
		const char *expect) {
	char src[160];

	out_reset();
	strcpy(src, "print(");
	strcat(src, call);
	strcat(src, ")");
	if (run(name, src)) {
		if (strstr(m_out, expect) == NULL) {
			qrt_puts("  ");
			qrt_puts(name);
			qrt_puts(": got [");
			qrt_puts(m_out);
			qrt_puts("] want [");
			qrt_puts(expect);
			qrt_puts("]\r\n");
		}
		qrt_expect_ok(name, strstr(m_out, expect) != NULL);
	}
}

/*
 * Every setter in the table, with the call to make and where the fake should
 * record it.
 *
 * Driven from a list rather than written out one at a time because the
 * property that matters applies to all of them equally: a setter that forgets
 * timeout_reset works until the first gap longer than the timeout, and then
 * the output drops out with nothing to show why. Adding a setter without
 * adding it here is the only way to miss that, which is a smaller hole than
 * remembering the rule by hand twenty times.
 */
typedef struct {
	const char *name;
	const char *call;
	float *recorded;
	float expect;
} setter_t;

static THD_FUNCTION(engine, arg) {
	(void)arg;
	chRegSetThreadName("lua");

	script_lua_cfg_t cfg = {0};
	cfg.mem_limit = 20u * 1024u;
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

	/* --- the whole table is attached ---------------------------------- */
	out_reset();
	run("table attached", "print(type(vesc.get_rpm) .. type(vesc.set_duty))");
	qrt_expect_ok("bindings are functions",
			strstr(m_out, "functionfunction") != NULL);

	/* --- getters read the field they claim to ------------------------- */
	fake_mc.rpm         = 1234.0f;
	fake_mc.current     = 11.0f;
	fake_mc.current_dir = -12.0f;
	fake_mc.current_in  = 13.0f;
	fake_mc.duty        = 0.5f;
	fake_mc.vin         = 48.0f;
	fake_mc.temp_fet    = 41.0f;
	fake_mc.temp_motor  = 42.0f;
	fake_mc.speed       = 7.0f;
	fake_mc.dist        = 100.0f;
	fake_mc.dist_abs    = 200.0f;
	fake_mc.ah          = 3.0f;
	fake_mc.ah_chg      = 4.0f;
	fake_mc.wh          = 5.0f;
	fake_mc.wh_chg      = 6.0f;
	fake_mc.tacho       = 777;
	fake_mc.tacho_abs   = 888;
	fake_mc.fault       = FAULT_CODE_OVER_TEMP_FET;

	expect_getter("get_rpm",         "vesc.get_rpm()",         "1234");
	expect_getter("get_current",     "vesc.get_current()",     "11");
	expect_getter("get_current_dir", "vesc.get_current_dir()", "-12");
	expect_getter("get_current_in",  "vesc.get_current_in()",  "13");
	expect_getter("get_duty",        "vesc.get_duty()",        "0.5");
	expect_getter("get_vin",         "vesc.get_vin()",         "48");
	expect_getter("get_temp_fet",    "vesc.get_temp_fet()",    "41");
	expect_getter("get_temp_mot",    "vesc.get_temp_mot()",    "42");
	expect_getter("get_speed",       "vesc.get_speed()",       "7");
	expect_getter("get_dist",        "vesc.get_dist()",        "100");
	expect_getter("get_dist_abs",    "vesc.get_dist_abs()",    "200");
	expect_getter("get_ah",          "vesc.get_ah()",          "3");
	expect_getter("get_ah_chg",      "vesc.get_ah_chg()",      "4");
	expect_getter("get_wh",          "vesc.get_wh()",          "5");
	expect_getter("get_wh_chg",      "vesc.get_wh_chg()",      "6");
	expect_getter("get_tacho",       "vesc.get_tacho()",       "777");
	expect_getter("get_tacho_abs",   "vesc.get_tacho_abs()",   "888");
	/* the fault is an integer code, not a string, same as lisp */
	expect_getter("get_fault",       "vesc.get_fault()",       "5");

	/* --- a plain counter read must not reset the counter --------------- */
	fake_mc.ah_reset = true;
	fake_mc.wh_reset = true;
	fake_mc.tacho_reset = true;
	run("counter read",
			"vesc.get_ah()\n"
			"vesc.get_wh()\n"
			"vesc.get_tacho()\n");
	qrt_puts("  ah_reset=");
	qrt_putu(fake_mc.ah_reset ? 1u : 0u);
	qrt_puts(" sizeof(fake_mc)=");
	qrt_putu((unsigned)sizeof(fake_mc));
	qrt_puts("\r\n");
	qrt_expect_ok("reading amp-hours does not reset them", !fake_mc.ah_reset);
	qrt_expect_ok("reading watt-hours does not reset them", !fake_mc.wh_reset);
	qrt_expect_ok("reading the tachometer does not reset it",
			!fake_mc.tacho_reset);

	run("counter reset", "vesc.get_ah(true) vesc.get_wh(true)");
	qrt_expect_ok("amp-hours can be reset explicitly", fake_mc.ah_reset);
	qrt_expect_ok("watt-hours can be reset explicitly", fake_mc.wh_reset);

	/* --- every setter passes its value through and refreshes the timeout */
	static const setter_t setters[] = {
		{"set_current",       "vesc.set_current(7.5)",        &fake_mc.set_current,       7.5f},
		{"set_current_rel",   "vesc.set_current_rel(0.25)",   &fake_mc.set_current_rel,   0.25f},
		{"set_duty",          "vesc.set_duty(0.4)",           &fake_mc.set_duty,          0.4f},
		{"set_rpm",           "vesc.set_rpm(3000)",           &fake_mc.set_rpm,           3000.0f},
		{"set_pos",           "vesc.set_pos(90)",             &fake_mc.set_pos,           90.0f},
		{"set_brake",         "vesc.set_brake(5)",            &fake_mc.set_brake,         5.0f},
		{"set_brake_rel",     "vesc.set_brake_rel(0.6)",      &fake_mc.set_brake_rel,     0.6f},
		{"set_handbrake",     "vesc.set_handbrake(8)",        &fake_mc.set_handbrake,     8.0f},
		{"set_handbrake_rel", "vesc.set_handbrake_rel(0.75)", &fake_mc.set_handbrake_rel, 0.75f},
	};

	for (unsigned i = 0; i < (sizeof(setters) / sizeof(setters[0])); i++) {
		const setter_t *s = &setters[i];
		char what[64];

		*s->recorded = 0.0f;
		fake_timeout_resets = 0;
		fake_mc.set_calls = 0;

		if (!run(s->name, s->call)) {
			continue;
		}

		strcpy(what, s->name);
		strcat(what, " reached the motor");
		qrt_expect_ok(what, (*s->recorded > (s->expect - 0.001f))
				&& (*s->recorded < (s->expect + 0.001f)));

		strcpy(what, s->name);
		strcat(what, " refreshed the timeout");
		qrt_expect(what, (unsigned)fake_timeout_resets, 1u);

		strcpy(what, s->name);
		strcat(what, " called the motor once");
		qrt_expect(what, (unsigned)fake_mc.set_calls, 1u);
	}

	/* --- release and the bare timeout refresh ------------------------- */
	fake_mc.released = 0;
	fake_timeout_resets = 0;
	run("release_motor", "vesc.release_motor()");
	qrt_expect("release_motor released", (unsigned)fake_mc.released, 1u);
	qrt_expect("release_motor refreshed the timeout",
			(unsigned)fake_timeout_resets, 1u);

	fake_timeout_resets = 0;
	run("timeout_reset", "vesc.timeout_reset()");
	qrt_expect("timeout_reset refreshed the timeout",
			(unsigned)fake_timeout_resets, 1u);

	/* --- the optional off-delay second argument ----------------------- */
	fake_mc.set_off_delay = 0.0f;
	fake_mc.off_delay_calls = 0;
	run("off delay default", "vesc.set_current(1)");
	/*
	 * Counted, not compared. Reading an absent argument with lua_tonumber
	 * yields 0, and calling set_current_off_delay(0) is not the same as not
	 * calling it -- zero clears whatever delay was set before. Asserting the
	 * recorded value was still 0 cannot tell those apart, and did not.
	 */
	qrt_expect("one argument does not touch the off-delay",
			(unsigned)fake_mc.off_delay_calls, 0u);

	fake_mc.off_delay_calls = 0;
	run("off delay given", "vesc.set_current(1, 0.75)");
	qrt_expect("a second argument sets the off-delay once",
			(unsigned)fake_mc.off_delay_calls, 1u);
	qrt_expect_ok("the off-delay has its value",
			(fake_mc.set_off_delay > 0.74f)
			&& (fake_mc.set_off_delay < 0.76f));

	/* same for the relative form, which takes the same optional argument */
	fake_mc.off_delay_calls = 0;
	run("rel off delay default", "vesc.set_current_rel(0.5)");
	qrt_expect("set_current_rel with one argument leaves it alone",
			(unsigned)fake_mc.off_delay_calls, 0u);

	fake_mc.off_delay_calls = 0;
	run("rel off delay given", "vesc.set_current_rel(0.5, 0.25)");
	qrt_expect("set_current_rel passes its off-delay",
			(unsigned)fake_mc.off_delay_calls, 1u);

	/* --- a bad argument is a Lua error, not a wild value --------------- */
	{
		char err[128];

		fake_mc.set_duty = 0.0f;
		fake_mc.set_calls = 0;
		qrt_expect_ok("a non-number argument is refused",
				!script_lua_run(m_s, "vesc.set_duty('fast')", -1, "t",
						err, sizeof(err)));
		qrt_expect("a refused call never reached the motor",
				(unsigned)fake_mc.set_calls, 0u);

		qrt_expect_ok("a missing argument is refused",
				!script_lua_run(m_s, "vesc.set_duty()", -1, "t",
						err, sizeof(err)));
		qrt_expect("a missing argument never reached the motor",
				(unsigned)fake_mc.set_calls, 0u);
	}

	script_lua_close(m_s);
	m_s = NULL;
}

int main(void) {
	qrt_console_init();
	chSysInit();
	qrt_systick_init();

	qrt_puts("bindings test\r\n");
	script_alloc_init(m_arena, sizeof(m_arena), sizeof(m_arena));

	thread_t *tp = chThdCreateStatic(wa_engine, sizeof(wa_engine),
			NORMALPRIO - 1, engine, NULL);
	qrt_expect_ok("engine thread created", tp != NULL);
	chThdWait(tp);

	qrt_puts("engine_stack_used=");
	qrt_putu(qrt_stack_used(wa_engine, sizeof(wa_engine)));
	qrt_puts(" of ");
	qrt_putu((unsigned)qrt_stack_total(sizeof(wa_engine)));
	qrt_puts("\r\n");

	qrt_report();
	return 0;
}
