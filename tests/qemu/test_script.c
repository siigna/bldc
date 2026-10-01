/*
 * script/examples/bench_throttle.lua, on the simulated F405.
 *
 * The script is a bring-up artifact: it is what will drive a motor the first
 * time a Lua build drives one, so its safety properties are worth checking
 * before the bench rather than during it. Each of the four notes at the top
 * of the script has a check here.
 *
 * The script source is embedded by the Makefile from the real file, so this
 * cannot drift from what gets uploaded. BENCH_TEST is set as a global before
 * the chunk runs, which is how the script knows to define M and return
 * without entering its own loop.
 *
 * What this does NOT cover: the input bindings are the stand-ins in
 * fake_io.c, not script/lua_vesc_io.c, which cannot be linked here. So this
 * tests the script's logic against a controllable input, not the plumbing
 * underneath it.
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

/* cut from the real file by the Makefile */
static const char m_script[] =
#include "bench_throttle.inc"
;

#define ARENA_SIZE  (40u * 1024u)
static uint8_t m_arena[ARENA_SIZE] __attribute__((aligned(8)));
static THD_WORKING_AREA(wa_engine, 12288) __attribute__((section(".ram4")));

static char m_out[1024];
static size_t m_out_len;

static void capture(const char *msg) {
	size_t n = strlen(msg);

	if (m_out_len + n + 1u < sizeof(m_out)) {
		memcpy(m_out + m_out_len, msg, n);
		m_out_len += n;
		m_out[m_out_len] = '\0';
	}
}

int commands_printf_lisp(const char *format, ...) {
	(void)format;
	return 0;
}

static bool no_stop(void) {
	return false;
}

static script_lua_t *m_s;

/* Run one M.step(dt) and report the commanded current via a global. */
static float step(float dt) {
	char src[96];
	char err[160];

	snprintf(src, sizeof(src), "_a = M.step(%d/1000.0) _r = M.reason",
			(int)(dt * 1000.0f));
	if (!script_lua_run(m_s, src, -1, "step", err, sizeof(err))) {
		qrt_puts("  step error: ");
		qrt_puts(err);
		qrt_puts("\r\n");
		return -1.0f;
	}

	lua_State *L = script_lua_state(m_s);
	lua_getglobal(L, "_a");
	float v = (float)lua_tonumber(L, -1);
	lua_pop(L, 1);
	return v;
}

/*
 * Several steps in one chunk. Doing this from C compiled a fresh chunk per
 * step, and a few hundred of those is slow enough under QEMU to hit the
 * harness timeout.
 */
static float ramp(int n, float dt) {
	char src[128];
	char err[160];

	snprintf(src, sizeof(src),
			"for _ = 1, %d do _a = M.step(%d/1000.0) end _r = M.reason",
			n, (int)(dt * 1000.0f));
	if (!script_lua_run(m_s, src, -1, "ramp", err, sizeof(err))) {
		qrt_puts("  ramp error: ");
		qrt_puts(err);
		qrt_puts("\r\n");
		return -1.0f;
	}

	lua_State *L = script_lua_state(m_s);
	lua_getglobal(L, "_a");
	float v = (float)lua_tonumber(L, -1);
	lua_pop(L, 1);
	return v;
}

static const char *reason(void) {
	lua_State *L = script_lua_state(m_s);
	const char *r;

	lua_getglobal(L, "_r");
	r = lua_tostring(L, -1);
	lua_pop(L, 1);
	return (r != NULL) ? r : "";
}

static bool armed(void) {
	lua_State *L = script_lua_state(m_s);
	bool a;

	lua_getglobal(L, "M");
	lua_getfield(L, -1, "armed");
	a = lua_toboolean(L, -1) != 0;
	lua_pop(L, 2);
	return a;
}

static void set_cfg(const char *field, const char *value) {
	char src[128];
	char err[160];

	snprintf(src, sizeof(src), "M.cfg.%s = %s", field, value);
	(void)script_lua_run(m_s, src, -1, "cfg", err, sizeof(err));
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

	/* tell the script a test is driving it, so it defines M and returns */
	{
		lua_State *L = script_lua_state(m_s);

		lua_pushboolean(L, 1);
		lua_setglobal(L, "BENCH_TEST");
	}

	fake_io_reset();
	fake_mc.set_current = 0.0f;

	/*
	 * The script declares its table local and returns it, which is what a
	 * require() of it would use. script_lua_run discards a chunk's return
	 * value, so the source is wrapped in a function call whose result is
	 * assigned to a global the checks below can reach. Wrapping rather than
	 * adding a global to the script keeps the file deployable as it stands.
	 */
	{
		static char wrapped[8192];
		char err[256];
		int n = snprintf(wrapped, sizeof(wrapped),
				"M = (function()\n%s\nend)()", m_script);

		qrt_expect_ok("the wrapped script fits the buffer",
				(n > 0) && (n < (int)sizeof(wrapped)));

		if (!script_lua_run(m_s, wrapped, -1, "bench_throttle",
				err, sizeof(err))) {
			qrt_puts("  load error: ");
			qrt_puts(err);
			qrt_puts("\r\n");
		}
		qrt_expect_ok("the script loads and returns its table",
				script_lua_run(m_s, "_ok = (type(M) == 'table')", -1, "x",
						err, sizeof(err)));
	}

	{
		lua_State *L = script_lua_state(m_s);

		lua_getglobal(L, "_ok");
		qrt_expect_ok("the script exposed its state table",
				lua_toboolean(L, -1) != 0);
		lua_pop(L, 1);
	}

	/* ---- note 2: it starts disarmed ---------------------------------- */
	set_cfg("input", "'ppm'");
	fake_io.ppm = 0.8f;			/* throttle already held at power-up */
	fake_io.ppm_age = 0.0f;

	qrt_expect_ok("a held throttle commands nothing", step(0.02f) == 0.0f);
	qrt_expect_ok("and the script stays disarmed", !armed());
	qrt_expect_ok("and says why",
			strstr(reason(), "waiting for throttle") != NULL);

	/* it arms once the stick comes back */
	fake_io.ppm = 0.0f;
	(void)step(0.02f);
	qrt_expect_ok("returning the throttle arms it", armed());

	/* ---- the ramp is rate limited upward ----------------------------- */
	fake_io.ppm = 1.0f;
	{
		/* ramp_up 6 A/s, so 20 ms should move about 0.12 A */
		float a = step(0.02f);

		qrt_puts("  first step after full throttle: ");
		qrt_putu((unsigned)(a * 1000.0f));
		qrt_puts(" mA\r\n");
		qrt_expect_ok("the first step is rate limited, not the full 4 A",
				(a > 0.0f) && (a < 0.5f));
	}

	/* and reaches the cap given enough steps */
	{
		float a = ramp(100, 0.02f);

		qrt_puts("  settled at ");
		qrt_putu((unsigned)(a * 1000.0f));
		qrt_puts(" mA\r\n");
		qrt_expect_ok("it settles at the configured cap",
				(a > 3.9f) && (a < 4.1f));
		qrt_expect_ok("and that is what reached the motor",
				(fake_mc.set_current > 3.9f) && (fake_mc.set_current < 4.1f));
	}

	/* releasing the throttle is immediate, not ramped down */
	fake_io.ppm = 0.0f;
	qrt_expect_ok("releasing the throttle drops to zero in one step",
			step(0.02f) == 0.0f);

	/*
	 * Zero throttle coasts rather than holding zero current. Holding zero
	 * still regulates, which on a bench stand is a shaft that resists being
	 * turned by hand -- and makes it look like the motor is still driven.
	 */
	fake_mc.released = 0;
	fake_mc.set_calls = 0;
	fake_io.ppm = 0.0f;
	(void)step(0.02f);
	qrt_expect("zero throttle coasts the motor",
			(unsigned)fake_mc.released, 1u);
	qrt_expect("and commands no current at all",
			(unsigned)fake_mc.set_calls, 1u);	/* release_motor only */

	/*
	 * An input reading past full must not exceed the cap.
	 *
	 * Two things prevent it: the throttle is clamped to 0..1 on read, and the
	 * commanded current is clamped to max_current after the ramp. They are
	 * redundant by design, which has a consequence worth recording -- neither
	 * is individually detectable by mutation, because removing either leaves
	 * the other covering it. Removing *both* gives 8 A against a 4 A cap, so
	 * the pair is load-bearing even though each half looks dead on its own.
	 *
	 * This check was added because removing the cap alone survived, which was
	 * a real gap at the time: the clamp on the ppm path only covered the
	 * lower end.
	 */
	fake_io.ppm = 2.0f;
	{
		float a = ramp(120, 0.02f);

		qrt_puts("  with the input at 2.0, settled at ");
		qrt_putu((unsigned)(a * 1000.0f));
		qrt_puts(" mA\r\n");
		qrt_expect_ok("an over-range input still caps at the configured limit",
				(a > 3.9f) && (a < 4.1f));
	}

	/* ---- note 3: a stale input stops it and disarms it ---------------- */
	fake_io.ppm = 1.0f;
	(void)ramp(100, 0.02f);
	qrt_expect_ok("back at the cap before the dropout", armed());

	fake_mc.released = 0;
	fake_io.ppm_age = 1.0f;			/* older than stale_s */
	qrt_expect_ok("a stale input commands nothing", step(0.02f) == 0.0f);
	qrt_expect("and coasts the motor", (unsigned)fake_mc.released, 1u);
	qrt_expect_ok("and disarms", !armed());
	qrt_expect_ok("and says the input is stale",
			strstr(reason(), "stale") != NULL);

	/*
	 * The property this is really for: the link coming back at full throttle
	 * must not spin the motor up. It has to go through zero first.
	 */
	fake_io.ppm_age = 0.0f;			/* link restored, stick still at full */
	qrt_expect_ok("a restored link at full throttle still commands nothing",
			step(0.02f) == 0.0f);
	qrt_expect_ok("and is still disarmed", !armed());

	fake_io.ppm = 0.0f;
	(void)step(0.02f);
	qrt_expect_ok("only a return to zero re-arms it", armed());

	/* a decoder that never ran reads as stale, not as zero throttle */
	fake_io.ppm_age_is_nil = true;
	qrt_expect_ok("an absent decoder counts as no input", step(0.02f) == 0.0f);
	qrt_expect_ok("and disarms", !armed());
	fake_io.ppm_age_is_nil = false;

	/* ---- the adc path, including a disconnected pot ------------------- */
	set_cfg("input", "'adc'");
	fake_io.adc_volts = 0.8f;		/* at rest */
	(void)step(0.02f);
	qrt_expect_ok("a pot at rest arms", armed());

	fake_io.adc_volts = 2.4f;		/* full */
	{
		float a = ramp(120, 0.02f);

		qrt_expect_ok("a pot at full reaches the cap",
				(a > 3.9f) && (a < 4.1f));
	}

	/*
	 * A broken pot wire reads below the rest voltage. That must not look
	 * like a closed throttle, because a closed throttle is indistinguishable
	 * from working correctly.
	 */
	fake_mc.released = 0;
	fake_io.adc_volts = 0.2f;
	qrt_expect_ok("a pot below its range commands nothing",
			step(0.02f) == 0.0f);
	qrt_expect("and coasts", (unsigned)fake_mc.released, 1u);
	qrt_expect_ok("and reports it rather than reading zero throttle",
			strstr(reason(), "below range") != NULL);

	/* a board with no such pin is reported, not read as channel 0 */
	fake_io.adc_is_nil = true;
	qrt_expect_ok("a missing adc pin commands nothing", step(0.02f) == 0.0f);
	qrt_expect_ok("and says so",
			strstr(reason(), "no adc pin") != NULL);
	fake_io.adc_is_nil = false;

	/*
	 * ---- note 1: the script does not rewrite the erpm limit ----------
	 *
	 * Checked against the embedded source, not by running anything. The
	 * first version of this check looked for a global the script never sets
	 * and ended in `|| true`, so it passed unconditionally -- it was not a
	 * check at all.
	 */
	qrt_expect_ok("the script never calls conf_set",
			strstr(m_script, "conf_set") == NULL);
	qrt_expect_ok("and says why not, in the notes",
			strstr(m_script, "l_max_erpm") != NULL);
	/* it does disable the apps, which is the other half of note 4 */
	qrt_expect_ok("the script disables the apps' output",
			strstr(m_script, "app_disable_output") != NULL);

	script_lua_close(m_s);
	m_s = NULL;
}

int main(void) {
	qrt_console_init();
	chSysInit();
	qrt_systick_init();

	qrt_puts("bench script test\r\n");
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
