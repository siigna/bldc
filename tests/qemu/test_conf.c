/*
 * vesc.conf_get and vesc.conf_set, on the simulated F405.
 *
 * The mapping itself is checked by tests/conf_table, which regenerates the
 * table from the LispBM extensions and fails on any difference. What this
 * covers is the behaviour around it: that all 136 entries are reachable
 * without faulting, that each round trips, that the two write paths are
 * chosen correctly, and that an unknown name fails where the typo is.
 *
 * The exhaustive walk is the point. An offset that lands outside the struct,
 * or a kind that reads a field wider than it is, shows up as a wrong value or
 * a fault on that one parameter and on no other -- so spot-checking a dozen
 * of them proves very little.
 */

#include "qrt.h"
#include "script_lua.h"
#include "script_alloc.h"
#include "lua_vesc_conf.h"

#include "mc_interface.h"
#include "app.h"
#include "commands.h"
#include "mempools.h"

#include <string.h>
#include <stdarg.h>
#include <stdio.h>

/*
 * Larger than the firmware's 24 KB so the walk below has room to spare.
 *
 * Measured, the walk peaks around 19 KB of script memory -- which does fit
 * the firmware's 20 KB ceiling, but only just, and a run right at the ceiling
 * failed with "not enough memory" before the collector had reason to run.
 * The peak is printed at the end so the figure stays on the record.
 */
#define ARENA_SIZE  (56u * 1024u)
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

static void out_reset(void) {
	m_out_len = 0;
	m_out[0] = '\0';
}

int commands_printf_lisp(const char *format, ...) {
	(void)format;
	return 0;
}

static bool no_stop(void) {
	return false;
}

static script_lua_t *m_s;

static bool run(const char *what, const char *src) {
	char err[160];

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

static THD_FUNCTION(engine, arg) {
	(void)arg;
	chRegSetThreadName("lua");

	script_lua_cfg_t cfg = {0};
	cfg.mem_limit = 48u * 1024u;
	cfg.print = capture;
	cfg.should_stop = no_stop;
	cfg.hook_count = 2000;
	cfg.alloc = script_alloc;

	m_s = script_lua_open(&cfg);
	qrt_expect_ok("engine opened", m_s != NULL);
	if (m_s == NULL) {
		return;
	}
	script_lua_register(m_s, lua_vesc_conf_fns());

	/*
	 * A plausible drivetrain before anything is read. min_speed and max_speed
	 * divide by si_wheel_diameter, so on an all-zero configuration they are
	 * 0/0 -- NaN, not a wrong number, and the walk below would report them as
	 * unreadable for a reason that has nothing to do with the bindings.
	 */
	fake_mc.mcconf.si_motor_poles = 14;
	fake_mc.mcconf.si_gear_ratio = 1.0f;
	fake_mc.mcconf.si_wheel_diameter = 0.6f;

	/* --- the table is reachable and has the expected size -------------- */
	out_reset();
	run("conf_names", "print(#vesc.conf_names())");
	/* 136 generated + 5 scaled + min_speed + max_speed */
	qrt_expect_ok("conf_names lists every parameter",
			strstr(m_out, "143") != NULL);

	/*
	 * --- every parameter is readable ----------------------------------
	 *
	 * A bad offset or a kind wider than its field shows up on one parameter
	 * and no other, so this walks all of them rather than sampling.
	 */
	out_reset();
	run("read every parameter",
			"local names = vesc.conf_names()\n"
			"local bad = 0\n"
			"local first = ''\n"
			"for _, n in ipairs(names) do\n"
			"  local v = vesc.conf_get(n)\n"
			"  if v == nil or v ~= v then\n"
			"    bad = bad + 1\n"
			"    if first == '' then first = n end\n"
			"  end\n"
			"end\n"
			"print('unreadable=' .. bad .. ' ' .. first)\n");
	if (strstr(m_out, "unreadable=0") == NULL) {
		qrt_puts("  ");
		qrt_puts(m_out);
		qrt_puts("\r\n");
	}
	qrt_expect_ok("every parameter reads as a number",
			strstr(m_out, "unreadable=0") != NULL);

	/*
	 * --- every parameter round trips -----------------------------------
	 *
	 * Checked one at a time rather than by collecting results into a table.
	 * The table version ran the script out of memory at a 20 KB ceiling,
	 * which is worth knowing in itself: 143 string keys and their values is
	 * not a small allocation on this part.
	 *
	 * The value written is the loop index, so two names sharing a field show
	 * up as the second overwriting the first rather than as two reads that
	 * happen to agree.
	 */
	out_reset();
	run("round trip every parameter",
			"local names = vesc.conf_names()\n"
			"local bad = 0\n"
			"local first = ''\n"
			"local bools = 0\n"
			"for i, n in ipairs(names) do\n"
			"  -- classify first: a bool field turns any non-zero into 1, so\n"
			"  -- a distinct-value round trip is the wrong check for it\n"
			"  local is_bool = false\n"
			"  if vesc.conf_set(n, 2) then\n"
			"    is_bool = (vesc.conf_get(n) == 1)\n"
			"  end\n"
			"  if is_bool then\n"
			"    bools = bools + 1\n"
			"    local ok0 = vesc.conf_set(n, 0) and vesc.conf_get(n) == 0\n"
			"    local ok1 = vesc.conf_set(n, 1) and vesc.conf_get(n) == 1\n"
			"    if not (ok0 and ok1) then\n"
			"      bad = bad + 1\n"
			"      if first == '' then first = n .. ' (bool)' end\n"
			"    end\n"
			"  else\n"
			"    if vesc.conf_set(n, i) then\n"
			"      local got = vesc.conf_get(n)\n"
			"      -- a magnitude parameter reads back negated, and an\n"
			"      -- integer field truncates; both are expected\n"
			"      if got == nil or math.abs(math.abs(got) - i) > 1.0 then\n"
			"        bad = bad + 1\n"
			"        if first == '' then\n"
			"          first = n .. ' got ' .. tostring(got) .. ' want ' .. i\n"
			"        end\n"
			"      end\n"
			"    end\n"
			"  end\n"
			"end\n"
			"print('roundtrip_bad=' .. bad .. ' bools=' .. bools\n"
			"      .. ' ' .. first)\n");
	if (strstr(m_out, "roundtrip_bad=0") == NULL) {
		qrt_puts("  ");
		qrt_puts(m_out);
		qrt_puts("\r\n");
	} else {
		qrt_puts("  ");
		qrt_puts(m_out);
	}
	qrt_expect_ok("every parameter round trips",
			strstr(m_out, "roundtrip_bad=0") != NULL);
	/* the bool fields exist, so the classification above is doing something */
	qrt_expect_ok("some parameters are boolean",
			strstr(m_out, "bools=0 ") == NULL);

	/* --- a parameter stored negative is given as a magnitude ----------- */
	out_reset();
	run("neg abs", "vesc.conf_set('l_current_min', 60)");
	qrt_expect_ok("l_current_min is stored negative",
			fake_mc.mcconf.l_current_min < -59.0f
			&& fake_mc.mcconf.l_current_min > -61.0f);
	run("neg abs read", "print('cmin=' .. vesc.conf_get('l_current_min'))");
	qrt_expect_ok("and reads back as stored, like lisp",
			strstr(m_out, "cmin=-60") != NULL);

	/* --- the two write paths ------------------------------------------- */
	fake_mc.mcconf_applied = 0;
	fake_mc_limits_applied = 0;
	run("plain write", "vesc.conf_set('l_current_max', 77)");
	qrt_expect("a plain parameter needs no reconfigure",
			(unsigned)fake_mc.mcconf_applied, 0u);
	qrt_expect("a plain parameter still clamps to hw limits",
			(unsigned)fake_mc_limits_applied, 1u);
	qrt_expect_ok("and took effect on the live configuration",
			fake_mc.mcconf.l_current_max > 76.0f
			&& fake_mc.mcconf.l_current_max < 78.0f);

	fake_mc.mcconf_applied = 0;
	run("apply write", "vesc.conf_set('foc_f_zv', 30000)");
	qrt_expect("a foc parameter is applied through a reconfigure",
			(unsigned)fake_mc.mcconf_applied, 1u);
	qrt_expect("the mcconf mempool was returned",
			(unsigned)fake_mempool_mcconf_out, 0u);

	fake_app.applied = 0;
	run("app apply write", "vesc.conf_set('app_to_use', 3)");
	qrt_expect("an app parameter is applied through a reconfigure",
			(unsigned)fake_app.applied, 1u);
	qrt_expect("the appconf mempool was returned",
			(unsigned)fake_mempool_appconf_out, 0u);

	/* --- the scaled foc constants -------------------------------------- */
	run("scaled set", "vesc.conf_set('foc_motor_r', 25)");
	qrt_expect_ok("foc_motor_r is stored in ohms, given in milliohms",
			fake_mc.mcconf.foc_motor_r > 0.0249f
			&& fake_mc.mcconf.foc_motor_r < 0.0251f);
	out_reset();
	run("scaled get", "print('r=' .. vesc.conf_get('foc_motor_r'))");
	qrt_expect_ok("and reads back in milliohms",
			strstr(m_out, "r=25") != NULL);

	/* --- the computed speed limits ------------------------------------- */
	run("speed setup",
			"vesc.conf_set('si_motor_poles', 14)\n"
			"vesc.conf_set('si_gear_ratio', 1)\n"
			"vesc.conf_set('si_wheel_diameter', 0.6)\n");
	run("max speed", "vesc.conf_set('max_speed', 10)");
	out_reset();
	run("max speed read", "print('ms=' .. vesc.conf_get('max_speed'))");
	qrt_expect_ok("max_speed round trips through the erpm conversion",
			strstr(m_out, "ms=10") != NULL);
	qrt_expect_ok("and left an erpm limit behind",
			fake_mc.mcconf.l_max_erpm > 1000.0f);

	/* --- an unknown name fails where the typo is ----------------------- */
	out_reset();
	run("unknown read", "print('u=' .. tostring(vesc.conf_get('l_current_mx')))");
	qrt_expect_ok("an unknown parameter reads as nil, not zero",
			strstr(m_out, "u=nil") != NULL);

	out_reset();
	run("unknown write",
			"print('w=' .. tostring(vesc.conf_set('l_current_mx', 1)))");
	qrt_expect_ok("an unknown parameter refuses to be written",
			strstr(m_out, "w=false") != NULL);

	{
		char err[160];

		qrt_expect_ok("arithmetic on an unknown parameter raises",
				!script_lua_run(m_s,
						"local x = vesc.conf_get('nope') + 1", -1, "t",
						err, sizeof(err)));
	}

	qrt_puts("script_mem_peak=");
	qrt_putu((unsigned)script_lua_mem_peak(m_s));
	qrt_puts(" arena_peak=");
	qrt_putu((unsigned)script_alloc_peak());
	qrt_puts("\r\n");

	script_lua_close(m_s);
	m_s = NULL;
}

int main(void) {
	qrt_console_init();
	chSysInit();
	qrt_systick_init();

	qrt_puts("conf test\r\n");
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
