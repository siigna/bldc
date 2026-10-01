/*
 * The CAN bindings, on the simulated F405.
 *
 * Two things here are worth more than the rest: that a reader goes to the
 * right status cache and the right field in it, and that the two documented
 * decisions hold -- reading an absent controller gives 0 while can_msg_age
 * gives nil, and commanding another controller does not touch the local motor
 * timeout. Both are the kind of thing that gets "fixed" later by someone
 * reasoning from analogy, so both are pinned.
 *
 * What this cannot show is what happens on a real bus: arbitration, a
 * controller that answers late, or what another board does when it is told to
 * spin. That is bench work with two ESCs.
 */

#include "qrt.h"
#include "script_lua.h"
#include "script_alloc.h"
#include "lua_vesc_can.h"
#include "lua_vesc_mc.h"

#include "comm_can.h"
#include "app.h"
#include "timeout.h"

#include <string.h>

#define ARENA_SIZE  (32u * 1024u)
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

// true if the chunk raised, which is what an argument check should do
static bool raises(const char *src) {
	char err[160];

	return !script_lua_run(m_s, src, -1, "t", err, sizeof(err));
}

static void expect_print(const char *name, const char *call,
		const char *expect) {
	char src[192];

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

typedef struct {
	const char *name;
	const char *call;
	const char *expect_call;	// what the fake should have recorded
	float expect_value;
} canset_t;

static THD_FUNCTION(engine, arg) {
	(void)arg;
	chRegSetThreadName("lua");

	script_lua_cfg_t cfg = {0};
	cfg.mem_limit = 24u * 1024u;
	cfg.print = capture;
	cfg.should_stop = no_stop;
	cfg.hook_count = 2000;
	cfg.alloc = script_alloc;

	m_s = script_lua_open(&cfg);
	qrt_expect_ok("engine opened", m_s != NULL);
	if (m_s == NULL) {
		return;
	}
	script_lua_register(m_s, lua_vesc_can_fns());
	script_lua_register(m_s, lua_vesc_mc_fns());

	fake_can_reset();
	fake_can_publish(7, 2500.0f, 11.0f, 0.5f, 44.0f, 48.0f);

	/* --- each reader goes to the right cache and the right field ------- */
	expect_print("canget_rpm",         "vesc.canget_rpm(7)",         "2500");
	expect_print("canget_current",     "vesc.canget_current(7)",     "11");
	expect_print("canget_duty",        "vesc.canget_duty(7)",        "0.5");
	expect_print("canget_ah",          "vesc.canget_ah(7)",          "2");
	expect_print("canget_ah_chg",      "vesc.canget_ah_chg(7)",      "3");
	expect_print("canget_wh",          "vesc.canget_wh(7)",          "4");
	expect_print("canget_wh_chg",      "vesc.canget_wh_chg(7)",      "5");
	expect_print("canget_temp_fet",    "vesc.canget_temp_fet(7)",    "44");
	expect_print("canget_temp_mot",    "vesc.canget_temp_mot(7)",    "55");
	expect_print("canget_current_in",  "vesc.canget_current_in(7)",  "6");
	expect_print("canget_pid_pos",     "vesc.canget_pid_pos(7)",     "7");
	expect_print("canget_vin",         "vesc.canget_vin(7)",         "48");
	expect_print("canget_tacho",       "vesc.canget_tacho(7)",       "9999");
	/* current * SIGN(duty), which is how lisp recovers a direction */
	expect_print("canget_current_dir", "vesc.canget_current_dir(7)", "11");

	/*
	 * --- an absent controller reads as zero, and that is the hazard -----
	 *
	 * Matching lisp deliberately: a script ported between the engines must
	 * not change behaviour. But zero is a plausible reading, so the pair of
	 * checks below is really one statement -- the value tells you nothing
	 * about whether anybody is there, and can_msg_age is what does.
	 */
	expect_print("absent reads zero", "vesc.canget_rpm(9)", "0");
	expect_print("absent has no age",
			"tostring(vesc.can_msg_age(9))", "nil");
	expect_print("present has an age",
			"type(vesc.can_msg_age(7))", "number");
	expect_print("a present age is small",
			"vesc.can_msg_age(7) < 5", "true");

	/* each status frame is aged separately */
	expect_print("frame 5 has an age",
			"type(vesc.can_msg_age(7, 5))", "number");
	qrt_expect_ok("an out-of-range frame number raises",
			raises("vesc.can_msg_age(7, 9)"));

	/* --- controller ids are validated, not silently wrapped ------------ */
	qrt_expect_ok("a negative id raises", raises("vesc.canget_rpm(-1)"));
	qrt_expect_ok("an id past 253 raises", raises("vesc.canget_rpm(254)"));
	qrt_expect_ok("a non-numeric id raises",
			raises("vesc.canget_rpm('seven')"));
	qrt_expect_ok("a nil id raises", raises("vesc.canget_rpm(nil)"));
	/*
	 * A numeric string is accepted, because luaL_checkinteger applies Lua's
	 * own string-to-number coercion. Not worth fighting: it is what every
	 * other Lua function does, and '7' meaning 7 is not a hazard. Recorded
	 * so the next reader does not take it for a missing check.
	 */
	qrt_expect_ok("a numeric string is coerced, as elsewhere in Lua",
			!raises("vesc.canget_rpm('7')"));

	/* --- discovery ----------------------------------------------------- */
	expect_print("can_local_id", "type(vesc.can_local_id())", "number");
	fake_can_publish(9, 100.0f, 1.0f, 0.1f, 20.0f, 40.0f);
	expect_print("can_scan finds both", "#vesc.can_scan()", "2");
	out_reset();
	run("scan contents",
			"local t = vesc.can_scan()\n"
			"table.sort(t)\n"
			"print('ids=' .. t[1] .. ',' .. t[2])\n");
	qrt_expect_ok("can_scan reports the ids heard from",
			strstr(m_out, "ids=7,9") != NULL);

	fake_can.ping_answers = 7;
	expect_print("a ping that answers", "type(vesc.can_ping(7))", "number");
	expect_print("a ping that does not", "tostring(vesc.can_ping(8))", "nil");

	/*
	 * --- commanding another controller ---------------------------------
	 *
	 * Each one has to reach the right comm_can call with the right value: a
	 * canset_brake wired to comm_can_set_current would read fine and spin a
	 * motor that was asked to stop.
	 */
	static const canset_t sets[] = {
		{"canset_current",     "vesc.canset_current(7, 12.5)",     "current",     12.5f},
		{"canset_current_rel", "vesc.canset_current_rel(7, 0.25)", "current_rel", 0.25f},
		{"canset_duty",        "vesc.canset_duty(7, 0.4)",         "duty",        0.4f},
		{"canset_rpm",         "vesc.canset_rpm(7, 3000)",         "rpm",         3000.0f},
		{"canset_pos",         "vesc.canset_pos(7, 90)",           "pos",         90.0f},
		{"canset_brake",       "vesc.canset_brake(7, 5)",          "brake",       5.0f},
		{"canset_brake_rel",   "vesc.canset_brake_rel(7, 0.6)",    "brake_rel",   0.6f},
	};

	for (unsigned i = 0; i < (sizeof(sets) / sizeof(sets[0])); i++) {
		const canset_t *s = &sets[i];
		char what[80];

		fake_can.last_id = -1;
		fake_can.last_value = 0.0f;
		fake_can.last_call = "";
		fake_can.set_calls = 0;
		fake_timeout_resets = 0;

		if (!run(s->name, s->call)) {
			continue;
		}

		strcpy(what, s->name);
		strcat(what, " reached the right call");
		qrt_expect_ok(what, strcmp(fake_can.last_call, s->expect_call) == 0);

		strcpy(what, s->name);
		strcat(what, " addressed the right controller");
		qrt_expect(what, (unsigned)fake_can.last_id, 7u);

		strcpy(what, s->name);
		strcat(what, " passed its value");
		qrt_expect_ok(what,
				(fake_can.last_value > (s->expect_value - 0.001f))
				&& (fake_can.last_value < (s->expect_value + 0.001f)));

		/*
		 * The documented decision, pinned. The local motor timeout is about
		 * this board's output; the controller being commanded runs its own.
		 * Refreshing ours here would be meaningless, and adding it by
		 * analogy with the motor bindings is the obvious mistake.
		 */
		strcpy(what, s->name);
		strcat(what, " leaves the local timeout alone");
		qrt_expect(what, (unsigned)fake_timeout_resets, 0u);
	}

	/* the optional off-delay, counted rather than compared */
	fake_can.off_delay_calls = 0;
	run("no off delay", "vesc.canset_current(7, 1)");
	qrt_expect("two arguments do not set an off-delay",
			(unsigned)fake_can.off_delay_calls, 0u);

	fake_can.off_delay_calls = 0;
	run("off delay", "vesc.canset_current(7, 1, 0.75)");
	qrt_expect("a third argument sets the off-delay",
			(unsigned)fake_can.off_delay_calls, 1u);
	qrt_expect_ok("and passes its value",
			(fake_can.last_off_delay > 0.74f)
			&& (fake_can.last_off_delay < 0.76f));

	/* --- raw frames ---------------------------------------------------- */
	fake_can.tx_sid_calls = 0;
	out_reset();
	run("send sid", "print(vesc.can_send_sid(0x123, '\\1\\2\\3'))");
	qrt_expect("a standard frame went out once",
			(unsigned)fake_can.tx_sid_calls, 1u);
	qrt_expect("with the id given", fake_can.tx_id, 0x123u);
	qrt_expect("and the length given", (unsigned)fake_can.tx_len, 3u);
	qrt_expect_ok("and the bytes given",
			fake_can.tx_data[0] == 1 && fake_can.tx_data[1] == 2
			&& fake_can.tx_data[2] == 3);
	qrt_expect_ok("and reported success", strstr(m_out, "true") != NULL);

	fake_can.tx_eid_calls = 0;
	run("send eid", "vesc.can_send_eid(0x1FFFFFFF, '\\9')");
	qrt_expect("an extended frame went out once",
			(unsigned)fake_can.tx_eid_calls, 1u);
	qrt_expect("with its 29-bit id", fake_can.tx_id, 0x1FFFFFFFu);

	/*
	 * Over-long payloads raise rather than truncate. A frame silently cut
	 * short is a different message and the receiver cannot tell.
	 */
	fake_can.tx_sid_calls = 0;
	qrt_expect_ok("a nine-byte payload raises",
			raises("vesc.can_send_sid(1, '123456789')"));
	qrt_expect("and sent nothing", (unsigned)fake_can.tx_sid_calls, 0u);

	qrt_expect_ok("a standard id past 11 bits raises",
			raises("vesc.can_send_sid(0x800, '\\1')"));
	qrt_expect_ok("an extended id past 29 bits raises",
			raises("vesc.can_send_eid(0x20000000, '\\1')"));
	qrt_expect_ok("a negative frame id raises",
			raises("vesc.can_send_sid(-1, '\\1')"));
	qrt_expect("none of those reached the bus",
			(unsigned)fake_can.tx_sid_calls, 0u);

	/* an empty payload is a real frame, not an error */
	fake_can.tx_sid_calls = 0;
	run("empty frame", "vesc.can_send_sid(0x10, '')");
	qrt_expect("a zero-length frame is allowed",
			(unsigned)fake_can.tx_sid_calls, 1u);
	qrt_expect("with length zero", (unsigned)fake_can.tx_len, 0u);

	/* a transmit failure is reported, not swallowed */
	fake_can.tx_fail = 1;
	out_reset();
	run("send fails", "print(vesc.can_send_sid(0x11, '\\1'))");
	qrt_expect_ok("a failed transmit reports false",
			strstr(m_out, "false") != NULL);
	fake_can.tx_fail = 0;

	/* --- can_cmd is deliberately absent -------------------------------- */
	expect_print("can_cmd is not exposed", "tostring(vesc.can_cmd)", "nil");

	script_lua_close(m_s);
	m_s = NULL;
}

int main(void) {
	qrt_console_init();
	chSysInit();
	qrt_systick_init();

	qrt_puts("can test\r\n");
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
