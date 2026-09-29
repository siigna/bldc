/*
 * Host tests for the PAS pedal decoder in applications/app_pas.c.
 *
 * The real app_pas.c is compiled against the stub headers in this directory,
 * so the tests exercise the shipping decoder rather than a copy of it. Pad
 * levels and the virtual timer are driven from here, which makes pedal timing
 * deterministic and lets uptime be fast-forwarded.
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

#include "datatypes.h"
#include "app.h"

// Stub state referenced by the stub headers.
systime_t test_now = 0;
uint8_t test_pad[2] = {0, 0};
mc_fault_code test_fault = FAULT_CODE_NONE;
float test_current_rel = 0.0;

// Stubs for the rest of the firmware that app_pas.c calls.
static app_configuration test_appconf;
const app_configuration *app_get_configuration(void) { return &test_appconf; }
bool app_is_output_disabled(void) { return false; }
int commands_printf(const char *format, ...) { (void)format; return 0; }
void terminal_register_command_callback(const char *command, const char *help,
		const char *arg_names, void(*cbf)(int argc, const char **argv)) {
	(void)command; (void)help; (void)arg_names; (void)cbf;
}
void terminal_unregister_callback(void(*cbf)(int argc, const char **argv)) { (void)cbf; }
#ifdef TEST_TORQUE_SENSOR
float test_torque = 0.0;
float hw_get_PAS_torque(void) { return test_torque; }
#endif

// app_pas.c exposes the decoder entry point without a prototype in app.h.
void pas_event_handler(void);

// ---------------------------------------------------------------------------
// Test harness
// ---------------------------------------------------------------------------

static int checks = 0;
static int failures = 0;

static void check(bool ok, const char *what) {
	checks++;
	if (!ok) {
		failures++;
		printf("  FAIL: %s\n", what);
	}
}

static void check_near(float got, float want, float tol, const char *what) {
	checks++;
	if (!(fabsf(got - want) <= tol)) {
		failures++;
		printf("  FAIL: %s (got %.4f, want %.4f +/- %.4f)\n", what, (double)got,
				(double)want, (double)tol);
	}
}

// ---------------------------------------------------------------------------
// Pedal simulation
// ---------------------------------------------------------------------------

#define MAGNETS		24

// Quadrature states in the forward direction, as decoded by the QEM table in
// app_pas.c: 0 -> 2 -> 3 -> 1 -> 0. state = PAS2 * 2 + PAS1.
static const uint8_t fwd_cycle[4] = {2, 3, 1, 0};
// The reverse direction is the same ring traversed the other way.
static const uint8_t rev_cycle[4] = {1, 3, 2, 0};

static void set_state(uint8_t state) {
	test_pad[0] = state & 1;
	test_pad[1] = (state >> 1) & 1;
}

// One transition: advance the clock, apply the new pad levels, run the decoder.
static void transition(uint8_t state, systime_t ticks) {
	test_now += ticks;
	set_state(state);
	pas_event_handler();
}

// Feed one full quadrature cycle, which is one magnet, spread over the given
// number of ticks. Exactly one pulse is accepted per cycle once the direction
// gate has been satisfied.
static void cycle_fwd(systime_t ticks) {
	for (int i = 0; i < 4; i++) {
		transition(fwd_cycle[i], ticks / 4);
	}
}

__attribute__((unused)) static void cycle_rev(systime_t ticks) {
	for (int i = 0; i < 4; i++) {
		transition(rev_cycle[i], ticks / 4);
	}
}

// Ticks per magnet for a given crank cadence.
static systime_t ticks_for_rpm(float rpm) {
	return (systime_t)((60.0f / (rpm * (float)MAGNETS)) * (float)CH_CFG_ST_FREQUENCY);
}

// Pedal steadily for a number of magnets.
static void pedal(float rpm, int magnets) {
	systime_t t = ticks_for_rpm(rpm);
	for (int i = 0; i < magnets; i++) {
		cycle_fwd(t);
	}
}

static pas_config base_config(void) {
	pas_config c;
	memset(&c, 0, sizeof(c));
	c.ctrl_type = PAS_CTRL_TYPE_CADENCE;
	c.sensor_type = PAS_SENSOR_TYPE_QUADRATURE;
	c.current_scaling = 0.5;
	c.pedal_rpm_start = 10.0;
	c.pedal_rpm_end = 180.0;
	c.invert_pedal_direction = false;
	c.magnets = MAGNETS;
	c.use_filter = false;
	c.ramp_time_pos = 0.6;
	c.ramp_time_neg = 0.3;
	c.update_rate_hz = 500;
	return c;
}

// Reset to a known state. app_pas_configure clears the decoder, which is what
// makes the tests independent of each other.
static void setup(pas_config *c) {
	test_now = 1000;
	set_state(0);
	app_pas_configure(c);
}

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

// Cadence is decoded at the rate it is pedalled.
static void test_cadence_accuracy(void) {
	printf("cadence accuracy\n");
	pas_config c = base_config();
	setup(&c);

	pedal(50.0, 10);
	check_near(app_pas_get_pedal_rpm(), 50.0, 0.5, "50 rpm decoded");

	setup(&c);
	pedal(100.0, 10);
	check_near(app_pas_get_pedal_rpm(), 100.0, 1.0, "100 rpm decoded");
}

// Regression: timestamps were taken as absolute float seconds, so period
// resolution degraded as uptime grew. At 10 kHz a float cannot represent a
// 100 us step beyond about 28 minutes of uptime.
static void test_uptime_independence(void) {
	printf("uptime independence\n");
	pas_config c = base_config();

	setup(&c);
	pedal(50.0, 10);
	float early = app_pas_get_pedal_rpm();

	// About 83 hours of uptime, far beyond where the old float seconds lost
	// the ability to resolve a single tick.
	setup(&c);
	test_now = 3000000000u;
	pedal(50.0, 10);
	float late = app_pas_get_pedal_rpm();

	check_near(late, 50.0, 0.5, "50 rpm decoded after long uptime");
	check_near(late, early, 0.01, "cadence independent of uptime");
}

// A glitch on the sensor lines must not corrupt the reported cadence.
//
// Note on coverage: this does not reach the case where an illegal transition
// doubled the cadence. That needed the direction counter to be at 4 or more
// while the state machine sat in state 0, which is only reachable if an
// accepted pulse was rejected by a plausibility check without resetting the
// counter. Both of those were changed, so the path no longer exists, but this
// test alone does not demonstrate it.
static void test_illegal_transition(void) {
	printf("illegal transition\n");
	pas_config c = base_config();
	setup(&c);

	pedal(50.0, 10);
	check_near(app_pas_get_pedal_rpm(), 50.0, 0.5, "steady before glitch");

	// 0 -> 3 is a double transition, which QEM reports as 2.
	systime_t t = ticks_for_rpm(50.0) / 4;
	transition(0, t);
	transition(3, t);
	transition(1, t);
	transition(0, t);
	pedal(50.0, 4);

	float rpm = app_pas_get_pedal_rpm();
	check(rpm < 75.0, "illegal transition does not double cadence");
	check(rpm >= 0.0, "cadence stays non-negative");
}

// Pedalling backwards must not engage assist.
static void test_reverse_no_cadence(void) {
	printf("reverse direction\n");
	pas_config c = base_config();
	setup(&c);

	systime_t t = ticks_for_rpm(50.0);
	for (int i = 0; i < 12; i++) {
		cycle_rev(t);
	}
	check_near(app_pas_get_pedal_rpm(), 0.0, 0.001, "no cadence when pedalling backwards");
}

// Inverting the configured direction makes the reverse ring the assisted one.
static void test_inverted_direction(void) {
	printf("inverted direction\n");
	pas_config c = base_config();
	c.invert_pedal_direction = true;
	setup(&c);

	systime_t t = ticks_for_rpm(50.0);
	for (int i = 0; i < 12; i++) {
		cycle_rev(t);
	}
	check_near(app_pas_get_pedal_rpm(), 50.0, 0.5, "inverted direction decodes reverse ring");
}

// Cadence must fall to zero once the cranks stop.
static void test_idle_zeroes(void) {
	printf("idle\n");
	pas_config c = base_config();
	setup(&c);

	pedal(50.0, 10);
	check(app_pas_get_pedal_rpm() > 1.0, "cadence present before idle");

	// max_pulse_period is 0.3 s at these settings. Keep polling with the pads
	// unchanged, as the thread would.
	for (int i = 0; i < 100; i++) {
		test_now += 100;
		pas_event_handler();
	}
	check_near(app_pas_get_pedal_rpm(), 0.0, 0.001, "cadence zeroed after cranks stop");
}

// Resuming after an idle period must not produce a cadence spike from the long
// gap, and must converge on the real cadence.
static void test_resume_after_idle(void) {
	printf("resume after idle\n");
	pas_config c = base_config();
	setup(&c);

	pedal(50.0, 10);
	for (int i = 0; i < 100; i++) {
		test_now += 100;
		pas_event_handler();
	}

	pedal(50.0, 10);
	check_near(app_pas_get_pedal_rpm(), 50.0, 0.5, "cadence recovered after idle");
}

// A period far shorter than the configured maximum cadence allows is rejected.
static void test_implausibly_fast(void) {
	printf("implausibly fast\n");
	pas_config c = base_config();
	setup(&c);

	// min_pedal_period is 1/9 s of crank revolution at pedal_rpm_end 180.
	for (int i = 0; i < 20; i++) {
		cycle_fwd(20);
	}
	check_near(app_pas_get_pedal_rpm(), 0.0, 0.001, "implausibly short period rejected");
}

// use_filter had no effect: the low-pass was applied with a constant of 1.0,
// which assigns the new sample unchanged.
static void test_filter_is_honoured(void) {
	printf("filter\n");
	pas_config c = base_config();
	systime_t t = ticks_for_rpm(50.0);

	// Unfiltered: one long magnet period moves the reported cadence fully.
	c.use_filter = false;
	setup(&c);
	pedal(50.0, 10);
	cycle_fwd(t * 2);
	float unfiltered = app_pas_get_pedal_rpm();

	// Filtered: the same perturbation moves it less.
	c.use_filter = true;
	setup(&c);
	pedal(50.0, 10);
	cycle_fwd(t * 2);
	float filtered = app_pas_get_pedal_rpm();

	check(fabsf(unfiltered - 50.0f) > 10.0f, "unfiltered cadence follows the perturbation");
	check(fabsf(filtered - 50.0f) < fabsf(unfiltered - 50.0f),
			"filtered cadence is less disturbed than unfiltered");
	check(filtered < 50.5f, "filtered cadence still responds to the perturbation");
}

// An unsupported sensor type must not behave like a supported one.
static void test_unsupported_sensor_type(void) {
	printf("unsupported sensor type\n");
	pas_config c = base_config();
	c.sensor_type = (pas_sensor_type)7;
	setup(&c);

	pedal(50.0, 10);
	check_near(app_pas_get_pedal_rpm(), 0.0, 0.001, "no cadence for unsupported sensor type");
}

// Reconfiguring must not leave stale pedal timing behind.
static void test_reconfigure_resets(void) {
	printf("reconfigure\n");
	pas_config c = base_config();
	setup(&c);

	pedal(50.0, 10);
	check(app_pas_get_pedal_rpm() > 1.0, "cadence present before reconfigure");

	app_pas_configure(&c);
	check_near(app_pas_get_pedal_rpm(), 0.0, 0.001, "cadence cleared by reconfigure");

	// And the first pulse after the reset is treated as a reference rather than
	// being timed against the pre-reset timestamp.
	pedal(50.0, 10);
	check_near(app_pas_get_pedal_rpm(), 50.0, 0.5, "cadence re-acquired after reconfigure");
}

// Changing the magnet count changes the decoded cadence for the same pulse rate.
static void test_magnet_count(void) {
	printf("magnet count\n");
	pas_config c = base_config();
	setup(&c);
	pedal(50.0, 10);
	float rpm24 = app_pas_get_pedal_rpm();

	// Halving the magnet count halves the revolutions the same pulse rate implies.
	pas_config c12 = base_config();
	c12.magnets = 12;
	setup(&c12);
	systime_t t = ticks_for_rpm(50.0);
	for (int i = 0; i < 10; i++) {
		cycle_fwd(t);
	}
	float rpm12 = app_pas_get_pedal_rpm();

	// The same pulse rate with half the magnets is twice the cadence.
	check_near(rpm24 / rpm12, 0.5, 0.02, "cadence scales with magnet count");
}

// The pedal sensor pins are only claimed when they are actually usable. On
// hardware without dedicated pins the fallback is the COMM UART pads, which must
// not be reconfigured as inputs while UART comms is in use.
static void test_pin_claim_gate(void) {
	printf("pin claim gate\n");
	pas_config c = base_config();

	// UART comms off: the pins are claimed wherever they exist.
	test_appconf.permanent_uart_enabled = false;
	test_appconf.app_to_use = APP_PAS;
	app_pas_start(true);
	setup(&c);
	pedal(50.0, 10);
#ifdef TEST_NO_PAS_PINS
	check_near(app_pas_get_pedal_rpm(), 0.0, 0.001, "no cadence without pedal sensor pins");
#else
	check_near(app_pas_get_pedal_rpm(), 50.0, 0.5, "cadence decoded when pins are claimed");
#endif
	app_pas_stop();

	// UART comms on.
	test_appconf.permanent_uart_enabled = true;
	app_pas_start(true);
	setup(&c);
	pedal(50.0, 10);
#if defined(TEST_NO_PAS_PINS) || defined(TEST_PAS_PINS_SHARED)
	check_near(app_pas_get_pedal_rpm(), 0.0, 0.001,
			"pads not claimed, so no cadence, while UART comms is enabled");
#else
	check_near(app_pas_get_pedal_rpm(), 50.0, 0.5,
			"dedicated pins are unaffected by UART comms");
#endif
	app_pas_stop();

	// A UART application has the same effect as permanent UART comms.
	test_appconf.permanent_uart_enabled = false;
	test_appconf.app_to_use = APP_ADC_UART;
	app_pas_start(false);
	setup(&c);
	pedal(50.0, 10);
#if defined(TEST_NO_PAS_PINS) || defined(TEST_PAS_PINS_SHARED)
	check_near(app_pas_get_pedal_rpm(), 0.0, 0.001, "pads not claimed for a UART application");
#else
	check_near(app_pas_get_pedal_rpm(), 50.0, 0.5, "dedicated pins unaffected by a UART application");
#endif
	app_pas_stop();

	// Restart into a working configuration must clear the previous failure.
	test_appconf.app_to_use = APP_PAS;
	app_pas_start(true);
	setup(&c);
	pedal(50.0, 10);
#ifdef TEST_NO_PAS_PINS
	check_near(app_pas_get_pedal_rpm(), 0.0, 0.001, "still no pins after restart");
#else
	check_near(app_pas_get_pedal_rpm(), 50.0, 0.5, "cadence recovered after restart");
#endif
	app_pas_stop();
}

int main(void) {
	memset(&test_appconf, 0, sizeof(test_appconf));
	test_appconf.app_to_use = APP_PAS;
	test_appconf.permanent_uart_enabled = false;

	printf("app_pas host tests\n\n");

#ifndef TEST_NO_PAS_PINS
	// The decoder tests need pedal sensor pins to read.
	test_cadence_accuracy();
	test_uptime_independence();
	test_illegal_transition();
	test_reverse_no_cadence();
	test_inverted_direction();
	test_idle_zeroes();
	test_resume_after_idle();
	test_implausibly_fast();
	test_filter_is_honoured();
	test_unsupported_sensor_type();
	test_reconfigure_resets();
	test_magnet_count();
#endif
	test_pin_claim_gate();

	printf("\n%d checks, %d failures\n", checks, failures);
	return failures == 0 ? 0 : 1;
}
