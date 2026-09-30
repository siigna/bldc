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
#include "sched.h"
#include <math.h>

#include "datatypes.h"
#include "app.h"

// The stub firmware lives in fixture.c, which the fuzzer shares.
extern systime_t test_now;
extern uint8_t test_pad[2];
extern mc_fault_code test_fault;
extern float test_current_rel;
extern volatile uint16_t ADC_Value[16];
extern float test_v_in;
extern float test_speed;
extern float test_current_in;
extern mc_configuration test_mcconf;
extern app_configuration test_appconf;
extern float test_torque;

// app_pas.c exposes the sensor front end without prototypes in app.h.
void pas_event_handler(void);
float pas_read_torque_nm(void);
float pas_compute_output(float dt_ms);
float pas_mix_throttle(float throttle_rel, float pas_rel);
void app_pas_walk_set(bool active);

// Set the voltage an ADC channel reads.
static void set_adc_volts(int ch, float volts) {
	float counts = volts / 3.3f * 4096.0f;
	if (counts < 0.0f) {
		counts = 0.0f;
	}
	if (counts > 4095.0f) {
		counts = 4095.0f;
	}
	ADC_Value[ch] = (uint16_t)(counts + 0.5f);
}

// The voltage that actually comes back out of the ADC for a requested voltage,
// which is quantised to 12 bits.
static float adc_quantised(float volts) {
	set_adc_volts(6, volts);
	return (float)ADC_Value[6] / 4096.0f * 3.3f;
}

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

// One quadrature cycle with the control loop run at a realistic rate. The
// thread samples torque at update_rate_hz, which at 500 Hz and 60 rpm with 24
// magnets is about 20 samples per pedal pulse, so a test that calls it once per
// pulse would misrepresent the sensor filter.
#define CTRL_CALLS_PER_PULSE	20

static void cycle_fwd_with_control(systime_t ticks) {
	for (int i = 0; i < 4; i++) {
		transition(fwd_cycle[i], ticks / 4);
		for (int k = 0; k < CTRL_CALLS_PER_PULSE / 4; k++) {
			pas_compute_output(2.0);
		}
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

// A single-wire sensor gives one edge per magnet and no direction.
static void pulse_single_wire(float rpm, int magnets_count) {
	systime_t t = ticks_for_rpm(rpm);
	for (int i = 0; i < magnets_count; i++) {
		// Half the period low, half high, so each magnet is one rising edge.
		test_now += t / 2;
		test_pad[0] = 0;
		pas_event_handler();
		test_now += t - (t / 2);
		test_pad[0] = 1;
		pas_event_handler();
	}
}

// The ramp is part of the output path, so tests that are not about the ramp
// disable it and read the control law directly. Zero is below the threshold the
// firmware uses to apply a ramp at all.
static void ramp_off(pas_config *c) {
	c->ramp_time_pos = 0.0;
	c->ramp_time_neg = 0.0;
}

static pas_config base_config(void) {
	pas_config c;
	memset(&c, 0, sizeof(c));
	c.assist_gain = 1.0;
	c.torque_avg_pulses = 0;
	// The shipped defaults, so the tests run against realistic values.
	c.cadence_floor_rpm = 55.0;
	c.start_timeout_s = 0.3;
	c.stop_timeout_s = 0.25;
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
	// A value outside the enum is the whole point here: the firmware must
	// report an unsupported sensor type rather than fall through to a default
	// that silently produces no assist, and a configuration can arrive over
	// CAN from anything.
	// NOLINTNEXTLINE(clang-analyzer-optin.core.EnumCastOutOfRange)
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

static void test_single_wire(void) {
	printf("single wire\n");
	pas_config c = base_config();
	c.sensor_type = PAS_SENSOR_TYPE_SINGLE_WIRE;
	setup(&c);

	pulse_single_wire(50.0, 10);
	check_near(app_pas_get_pedal_rpm(), 50.0, 0.5, "50 rpm decoded from a single wire");

	setup(&c);
	pulse_single_wire(90.0, 10);
	check_near(app_pas_get_pedal_rpm(), 90.0, 1.0, "90 rpm decoded from a single wire");

	// Idle must zero it, as for quadrature.
	for (int i = 0; i < 100; i++) {
		test_now += 100;
		pas_event_handler();
	}
	check_near(app_pas_get_pedal_rpm(), 0.0, 0.001, "single wire cadence zeroed when idle");

	// And it must recover.
	pulse_single_wire(50.0, 10);
	check_near(app_pas_get_pedal_rpm(), 50.0, 0.5, "single wire cadence recovered");
}

// A quadrature sensor read as single wire still produces a cadence, because
// PAS1 toggles, but the count differs. What matters is that selecting the wrong
// type does not produce a wildly high reading.
static void test_single_wire_ignores_direction(void) {
	printf("single wire direction\n");
	pas_config c = base_config();
	c.sensor_type = PAS_SENSOR_TYPE_SINGLE_WIRE;
	c.invert_pedal_direction = true;
	setup(&c);

	// Inverting has no meaning for a single wire, so the cadence is unchanged.
	pulse_single_wire(50.0, 10);
	check_near(app_pas_get_pedal_rpm(), 50.0, 0.5,
			"invert_pedal_direction does not affect a single wire sensor");
}

// Volts to Nm: zero point, scale, deadband, clamp and sign.
static void test_torque_scaling(void) {
	printf("torque scaling\n");
	pas_config c = base_config();
	c.torque_source = PAS_TORQUE_SRC_ADC;
	c.torque_adc_ch = PAS_TORQUE_ADC_EXT1;
	c.torque_zero_v = 1.5;
	c.torque_nm_per_v = 70.0;
	c.torque_max_nm = 140.0;
	c.torque_deadband_nm = 0.0;

	// At the zero point there is no torque.
	setup(&c);
	set_adc_volts(ADC_IND_EXT, 1.5);
	// The reading is low-passed, so settle it.
	for (int i = 0; i < 40; i++) {
		pas_read_torque_nm();
	}
	check_near(pas_read_torque_nm(), 0.0, 0.05, "zero volts offset gives zero torque");

	// One volt above zero is one scale factor of torque.
	setup(&c);
	float v = adc_quantised(2.5);
	for (int i = 0; i < 60; i++) {
		pas_read_torque_nm();
	}
	check_near(pas_read_torque_nm(), (v - 1.5f) * 70.0f, 0.5,
			"one volt above zero gives the configured scale");

	// Below the zero point reads as no torque, not as negative torque.
	setup(&c);
	set_adc_volts(ADC_IND_EXT, 1.0);
	for (int i = 0; i < 60; i++) {
		pas_read_torque_nm();
	}
	check_near(pas_read_torque_nm(), 0.0, 0.001, "below the zero point gives zero, not negative");

	// Clamped to the configured full scale, when full scale is reachable.
	c.torque_max_nm = 100.0;
	setup(&c);
	set_adc_volts(ADC_IND_EXT, 3.3);
	for (int i = 0; i < 80; i++) {
		pas_read_torque_nm();
	}
	check_near(pas_read_torque_nm(), 100.0, 0.001, "torque clamped to full scale");
}

// The sensor this is aimed at outputs 1.5 V to 3.5 V at 70 Nm/V, which is
// 140 Nm of range, but the ADC reference on these boards is 3.3 V. So the top
// of the sensor range cannot be measured at all, and the assist would flatten
// out there rather than at full effort.
static void test_torque_reference_ceiling(void) {
	printf("torque reference ceiling\n");
	pas_config c = base_config();
	c.torque_source = PAS_TORQUE_SRC_ADC;
	c.torque_adc_ch = PAS_TORQUE_ADC_EXT1;
	c.torque_zero_v = 1.5;
	c.torque_nm_per_v = 70.0;
	c.torque_max_nm = 140.0;	// (3.5 - 1.5) * 70, the sensor's own range
	c.torque_deadband_nm = 0.0;
	setup(&c);

	set_adc_volts(ADC_IND_EXT, 3.3);
	for (int i = 0; i < 80; i++) {
		pas_read_torque_nm();
	}

	// (3.3 - 1.5) * 70 = 126, so 14 Nm of the sensor range is unreachable.
	float ceiling = pas_read_torque_nm();
	check_near(ceiling, 126.0, 0.5, "the measurable ceiling is set by the ADC reference");
	check(ceiling < c.torque_max_nm,
			"full scale is not reachable, so it must not be configured as if it were");
	check(app_pas_torque_saturated(), "the reading at the reference is reported as saturated");
}

static void test_torque_deadband(void) {
	printf("torque deadband\n");
	pas_config c = base_config();
	c.torque_source = PAS_TORQUE_SRC_ADC;
	c.torque_adc_ch = PAS_TORQUE_ADC_EXT1;
	c.torque_zero_v = 1.5;
	c.torque_nm_per_v = 70.0;
	c.torque_max_nm = 140.0;
	c.torque_deadband_nm = 7.0;	// 0.1 V worth

	// Just inside the deadband is zero.
	setup(&c);
	set_adc_volts(ADC_IND_EXT, 1.55);	// 3.5 Nm raw
	for (int i = 0; i < 80; i++) {
		pas_read_torque_nm();
	}
	check_near(pas_read_torque_nm(), 0.0, 0.001, "inside the deadband gives zero");

	// Above it, output is continuous rather than stepping to the raw value.
	setup(&c);
	set_adc_volts(ADC_IND_EXT, 1.65);	// 10.5 Nm raw, 3.5 Nm above the deadband
	for (int i = 0; i < 80; i++) {
		pas_read_torque_nm();
	}
	float just_above = pas_read_torque_nm();
	check(just_above > 0.0 && just_above < 10.5,
			"just above the deadband is small and positive, not the raw value");

	// Full scale is still reachable, given a full scale the ADC range can reach.
	c.torque_max_nm = 100.0;
	setup(&c);
	set_adc_volts(ADC_IND_EXT, 3.3);
	for (int i = 0; i < 80; i++) {
		pas_read_torque_nm();
	}
	check_near(pas_read_torque_nm(), 100.0, 0.001, "full scale still reached with a deadband");
}

// The sensor the plan targets outputs up to 3.5 V, above the ADC reference on
// these boards, so clipping has to be visible rather than looking like a
// plateau in the assist.
static void test_torque_saturation(void) {
	printf("torque saturation\n");
	pas_config c = base_config();
	c.torque_source = PAS_TORQUE_SRC_ADC;
	c.torque_adc_ch = PAS_TORQUE_ADC_EXT1;
	setup(&c);

	set_adc_volts(ADC_IND_EXT, 1.5);
	pas_read_torque_nm();
	check(!app_pas_torque_saturated(), "not saturated at the zero point");

	set_adc_volts(ADC_IND_EXT, 3.3);
	pas_read_torque_nm();
	check(app_pas_torque_saturated(), "saturated at the ADC reference");

	set_adc_volts(ADC_IND_EXT, 1.5);
	pas_read_torque_nm();
	check(!app_pas_torque_saturated(), "saturation clears when the reading falls");
}

// hw.h aliases the unavailable ADC channels to the first one, so selecting a
// channel a board does not have would otherwise read the throttle input.
static void test_torque_channel_validation(void) {
	printf("torque channel validation\n");
	pas_config c = base_config();
	c.torque_source = PAS_TORQUE_SRC_ADC;
	c.torque_zero_v = 0.0;
	c.torque_nm_per_v = 70.0;
	c.torque_max_nm = 140.0;
	c.torque_deadband_nm = 0.0;

	// Put a large signal on the first channel, which is normally the throttle.
	set_adc_volts(ADC_IND_EXT, 3.0);

	// A channel this hardware does have.
	c.torque_adc_ch = PAS_TORQUE_ADC_EXT3;
	setup(&c);
	set_adc_volts(ADC_IND_EXT3, 0.0);
	for (int i = 0; i < 80; i++) {
		pas_read_torque_nm();
	}
	check(!app_pas_torque_ch_invalid(), "an available channel is accepted");
	check_near(pas_read_torque_nm(), 0.0, 0.01, "reads the selected channel, not the first one");

	// A channel it does not.
	c.torque_adc_ch = PAS_TORQUE_ADC_EXT6;
	setup(&c);
	for (int i = 0; i < 80; i++) {
		pas_read_torque_nm();
	}
	check(app_pas_torque_ch_invalid(), "an unavailable channel is rejected");
	check_near(pas_read_torque_nm(), 0.0, 0.001,
			"an unavailable channel gives no torque rather than the throttle reading");
}

static void test_torque_source_none(void) {
	printf("torque source none\n");
	pas_config c = base_config();
	c.torque_source = PAS_TORQUE_SRC_NONE;
	c.torque_adc_ch = PAS_TORQUE_ADC_EXT1;
	setup(&c);

	set_adc_volts(ADC_IND_EXT, 3.0);
	for (int i = 0; i < 40; i++) {
		pas_read_torque_nm();
	}
	check_near(pas_read_torque_nm(), 0.0, 0.001, "no torque source gives no torque");
}

#ifdef TEST_TORQUE_SENSOR
// A board implementation reports a ratio, which is expressed in Nm using the
// configured full scale.
static void test_torque_source_hw(void) {
	printf("torque source hardware\n");
	pas_config c = base_config();
	c.torque_source = PAS_TORQUE_SRC_HW;
	c.torque_max_nm = 140.0;
	setup(&c);

	test_torque = 0.0;
	check_near(pas_read_torque_nm(), 0.0, 0.001, "hardware ratio 0 is 0 Nm");

	test_torque = 0.5;
	check_near(pas_read_torque_nm(), 70.0, 0.001, "hardware ratio 0.5 is half full scale");

	test_torque = 1.0;
	check_near(pas_read_torque_nm(), 140.0, 0.001, "hardware ratio 1 is full scale");
}
#else
// Selecting the hardware source on a board without one must report rather than
// look like an inactive app.
static void test_torque_source_hw_absent(void) {
	printf("torque source hardware absent\n");
	pas_config c = base_config();
	c.torque_source = PAS_TORQUE_SRC_HW;
	setup(&c);

	check_near(pas_read_torque_nm(), 0.0, 0.001, "no torque without a hardware implementation");
}
#endif

// Rider power is torque times angular velocity, and it is the concept the app
// was missing entirely.
static void test_rider_power(void) {
	printf("rider power\n");
	pas_config c = base_config();
	c.ctrl_type = PAS_CTRL_TYPE_POWER;
	c.torque_source = PAS_TORQUE_SRC_ADC;
	c.torque_adc_ch = PAS_TORQUE_ADC_EXT1;
	c.torque_zero_v = 0.0;
	c.torque_nm_per_v = 30.0;
	c.torque_max_nm = 200.0;
	c.torque_deadband_nm = 0.0;
	c.current_scaling = 1.0;
	c.assist_gain = 1.0;
	setup(&c);

	// 1.0 V at 30 Nm/V is 30 Nm. Pedal at 60 rpm, which is 2*pi rad/s, so
	// rider power is 30 * 2*pi = about 188 W.
	set_adc_volts(ADC_IND_EXT, 1.0);
	pedal(60.0, 12);
	for (int i = 0; i < 80; i++) {
		pas_compute_output(2.0);
	}

	float nm = app_pas_get_torque_nm();
	float expect = nm * (60.0f * 2.0f * (float)M_PI / 60.0f);
	check_near(app_pas_get_rider_power(), expect, 1.0, "rider power is torque times cadence");
	check(app_pas_get_rider_power() > 150.0 && app_pas_get_rider_power() < 220.0,
			"30 Nm at 60 rpm is about 190 W");
}

// Stopped cranks must mean no assist. With power control that falls out of the
// arithmetic rather than needing an interlock, which is worth pinning down.
static void test_power_zero_without_cadence(void) {
	printf("power without cadence\n");
	pas_config c = base_config();
	c.ctrl_type = PAS_CTRL_TYPE_POWER;
	c.torque_source = PAS_TORQUE_SRC_ADC;
	c.torque_adc_ch = PAS_TORQUE_ADC_EXT1;
	c.torque_zero_v = 0.0;
	c.torque_nm_per_v = 30.0;
	c.torque_max_nm = 200.0;
	c.torque_deadband_nm = 0.0;
	c.current_scaling = 1.0;
	setup(&c);

	// Full weight on the pedals, cranks not turning.
	set_adc_volts(ADC_IND_EXT, 2.0);
	for (int i = 0; i < 100; i++) {
		test_now += 100;
		pas_event_handler();
		pas_compute_output(10.0);
	}

	check(app_pas_get_torque_nm() > 10.0, "torque is being read");
	check_near(app_pas_get_rider_power(), 0.0, 0.001, "no rider power without cadence");
	check_near(pas_compute_output(2.0), 0.0, 0.001, "no assist without cadence");
}

// The gain is motor watts per rider watt.
static void test_assist_gain(void) {
	printf("assist gain\n");
	pas_config c = base_config();
	c.ctrl_type = PAS_CTRL_TYPE_POWER;
	c.torque_source = PAS_TORQUE_SRC_ADC;
	c.torque_adc_ch = PAS_TORQUE_ADC_EXT1;
	c.torque_zero_v = 0.0;
	c.torque_nm_per_v = 30.0;
	c.torque_max_nm = 200.0;
	c.torque_deadband_nm = 0.0;
	c.current_scaling = 1.0;

	float outs[3];
	const float gains[3] = {0.5, 1.0, 2.0};

	for (int g = 0; g < 3; g++) {
		c.assist_gain = gains[g];
		setup(&c);
		set_adc_volts(ADC_IND_EXT, 1.0);
		pedal(60.0, 12);
		for (int i = 0; i < 80; i++) {
			outs[g] = pas_compute_output(2.0);
		}
		check_near(app_pas_get_motor_power_target(),
				app_pas_get_rider_power() * gains[g], 1.0,
				"motor power target is gain times rider power");
	}

	check_near(outs[1] / outs[0], 2.0, 0.05, "doubling the gain doubles the request");
	check_near(outs[2] / outs[1], 2.0, 0.05, "and again");
}

// Watts become a current against the measured input voltage, and the assist
// ceiling still applies.
static void test_power_to_current(void) {
	printf("power to current\n");
	pas_config c = base_config();
	c.ctrl_type = PAS_CTRL_TYPE_POWER;
	c.torque_source = PAS_TORQUE_SRC_ADC;
	c.torque_adc_ch = PAS_TORQUE_ADC_EXT1;
	c.torque_zero_v = 0.0;
	c.torque_nm_per_v = 30.0;
	c.torque_max_nm = 200.0;
	c.torque_deadband_nm = 0.0;
	c.current_scaling = 1.0;
	c.assist_gain = 1.0;

	test_mcconf.lo_current_max = 100.0;

	// At twice the voltage, the same power is half the current.
	test_v_in = 25.0;
	setup(&c);
	set_adc_volts(ADC_IND_EXT, 1.0);
	pedal(60.0, 12);
	float out_low = 0.0;
	for (int i = 0; i < 80; i++) {
		out_low = pas_compute_output(2.0);
	}

	test_v_in = 50.0;
	setup(&c);
	set_adc_volts(ADC_IND_EXT, 1.0);
	pedal(60.0, 12);
	float out_high = 0.0;
	for (int i = 0; i < 80; i++) {
		out_high = pas_compute_output(2.0);
	}

	check_near(out_low / out_high, 2.0, 0.05,
			"halving the input voltage doubles the current for the same power");

	// And the relative current is watts / volts / lo_current_max.
	float expect = (app_pas_get_motor_power_target() / 50.0f) / 100.0f;
	check_near(out_high, expect, 0.001, "relative current is against lo_current_max");

	// The assist ceiling caps it.
	c.current_scaling = 0.01;
	setup(&c);
	set_adc_volts(ADC_IND_EXT, 2.0);
	pedal(90.0, 12);
	float capped = 0.0;
	for (int i = 0; i < 80; i++) {
		capped = pas_compute_output(2.0);
	}
	check_near(capped, 0.01, 0.0001, "PAS max current caps the power request");

	test_v_in = 50.0;
	c.current_scaling = 1.0;
}

// An input voltage reading that is too low would inflate the current a power
// request converts to, so it is floored.
static void test_vin_guard(void) {
	printf("input voltage guard\n");
	pas_config c = base_config();
	c.ctrl_type = PAS_CTRL_TYPE_POWER;
	c.torque_source = PAS_TORQUE_SRC_ADC;
	c.torque_adc_ch = PAS_TORQUE_ADC_EXT1;
	c.torque_zero_v = 0.0;
	c.torque_nm_per_v = 30.0;
	c.torque_max_nm = 200.0;
	c.torque_deadband_nm = 0.0;
	c.current_scaling = 1.0;
	c.assist_gain = 1.0;
	test_mcconf.lo_current_max = 100.0;

	// 8 V is the floor.
	test_v_in = 8.0;
	setup(&c);
	set_adc_volts(ADC_IND_EXT, 1.0);
	pedal(60.0, 12);
	float at_floor = 0.0;
	for (int i = 0; i < 80; i++) {
		at_floor = pas_compute_output(2.0);
	}

	// A reading far below it must not ask for more than the floor does.
	test_v_in = 0.1;
	setup(&c);
	set_adc_volts(ADC_IND_EXT, 1.0);
	pedal(60.0, 12);
	float below = 0.0;
	for (int i = 0; i < 80; i++) {
		below = pas_compute_output(2.0);
	}

	check_near(below, at_floor, 0.0001,
			"a voltage below the floor is treated as the floor, not as more current");

	// Zero must not divide.
	test_v_in = 0.0;
	setup(&c);
	set_adc_volts(ADC_IND_EXT, 1.0);
	pedal(60.0, 12);
	float at_zero = 0.0;
	for (int i = 0; i < 80; i++) {
		at_zero = pas_compute_output(2.0);
	}
	check(isfinite(at_zero), "a zero voltage reading does not produce a non-finite request");
	check_near(at_zero, at_floor, 0.0001, "and is also treated as the floor");

	test_v_in = 50.0;
}

// Averaging over whole pulse intervals rather than applying a time constant.
static void test_torque_averaging(void) {
	printf("torque averaging\n");
	pas_config c = base_config();
	c.ctrl_type = PAS_CTRL_TYPE_POWER;
	c.torque_source = PAS_TORQUE_SRC_ADC;
	c.torque_adc_ch = PAS_TORQUE_ADC_EXT1;
	c.torque_zero_v = 0.0;
	c.torque_nm_per_v = 30.0;
	c.torque_max_nm = 200.0;
	c.torque_deadband_nm = 0.0;
	c.current_scaling = 1.0;

	// Pedalling and computing have to be interleaved, as they are on the board:
	// the averaging window closes an interval when a pedal pulse arrives, so it
	// only fills if torque is being sampled while the cranks turn.
	systime_t t = ticks_for_rpm(60.0);

	c.torque_avg_pulses = 0;
	setup(&c);
	set_adc_volts(ADC_IND_EXT, 1.0);
	for (int i = 0; i < 60; i++) {
		cycle_fwd_with_control(t);
	}
	float steady_off = app_pas_get_rider_power();
	check(steady_off > 0.0, "power with averaging off");

	// With a window, a steady signal gives the same answer.
	c.torque_avg_pulses = c.magnets;
	setup(&c);
	set_adc_volts(ADC_IND_EXT, 1.0);
	for (int i = 0; i < 60; i++) {
		cycle_fwd_with_control(t);
	}
	float steady_on = app_pas_get_rider_power();
	check_near(steady_on, steady_off, steady_off * 0.05f,
			"a steady torque gives the same power averaged or not");
}

// The window must not carry torque across a stop and pair it with a fresh
// cadence when pedalling resumes.
static void test_averaging_reset_on_stop(void) {
	printf("averaging reset on stop\n");
	pas_config c = base_config();
	c.ctrl_type = PAS_CTRL_TYPE_POWER;
	c.torque_source = PAS_TORQUE_SRC_ADC;
	c.torque_adc_ch = PAS_TORQUE_ADC_EXT1;
	c.torque_zero_v = 0.0;
	c.torque_nm_per_v = 30.0;
	c.torque_max_nm = 200.0;
	c.torque_deadband_nm = 0.0;
	c.current_scaling = 1.0;
	c.torque_avg_pulses = c.magnets;
	setup(&c);

	systime_t t = ticks_for_rpm(60.0);

	// Pedal hard, filling the window with a high torque.
	set_adc_volts(ADC_IND_EXT, 2.0);
	for (int i = 0; i < 60; i++) {
		cycle_fwd_with_control(t);
	}
	float loaded = app_pas_get_rider_power();
	check(loaded > 0.0, "power while pedalling under load");

	// Stop, and drop the torque to nothing.
	for (int i = 0; i < 100; i++) {
		test_now += 100;
		pas_event_handler();
		pas_compute_output(10.0);
	}
	set_adc_volts(ADC_IND_EXT, 0.0);
	check_near(app_pas_get_rider_power(), 0.0, 0.001, "no power while stopped");

	// Resume with no force on the pedals. The first pulses must not be assisted
	// from the torque recorded before the stop.
	float peak = 0.0;
	for (int i = 0; i < 12; i++) {
		cycle_fwd_with_control(t);
		float pw = app_pas_get_rider_power();
		if (pw > peak) {
			peak = pw;
		}
	}
	check(peak < loaded * 0.1f,
			"resuming without pedal force does not assist from the pre-stop torque");
}

// A window must actually smooth a torque that varies within the stroke, which
// is what a bottom bracket sensor does.
static void test_torque_averaging_smooths(void) {
	printf("torque averaging smooths\n");
	pas_config c = base_config();
	c.ctrl_type = PAS_CTRL_TYPE_POWER;
	c.torque_source = PAS_TORQUE_SRC_ADC;
	c.torque_adc_ch = PAS_TORQUE_ADC_EXT1;
	c.torque_zero_v = 0.0;
	c.torque_nm_per_v = 30.0;
	c.torque_max_nm = 200.0;
	c.torque_deadband_nm = 0.0;
	c.current_scaling = 1.0;
	c.magnets = 8;

	// Drive a torque that alternates between pulses, as a pedal stroke does,
	// and record the spread of the resulting power.
	float spread[2];
	const int windows[2] = {0, 8};

	for (int w = 0; w < 2; w++) {
		c.torque_avg_pulses = windows[w];
		setup(&c);

		// Fill the window first.
		systime_t t = ticks_for_rpm(60.0);
		for (int rev = 0; rev < 6; rev++) {
			for (int m = 0; m < 8; m++) {
				set_adc_volts(ADC_IND_EXT, (m < 4) ? 1.5f : 0.5f);
				cycle_fwd_with_control(t);
			}
		}

		float lo = 1e9f;
		float hi = -1e9f;
		for (int rev = 0; rev < 4; rev++) {
			for (int m = 0; m < 8; m++) {
				set_adc_volts(ADC_IND_EXT, (m < 4) ? 1.5f : 0.5f);
				cycle_fwd_with_control(t);
				float pw = app_pas_get_rider_power();
				if (pw < lo) {
					lo = pw;
				}
				if (pw > hi) {
					hi = pw;
				}
			}
		}
		spread[w] = hi - lo;
	}

	check(spread[0] > 0.0, "an unaveraged reading varies through the stroke");
	check(spread[1] < spread[0] * 0.5f,
			"a one revolution window at least halves the variation through the stroke");
}

// Cadence control at a known cadence, with the ramp out of the way, as a
// baseline the limits can be measured against.
static float cadence_output(pas_config *c) {
	setup(c);
	pedal(50.0, 12);
	float out = 0.0;
	for (int i = 0; i < 10; i++) {
		out = pas_compute_output(2.0);
	}
	return out;
}

static void test_speed_taper(void) {
	printf("speed taper\n");
	pas_config c = base_config();
	ramp_off(&c);
	c.ctrl_type = PAS_CTRL_TYPE_CADENCE;
	c.current_scaling = 1.0;
	c.pedal_rpm_start = 10.0;
	c.pedal_rpm_end = 100.0;

	// Taper off: speed makes no difference.
	c.taper_start_kmh = 0.0;
	c.taper_end_kmh = 0.0;
	test_speed = 0.0;
	float unlimited = cadence_output(&c);
	test_speed = 30.0 / 3.6;
	check_near(cadence_output(&c), unlimited, 0.001, "no taper configured means no limiting");

	// Tapering from 20 to 25 km/h.
	c.taper_start_kmh = 20.0;
	c.taper_end_kmh = 25.0;

	test_speed = 10.0 / 3.6;
	check_near(cadence_output(&c), unlimited, 0.001, "full assist below the taper start");

	test_speed = 20.0 / 3.6;
	check_near(cadence_output(&c), unlimited, 0.001, "full assist at the taper start");

	test_speed = 22.5 / 3.6;
	check_near(cadence_output(&c), unlimited * 0.5f, unlimited * 0.02f,
			"half assist half way through the taper");

	test_speed = 25.0 / 3.6;
	check_near(cadence_output(&c), 0.0, 0.001, "no assist at the taper end");

	test_speed = 40.0 / 3.6;
	check_near(cadence_output(&c), 0.0, 0.001, "no assist above the taper end");

	test_speed = 0.0;
}

// Setting the two speeds equal is how a hard cutoff is expressed, which is what
// the pedelec preset uses.
static void test_speed_cutoff(void) {
	printf("speed cutoff\n");
	pas_config c = base_config();
	ramp_off(&c);
	c.ctrl_type = PAS_CTRL_TYPE_CADENCE;
	c.current_scaling = 1.0;
	c.pedal_rpm_start = 10.0;
	c.pedal_rpm_end = 100.0;
	c.taper_start_kmh = 25.0;
	c.taper_end_kmh = 25.0;

	test_speed = 24.0 / 3.6;
	float below = cadence_output(&c);
	check(below > 0.0, "assist just below the cutoff");

	test_speed = 25.5 / 3.6;
	check_near(cadence_output(&c), 0.0, 0.001, "no assist just above the cutoff");

	// An end below the start is the same thing rather than an inverted taper.
	c.taper_start_kmh = 25.0;
	c.taper_end_kmh = 10.0;
	test_speed = 24.0 / 3.6;
	check(cadence_output(&c) > 0.0, "an end below the start still assists below the start");
	test_speed = 26.0 / 3.6;
	check_near(cadence_output(&c), 0.0, 0.001, "and cuts off above it");

	test_speed = 0.0;
}

// The cap is on watts and has to bite whatever the control type is, not only
// the one that works in watts.
static void test_power_cap(void) {
	printf("power cap\n");
	pas_config c = base_config();
	ramp_off(&c);
	c.ctrl_type = PAS_CTRL_TYPE_CADENCE;
	c.current_scaling = 1.0;
	c.pedal_rpm_start = 10.0;
	c.pedal_rpm_end = 60.0;

	test_v_in = 50.0;
	test_mcconf.lo_current_max = 100.0;

	c.power_max_w = 0.0;
	float uncapped = cadence_output(&c);
	check(uncapped > 0.5, "cadence control at full scale without a cap");

	// 250 W of 5000 W full scale is 0.05 relative.
	c.power_max_w = 250.0;
	check_near(cadence_output(&c), 0.05, 0.001, "the cap applies to cadence control");

	// The cap is in watts, so halving the voltage doubles the relative current
	// it allows.
	test_v_in = 25.0;
	check_near(cadence_output(&c), 0.10, 0.001, "the cap tracks the input voltage");

	// A cap above what is available changes nothing.
	test_v_in = 50.0;
	c.power_max_w = 100000.0;
	check_near(cadence_output(&c), uncapped, 0.001, "a cap above full scale does not limit");

	test_v_in = 50.0;
}

// A hard pedal stop ends assist with the pedalling instead of fading out over
// the negative ramp time.
static void test_pedal_stop_hard(void) {
	printf("hard pedal stop\n");
	pas_config c = base_config();
	c.ctrl_type = PAS_CTRL_TYPE_CADENCE;
	c.current_scaling = 1.0;
	c.pedal_rpm_start = 10.0;
	c.pedal_rpm_end = 100.0;
	// A ramp long enough that the difference is unmistakable.
	c.ramp_time_pos = 0.2;
	c.ramp_time_neg = 2.0;

	// Ramping down: still assisting shortly after the cranks stop.
	c.pedal_stop_hard = false;
	setup(&c);
	pedal(50.0, 12);
	for (int i = 0; i < 400; i++) {
		pas_compute_output(2.0);
	}
	float running = pas_compute_output(2.0);
	check(running > 0.0, "assisting while pedalling");

	for (int i = 0; i < 100; i++) {
		test_now += 100;
		pas_event_handler();
	}
	float after_ramp = pas_compute_output(2.0);
	check(after_ramp > 0.0, "still assisting just after the cranks stop when ramping");

	// Hard cut: nothing, immediately.
	c.pedal_stop_hard = true;
	setup(&c);
	pedal(50.0, 12);
	for (int i = 0; i < 400; i++) {
		pas_compute_output(2.0);
	}
	check(pas_compute_output(2.0) > 0.0, "assisting while pedalling with hard stop set");

	for (int i = 0; i < 100; i++) {
		test_now += 100;
		pas_event_handler();
	}
	check_near(pas_compute_output(2.0), 0.0, 0.001,
			"no assist immediately after the cranks stop with hard stop set");
}

static void test_brake(void) {
	printf("brake\n");
	pas_config c = base_config();
	c.ctrl_type = PAS_CTRL_TYPE_CADENCE;
	c.current_scaling = 1.0;
	c.pedal_rpm_start = 10.0;
	c.pedal_rpm_end = 100.0;
	c.ramp_time_pos = 0.2;
	c.ramp_time_neg = 2.0;
	c.brake_source = PAS_BRAKE_SRC_ADC;
	c.brake_adc_ch = PAS_TORQUE_ADC_EXT2;
	c.brake_threshold_v = 1.65;
	c.brake_invert = false;

	setup(&c);
	set_adc_volts(ADC_IND_EXT2, 0.0);
	pedal(50.0, 12);
	for (int i = 0; i < 400; i++) {
		pas_compute_output(2.0);
	}
	check(pas_compute_output(2.0) > 0.0, "assisting with the brake released");

	// The brake cuts immediately, through the ramp rather than over it.
	set_adc_volts(ADC_IND_EXT2, 3.0);
	check_near(pas_compute_output(2.0), 0.0, 0.001, "the brake cuts assist immediately");

	// And releasing it does not restore the old ramp state in one step.
	set_adc_volts(ADC_IND_EXT2, 0.0);
	float resumed = pas_compute_output(2.0);
	check(resumed >= 0.0 && resumed < 0.05,
			"releasing the brake ramps back up rather than jumping");

	// Below the threshold is released, above is applied.
	setup(&c);
	set_adc_volts(ADC_IND_EXT2, 1.6);
	pedal(50.0, 12);
	for (int i = 0; i < 400; i++) {
		pas_compute_output(2.0);
	}
	check(pas_compute_output(2.0) > 0.0, "just below the threshold is released");

	set_adc_volts(ADC_IND_EXT2, 1.7);
	check_near(pas_compute_output(2.0), 0.0, 0.001, "just above the threshold is applied");
}

// A switch that pulls the input low when the brake is used.
static void test_brake_inverted(void) {
	printf("brake inverted\n");
	pas_config c = base_config();
	ramp_off(&c);
	c.ctrl_type = PAS_CTRL_TYPE_CADENCE;
	c.current_scaling = 1.0;
	c.pedal_rpm_start = 10.0;
	c.pedal_rpm_end = 100.0;
	c.brake_source = PAS_BRAKE_SRC_ADC;
	c.brake_adc_ch = PAS_TORQUE_ADC_EXT2;
	c.brake_threshold_v = 1.65;
	c.brake_invert = true;

	set_adc_volts(ADC_IND_EXT2, 3.0);
	check(cadence_output(&c) > 0.0, "high is released when inverted");

	set_adc_volts(ADC_IND_EXT2, 0.0);
	check_near(cadence_output(&c), 0.0, 0.001, "low is applied when inverted");
}

// As for the torque channel, a brake channel the hardware lacks must not fall
// through to the first ADC channel.
static void test_brake_channel_validation(void) {
	printf("brake channel validation\n");
	pas_config c = base_config();
	ramp_off(&c);
	c.ctrl_type = PAS_CTRL_TYPE_CADENCE;
	c.current_scaling = 1.0;
	c.pedal_rpm_start = 10.0;
	c.pedal_rpm_end = 100.0;
	c.brake_source = PAS_BRAKE_SRC_ADC;
	c.brake_adc_ch = PAS_TORQUE_ADC_EXT6;
	c.brake_threshold_v = 1.65;

	// A voltage that would read as braking on the first channel.
	set_adc_volts(ADC_IND_EXT, 3.0);

	float out = cadence_output(&c);
	check(out > 0.0, "an unavailable brake channel does not brake from another channel");
}

// The pedelec preset is a combination of the limits rather than a mode of its
// own, so what matters is that the combination behaves as intended.
static void test_pedelec_combination(void) {
	printf("pedelec combination\n");
	pas_config c = base_config();
	c.ctrl_type = PAS_CTRL_TYPE_CADENCE;
	c.current_scaling = 1.0;
	c.pedal_rpm_start = 10.0;
	c.pedal_rpm_end = 60.0;
	c.ramp_time_pos = 0.2;
	c.ramp_time_neg = 2.0;

	// What "pas_preset pedelec store" sets.
	c.taper_start_kmh = 25.0;
	c.taper_end_kmh = 25.0;
	c.power_max_w = 250.0;
	c.pedal_stop_hard = true;

	test_v_in = 50.0;
	test_mcconf.lo_current_max = 100.0;

	// Under way and pedalling: capped at 250 W of the 5000 W full scale.
	test_speed = 20.0 / 3.6;
	setup(&c);
	pedal(50.0, 12);
	float out = 0.0;
	for (int i = 0; i < 400; i++) {
		out = pas_compute_output(2.0);
	}
	check_near(out, 0.05, 0.002, "assist is capped at 250 W below the cutoff");

	// Over the cutoff: nothing.
	test_speed = 26.0 / 3.6;
	setup(&c);
	pedal(50.0, 12);
	for (int i = 0; i < 400; i++) {
		out = pas_compute_output(2.0);
	}
	check_near(out, 0.0, 0.001, "no assist above the cutoff");

	// Pedalling stops: nothing, immediately.
	test_speed = 20.0 / 3.6;
	setup(&c);
	pedal(50.0, 12);
	for (int i = 0; i < 400; i++) {
		pas_compute_output(2.0);
	}
	for (int i = 0; i < 100; i++) {
		test_now += 100;
		pas_event_handler();
	}
	check_near(pas_compute_output(2.0), 0.0, 0.001, "no assist once pedalling stops");

	test_speed = 0.0;
}

// Torque times cadence collapses as the cranks slow, so the assist calculation
// floors the cadence. Reported rider power must still use the real one.
static void test_cadence_floor(void) {
	printf("cadence floor\n");
	pas_config c = base_config();
	c.ctrl_type = PAS_CTRL_TYPE_POWER;
	c.torque_source = PAS_TORQUE_SRC_ADC;
	c.torque_adc_ch = PAS_TORQUE_ADC_EXT1;
	c.torque_zero_v = 0.0;
	c.torque_nm_per_v = 30.0;
	c.torque_max_nm = 200.0;
	c.torque_deadband_nm = 0.0;
	c.current_scaling = 1.0;
	c.assist_gain = 1.0;
	c.cadence_floor_rpm = 55.0;

	// Pedal well below the floor.
	setup(&c);
	set_adc_volts(ADC_IND_EXT, 1.0);
	systime_t t = ticks_for_rpm(25.0);
	for (int i = 0; i < 40; i++) {
		cycle_fwd_with_control(t);
	}

	float rider = app_pas_get_rider_power();
	float basis = app_pas_get_assist_basis_power();

	check_near(app_pas_get_pedal_rpm(), 25.0, 1.0, "pedalling at 25 rpm");
	check(basis > rider * 1.5f, "the assist basis is lifted by the floor");
	check_near(basis / rider, 55.0f / 25.0f, 0.1,
			"the assist basis is scaled by the ratio of the floor to the cadence");
	check_near(app_pas_get_motor_power_target(), basis * c.assist_gain, 1.0,
			"motor power comes from the floored basis");

	// Above the floor the two agree.
	setup(&c);
	set_adc_volts(ADC_IND_EXT, 1.0);
	t = ticks_for_rpm(80.0);
	for (int i = 0; i < 40; i++) {
		cycle_fwd_with_control(t);
	}
	check_near(app_pas_get_assist_basis_power(), app_pas_get_rider_power(), 1.0,
			"above the floor the basis is the rider power");
}

static void test_cadence_floor_off(void) {
	printf("cadence floor off\n");
	pas_config c = base_config();
	c.ctrl_type = PAS_CTRL_TYPE_POWER;
	c.torque_source = PAS_TORQUE_SRC_ADC;
	c.torque_adc_ch = PAS_TORQUE_ADC_EXT1;
	c.torque_zero_v = 0.0;
	c.torque_nm_per_v = 30.0;
	c.torque_max_nm = 200.0;
	c.torque_deadband_nm = 0.0;
	c.current_scaling = 1.0;
	c.cadence_floor_rpm = 0.0;
	setup(&c);

	set_adc_volts(ADC_IND_EXT, 1.0);
	systime_t t = ticks_for_rpm(25.0);
	for (int i = 0; i < 40; i++) {
		cycle_fwd_with_control(t);
	}
	check_near(app_pas_get_assist_basis_power(), app_pas_get_rider_power(), 0.5,
			"with no floor the basis is the rider power");
}

// The floor must not create assist out of a stopped crank, which would undo the
// property that makes the power type safe against a stuck torque sensor.
static void test_cadence_floor_not_applied_when_stopped(void) {
	printf("cadence floor when stopped\n");
	pas_config c = base_config();
	c.ctrl_type = PAS_CTRL_TYPE_POWER;
	c.torque_source = PAS_TORQUE_SRC_ADC;
	c.torque_adc_ch = PAS_TORQUE_ADC_EXT1;
	c.torque_zero_v = 0.0;
	c.torque_nm_per_v = 30.0;
	c.torque_max_nm = 200.0;
	c.torque_deadband_nm = 0.0;
	c.current_scaling = 1.0;
	c.cadence_floor_rpm = 55.0;
	setup(&c);

	// Standing on the pedals, cranks still.
	set_adc_volts(ADC_IND_EXT, 2.0);
	for (int i = 0; i < 100; i++) {
		test_now += 100;
		pas_event_handler();
		pas_compute_output(10.0);
	}

	check(app_pas_get_torque_nm() > 10.0, "torque is being read");
	check_near(app_pas_get_assist_basis_power(), 0.0, 0.001,
			"no assist basis from a stopped crank even with a floor set");
	check_near(pas_compute_output(2.0), 0.0, 0.001, "and no assist");
}

// pedal() returns two transitions after the pulse that was last accepted, since
// a pulse is accepted on arriving at state 3 and a cycle is four transitions.
// The measurement below therefore starts half a pulse interval into the stop
// window, and the interval is set by ticks_for_rpm, not by config.magnets.
#define CUTOFF_MEASURE_OFFSET_S		(0.5f * 60.0f / (50.0f * (float)MAGNETS))

// Time from the start of the measurement to the cadence being reported as zero.
static float time_to_cutoff(pas_config *c) {
	setup(c);
	pedal(50.0, 16);

	for (int i = 0; i < 2000; i++) {
		test_now += 10;			// 1 ms
		pas_event_handler();
		if (app_pas_get_pedal_rpm() < 0.001) {
			return (float)(i + 1) * 0.001f;
		}
	}
	return -1.0;
}

// The stop threshold is what controls how long assist lingers, and it has to be
// independent of the start threshold because the two are tuned oppositely.
static void test_stop_threshold(void) {
	printf("stop threshold\n");
	pas_config c = base_config();
	c.magnets = 18;			// as on the sensor this was written against

	c.start_timeout_s = 0.4;
	c.stop_timeout_s = 0.15;
	float quick = time_to_cutoff(&c);
	check_near(quick, 0.15 - CUTOFF_MEASURE_OFFSET_S, 0.01,
			"a short stop threshold cuts off quickly");

	c.stop_timeout_s = 0.5;
	float slow = time_to_cutoff(&c);
	check_near(slow, 0.5 - CUTOFF_MEASURE_OFFSET_S, 0.01,
			"a long stop threshold lingers");

	check(slow > quick * 2.0f, "the stop threshold controls the cutoff delay");

	// And changing the start threshold does not move the cutoff.
	c.stop_timeout_s = 0.15;
	c.start_timeout_s = 1.0;
	check_near(time_to_cutoff(&c), quick, 0.005,
			"the start threshold does not affect the cutoff");
}

// Zero means derive from the start cadence and magnet count, which is what the
// app did when one value served both roles.
static void test_thresholds_derived(void) {
	printf("thresholds derived\n");
	pas_config c = base_config();
	c.magnets = 18;
	c.pedal_rpm_start = 10.0;
	c.start_timeout_s = 0.0;
	c.stop_timeout_s = 0.0;

	// 60 / 10 / 18 * 1.2 = 0.4 s, which is above the 0.15 to 0.30 s Grin
	// recommend, and is the reason for making it explicit.
	check_near(time_to_cutoff(&c), 0.4 - CUTOFF_MEASURE_OFFSET_S, 0.01,
			"the derived cutoff is the legacy value");

	// A low pole count sensor gets a much longer cutoff from the same formula.
	c.magnets = 8;
	check_near(time_to_cutoff(&c), 0.9 - CUTOFF_MEASURE_OFFSET_S, 0.01,
			"the derived cutoff scales with the magnet count");
}

// The start threshold gates whether a gap counts as continued pedalling.
static void test_start_threshold(void) {
	printf("start threshold\n");
	pas_config c = base_config();
	c.stop_timeout_s = 1.0;			// out of the way

	// At 50 rpm with 24 magnets the gap between pulses is 50 ms. A start
	// threshold below that means no gap ever counts as continued pedalling, so
	// every pulse is only a reference and no cadence is established.
	c.start_timeout_s = 0.02;
	setup(&c);
	pedal(50.0, 20);
	check_near(app_pas_get_pedal_rpm(), 0.0, 0.001,
			"a start threshold below the pulse gap never establishes a cadence");

	// Above the gap it works normally.
	c.start_timeout_s = 0.3;
	setup(&c);
	pedal(50.0, 20);
	check_near(app_pas_get_pedal_rpm(), 50.0, 0.5,
			"a start threshold above the pulse gap decodes normally");
}

// A Thun-style sensor rests mid range and swings both ways, so both directions
// are pedal effort.
static void test_torque_bipolar(void) {
	printf("torque bipolar\n");
	pas_config c = base_config();
	c.torque_source = PAS_TORQUE_SRC_ADC;
	c.torque_adc_ch = PAS_TORQUE_ADC_EXT1;
	c.torque_zero_v = 1.65;			// mid rail
	c.torque_nm_per_v = 100.0;
	c.torque_max_nm = 200.0;
	c.torque_deadband_nm = 0.0;
	c.torque_bipolar = true;

	// Half a volt either side of the zero point is the same torque.
	setup(&c);
	set_adc_volts(ADC_IND_EXT, 2.15);
	for (int i = 0; i < 80; i++) {
		pas_read_torque_nm();
	}
	float above = pas_read_torque_nm();

	setup(&c);
	set_adc_volts(ADC_IND_EXT, 1.15);
	for (int i = 0; i < 80; i++) {
		pas_read_torque_nm();
	}
	float below = pas_read_torque_nm();

	check(above > 40.0, "torque above the zero point");
	check_near(below, above, 0.5, "the same magnitude below the zero point");

	// Unipolar discards the low side, which is the behaviour a sensor resting
	// at the bottom of its range needs.
	c.torque_bipolar = false;
	setup(&c);
	set_adc_volts(ADC_IND_EXT, 1.15);
	for (int i = 0; i < 80; i++) {
		pas_read_torque_nm();
	}
	check_near(pas_read_torque_nm(), 0.0, 0.001, "unipolar reads nothing below the zero point");
}

// A minimum rider power before the motor contributes, which is a power
// threshold rather than a force one.
static void test_assist_start_power(void) {
	printf("assist start power\n");
	pas_config c = base_config();
	c.ctrl_type = PAS_CTRL_TYPE_POWER;
	c.torque_source = PAS_TORQUE_SRC_ADC;
	c.torque_adc_ch = PAS_TORQUE_ADC_EXT1;
	c.torque_zero_v = 0.0;
	c.torque_nm_per_v = 30.0;
	c.torque_max_nm = 200.0;
	c.torque_deadband_nm = 0.0;
	c.current_scaling = 1.0;
	c.assist_gain = 1.0;
	c.cadence_floor_rpm = 0.0;
	ramp_off(&c);

	// Without a start level, assist tracks the basis.
	c.assist_start_w = 0.0;
	setup(&c);
	set_adc_volts(ADC_IND_EXT, 1.0);
	systime_t t = ticks_for_rpm(60.0);
	for (int i = 0; i < 40; i++) {
		cycle_fwd_with_control(t);
	}
	float basis = app_pas_get_assist_basis_power();
	check_near(app_pas_get_motor_power_target(), basis, 1.0, "no start level, assist is the basis");

	// With one, it is subtracted before the gain.
	c.assist_start_w = 50.0;
	setup(&c);
	set_adc_volts(ADC_IND_EXT, 1.0);
	for (int i = 0; i < 40; i++) {
		cycle_fwd_with_control(t);
	}
	check_near(app_pas_get_motor_power_target(),
			app_pas_get_assist_basis_power() - 50.0, 1.5,
			"the start level is subtracted before the gain");

	// A start level above the rider's effort means no assist at all, not
	// negative assist.
	c.assist_start_w = 10000.0;
	setup(&c);
	set_adc_volts(ADC_IND_EXT, 1.0);
	for (int i = 0; i < 40; i++) {
		cycle_fwd_with_control(t);
	}
	check_near(app_pas_get_motor_power_target(), 0.0, 0.001,
			"a start level above the rider effort gives no assist");
	check_near(pas_compute_output(2.0), 0.0, 0.001, "and no output");
}

// Both throttle mixing policies, since both are defensible and the choice is
// now a setting.
static void test_throttle_mixing(void) {
	printf("throttle mixing\n");
	pas_config c = base_config();
	c.throttle_no_pedal_kmh = 0.0;

	// Highest wins: a throttle can only add.
	c.throttle_mode = PAS_THROTTLE_MAX;
	setup(&c);
	check_near(pas_mix_throttle(0.2, 0.5), 0.5, 0.001, "highest wins takes the PAS output");
	check_near(pas_mix_throttle(0.7, 0.5), 0.7, 0.001, "highest wins takes the throttle");
	check_near(pas_mix_throttle(0.0, 0.5), 0.5, 0.001, "no throttle leaves the assist");

	// Throttle priority: any throttle takes over, even below the assist.
	c.throttle_mode = PAS_THROTTLE_PRIORITY;
	setup(&c);
	check_near(pas_mix_throttle(0.2, 0.5), 0.2, 0.001,
			"throttle priority takes the throttle even when it asks for less");
	check_near(pas_mix_throttle(0.7, 0.5), 0.7, 0.001, "and when it asks for more");
	check_near(pas_mix_throttle(0.0, 0.5), 0.5, 0.001,
			"a released throttle leaves the assist");
	check_near(pas_mix_throttle(0.005, 0.5), 0.5, 0.001,
			"and so does a throttle below the takeover threshold");
}

// Above a configured speed the throttle needs pedalling, which is how a pedelec
// arrangement is usually expressed.
static void test_throttle_needs_pedalling(void) {
	printf("throttle needs pedalling\n");
	pas_config c = base_config();
	c.throttle_mode = PAS_THROTTLE_MAX;
	c.throttle_no_pedal_kmh = 6.0;

	// Below the limit, the throttle works without pedalling.
	setup(&c);
	test_speed = 4.0 / 3.6;
	check_near(pas_mix_throttle(0.6, 0.0), 0.6, 0.001,
			"below the limit the throttle works without pedalling");

	// Above it, not while stopped.
	setup(&c);
	test_speed = 10.0 / 3.6;
	check_near(pas_mix_throttle(0.6, 0.0), 0.0, 0.001,
			"above the limit a throttle alone does nothing");

	// Above it while pedalling, it works.
	setup(&c);
	test_speed = 10.0 / 3.6;
	pedal(50.0, 12);
	check(app_pas_get_pedal_rpm() > 1.0, "pedalling");
	check_near(pas_mix_throttle(0.6, 0.0), 0.6, 0.001,
			"above the limit the throttle works while pedalling");

	// Zero disables the check, which is the difference from a Cycle Analyst.
	c.throttle_no_pedal_kmh = 0.0;
	setup(&c);
	test_speed = 40.0 / 3.6;
	check_near(pas_mix_throttle(0.6, 0.0), 0.6, 0.001,
			"zero leaves the throttle working at any speed");

	test_speed = 0.0;
}

static pas_config walk_config(void) {
	pas_config c = base_config();
	c.ctrl_type = PAS_CTRL_TYPE_CADENCE;
	c.current_scaling = 1.0;
	c.pedal_rpm_start = 10.0;
	c.pedal_rpm_end = 100.0;
	c.walk_source = PAS_WALK_SRC_LISP;
	c.walk_max_kmh = 6.0;
	c.walk_current = 0.05;
	c.walk_require_pedal = false;
	return c;
}

static void test_walk_basic(void) {
	printf("walk assist\n");
	pas_config c = walk_config();
	setup(&c);
	test_speed = 0.0;

	// Nothing until it is requested, even though the cranks are still, which is
	// the difference from the pedal assist types.
	check_near(pas_compute_output(2.0), 0.0, 0.001, "no walk assist until requested");

	app_pas_walk_set(true);
	check_near(pas_compute_output(2.0), 0.05, 0.0001,
			"walk assist drives with the cranks stopped");
	check(0 != (app_pas_get_flags() & PAS_FLAG_WALK_ACTIVE), "the walk flag is set");

	// Releasing stops it at once rather than over the ramp.
	app_pas_walk_set(false);
	check_near(pas_compute_output(2.0), 0.0, 0.001, "releasing stops it immediately");
	check(0 == (app_pas_get_flags() & PAS_FLAG_WALK_ACTIVE), "the walk flag clears");

	// A source of None ignores the request entirely.
	c.walk_source = PAS_WALK_SRC_NONE;
	setup(&c);
	app_pas_walk_set(true);
	check_near(pas_compute_output(2.0), 0.0, 0.001, "no walk assist without a source");
	app_pas_walk_set(false);
}

// The script request is a keepalive. A display that stops talking must release
// walk assist rather than leave the motor driving.
static void test_walk_keepalive_expires(void) {
	printf("walk keepalive\n");
	pas_config c = walk_config();
	setup(&c);
	test_speed = 0.0;

	app_pas_walk_set(true);
	check(pas_compute_output(2.0) > 0.0, "driving while the request is fresh");

	// Refreshed inside the window, it keeps going indefinitely.
	for (int i = 0; i < 10; i++) {
		test_now += 3000;			// 300 ms, inside the 500 ms window
		app_pas_walk_set(true);
		check_near(pas_compute_output(2.0), 0.05, 0.0001, "refreshed, still driving");
	}

	// Left unrefreshed, it expires.
	test_now += 6000;				// 600 ms, past the 500 ms timeout
	check_near(pas_compute_output(2.0), 0.0, 0.001,
			"an unrefreshed request expires and stops the motor");

	app_pas_walk_set(false);
}

// The speed limit fades it out rather than cutting, and stops it above.
static void test_walk_speed_limit(void) {
	printf("walk speed limit\n");
	pas_config c = walk_config();
	setup(&c);

	app_pas_walk_set(true);

	test_speed = 2.0 / 3.6;
	check_near(pas_compute_output(2.0), 0.05, 0.0001, "full walk current well below the limit");

	// The taper band is the last 1 km/h, so 5.5 of 6 is half.
	test_speed = 5.5 / 3.6;
	check_near(pas_compute_output(2.0), 0.025, 0.002, "half way through the taper band");

	test_speed = 6.0 / 3.6;
	check_near(pas_compute_output(2.0), 0.0, 0.001, "nothing at the limit");

	test_speed = 9.0 / 3.6;
	check_near(pas_compute_output(2.0), 0.0, 0.001, "nothing above the limit");

	app_pas_walk_set(false);
	test_speed = 0.0;
}

// Both behaviours, since either is reasonable.
static void test_walk_require_pedal(void) {
	printf("walk requires pedalling\n");
	pas_config c = walk_config();
	test_speed = 0.0;

	// Off: it drives with the cranks stopped, which is the point of it.
	c.walk_require_pedal = false;
	setup(&c);
	app_pas_walk_set(true);
	check(pas_compute_output(2.0) > 0.0, "off, walk assist drives with the cranks stopped");
	app_pas_walk_set(false);

	// On: the request alone does nothing.
	c.walk_require_pedal = true;
	setup(&c);
	app_pas_walk_set(true);
	check_near(pas_compute_output(2.0), 0.0, 0.001,
			"on, the request alone does nothing with the cranks stopped");

	// On, and pedalling: it drives.
	pedal(50.0, 12);
	app_pas_walk_set(true);
	check(app_pas_get_pedal_rpm() > 1.0, "pedalling");
	check(pas_compute_output(2.0) > 0.0, "on, walk assist drives while pedalling");
	app_pas_walk_set(false);
}

// The brake has to win over walk assist as it does over pedal assist.
static void test_walk_brake_overrides(void) {
	printf("walk brake override\n");
	pas_config c = walk_config();
	c.brake_source = PAS_BRAKE_SRC_ADC;
	c.brake_adc_ch = PAS_TORQUE_ADC_EXT2;
	c.brake_threshold_v = 1.65;
	setup(&c);
	test_speed = 0.0;

	set_adc_volts(ADC_IND_EXT2, 0.0);
	app_pas_walk_set(true);
	check(pas_compute_output(2.0) > 0.0, "driving with the brake released");

	set_adc_volts(ADC_IND_EXT2, 3.0);
	check_near(pas_compute_output(2.0), 0.0, 0.001, "the brake stops walk assist");

	app_pas_walk_set(false);
}

// Walk assist replaces the control type rather than adding to it, and leaving it
// must not ramp down from the walk current.
static void test_walk_replaces_and_releases(void) {
	printf("walk replaces assist\n");
	pas_config c = walk_config();
	c.walk_current = 0.4;
	c.ramp_time_pos = 0.2;
	c.ramp_time_neg = 2.0;
	setup(&c);
	test_speed = 0.0;

	// Pedalling gives cadence assist, ramping up.
	pedal(50.0, 12);
	for (int i = 0; i < 400; i++) {
		pas_compute_output(2.0);
	}
	float pedalling = pas_compute_output(2.0);
	check(pedalling > 0.0, "cadence assist while pedalling");

	// Walk takes over at its own current, not the sum.
	app_pas_walk_set(true);
	check_near(pas_compute_output(2.0), 0.4, 0.0001, "walk assist replaces the assist output");

	// Releasing walk drops straight out rather than ramping down from 0.4.
	app_pas_walk_set(false);
	float after = pas_compute_output(2.0);
	check(after < 0.05, "leaving walk assist does not ramp down from the walk current");
}

// Config for the power-control tests: a known torque, cadence and full scale.
static pas_config power_config(void) {
	pas_config c = base_config();
	c.ctrl_type = PAS_CTRL_TYPE_POWER;
	c.torque_source = PAS_TORQUE_SRC_ADC;
	c.torque_adc_ch = PAS_TORQUE_ADC_EXT1;
	c.torque_zero_v = 0.0;
	c.torque_nm_per_v = 30.0;
	c.torque_max_nm = 200.0;
	c.torque_deadband_nm = 0.0;
	c.current_scaling = 1.0;
	c.assist_gain = 1.0;
	c.cadence_floor_rpm = 0.0;
	c.power_gain = 2.0;
	ramp_off(&c);
	return c;
}

// Pedal and run the loop, with the measured input power supplied by the caller
// as a fraction of what the output is actually asking for. 1.0 means the motor
// delivers exactly the request, which is the case the trim should settle at.
static float run_power_loop(pas_config *c, float delivered_fraction, int iters) {
	setup(c);
	set_adc_volts(ADC_IND_EXT, 1.0);
	systime_t t = ticks_for_rpm(60.0);

	// Fill the torque filter and establish cadence.
	for (int i = 0; i < 20; i++) {
		cycle_fwd_with_control(t);
	}

	float out = 0.0;
	for (int i = 0; i < iters; i++) {
		// What the motor is drawing, given the output the loop just asked for.
		test_current_in = out * test_mcconf.lo_current_max * delivered_fraction;
		out = pas_compute_output(2.0);
		// Keep the cranks turning so the cadence does not lapse.
		if ((i % 10) == 0) {
			cycle_fwd(t);
		}
	}
	return out;
}

static void test_power_open_loop_default(void) {
	printf("power open loop\n");
	pas_config c = power_config();
	c.power_ctrl_mode = PAS_POWER_OPEN_LOOP;
	test_v_in = 50.0;
	test_mcconf.lo_current_max = 100.0;

	// Whatever the motor actually draws, open loop does not react to it.
	float full = run_power_loop(&c, 1.0, 200);
	float starved = run_power_loop(&c, 0.3, 200);
	check_near(starved, full, 0.001, "open loop ignores measured power");

	// And it is the request over the voltage over the current limit.
	float expect = (app_pas_get_motor_power_target() / 50.0f) / 100.0f;
	check_near(starved, expect, 0.002, "open loop is the feedforward estimate");

	test_current_in = 0.0;
}

// Closed loop trims towards the request when the motor is delivering less than
// asked, which is the case open loop cannot correct.
static void test_power_closed_loop_trims_up(void) {
	printf("power closed loop trims up\n");
	pas_config c = power_config();
	test_v_in = 50.0;
	test_mcconf.lo_current_max = 100.0;

	c.power_ctrl_mode = PAS_POWER_OPEN_LOOP;
	float open = run_power_loop(&c, 0.5, 300);

	c.power_ctrl_mode = PAS_POWER_CLOSED_LOOP;
	float closed = run_power_loop(&c, 0.5, 300);

	check(closed > open * 1.5f,
			"closed loop raises the request when the motor delivers less than asked");
	check(closed <= 1.0, "and stays within the relative current range");

	test_current_in = 0.0;
}

// And backs off when more is being drawn than asked for.
static void test_power_closed_loop_trims_down(void) {
	printf("power closed loop trims down\n");
	pas_config c = power_config();
	test_v_in = 50.0;
	test_mcconf.lo_current_max = 100.0;

	c.power_ctrl_mode = PAS_POWER_OPEN_LOOP;
	float open = run_power_loop(&c, 1.0, 300);

	c.power_ctrl_mode = PAS_POWER_CLOSED_LOOP;
	float closed = run_power_loop(&c, 2.0, 300);

	check(closed < open, "closed loop backs off when more power is drawn than asked");
	check(closed >= 0.0, "and never goes negative");

	test_current_in = 0.0;
}

// With the motor delivering exactly the request, the trim should settle rather
// than drift, so closed loop matches open loop in steady state.
static void test_power_closed_loop_settles(void) {
	printf("power closed loop settles\n");
	pas_config c = power_config();
	test_v_in = 50.0;
	test_mcconf.lo_current_max = 100.0;

	c.power_ctrl_mode = PAS_POWER_OPEN_LOOP;
	float open = run_power_loop(&c, 1.0, 400);

	c.power_ctrl_mode = PAS_POWER_CLOSED_LOOP;
	float closed = run_power_loop(&c, 1.0, 400);

	check_near(closed, open, open * 0.1f,
			"with the request being delivered, the trim settles near the feedforward");
}

// The trim must not wind up past the ceiling while the output is clamped.
static void test_power_closed_loop_antiwindup(void) {
	printf("power closed loop anti-windup\n");
	pas_config c = power_config();
	test_v_in = 50.0;
	test_mcconf.lo_current_max = 100.0;

	// A tight assist limit, and a motor that never delivers, so the loop is
	// permanently asking for more and permanently clamped.
	c.power_ctrl_mode = PAS_POWER_CLOSED_LOOP;
	c.current_scaling = 0.05;
	float clamped = run_power_loop(&c, 0.0, 600);
	check_near(clamped, 0.05, 0.001, "the output stays at the assist limit");

	// Now let the motor deliver. The output must come back promptly rather
	// than having to unwind a large accumulated correction.
	for (int i = 0; i < 30; i++) {
		test_current_in = 10.0 * test_mcconf.lo_current_max;
		pas_compute_output(2.0);
	}
	check(pas_compute_output(2.0) < 0.05,
			"and comes back down promptly once power is being delivered");

	test_current_in = 0.0;
}

// In the combined ADC and PAS mode the measured power includes the throttle, so
// the loop stays open rather than fighting it.
static void test_power_closed_loop_needs_primary(void) {
	printf("power closed loop needs primary output\n");
	pas_config c = power_config();
	test_v_in = 50.0;
	test_mcconf.lo_current_max = 100.0;
	c.power_ctrl_mode = PAS_POWER_CLOSED_LOOP;

	// Primary: the loop closes and trims up on a starved motor.
	app_pas_start(true);
	float primary = run_power_loop(&c, 0.5, 300);
	app_pas_stop();

	// Sharing the output with a throttle: open loop regardless of the setting.
	app_pas_start(false);
	float shared = run_power_loop(&c, 0.5, 300);
	app_pas_stop();

	check(primary > shared * 1.5f,
			"closed loop only applies when PAS is the only thing driving");

	c.power_ctrl_mode = PAS_POWER_OPEN_LOOP;
	app_pas_start(false);
	check_near(run_power_loop(&c, 0.5, 300), shared, 0.001,
			"and the shared case matches open loop exactly");
	app_pas_stop();

	test_current_in = 0.0;
}

// The getters the dash and the lisp bindings read, and the throttle wrapper
// app_adc.c calls. Coverage showed every one of these unreached: they are one
// or three lines each, but they are the whole of the interface this app
// presents to the rest of the firmware, and a wrapper that returned the wrong
// one of its two arguments would be invisible to every other test here.
static void test_public_interface(void) {
	printf("public interface\n");
	pas_config c = base_config();
	c.ctrl_type = PAS_CTRL_TYPE_CADENCE;
	setup(&c);

	// Not running: the throttle passes straight through, untouched. This is
	// the path every board without PAS configured takes on every ADC cycle.
	app_pas_stop();
	check(!app_pas_is_running(), "stopped app reports not running");
	check_near(app_pas_apply_to_throttle(0.4), 0.4, 0.0001,
			"throttle passes through while stopped");
	check_near(app_pas_apply_to_throttle(-0.3), -0.3, 0.0001,
			"including a negative throttle");

	// is_running is set by the thread, not by app_pas_start, and the host
	// stub for chThdCreateStatic does not run one -- so the mixing branch of
	// app_pas_apply_to_throttle cannot be reached from here. That is worth
	// knowing rather than working around: it means the wrapper passes the
	// throttle through unchanged for however long it takes the thread to be
	// scheduled after a start, and the mixing itself is covered directly
	// through pas_mix_throttle below.
	test_appconf.app_to_use = APP_PAS;
	app_pas_start(true);
	check_near(app_pas_apply_to_throttle(0.4), 0.4, 0.0001,
			"throttle still passes through before the thread has run");

	pedal(60.0, 12);
	// The return value, not app_pas_get_current_target_rel: that getter
	// reports output_current_rel, which the thread writes after calling this.
	// So it, like app_pas_is_running, reads as its initial value in a host
	// test however much the assist law is exercised.
	float assist = pas_compute_output(2.0);
	check(assist > 0.0, "assist is being produced");

	// What the wrapper would return once the thread has set is_running: the
	// same call it makes, with the same two arguments.
	check_near(pas_mix_throttle(0.0, assist), assist, 0.0001,
			"assist becomes the throttle when the lever is at rest");
	check_near(pas_mix_throttle(0.9, assist), 0.9, 0.0001,
			"a larger throttle wins over the assist");

	// The reporting getters. Values, not just finiteness: a getter returning
	// the wrong field is the failure mode, and every one of these is read by
	// the dash over CAN.
	check_near(app_pas_get_speed_taper(), 1.0, 0.0001,
			"taper is fully open below the taper start");
	check_near(app_pas_get_torque_ratio(), 0.0, 0.0001,
			"no torque ratio without a torque sensor");
	check(app_pas_get_measured_power() >= 0.0, "measured power is not negative");

	// The sub scaling another app can apply at runtime scales the ceiling,
	// which is the only way that value is ever set.
	app_pas_set_current_sub_scaling(0.5);
	pedal(60.0, 12);
	float scaled = pas_compute_output(2.0);
	app_pas_set_current_sub_scaling(1.0);
	pedal(60.0, 12);
	float full = pas_compute_output(2.0);
	check(scaled < full, "sub scaling reduces the output");

	app_pas_stop();
	test_appconf.app_to_use = APP_NONE;
}

// A negative PAS Max Current made pedalling produce braking current: the
// ceiling is used as the maximum of utils_truncate_number, which tests the
// maximum before the minimum, so a negative maximum clamped the output to it
// rather than to zero. VESC Tool will not send one, but nothing in the
// firmware enforces the XML limits and any sender of COMM_SET_APPCONF can.
//
// Found by fuzz.c. Kept here because a fuzzer finding is only a regression
// test once it is one.
static void test_negative_scaling_cannot_brake(void) {
	printf("negative scaling cannot brake\n");

	pas_config c = base_config();
	c.ctrl_type = PAS_CTRL_TYPE_CADENCE;
	c.current_scaling = -0.88;
	setup(&c);

	pedal(60.0, 20);
	for (int i = 0; i < 20; i++) {
		float out = pas_compute_output(2.0);
		check(out >= 0.0, "no negative output from a negative scaling");
	}

	// The same through the sub scaling, which is not a configuration field at
	// all and arrives from another app at runtime.
	c.current_scaling = 0.5;
	setup(&c);
	app_pas_set_current_sub_scaling(-1.0);
	pedal(60.0, 20);
	for (int i = 0; i < 20; i++) {
		check(pas_compute_output(2.0) >= 0.0,
				"no negative output from a negative sub scaling");
	}
	app_pas_set_current_sub_scaling(1.0);
}

// --- The thread, and the terminal commands -------------------------------
//
// Coverage put these at zero: the three terminal commands are 214 of the
// file's 680 lines and the thread another 34. They are not pure arithmetic, so
// they were left out at first, but they are not glue either -- the thread holds
// the safe-start interlock, the fault check and the output-disabled path, and
// each of those decides whether the motor gets current.
//
// The thread runs on the cooperative scheduler in sched.h, which is what makes
// its stop path reachable as well as its loop.

extern const char *test_cmd_name[];
extern void (*test_cmd_fn[])(int argc, const char **argv);
extern int test_cmd_count;
extern char test_out[];
extern void test_out_clear(void);

// Run the app's thread up to n of its sleeps. Each switch resumes it where it
// last slept, so this is n iterations of its loop, deterministically and with
// the test in control between each one. See sched.h.
static void run_thread(int n) {
	for (int i = 0; i < n && sched_alive(); i++) {
		sched_switch();
	}
}

static void (*find_cmd(const char *name))(int, const char **) {
	for (int i = 0; i < test_cmd_count; i++) {
		if (test_cmd_name[i] && strcmp(test_cmd_name[i], name) == 0) {
			return test_cmd_fn[i];
		}
	}
	return 0;
}

static void call_cmd(const char *name, int argc, const char **argv) {
	void (*fn)(int, const char **) = find_cmd(name);
	test_out_clear();
	if (fn) {
		fn(argc, argv);
	}
}

static bool out_has(const char *needle) {
	return strstr(test_out, needle) != 0;
}

static void test_thread_loop(void) {
	printf("thread loop\n");
	pas_config c = base_config();
	c.ctrl_type = PAS_CTRL_TYPE_CADENCE;
	setup(&c);

	test_appconf.app_to_use = APP_PAS;
	test_fault = FAULT_CODE_NONE;
	app_pas_start(true);
	check(sched_alive(), "starting the app created a thread");
	check(!app_pas_is_running(), "which has not run yet, so is_running is false");

	// Safe start: the output must stay at zero until it has been zero for long
	// enough, so a display that comes up with the cranks already turning
	// cannot hand the motor current immediately.
	//
	// Note what it actually does: the interlock zeroes the published value and
	// skips the rest of the iteration, so mc_interface_set_current_rel is not
	// called at all rather than called with zero. The sentinel checks that,
	// which is the stronger statement -- nothing else can be driving the motor
	// through this app while the interlock holds.
	pedal(60.0, 12);
	test_current_rel = -1.0;
	run_thread(3);
	check_near(test_current_rel, -1.0, 0.0001,
			"safe start does not write the motor at all");
	check_near(app_pas_get_current_target_rel(), 0.0, 0.0001,
			"and publishes zero for the throttle app to read");

	// Given enough idle iterations the interlock clears and pedalling reaches
	// the motor.
	app_pas_start(true);
	run_thread(400);
	pedal(60.0, 12);
	app_pas_start(true);
	run_thread(20);
	check(app_pas_get_current_target_rel() > 0.0,
			"assist reaches the output once safe start has cleared");
	check(test_current_rel > 0.0, "and is written to the motor");

	// A fault resets the interlock, which is the whole point of it: the motor
	// must not pick up again the instant a fault clears.
	test_fault = FAULT_CODE_OVER_VOLTAGE;
	app_pas_start(true);
	pedal(60.0, 12);
	test_current_rel = -1.0;
	run_thread(3);
	check_near(test_current_rel, -1.0, 0.0001,
			"a fault stops the motor being written");
	check_near(app_pas_get_current_target_rel(), 0.0, 0.0001,
			"and publishes zero while it stands");
	test_fault = FAULT_CODE_NONE;

	// Not the primary output: the value is still published for app_adc to read
	// but nothing is written to the motor.
	app_pas_start(false);
	run_thread(400);
	pedal(60.0, 12);
	app_pas_start(false);
	run_thread(20);
	test_current_rel = -1.0;
	pedal(60.0, 12);
	app_pas_start(false);
	run_thread(20);
	check_near(test_current_rel, -1.0, 0.0001,
			"nothing is written to the motor when PAS is not primary");
	check(app_pas_get_current_target_rel() > 0.0,
			"but the value is still published");

	// is_running, which only the thread sets.
	check(app_pas_is_running(), "the thread sets is_running");

	// The mixing branch of app_pas_apply_to_throttle, which needs is_running
	// and is what app_adc.c calls on every ADC cycle. Unreachable before the
	// scheduler existed.
	pedal(60.0, 12);
	run_thread(20);
	float assist = app_pas_get_current_target_rel();
	check(assist > 0.0, "the thread published an assist value");
	check_near(app_pas_apply_to_throttle(0.0), assist, 0.0001,
			"a resting throttle is replaced by the assist");
	check_near(app_pas_apply_to_throttle(0.9), 0.9, 0.0001,
			"a larger throttle wins");

	// And the real stop path. app_pas_stop sets the flag and then sleeps until
	// the thread acknowledges; each of those sleeps runs the thread, which
	// sees the flag, clears is_running and returns. This is the handshake the
	// firmware actually performs at shutdown and on an app change.
	app_pas_stop();
	check(!app_pas_is_running(), "stopping clears is_running");
	check(!sched_alive(), "and the thread has returned");
	check_near(app_pas_get_current_target_rel(), 0.0, 0.0001,
			"and the published value is cleared, so a later throttle-only "
			"session cannot inherit it");

	// Stopping unregisters the terminal commands, so a restart does not
	// register duplicates.
	check(find_cmd("pas_status") == 0, "and the terminal commands are gone");

	sched_reset();
	test_appconf.app_to_use = APP_NONE;
}

// The flag bits, one condition at a time. These are the whole of what a
// display can say about why assist is not what the rider expects -- the dash
// turns them into its status word -- so a bit set from the wrong condition
// would have it reporting the wrong fault with no way to tell from the outside.
static void test_flag_bits(void) {
	printf("flag bits\n");

	// A channel that does not exist on this hardware, which is the one every
	// board hits if it is configured for a sensor it has no pin for.
	pas_config c = base_config();
	c.ctrl_type = PAS_CTRL_TYPE_TORQUE;
	c.torque_source = PAS_TORQUE_SRC_ADC;
	c.torque_adc_ch = PAS_TORQUE_ADC_EXT8;
	setup(&c);
	pas_compute_output(2.0);
	check(app_pas_get_flags() & PAS_FLAG_TORQUE_CH_INVALID,
			"an impossible torque channel raises TORQUE_CH_INVALID");

	c = base_config();
	c.brake_source = PAS_BRAKE_SRC_ADC;
	c.brake_adc_ch = PAS_TORQUE_ADC_EXT8;
	setup(&c);
	pas_compute_output(2.0);
	check(app_pas_get_flags() & PAS_FLAG_BRAKE_CH_INVALID,
			"an impossible brake channel raises BRAKE_CH_INVALID");

	c = base_config();
	c.walk_source = PAS_WALK_SRC_ADC;
	c.walk_adc_ch = PAS_TORQUE_ADC_EXT8;
	setup(&c);
	pas_compute_output(2.0);
	check(app_pas_get_flags() & PAS_FLAG_WALK_CH_INVALID,
			"an impossible walk channel raises WALK_CH_INVALID");

	// A sensor type the build has no decoder for.
	c = base_config();
	c.ctrl_type = PAS_CTRL_TYPE_TORQUE;
	c.torque_source = PAS_TORQUE_SRC_HW;
	c.torque_max_nm = 140.0;
	setup(&c);
	pedal(60.0, 12);
	pas_compute_output(2.0);
	// On a board without hw_get_PAS_torque the override is not compiled in,
	// so asking for it has to be reported rather than silently read as zero.
#ifndef TEST_TORQUE_SENSOR
	check(app_pas_get_flags() & PAS_FLAG_TORQUE_SRC_UNSUPPORTED,
			"asking for a hardware torque source it does not have is reported");
#else
	check(!(app_pas_get_flags() & PAS_FLAG_TORQUE_SRC_UNSUPPORTED),
			"a board with the hardware torque source does not report it missing");
#endif

	// The brake, engaged through the channel that does exist.
	c = base_config();
	c.brake_source = PAS_BRAKE_SRC_ADC;
	c.brake_adc_ch = PAS_TORQUE_ADC_EXT3;
	c.brake_threshold_v = 1.0;
	setup(&c);
	// ADC_IND_EXT3, not 3: the channel enum is not the array index, and the
	// stub hw.h reproduces the aliasing the real hwconf/hw.h does.
	set_adc_volts(ADC_IND_EXT3, 2.0);
	pedal(60.0, 12);
	pas_compute_output(2.0);
	check(app_pas_get_flags() & PAS_FLAG_BRAKE_ENGAGED,
			"an engaged brake raises BRAKE_ENGAGED");
	set_adc_volts(ADC_IND_EXT3, 0.0);

	// The speed taper, which is the flag the dash shows as "splim".
	c = base_config();
	c.taper_start_kmh = 20.0;
	c.taper_end_kmh = 30.0;
	setup(&c);
	test_speed = 25.0 / 3.6;
	pedal(60.0, 12);
	pas_compute_output(2.0);
	check(app_pas_get_flags() & PAS_FLAG_SPEED_LIMITED,
			"riding inside the taper raises SPEED_LIMITED");
	test_speed = 0.0;

	// And nothing set at all, which is the state a working bike is in: a flag
	// word that is never zero would make the dash cry wolf for a whole ride.
	c = base_config();
	setup(&c);
	pedal(60.0, 12);
	pas_compute_output(2.0);
	check(app_pas_get_flags() == 0, "a working configuration raises no flags");
}

// Walk assist can be asked for from a script or from an ADC channel, and the
// two paths share nothing. The lisp one expires on its own, which is what stops
// a display that has lost power from leaving the motor driving.
static void test_walk_sources(void) {
	printf("walk sources\n");

	pas_config c = base_config();
	c.walk_source = PAS_WALK_SRC_LISP;
	c.walk_max_kmh = 6.0;
	c.walk_current = 0.2;
	setup(&c);

	check_near(pas_compute_output(2.0), 0.0, 0.0001, "no walk without a request");

	app_pas_walk_set(true);
	check(pas_compute_output(2.0) > 0.0, "a script request starts walk assist");
	check(app_pas_get_flags() & PAS_FLAG_WALK_ACTIVE, "and raises WALK_ACTIVE");

	// Expiry. The controller must not keep walking because a display stopped
	// talking, so the request is good for WALK_LISP_TIMEOUT_S and no longer.
	test_now += (systime_t)(CH_CFG_ST_FREQUENCY * 2);
	check_near(pas_compute_output(2.0), 0.0, 0.0001,
			"an unrefreshed script request expires");

	app_pas_walk_set(false);
	check_near(pas_compute_output(2.0), 0.0, 0.0001, "and a release stops it");

	// The ADC source, on a channel that exists, with and without inversion.
	c = base_config();
	c.walk_source = PAS_WALK_SRC_ADC;
	c.walk_adc_ch = PAS_TORQUE_ADC_EXT3;
	c.walk_threshold_v = 1.0;
	c.walk_max_kmh = 6.0;
	c.walk_current = 0.2;
	setup(&c);

	set_adc_volts(ADC_IND_EXT3, 0.2);
	check_near(pas_compute_output(2.0), 0.0, 0.0001, "below the threshold, no walk");
	set_adc_volts(ADC_IND_EXT3, 2.0);
	check(pas_compute_output(2.0) > 0.0, "above the threshold, walk");

	c.walk_invert = true;
	setup(&c);
	set_adc_volts(ADC_IND_EXT3, 2.0);
	check_near(pas_compute_output(2.0), 0.0, 0.0001,
			"inverted: above the threshold is no walk");
	set_adc_volts(ADC_IND_EXT3, 0.2);
	check(pas_compute_output(2.0) > 0.0, "inverted: below it is walk");

	set_adc_volts(ADC_IND_EXT3, 0.0);
}

static void test_terminal_commands(void) {
	printf("terminal commands\n");
	pas_config c = base_config();
	c.ctrl_type = PAS_CTRL_TYPE_POWER;
	c.sensor_type = PAS_SENSOR_TYPE_QUADRATURE;
	setup(&c);

	test_appconf.app_to_use = APP_PAS;
	app_pas_start(true);

	check(find_cmd("pas_status") != 0, "pas_status is registered");
	check(find_cmd("pas_torque") != 0, "pas_torque is registered");
	check(find_cmd("pas_preset") != 0, "pas_preset is registered");

	// pas_status is the command a rider runs on the bench before riding, so
	// what it says matters more than that it runs. It must report the control
	// type, whether the app is running, and the pin and sensor situation.
	pedal(60.0, 12);
	pas_compute_output(2.0);
	call_cmd("pas_status", 1, 0);
	check(out_has("PAS running"), "pas_status reports whether it is running");
	check(out_has("Control type"), "pas_status reports the control type");
	check(out_has("Pedal RPM"), "pas_status reports the pedal rpm");
	check(out_has("Start period"), "pas_status reports the derived periods");

	// pas_torque. With no torque source configured it has to say so rather
	// than print a plausible zero, which a rider on the bench would read as
	// "the sensor is connected and reading nothing".
	call_cmd("pas_torque", 1, 0);
	check(out_has("not set to ADC"), "pas_torque says when there is no ADC source");

	// A channel this hardware does not have, which is the next thing a
	// misconfigured bench setup hits.
	pas_config t = base_config();
	t.torque_source = PAS_TORQUE_SRC_ADC;
	t.torque_adc_ch = PAS_TORQUE_ADC_EXT8;
	setup(&t);
	app_pas_start(true);
	call_cmd("pas_torque", 1, 0);
	check(out_has("not available on this hardware"),
			"pas_torque names an impossible channel");

	// A real channel with a real voltage on it. This is the reading the bench
	// procedure depends on, so the numbers have to be there.
	t.torque_adc_ch = PAS_TORQUE_ADC_EXT3;
	t.torque_zero_v = 1.5;
	t.torque_nm_per_v = 70.0;
	t.torque_max_nm = 140.0;
	setup(&t);
	app_pas_start(true);
	set_adc_volts(ADC_IND_EXT3, 2.0);
	call_cmd("pas_torque", 1, 0);
	check(out_has("ADC channel"), "pas_torque reports which channel");
	check(out_has("Voltage"), "and the raw voltage");
	check(out_has("Nm"), "and the decoded torque");

	// Saturation has to be called out: a sensor pinned at the top of the ADC
	// range reads as maximum effort and would otherwise look like hard
	// pedalling rather than a wiring problem.
	set_adc_volts(ADC_IND_EXT3, 3.3);
	call_cmd("pas_torque", 1, 0);
	check(out_has("SATURATED"), "pas_torque reports a saturated reading");

	// The zero-point form averages a second of samples. The spread is the
	// useful part -- it is how a rider tells a noisy channel from a quiet one.
	set_adc_volts(ADC_IND_EXT3, 1.52);
	const char *argv_zero[] = {"pas_torque", "zero"};
	call_cmd("pas_torque", 2, argv_zero);
	check(out_has("Mean"), "pas_torque zero reports the mean");
	check(out_has("Spread"), "and the spread");
	check(out_has("1.52"), "and the mean is the voltage that was there");
	check(out_has("does not write"),
			"and says it has not written the configuration");

	set_adc_volts(ADC_IND_EXT3, 0.0);

	// Back to the configuration the rest of this test expects.
	setup(&c);
	app_pas_start(true);

	// The preset prints by default and writes only with "store", which is the
	// behaviour worth pinning: it is a configuration change to a vehicle, and
	// a rider exploring the command must not have applied one by typing it.
	memset(&test_appconf, 0, sizeof(test_appconf));
	const char *argv_preset[] = {"pas_preset", "pedelec"};
	call_cmd("pas_preset", 2, argv_preset);
	check(out_has("Pedelec preset"), "the preset prints what it would set");
	check(out_has("25 km/h"), "including the speed cutoff");
	check(out_has("250 W"), "and the power cap");
	check(out_has("Not written"), "and says it has not written anything");
	check_near(test_appconf.app_pas_conf.taper_end_kmh, 0.0, 0.01,
			"and really has not");

	// It also refuses to claim compliance, which is deliberate wording rather
	// than decoration.
	check(out_has("not a statement"), "the preset does not claim compliance");

	// With "store" it writes, and the well-known numbers are the point of it.
	const char *argv_store[] = {"pas_preset", "pedelec", "store"};
	call_cmd("pas_preset", 3, argv_store);
	check_near(test_appconf.app_pas_conf.taper_start_kmh, 25.0, 0.01,
			"stored: the taper starts at 25 km/h");
	check_near(test_appconf.app_pas_conf.taper_end_kmh, 25.0, 0.01,
			"stored: and ends there, so the cutoff is abrupt");
	check_near(test_appconf.app_pas_conf.power_max_w, 250.0, 0.01,
			"stored: the power cap is 250 W");
	check(test_appconf.app_pas_conf.pedal_stop_hard, "stored: pedal stop is hard");

	// An unknown preset must be refused rather than applying a partial one.
	// The four fields the preset writes, rather than memcmp over the struct:
	// pas_config has padding, whose bytes are unspecified, so a comparison of
	// the object representation can report a difference that is not one.
	// clang-tidy flagged that, and it was right to.
	const pas_config before = test_appconf.app_pas_conf;
	const char *argv_bad[] = {"pas_preset", "nonsense"};
	call_cmd("pas_preset", 2, argv_bad);
	check(before.taper_start_kmh == test_appconf.app_pas_conf.taper_start_kmh &&
			before.taper_end_kmh == test_appconf.app_pas_conf.taper_end_kmh &&
			before.power_max_w == test_appconf.app_pas_conf.power_max_w &&
			before.pedal_stop_hard == test_appconf.app_pas_conf.pedal_stop_hard,
			"an unknown preset changes nothing");
	check(out_has("Usage"), "and prints its usage");

	// No argument at all, which is how a rider first finds the command.
	call_cmd("pas_preset", 1, 0);
	check(out_has("Usage"), "pas_preset with no argument prints its usage");

	// pas_status under a fully configured setup, which is what reaches the
	// advisory branches. Half the command is conditional diagnostics, and they
	// are the whole reason to run it on a bench: they are what tells a rider
	// which of a dozen settings is the one stopping assist.
	pas_config f = base_config();
	f.ctrl_type = PAS_CTRL_TYPE_POWER;
	f.power_ctrl_mode = PAS_POWER_CLOSED_LOOP;
	f.torque_source = PAS_TORQUE_SRC_ADC;
	f.torque_adc_ch = PAS_TORQUE_ADC_EXT3;
	f.torque_zero_v = 1.5;
	f.torque_nm_per_v = 70.0;
	f.torque_max_nm = 140.0;
	f.torque_avg_pulses = 8;
	f.assist_gain = 2.0;
	f.power_max_w = 250.0;
	f.taper_start_kmh = 20.0;
	f.taper_end_kmh = 25.0;
	f.brake_source = PAS_BRAKE_SRC_ADC;
	f.brake_adc_ch = PAS_TORQUE_ADC_EXT3;
	f.brake_threshold_v = 2.5;
	setup(&f);
	app_pas_start(true);
	set_adc_volts(ADC_IND_EXT3, 1.8);
	pedal(60.0, 16);
	pas_compute_output(2.0);
	call_cmd("pas_status", 1, 0);
	check(out_has("Torque ADC ch"), "reports the torque channel");
	check(out_has("Torque voltage"), "and its voltage");
	check(out_has("ADC reference"), "and the ADC reference it is measured against");
	check(out_has("Power control"), "reports the power control mode");
	check(out_has("Measured power"), "and the measured power it trims against");
	check(out_has("Torque averaging"), "reports the averaging window");
	check(out_has("Speed limit"), "reports the speed limit");
	check(out_has("Power cap"), "and the power cap");
	check(out_has("Brake"), "and the brake input");

	// The advisory for a stop period long enough that assist lingers. This is
	// the one that matters most on a first setup: the stock default is long
	// enough to keep driving for over a second after the cranks stop.
	pas_config sp = base_config();
	sp.stop_timeout_s = 1.25;
	setup(&sp);
	app_pas_start(true);
	call_cmd("pas_status", 1, 0);
	check(out_has("stop period is long"),
			"a long stop period is called out");
	check(out_has("0.15 to 0.30"), "with the figure Grin suggest");

	// A torque control type with no torque source, which produces no assist at
	// all and would otherwise look like a wiring fault.
	pas_config ns = base_config();
	ns.ctrl_type = PAS_CTRL_TYPE_TORQUE;
	ns.torque_source = PAS_TORQUE_SRC_NONE;
	setup(&ns);
	app_pas_start(true);
	call_cmd("pas_status", 1, 0);
	check(out_has("no torque source"),
			"a torque control type with no source is called out");

	// A hardware torque source on a board that has none.
	pas_config hw = base_config();
	hw.ctrl_type = PAS_CTRL_TYPE_TORQUE;
	hw.torque_source = PAS_TORQUE_SRC_HW;
	setup(&hw);
	app_pas_start(true);
	pedal(60.0, 12);
	pas_compute_output(2.0);
	call_cmd("pas_status", 1, 0);
#ifndef TEST_TORQUE_SENSOR
	// Two printf calls, so the sentence spans a newline in the buffer -- a
	// substring across it never matches.
	check(out_has("board does not"),
			"asking for a hardware torque source it lacks is called out");
	check(out_has("Use the ADC source"), "with the fix for it");
#endif

	// A saturated torque reading, which reads as maximum effort and is the
	// failure a sensor above the ADC reference actually produces.
	pas_config sat = base_config();
	sat.ctrl_type = PAS_CTRL_TYPE_TORQUE;
	sat.torque_source = PAS_TORQUE_SRC_ADC;
	sat.torque_adc_ch = PAS_TORQUE_ADC_EXT3;
	sat.torque_zero_v = 1.5;
	sat.torque_nm_per_v = 70.0;
	sat.torque_max_nm = 140.0;
	setup(&sat);
	app_pas_start(true);
	set_adc_volts(ADC_IND_EXT3, 3.3);
	pedal(60.0, 12);
	pas_compute_output(2.0);
	call_cmd("pas_status", 1, 0);
	check(out_has("SATURATED"), "a saturated torque reading is called out");
	check(out_has("divider"), "with the fix for it");
	set_adc_volts(ADC_IND_EXT3, 0.0);

	// Stopping unregisters them, or a second start would register duplicates.
	app_pas_stop();
	check(find_cmd("pas_status") == 0, "stopping unregisters pas_status");
	sched_reset();

	test_appconf.app_to_use = APP_NONE;
}

int main(void) {
	memset(&test_appconf, 0, sizeof(test_appconf));
	memset(&test_mcconf, 0, sizeof(test_mcconf));
	test_mcconf.lo_current_max = 100.0;
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

#ifndef TEST_NO_PAS_PINS
	test_single_wire();
	test_single_wire_ignores_direction();
#endif
	test_torque_scaling();
	test_torque_deadband();
	test_torque_reference_ceiling();
	test_torque_saturation();
	test_torque_channel_validation();
	test_torque_source_none();
#ifdef TEST_TORQUE_SENSOR
	test_torque_source_hw();
#else
	test_torque_source_hw_absent();
#endif

#ifndef TEST_NO_PAS_PINS
	test_rider_power();
	test_power_zero_without_cadence();
	test_assist_gain();
	test_power_to_current();
	test_vin_guard();
	test_torque_averaging();
	test_averaging_reset_on_stop();
	test_torque_averaging_smooths();
	test_speed_taper();
	test_speed_cutoff();
	test_power_cap();
	test_pedal_stop_hard();
	test_brake();
	test_brake_inverted();
	test_brake_channel_validation();
	test_pedelec_combination();
	test_cadence_floor();
	test_cadence_floor_off();
	test_cadence_floor_not_applied_when_stopped();
	test_stop_threshold();
	test_thresholds_derived();
	test_start_threshold();
	test_torque_bipolar();
	test_assist_start_power();
	test_throttle_mixing();
	test_throttle_needs_pedalling();
	test_public_interface();
	test_negative_scaling_cannot_brake();
	test_walk_basic();
	test_walk_keepalive_expires();
	test_walk_speed_limit();
	test_walk_require_pedal();
	test_walk_brake_overrides();
	test_walk_replaces_and_releases();
	test_power_open_loop_default();
	test_power_closed_loop_trims_up();
	test_power_closed_loop_trims_down();
	test_power_closed_loop_settles();
	test_power_closed_loop_antiwindup();
	test_power_closed_loop_needs_primary();

	// These two go last on purpose. The terminal tests write the stored
	// configuration, and the thread test leaves the app's thread notionally
	// running because its exit path cannot be driven from here -- see
	// run_thread. Either would disturb a test that ran after them.
	test_flag_bits();
	test_walk_sources();
	test_terminal_commands();
	test_thread_loop();
#endif

	printf("\n%d checks, %d failures\n", checks, failures);
	return failures == 0 ? 0 : 1;
}
