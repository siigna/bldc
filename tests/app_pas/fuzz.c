// Property fuzzer for the PAS app.
//
// The unit tests in main.c drive the module through scenarios a rider would
// produce. This drives it through sequences no rider would: pedal pulses in
// illegal orders, time jumping backwards, a torque signal above the reference,
// a sagging or absurd pack voltage, a configuration reloaded mid-stroke. Then
// it asserts the things that must hold whatever the input was.
//
// The properties are the point. A crash means an invariant broke, not that the
// module was fed something silly -- it is polling real hardware, and real
// hardware produces silly things: a dirty sensor, a loose connector, a brownout
// that restarts one MCU and not the other.
//
//   make fuzz && ./fuzz -max_total_time=60
//
// A finding lands in ./crash-* and replays with ./fuzz <file>.

#include <stdint.h>
#include <stddef.h>
#include <string.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>

#include "ch.h"
#include "hal.h"
#include "datatypes.h"
#include "app.h"

// The same fixture the unit tests use.
extern uint8_t test_pad[2];
extern systime_t test_now;
extern float test_torque;
extern float test_v_in;
extern float test_speed;
extern float test_current_in;
extern float test_current_rel;
extern mc_fault_code test_fault;
extern volatile uint16_t ADC_Value[16];

float pas_compute_output(float dt_ms);
float pas_mix_throttle(float throttle_rel, float pas_rel);

#define FAIL(...) do { \
		fprintf(stderr, "INVARIANT: " __VA_ARGS__); \
		fprintf(stderr, "\n"); \
		abort(); \
	} while (0)

static bool finite_f(float v) {
	return !isnan(v) && !isinf(v);
}

// One byte of the input picks each field, so any byte string is a valid script
// and the fuzzer's mutations all land on something meaningful.
static float pick(uint8_t b, float lo, float hi) {
	return lo + (hi - lo) * ((float)b / 255.0f);
}

int LLVMFuzzerTestOneInput(const uint8_t *data, size_t size) {
	if (size < 16) {
		return 0;
	}

	pas_config c;
	memset(&c, 0, sizeof(c));

	// Configuration from the head of the input, over ranges wider than VESC
	// Tool allows: a config can also arrive over CAN or from a flash read that
	// half succeeded.
	size_t i = 0;
	c.ctrl_type = data[i++] % 4;
	c.sensor_type = data[i++] % 3;
	c.torque_source = data[i++] % 2;
	c.brake_source = data[i++] % 3;
	c.walk_source = data[i++] % 3;
	c.throttle_mode = data[i++] % 3;
	c.power_ctrl_mode = data[i++] % 2;
	c.magnets = data[i++];                       // includes 0, which is invalid
	c.current_scaling = pick(data[i++], -1.0f, 2.0f);
	c.pedal_rpm_start = pick(data[i++], 0.0f, 200.0f);
	c.pedal_rpm_end = pick(data[i++], 0.0f, 200.0f);   // may be below start
	c.assist_gain = pick(data[i++], -2.0f, 10.0f);
	c.power_max_w = pick(data[i++], -100.0f, 5000.0f);
	c.taper_start_kmh = pick(data[i++], 0.0f, 60.0f);
	c.taper_end_kmh = pick(data[i++], 0.0f, 60.0f);    // may be below start
	c.torque_zero_v = pick(data[i++], 0.0f, 4.0f);

	c.torque_nm_per_v = 70.0f;
	c.torque_deadband_nm = 2.0f;
	c.torque_avg_pulses = 8;
	c.ramp_time_pos = 0.6f;
	c.ramp_time_neg = 0.3f;
	c.update_rate_hz = 500;
	c.start_timeout_s = 0.3f;
	c.stop_timeout_s = 0.25f;
	c.power_gain = 0.5f;
	c.walk_max_kmh = 6.0f;
	c.walk_current = 0.2f;
	c.walk_threshold_v = 1.5f;

	test_now = 1000;
	test_pad[0] = 0;
	test_pad[1] = 0;
	test_torque = 0.0f;
	test_v_in = 50.0f;
	test_speed = 0.0f;
	test_current_in = 0.0f;
	test_current_rel = 0.0f;
	test_fault = FAULT_CODE_NONE;
	memset((void *)ADC_Value, 0, sizeof(ADC_Value));

	app_pas_configure(&c);

	// The rest of the input is a script: five bytes per step.
	float prev = 0.0f;
	for (; i + 4 < size; i += 5) {
		test_pad[0] = data[i] & 1;
		test_pad[1] = (data[i] >> 1) & 1;

		// Time advances by 0 to 255 ticks, so a step of zero and a long stall
		// are both reachable. A zero step is the interesting one: it makes
		// every period and rate a division by no elapsed time.
		test_now += data[i + 1];

		test_torque = pick(data[i + 2], -1.0f, 2.0f);
		ADC_Value[3] = (uint16_t)(data[i + 2] * 16);
		ADC_Value[4] = (uint16_t)(data[i + 3] * 16);
		ADC_Value[6] = (uint16_t)(data[i + 3] * 16);

		// A pack that sags to nothing, and one reading impossibly high. The
		// power law divides by this.
		test_v_in = pick(data[i + 4], 0.0f, 120.0f);
		test_speed = pick(data[i + 4], -20.0f, 40.0f);

		float dt_ms = pick(data[i + 3], 0.0f, 100.0f);
		float out = pas_compute_output(dt_ms);

		// --- The properties ------------------------------------------------

		if (!finite_f(out)) {
			FAIL("output %f is not finite (dt %f, v_in %f)", out, dt_ms, test_v_in);
		}
		// The output is a current fraction handed to mc_interface_set_current_rel.
		// Outside 0..1 it is either no assist at all or a demand for more than
		// the configured maximum.
		if (out < 0.0f || out > 1.0f) {
			FAIL("output %f outside 0..1", out);
		}

		float rpm = app_pas_get_pedal_rpm();
		if (!finite_f(rpm)) {
			FAIL("pedal rpm is not finite");
		}
		// A crank that reads as spinning faster than any human could pedal
		// means the decoder has divided by a bad period.
		if (rpm < 0.0f || rpm > 1000.0f) {
			FAIL("pedal rpm %f out of range", rpm);
		}

		float nm = app_pas_get_torque_nm();
		float rider = app_pas_get_rider_power();
		float target = app_pas_get_motor_power_target();
		float taper = app_pas_get_speed_taper();
		if (!finite_f(nm) || !finite_f(rider) || !finite_f(target)) {
			FAIL("torque %f rider %f target %f: not finite", nm, rider, target);
		}
		// The taper is a multiplier applied to the assist, so outside 0..1 it
		// either kills assist that should be there or amplifies it past the
		// configured maximum.
		if (!finite_f(taper) || taper < 0.0f || taper > 1.0f) {
			FAIL("speed taper %f outside 0..1", taper);
		}

		// Mixing is meant to be closed over the same range as its inputs.
		float mixed = pas_mix_throttle(pick(data[i], -1.0f, 1.0f), out);
		if (!finite_f(mixed) || mixed < -1.0f || mixed > 1.0f) {
			FAIL("mixed throttle %f outside -1..1", mixed);
		}

		prev = out;
	}
	(void)prev;

	return 0;
}
