/*
	Copyright 2016 Benjamin Vedder	benjamin@vedder.se
	Copyright 2020 Marcos Chaparro	mchaparro@powerdesigns.ca

	This file is part of the VESC firmware.

	The VESC firmware is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    The VESC firmware is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#pragma GCC optimize ("Os")

#include "app.h"

#include "ch.h"
#include "hal.h"
#include "stm32f4xx_conf.h"
#include "mc_interface.h"
#include "timeout.h"
#include "utils_math.h"
#include "utils_sys.h"
#include "terminal.h"
#include "commands.h"
#include "hw.h"
#include <math.h>

// Settings
#define MAX_MS_WITHOUT_CADENCE_OR_TORQUE	5000
#define MAX_MS_WITHOUT_CADENCE			1000
#define MIN_MS_WITHOUT_POWER			500
#define FILTER_SAMPLES					5

// Threads
static THD_FUNCTION(pas_thread, arg);
__attribute__((section(".ram4"))) static THD_WORKING_AREA(pas_thread_wa, 512);

// Private variables
static volatile pas_config config;
static volatile float sub_scaling = 1.0;
static volatile float output_current_rel = 0.0;
static volatile float ms_without_power = 0.0;
static volatile float max_pulse_period = 0.0;
static volatile float min_pedal_period = 0.0;
static volatile float direction_conf = 0.0;
static volatile float pedal_rpm = 0;
static volatile bool primary_output = false;
static volatile bool stop_now = true;
static volatile bool is_running = false;
static volatile float torque_ratio = 0.0;
static volatile bool sensor_type_unsupported = false;
static volatile bool pins_unavailable = false;

// Pedal decoder state. Kept at file scope so that it can be reset when the
// app is reconfigured or restarted, which function-local statics cannot be.
static uint8_t dec_old_state = 0;
static systime_t dec_last_pulse_time = 0;
// Whether dec_last_pulse_time refers to a real pulse yet. Without this the age
// of the first pulse would be measured from tick zero, which only looks correct
// because uptime is normally already larger than max_pulse_period.
static bool dec_have_reference = false;
static float dec_period_filtered = 0.0;
static int32_t dec_correct_direction_counter = 0;

// Output state. Also kept at file scope so that a stop/start or a settings write
// does not resume the ramp from where the previous configuration left off.
static systime_t out_last_time = 0;
static float out_ramp = 0.0;
static int out_pulses_without_power_before = 0;
#ifdef HW_HAS_PAS_TORQUE_SENSOR
static float out_ms_without_cadence_or_torque = 0.0;
static float out_ms_without_cadence = 0.0;
#else
static volatile bool torque_unsupported = false;
#endif

/**
 * Configure and initialize PAS application
 *
 * @param conf
 * App config
 */
void app_pas_configure(pas_config *conf) {
	config = *conf;
	ms_without_power = 0.0;
	output_current_rel = 0.0;
	sensor_type_unsupported = false;

	// Reset the pedal decoder so that stale timing from before the change cannot
	// produce a bogus cadence on the next pulse.
	dec_old_state = 0;
	dec_last_pulse_time = 0;
	dec_have_reference = false;
	dec_period_filtered = 0.0;
	dec_correct_direction_counter = 0;
	pedal_rpm = 0.0;

	out_last_time = chVTGetSystemTimeX();
	out_ramp = 0.0;
	out_pulses_without_power_before = 0;
#ifdef HW_HAS_PAS_TORQUE_SENSOR
	out_ms_without_cadence_or_torque = 0.0;
	out_ms_without_cadence = 0.0;
#endif

	// a period longer than this should immediately reduce power to zero
	max_pulse_period = 1.0 / ((config.pedal_rpm_start / 60.0) * config.magnets) * 1.2;

	// if pedal spins at x3 the end rpm, assume its beyond limits
	min_pedal_period = 1.0 / ((config.pedal_rpm_end * 3.0 / 60.0));

	(config.invert_pedal_direction) ? (direction_conf = -1.0) : (direction_conf = 1.0);
}

/**
 * True when the pedal sensor pins cannot be claimed.
 *
 * On hardware without dedicated PAS pins the fallback is the COMM UART pads.
 * Reconfiguring those as inputs takes down UART communication, including the
 * link to a VESC Express module, so it is only done when UART comms is off.
 */
static bool pas_pins_available(void) {
#ifndef HW_PAS1_PORT
	return false;
#elif defined(HW_PAS_PINS_SHARED_WITH_UART)
	const app_configuration *ac = app_get_configuration();

	if (ac->permanent_uart_enabled) {
		return false;
	}

	switch (ac->app_to_use) {
		case APP_UART:
		case APP_PPM_UART:
		case APP_ADC_UART:
			return false;
		default:
			return true;
	}
#else
	return true;
#endif
}

static void terminal_pas_status(int argc, const char **argv) {
	(void)argc; (void)argv;

	commands_printf("PAS running       : %s", is_running ? "true" : "false");
	commands_printf("Primary output    : %s", primary_output ? "true" : "false");
	commands_printf("Sensor type       : %d", (int)config.sensor_type);
	commands_printf("Control type      : %d", (int)config.ctrl_type);
	commands_printf("Pedal RPM         : %.1f", (double)pedal_rpm);
	commands_printf("Output (rel)      : %.3f", (double)out_ramp);
	commands_printf("Filter            : %s", config.use_filter ? "on" : "off");
	commands_printf("Pedal period      : %.4f s", (double)dec_period_filtered);
	commands_printf("Max pulse period  : %.4f s", (double)max_pulse_period);
	commands_printf("Min pedal period  : %.4f s", (double)min_pedal_period);

#ifdef HW_PAS1_PORT
#ifdef HW_PAS_PINS_SHARED_WITH_UART
	commands_printf("PAS pins          : COMM UART pads (no dedicated pins on this hardware)");
#else
	commands_printf("PAS pins          : dedicated");
#endif
#else
	commands_printf("PAS pins          : NONE (no cadence input is possible)");
#endif
	if (pins_unavailable) {
		commands_printf("  NOT CLAIMED: the pads are in use for UART comms, so there is no");
		commands_printf("  cadence input. Disable UART comms or use hardware with dedicated pins.");
	}

#ifdef HW_HAS_PAS_TORQUE_SENSOR
	commands_printf("Torque sensor     : present, ratio %.3f", (double)torque_ratio);
#else
	commands_printf("Torque sensor     : NOT SUPPORTED on this hardware");
	if (torque_unsupported) {
		commands_printf("  A torque control type is selected but cannot produce output.");
	}
#endif

	if (sensor_type_unsupported) {
		commands_printf("WARNING: sensor type %d is not supported, no cadence is decoded.",
				(int)config.sensor_type);
	}

	commands_printf(" ");
}

/**
 * Start PAS thread
 *
 * @param is_primary_output
 * True when PAS app takes direct control of the current target,
 * false when PAS app shares control with the ADC app for current command
 */
void app_pas_start(bool is_primary_output) {
	stop_now = false;
	primary_output = is_primary_output;

	// Assigned rather than only set, so that restarting into a working
	// configuration clears a previous failure.
	pins_unavailable = !pas_pins_available();

#ifdef HW_PAS1_PORT
	if (!pins_unavailable) {
		palSetPadMode(HW_PAS1_PORT, HW_PAS1_PIN, PAL_MODE_INPUT_PULLUP);
		palSetPadMode(HW_PAS2_PORT, HW_PAS2_PIN, PAL_MODE_INPUT_PULLUP);
	}
#endif

	chThdCreateStatic(pas_thread_wa, sizeof(pas_thread_wa), NORMALPRIO, pas_thread, NULL);

	terminal_register_command_callback(
			"pas_status",
			"Print the state of the PAS app and what the hardware supports",
			0,
			terminal_pas_status);
}

bool app_pas_is_running(void) {
	return is_running;
}

void app_pas_stop(void) {
	stop_now = true;
	while (is_running) {
		chThdSleepMilliseconds(1);
	}

	terminal_unregister_callback(terminal_pas_status);

	if (primary_output == true) {
		mc_interface_set_current_rel(0.0);
	}
	else {
		output_current_rel = 0.0;
	}
}

void app_pas_set_current_sub_scaling(float current_sub_scaling) {
	sub_scaling = current_sub_scaling;
}

float app_pas_get_current_target_rel(void) {
	return output_current_rel;
}

float app_pas_get_pedal_rpm(void) {
	return pedal_rpm;
}

/**
 * Decode a quadrature (two-wire) pedal sensor.
 *
 * Called at the app update rate. Updates pedal_rpm, or sets it to zero when the
 * cranks have been still for longer than max_pulse_period.
 */
static void pas_decode_quadrature(void) {
#ifdef HW_PAS1_PORT
	const int8_t QEM[] = {0,-1,1,2,1,0,2,-1,-1,2,0,1,2,1,-1,0}; // Quadrature Encoder Matrix

	uint8_t PAS1_level = palReadPad(HW_PAS1_PORT, HW_PAS1_PIN);
	uint8_t PAS2_level = palReadPad(HW_PAS2_PORT, HW_PAS2_PIN);

	uint8_t new_state = PAS2_level * 2 + PAS1_level;
	int8_t direction_qem = QEM[dec_old_state * 4 + new_state];
	dec_old_state = new_state;

	// Require several quadrature events in the right direction to prevent vibrations from
	// engaging PAS. Note that QEM yields 2 for an illegal double transition, which matches
	// neither case and therefore neither advances nor resets the counter.
	int8_t direction = (int8_t)direction_conf * direction_qem;

	switch(direction) {
		case 1: dec_correct_direction_counter++; break;
		case -1: dec_correct_direction_counter = 0; break;
	}

	// sensors are poorly placed, so use only one rising edge as reference
	if ((new_state == 3) && (dec_correct_direction_counter >= 4)) {
		// Time since the previous accepted pulse, taken as a difference rather than
		// as a difference of absolute float seconds, which loses resolution as
		// uptime grows.
		float pulse_age = UTILS_AGE_S(dec_last_pulse_time);
		dec_last_pulse_time = chVTGetSystemTimeX();
		dec_correct_direction_counter = 0;

		// The first pulse after a start, and the first after an idle period, has no
		// meaningful period, so use it only as the reference for the next one. The
		// idle test is against max_pulse_period, which is a single magnet period, so
		// it uses the age rather than the full revolution period computed below.
		if (!dec_have_reference || pulse_age > max_pulse_period) {
			dec_have_reference = true;
			dec_period_filtered = 0.0;
			return;
		}

		// One crank revolution.
		float period = pulse_age * (float)config.magnets;

		if (period < min_pedal_period) { // can't be that short, abort
			return;
		}

		if (config.use_filter && dec_period_filtered > 0.0) {
			UTILS_LP_MOVING_AVG_APPROX(dec_period_filtered, period, FILTER_SAMPLES);
		} else {
			// Unfiltered, or seeding the filter with the first valid period.
			dec_period_filtered = period;
		}

		// The direction gate above has already established that the cranks are turning
		// forwards, so the result is positive. Scaling by direction_qem here would
		// double the reported cadence whenever QEM returned 2.
		pedal_rpm = 60.0 / dec_period_filtered;
	} else {
		// If no pedal activity, set RPM as zero
		if (UTILS_AGE_S(dec_last_pulse_time) > max_pulse_period) {
			pedal_rpm = 0.0;
			dec_period_filtered = 0.0;
		}
	}
#endif
}

/**
 * Read the pedal sensor selected by config.sensor_type.
 */
void pas_event_handler(void) {
	if (pins_unavailable) {
		pedal_rpm = 0.0;
		return;
	}

	switch (config.sensor_type) {
		case PAS_SENSOR_TYPE_QUADRATURE:
			pas_decode_quadrature();
			break;

		default:
			// An unsupported sensor type must not silently behave like a supported
			// one, so produce no cadence and let the thread report it.
			pedal_rpm = 0.0;
			sensor_type_unsupported = true;
			break;
	}
}

static THD_FUNCTION(pas_thread, arg) {
	(void)arg;

	chRegSetThreadName("APP_PAS");

	is_running = true;

	for(;;) {
		// Sleep for a time according to the specified rate
		systime_t sleep_time = CH_CFG_ST_FREQUENCY / config.update_rate_hz;

		// At least one tick should be slept to not block the other threads
		if (sleep_time == 0) {
			sleep_time = 1;
		}
		chThdSleep(sleep_time);

		if (stop_now) {
			is_running = false;
			return;
		}

		pas_event_handler();	// this could happen inside an ISR instead of being polled

		// Declared per iteration: a control type that computes no output, or an
		// iteration that bails out early, must not leave a previous value latched
		// to be ramped and applied.
		float output = 0.0;

		// For safe start when fault codes occur
		if (mc_interface_get_fault() != FAULT_CODE_NONE) {
			ms_without_power = 0;
		}

		if (app_is_output_disabled()) {
			// The exported value is consumed by app_adc in APP_ADC_PAS mode, so it
			// has to be cleared here. Leaving it at its last value would let the
			// throttle app keep applying PAS output while output is disabled.
			output_current_rel = 0.0;
			out_ramp = 0.0;
			continue;
		}

		switch (config.ctrl_type) {
			case PAS_CTRL_TYPE_NONE:
				output = 0.0;
				break;
			case PAS_CTRL_TYPE_CADENCE:
				// Map pedal rpm to assist level

				// NOTE: If the limits are the same a numerical instability is approached, so in that case
				// just use on/off control (which is what setting the limits to the same value essentially means).
				if (config.pedal_rpm_end > (config.pedal_rpm_start + 1.0)) {
					output = utils_map(pedal_rpm, config.pedal_rpm_start, config.pedal_rpm_end, 0.0, config.current_scaling * sub_scaling);
					utils_truncate_number(&output, 0.0, config.current_scaling * sub_scaling);
				} else {
					if (pedal_rpm > config.pedal_rpm_end) {
						output = config.current_scaling * sub_scaling;
					} else {
						output = 0.0;
					}
				}
				break;

#ifdef HW_HAS_PAS_TORQUE_SENSOR
			case PAS_CTRL_TYPE_TORQUE:
			case PAS_CTRL_TYPE_TORQUE_WITH_CADENCE_TIMEOUT:
			{
				// Both torque types sample the sensor. Previously only the first case
				// did, and the second relied on falling into it, so selecting the
				// second one directly never computed an output at all.
				torque_ratio = hw_get_PAS_torque();
				output = torque_ratio * config.current_scaling * sub_scaling;
				utils_truncate_number(&output, 0.0, config.current_scaling * sub_scaling);

				// The cadence checks below are applied to both types. They were added
				// as a safety fix and reached the plain torque type through the
				// fall-through, so they are kept there rather than dropped.

				// disable assistance if torque has been sensed for >5sec without any pedal movement. Prevents
				// motor overtemps when the rider is just resting on the pedals
				if(output == 0.0 || pedal_rpm > 0) {
					out_ms_without_cadence_or_torque = 0.0;
				} else {
					out_ms_without_cadence_or_torque += (1000.0 * (float)sleep_time) / (float)CH_CFG_ST_FREQUENCY;
					if(out_ms_without_cadence_or_torque > MAX_MS_WITHOUT_CADENCE_OR_TORQUE) {
						output = 0.0;
					}
				}
				// if cranks are not moving, there should not be any output. This covers the case of a torque sensor
				// stuck with a non-zero signal.
				if(pedal_rpm < 0.01) {
					out_ms_without_cadence += (1000.0 * (float)sleep_time) / (float)CH_CFG_ST_FREQUENCY;
					if(out_ms_without_cadence > MAX_MS_WITHOUT_CADENCE) {
						output = 0.0;
					}
				} else {
					out_ms_without_cadence = 0.0;
				}
			}
			break;
#else
			case PAS_CTRL_TYPE_TORQUE:
			case PAS_CTRL_TYPE_TORQUE_WITH_CADENCE_TIMEOUT:
				// This hardware has no torque sensor. Without this case the torque
				// control types fell through to default and produced nothing at all,
				// which is indistinguishable from the app being inactive. Record it
				// so that "pas_status" can report it.
				output = 0.0;
				torque_unsupported = true;
				break;
#endif
			default:
				break;
		}

		// Apply ramping
		float ramp_time = fabsf(output) > fabsf(out_ramp) ? config.ramp_time_pos : config.ramp_time_neg;

		// The elapsed time is taken every iteration. Taking it only inside the
		// branch below let it grow without bound whenever the branch was skipped,
		// so the first enabled iteration took one unbounded step.
		const float dt = (float)ST2MS(chVTTimeElapsedSinceX(out_last_time)) / 1000.0;
		out_last_time = chVTGetSystemTimeX();

		if (ramp_time > 0.01) {
			const float ramp_step = dt / ramp_time;
			utils_step_towards(&out_ramp, output, ramp_step);
			utils_truncate_number(&out_ramp, 0.0, config.current_scaling * sub_scaling);

			output = out_ramp;
		}

		if (output < 0.001) {
			ms_without_power += (1000.0 * (float)sleep_time) / (float)CH_CFG_ST_FREQUENCY;
		}

		// Safe start is enabled if the output has not been zero for long enough
		if (ms_without_power < MIN_MS_WITHOUT_POWER) {
			if (ms_without_power == out_pulses_without_power_before) {
				ms_without_power = 0;
			}
			out_pulses_without_power_before = ms_without_power;
			output_current_rel = 0.0;
			continue;
		}

		// Reset timeout
		timeout_reset();

		if (primary_output == true) {
			mc_interface_set_current_rel(output);
		}
		else {
			output_current_rel = output;
		}
	}
}
