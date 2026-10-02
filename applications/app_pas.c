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
#include "conf_general.h"
#include "mempools.h"
#include "hw.h"
#include <math.h>
#include <string.h>

// Settings
#define MAX_MS_WITHOUT_CADENCE_OR_TORQUE	5000
#define MAX_MS_WITHOUT_CADENCE			1000
#define MIN_MS_WITHOUT_POWER			500
#define FILTER_SAMPLES					5

// One entry per pedal pulse, so that the averaging window can cover a whole
// crank revolution at the largest magnet count the configuration allows.
#define PAS_TORQUE_RING_MAX				128

// A measured input voltage that reads low inflates the current a power request
// converts to, so it is floored here. Reading high is harmless, since it can
// only ask for less current than intended.
#define PAS_MIN_VIN						8.0

// A walk request from a script is a keepalive, not a latch: it expires unless
// it is refreshed. Otherwise a display that stopped talking, or a script that
// died while the button was held, would leave the motor pushing.
#define WALK_LISP_TIMEOUT_S			0.5

// Walk assist fades out over this much speed below the limit, so that it does
// not switch on and off at the threshold.
#define WALK_TAPER_KMH				1.0

// Threads
static THD_FUNCTION(pas_thread, arg);
__attribute__((section(".ram4"))) static THD_WORKING_AREA(pas_thread_wa, 512);

// Private variables
static volatile pas_config config;
static volatile float sub_scaling = 1.0;
static volatile float output_current_rel = 0.0;
static volatile float ms_without_power = 0.0;
// The longest gap between pulses that still counts as continued pedalling, and
// the gap after which the cranks are considered stopped. These are tuned in
// opposite directions, which is why one value cannot serve both: a longer start
// makes pulling away from a standstill engage sooner, while a shorter stop makes
// assist end sooner once pedalling ceases.
static volatile float start_period = 0.0;
static volatile float stop_period = 0.0;
static volatile float min_pedal_period = 0.0;
static volatile float direction_conf = 0.0;
static volatile float pedal_rpm = 0;
static volatile bool primary_output = false;
static volatile bool stop_now = true;
static volatile bool is_running = false;
static volatile float torque_ratio = 0.0;
static volatile bool sensor_type_unsupported = false;
static volatile bool pins_unavailable = false;
static volatile float torque_nm = 0.0;
static volatile float torque_volts = 0.0;
static volatile bool torque_saturated = false;
static volatile bool torque_ch_invalid = false;
static volatile bool torque_src_unsupported = false;
static float torque_filter_v = 0.0;
// Reported rider power, from the actual cadence. Kept separate from the value
// the assist is computed from, which uses a floored cadence, so that telemetry
// does not overstate what the rider is contributing.
static volatile float rider_power_w = 0.0;
static volatile float assist_basis_w = 0.0;
static volatile float motor_power_w = 0.0;
static volatile float measured_power_w = 0.0;
static float power_integ = 0.0;
static volatile bool brake_engaged = false;
static volatile bool brake_ch_invalid = false;
static volatile float speed_taper = 1.0;
static volatile bool walk_active = false;
static volatile bool walk_ch_invalid = false;
static volatile bool walk_lisp_request = false;
static volatile systime_t walk_lisp_time = 0;
static bool walk_was_active = false;

// Revolution synchronous torque averaging. A bottom bracket torque signal
// varies strongly within a pedal stroke, so the mean over one pulse interval is
// stored per pulse and the window averages whole intervals rather than applying
// a blind time constant. Setting the window to the magnet count makes it
// exactly one crank revolution.
static float torque_ring[PAS_TORQUE_RING_MAX];
static uint8_t torque_ring_pos = 0;
static uint8_t torque_ring_fill = 0;
static float torque_pulse_sum = 0.0;
static uint32_t torque_pulse_samples = 0;
static float torque_avg_nm = 0.0;
static uint32_t torque_pulse_seen = 0;

// Pedal decoder state. Kept at file scope so that it can be reset when the
// app is reconfigured or restarted, which function-local statics cannot be.
static uint8_t dec_old_state = 0;
static systime_t dec_last_pulse_time = 0;
// Whether dec_last_pulse_time refers to a real pulse yet. Without this the age
// of the first pulse would be measured from tick zero, which only looks correct
// because uptime is normally already larger than the start period.
static bool dec_have_reference = false;
static float dec_period_filtered = 0.0;
// Incremented on every accepted pedal pulse, so that the output side can tell
// when a pulse interval has closed without duplicating the edge detection.
static volatile uint32_t dec_pulse_count = 0;
static int32_t dec_correct_direction_counter = 0;

// Output state. Also kept at file scope so that a stop/start or a settings write
// does not resume the ramp from where the previous configuration left off.
static systime_t out_last_time = 0;
static float out_ramp = 0.0;
static int out_pulses_without_power_before = 0;
static float out_ms_without_cadence_or_torque = 0.0;
static float out_ms_without_cadence = 0.0;

/**
 * Configure and initialize PAS application
 *
 * @param conf
 * App config
 */
// cppcheck-suppress constParameterPointer ; matches app.h and every other
// app_*_configure in the tree, which cannot take const without changing all of
// them and the app_configuration they are called with.
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
	out_ms_without_cadence_or_torque = 0.0;
	out_ms_without_cadence = 0.0;

	torque_nm = 0.0;
	torque_volts = 0.0;
	torque_ratio = 0.0;
	torque_filter_v = 0.0;
	torque_saturated = false;
	torque_ch_invalid = false;
	torque_src_unsupported = false;

	rider_power_w = 0.0;
	assist_basis_w = 0.0;
	motor_power_w = 0.0;
	measured_power_w = 0.0;
	power_integ = 0.0;
	brake_engaged = false;
	brake_ch_invalid = false;
	speed_taper = 1.0;
	walk_active = false;
	walk_ch_invalid = false;
	walk_was_active = false;
	torque_ring_pos = 0;
	torque_ring_fill = 0;
	torque_pulse_sum = 0.0;
	torque_pulse_samples = 0;
	torque_avg_nm = 0.0;
	torque_pulse_seen = dec_pulse_count;

	// Zero means derive from the start cadence and magnet count, which is what
	// this app did when it had a single period for both roles. Note that the
	// derivation scales with the magnet count, so a low pole count sensor gets a
	// long cutoff: eight magnets at a 10 rpm start gives 0.9 s, well beyond the
	// 0.15 to 0.30 s that Grin recommend for the stop threshold on a Cycle
	// Analyst. Setting the two explicitly is preferable.
	const float derived = 1.0 / ((config.pedal_rpm_start / 60.0) * config.magnets) * 1.2;

	start_period = config.start_timeout_s > 0.001 ? config.start_timeout_s : derived;
	stop_period = config.stop_timeout_s > 0.001 ? config.stop_timeout_s : derived;

	// if pedal spins at x3 the end rpm, assume its beyond limits
	min_pedal_period = 1.0 / ((config.pedal_rpm_end * 3.0 / 60.0));

	(config.invert_pedal_direction) ? (direction_conf = -1.0) : (direction_conf = 1.0);
}

/**
 * Resolve the configured torque sensor ADC channel, or -1 when this hardware
 * does not have it.
 *
 * hw.h aliases ADC_IND_EXT2 through ADC_IND_EXT8 to ADC_IND_EXT when a board
 * does not define them, so testing with #ifdef is always true and would
 * silently read the throttle input. The values are compared instead.
 */
static int pas_adc_index(pas_adc_ch sel) {
	switch (sel) {
		case PAS_TORQUE_ADC_EXT1:
			return ADC_IND_EXT;
#if ADC_IND_EXT2 != ADC_IND_EXT
		case PAS_TORQUE_ADC_EXT2:
			return ADC_IND_EXT2;
#endif
#if ADC_IND_EXT3 != ADC_IND_EXT
		case PAS_TORQUE_ADC_EXT3:
			return ADC_IND_EXT3;
#endif
#if ADC_IND_EXT4 != ADC_IND_EXT
		case PAS_TORQUE_ADC_EXT4:
			return ADC_IND_EXT4;
#endif
#if ADC_IND_EXT5 != ADC_IND_EXT
		case PAS_TORQUE_ADC_EXT5:
			return ADC_IND_EXT5;
#endif
#if ADC_IND_EXT6 != ADC_IND_EXT
		case PAS_TORQUE_ADC_EXT6:
			return ADC_IND_EXT6;
#endif
#if ADC_IND_EXT7 != ADC_IND_EXT
		case PAS_TORQUE_ADC_EXT7:
			return ADC_IND_EXT7;
#endif
#if ADC_IND_EXT8 != ADC_IND_EXT
		case PAS_TORQUE_ADC_EXT8:
			return ADC_IND_EXT8;
#endif
		default:
			return -1;
	}
}

/**
 * Read the crank torque in Nm.
 *
 * Torque is carried as a unit rather than as a ratio of full scale, because
 * that is what makes rider power expressible. Board implementations behind
 * HW_HAS_PAS_TORQUE_SENSOR report a ratio, so the configured full scale is used
 * to express those in Nm as well.
 *
 * Returns zero, and records why, when no torque reading is available.
 */
float pas_read_torque_nm(void) {
	switch (config.torque_source) {
		case PAS_TORQUE_SRC_HW:
#ifdef HW_HAS_PAS_TORQUE_SENSOR
			torque_src_unsupported = false;
			return hw_get_PAS_torque() * config.torque_max_nm;
#else
			torque_src_unsupported = true;
			return 0.0;
#endif

		case PAS_TORQUE_SRC_ADC: {
			int adc_ch = pas_adc_index(config.torque_adc_ch);

			if (adc_ch < 0) {
				torque_ch_invalid = true;
				return 0.0;
			}
			torque_ch_invalid = false;

			float volts = ADC_VOLTS(adc_ch);
			torque_volts = volts;

			// A sensor whose output runs above the ADC reference clips, and the
			// assist then flattens out at whatever the clip corresponds to
			// rather than at full effort. Surface that instead of leaving it to
			// be discovered on a hill.
			torque_saturated = volts >= (V_REG * 0.99);

			UTILS_LP_MOVING_AVG_APPROX(torque_filter_v, volts, FILTER_SAMPLES);

			float nm = (torque_filter_v - config.torque_zero_v) * config.torque_nm_per_v;

			if (config.torque_bipolar) {
				// A sensor that rests mid-rail and swings both ways, measuring the
				// left and right pedal separately. A Thun sits at 2.5 V and covers
				// -200 to +200 Nm over 0.5 to 4.5 V. Both directions are pedal
				// effort, so the magnitude is the torque.
				nm = fabsf(nm);
			} else if (nm < 0.0) {
				// A unipolar sensor rests at the bottom of its range, so below the
				// zero point is noise or backwards force, not a brake request.
				nm = 0.0;
			}

			utils_deadband(&nm, config.torque_deadband_nm, config.torque_max_nm);
			utils_truncate_number(&nm, 0.0, config.torque_max_nm);
			return nm;
		}

		default:
			return 0.0;
	}
}

/**
 * Feed a torque sample into the revolution synchronous average.
 *
 * Samples are accumulated between pedal pulses, and the mean of each pulse
 * interval is stored when that interval closes. Averaging whole intervals means
 * the window is tied to crank position rather than to time, so it does not
 * change character with cadence the way a fixed time constant does.
 */
static void pas_torque_track(float nm) {
	// With the cranks stopped the window would hold torque from before the stop
	// and pair it with a fresh cadence on resume, so it is discarded and refills
	// from the new pedalling.
	if (pedal_rpm < 0.01) {
		torque_ring_pos = 0;
		torque_ring_fill = 0;
		torque_pulse_sum = 0.0;
		torque_pulse_samples = 0;
		torque_avg_nm = 0.0;
		torque_pulse_seen = dec_pulse_count;

		// The low pass on the sensor voltage is stale for the same reason, and
		// letting it decay across the stop would assist the first pulses of the
		// next start from the previous load. Starting it from zero instead only
		// ever under-reads while it fills.
		torque_filter_v = 0.0;
		return;
	}

	torque_pulse_sum += nm;
	torque_pulse_samples++;

	if (dec_pulse_count == torque_pulse_seen) {
		return;
	}
	torque_pulse_seen = dec_pulse_count;

	if (torque_pulse_samples > 0) {
		torque_ring[torque_ring_pos] = torque_pulse_sum / (float)torque_pulse_samples;
		torque_ring_pos = (torque_ring_pos + 1) % PAS_TORQUE_RING_MAX;
		if (torque_ring_fill < PAS_TORQUE_RING_MAX) {
			torque_ring_fill++;
		}
	}
	torque_pulse_sum = 0.0;
	torque_pulse_samples = 0;

	// Recomputed only when an interval closes rather than every iteration.
	int window = config.torque_avg_pulses;
	if (window > PAS_TORQUE_RING_MAX) {
		window = PAS_TORQUE_RING_MAX;
	}
	if (window > torque_ring_fill) {
		window = torque_ring_fill;
	}

	if (window <= 0) {
		torque_avg_nm = 0.0;
		return;
	}

	float sum = 0.0;
	for (int i = 0; i < window; i++) {
		int idx = (int)torque_ring_pos - 1 - i;
		while (idx < 0) {
			idx += PAS_TORQUE_RING_MAX;
		}
		sum += torque_ring[idx];
	}
	torque_avg_nm = sum / (float)window;
}

/**
 * The torque the assist law should act on, which is the averaged value when a
 * window is configured and the instantaneous one otherwise.
 */
static float pas_torque_for_control(float instant_nm) {
	if (config.torque_avg_pulses > 0 && torque_ring_fill > 0) {
		return torque_avg_nm;
	}
	return instant_nm;
}

/**
 * Power in watts from a crank torque and a cadence in rpm.
 */
static float pas_power_w(float nm, float rpm) {
	return nm * (rpm * (2.0 * M_PI / 60.0));
}

/**
 * The cadence the assist calculation should use.
 *
 * Torque times cadence collapses towards zero as the cranks slow, so pulling
 * away from a standstill would get the least assist just where the most is
 * wanted. A floor under the cadence used for the calculation fixes that, and a
 * Cycle Analyst uses 55 rpm for the same reason.
 *
 * The floor only applies while the cranks are actually turning, so the property
 * that stopped cranks mean no assist still holds and still needs no separate
 * interlock for a torque sensor stuck at a non-zero reading.
 */
static float pas_assist_cadence(void) {
	if (pedal_rpm < 0.01) {
		return 0.0;
	}

	if (pedal_rpm < config.cadence_floor_rpm) {
		return config.cadence_floor_rpm;
	}
	return pedal_rpm;
}

/**
 * Convert a motor power request in watts to a relative current, and optionally
 * trim it against the power actually being drawn.
 *
 * Open loop is the request divided by the input voltage, bounded by the current
 * limits. It cannot overshoot or oscillate, but the delivered power sits below
 * the request wherever the motor cannot take the current, and nothing corrects
 * for that.
 *
 * Closed loop keeps that as a feedforward term and adds an integral trim on the
 * error against measured input power. A Cycle Analyst regulates this with a
 * power PID, and its tuning notes warn about lag from too low a gain and surge
 * from too high a one. Keeping the feedforward means the gain only has to
 * correct the residual rather than produce the whole output, so the lag case
 * does not arise and the gain can be small.
 *
 * power_gain is how much of a full scale error the trim closes per second, so
 * 2.0 closes a full scale error in half a second.
 *
 * The loop is only closed when PAS is the primary output. In the combined ADC
 * and PAS mode the measured input power includes whatever the throttle is
 * asking for, which is not attributable to this request, so trimming against it
 * would fight the throttle.
 */
static float pas_power_to_current_rel(float watts, float ceiling, float dt_ms) {
	float v_in = mc_interface_get_input_voltage_filtered();

	if (v_in < PAS_MIN_VIN) {
		v_in = PAS_MIN_VIN;
	}

	const volatile mc_configuration *mcconf = mc_interface_get_configuration();
	const float i_max = mcconf->lo_current_max;

	if (i_max < 0.01) {
		return 0.0;
	}

	const float feedforward = (watts / v_in) / i_max;
	const float full_scale_w = i_max * v_in;

	measured_power_w = mc_interface_get_tot_current_in_filtered() * v_in;

	if (config.power_ctrl_mode != PAS_POWER_CLOSED_LOOP || !primary_output ||
			full_scale_w < 1.0 || dt_ms <= 0.0) {
		power_integ = 0.0;
		return feedforward;
	}

	// Nothing to regulate towards, so let the trim go rather than holding a
	// correction across a pause in pedalling.
	if (watts < 1.0) {
		power_integ = 0.0;
		return feedforward;
	}

	const float err_rel = (watts - measured_power_w) / full_scale_w;
	const float step = err_rel * config.power_gain * (dt_ms / 1000.0);
	float next = power_integ + step;

	// Stop integrating in whichever direction is already against a limit, so
	// that the trim does not wind up while the output is clamped and then take
	// time to unwind once it is not.
	const float total = feedforward + next;
	if ((total > ceiling && step > 0.0) || (total < 0.0 && step < 0.0)) {
		next = power_integ;
	}

	utils_truncate_number(&next, -ceiling, ceiling);
	power_integ = next;

	return feedforward + power_integ;
}

/**
 * Assist multiplier from road speed.
 *
 * Full assist up to taper_start_kmh, falling linearly to nothing at
 * taper_end_kmh. Setting the two equal, or the end below the start, gives a
 * hard cutoff at that speed. Both zero disables the taper.
 *
 * Speed comes from mc_interface_get_speed(), which is derived from motor RPM,
 * si_motor_poles, si_wheel_diameter and si_gear_ratio unless the hardware has a
 * wheel speed sensor. Those have to be right for this to mean anything.
 */
static float pas_speed_taper(void) {
	if (config.taper_end_kmh <= 0.01 && config.taper_start_kmh <= 0.01) {
		return 1.0;
	}

	const float kmh = mc_interface_get_speed() * 3.6;

	if (kmh <= config.taper_start_kmh) {
		return 1.0;
	}

	// A taper that does not span a range is a cutoff at the start speed.
	if (config.taper_end_kmh <= (config.taper_start_kmh + 0.01)) {
		return 0.0;
	}

	if (kmh >= config.taper_end_kmh) {
		return 0.0;
	}

	float scale = utils_map(kmh, config.taper_start_kmh, config.taper_end_kmh, 1.0, 0.0);
	utils_truncate_number(&scale, 0.0, 1.0);
	return scale;
}

/**
 * True when the brake is being applied.
 *
 * The threshold comparison covers a proportional lever and a plain switch
 * alike, since a switch reads as one rail or the other, and brake_invert
 * handles a switch that pulls low when applied.
 */
static bool pas_brake_active(void) {
	if (config.brake_source != PAS_BRAKE_SRC_ADC) {
		return false;
	}

	int adc_ch = pas_adc_index(config.brake_adc_ch);

	if (adc_ch < 0) {
		brake_ch_invalid = true;
		return false;
	}
	brake_ch_invalid = false;

	const bool over = ADC_VOLTS(adc_ch) >= config.brake_threshold_v;
	return config.brake_invert ? !over : over;
}

/**
 * Set or clear the walk assist request from a script.
 *
 * This is a keepalive rather than a latch. It has to be called repeatedly while
 * the button is held, because it expires after WALK_LISP_TIMEOUT_S. A display
 * that loses power or a script that stops running therefore releases walk
 * assist rather than leaving the motor driving.
 */
void app_pas_walk_set(bool active) {
	walk_lisp_request = active;
	walk_lisp_time = chVTGetSystemTimeX();
}

/**
 * True when walk assist should be driving.
 *
 * Unlike pedal assist, this is meant to move the vehicle with nobody on it and
 * the cranks still, so it deliberately does not use the cadence interlock that
 * makes the pedal assist types safe. Everything that keeps it in check is here:
 * it needs an explicit source, it is capped in speed, and walk_require_pedal
 * can put the cadence requirement back for anyone who wants it.
 */
static bool pas_walk_active(void) {
	bool requested = false;

	switch (config.walk_source) {
		case PAS_WALK_SRC_LISP:
			requested = walk_lisp_request &&
					UTILS_AGE_S(walk_lisp_time) < WALK_LISP_TIMEOUT_S;
			break;

		case PAS_WALK_SRC_ADC: {
			int adc_ch = pas_adc_index(config.walk_adc_ch);

			if (adc_ch < 0) {
				walk_ch_invalid = true;
				return false;
			}
			walk_ch_invalid = false;

			const bool over = ADC_VOLTS(adc_ch) >= config.walk_threshold_v;
			requested = config.walk_invert ? !over : over;
		} break;

		default:
			return false;
	}

	if (!requested) {
		return false;
	}

	// Optional: only walk while the cranks are turning, for anyone who would
	// rather not have a button that drives the vehicle on its own.
	if (config.walk_require_pedal && pedal_rpm < 0.01) {
		return false;
	}

	return true;
}

/**
 * The relative current walk assist should apply.
 *
 * Faded out approaching the speed limit rather than cut at it, so that walk
 * assist settles at the limit instead of pulsing against it. There is no
 * closed loop on speed, so the limit is where assist stops rather than a speed
 * it will hold.
 */
static float pas_walk_output(void) {
	const float kmh = mc_interface_get_speed() * 3.6;
	const float limit = config.walk_max_kmh;

	if (limit <= 0.01 || kmh >= limit) {
		return 0.0;
	}

	float scale = 1.0;

	if (kmh > (limit - WALK_TAPER_KMH)) {
		scale = utils_map(kmh, limit - WALK_TAPER_KMH, limit, 1.0, 0.0);
		utils_truncate_number(&scale, 0.0, 1.0);
	}

	float output = config.walk_current * scale;
	utils_truncate_number(&output, 0.0, config.walk_current);
	return output;
}

/**
 * Cap a relative current request so that it does not exceed a power limit.
 *
 * Applied to the relative current rather than inside the power control type, so
 * that it limits every control type rather than only the one that works in
 * watts.
 */
/**
 * The power cap expressed as a relative current, or 1.0 when there is none.
 *
 * Shared with the closed loop so that its anti-windup ceiling accounts for the
 * cap. Otherwise the trim would wind up against a limit applied after it, and
 * then take time to unwind once the cap was no longer binding.
 */
/*
 * The ceiling every control type clamps its output to, as a fraction of the
 * global current limit.
 *
 * Clamped non-negative because it is used as the max argument of
 * utils_truncate_number, which tests the maximum first: given a negative
 * maximum it clamps to that rather than to the zero minimum, and the output
 * leaves here negative. mc_interface_set_current_rel reads a negative value as
 * braking current, so a negative PAS Max Current made pedalling brake the
 * bike, in proportion to cadence.
 *
 * VESC Tool will not produce one -- the parameter's minimum is zero -- but the
 * limits in the XML are not part of the configuration signature and are not
 * enforced anywhere in the firmware, so any sender of COMM_SET_APPCONF can
 * write whatever it likes. The same applies to the sub scaling, which arrives
 * from another app at runtime rather than from a configuration at all.
 *
 * Found by tests/app_pas/fuzz.c.
 */
static float pas_output_ceiling(void) {
	float ceiling = config.current_scaling * sub_scaling;
	if (!(ceiling > 0.0)) {
		// Also catches NaN, which compares false against everything and would
		// otherwise pass straight through the clamps.
		return 0.0;
	}
	return ceiling;
}

static float pas_power_cap_rel(void) {
	if (config.power_max_w <= 0.1) {
		return 1.0;
	}

	float v_in = mc_interface_get_input_voltage_filtered();

	if (v_in < PAS_MIN_VIN) {
		v_in = PAS_MIN_VIN;
	}

	const volatile mc_configuration *mcconf = mc_interface_get_configuration();
	const float full_scale_w = mcconf->lo_current_max * v_in;

	if (full_scale_w < 1.0) {
		return 1.0;
	}

	return config.power_max_w / full_scale_w;
}

static float pas_apply_power_cap(float output) {
	const float max_rel = pas_power_cap_rel();

	if (output > max_rel) {
		return max_rel;
	}
	return output;
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

// The well known numbers for an EU pedelec. Presented as a preset: setting
// these does not make an installation compliant, which depends on the whole
// vehicle and on how it is used.
#define PEDELEC_CUTOFF_KMH		25.0
#define PEDELEC_POWER_W			250.0

/*
 * The eRider GTL-T17-73 torque bottom bracket, from its datasheet.
 *
 * Torque ratio 14.7 mV/Nm inverts to 68.03 Nm/V, and the 1.50 to 3.00 V output
 * range gives 1.5 V of span, so full scale is 102 Nm. That agrees with the
 * other figure the datasheet gives, 60 kgf of pedal force, which on the 170 mm
 * crank it is specified against is 100 Nm -- two independent numbers landing in
 * the same place is what makes them worth trusting. None of it depends on the
 * crank actually fitted: the ratio is torque per volt, and crank length only
 * changes how much pedal force reaches full scale.
 *
 * 3.00 V sits under every ADC reference in the tree -- 91 % of 3.3 V on a
 * Flipsky, 87 % of 3.44 V on a Ubox -- so no divider is needed and the
 * saturation flag stays a fault detector rather than an expected condition.
 *
 * 18 square waves per rotation on each of two channels is quadrature with 18
 * cycles per crank revolution, which is what magnets counts.
 */
#define ERIDER_NM_PER_V			68.03		// 1000 / 14.7 mV per Nm
#define ERIDER_MAX_NM			102.0		// 1.5 V of span at that ratio
#define ERIDER_ZERO_V			1.50		// datasheet Vo, +/- 0.05
#define ERIDER_MAGNETS			18
#define ERIDER_ZERO_TOL_V		0.05

static void pas_preset_apply_pedelec(volatile pas_config *c) {
	c->taper_start_kmh = PEDELEC_CUTOFF_KMH;
	c->taper_end_kmh = PEDELEC_CUTOFF_KMH;
	c->power_max_w = PEDELEC_POWER_W;
	c->pedal_stop_hard = true;
}

static void pas_preset_apply_erider(volatile pas_config *c) {
	c->sensor_type = PAS_SENSOR_TYPE_QUADRATURE;
	c->magnets = ERIDER_MAGNETS;
	c->torque_source = PAS_TORQUE_SRC_ADC;
	c->torque_zero_v = ERIDER_ZERO_V;
	c->torque_nm_per_v = ERIDER_NM_PER_V;
	c->torque_max_nm = ERIDER_MAX_NM;
	c->torque_avg_pulses = ERIDER_MAGNETS;
	c->ctrl_type = PAS_CTRL_TYPE_POWER;
}

/*
 * Write one preset.
 *
 * Each preset touches only its own fields, on a copy of the running
 * configuration, so applying both leaves both in place: one sets limits and the
 * other sets what the sensor is, and neither has an opinion about the other.
 */
static void pas_preset_write(void (*apply)(volatile pas_config *)) {
	app_configuration *appconf = mempools_alloc_appconf();

	if (appconf == 0) {
		commands_printf("Could not allocate the configuration, nothing written.");
		commands_printf(" ");
		return;
	}

	*appconf = *app_get_configuration();
	apply(&appconf->app_pas_conf);

	if (conf_general_store_app_configuration(appconf)) {
		app_set_configuration(appconf);
		commands_printf("Written and applied.");
	} else {
		commands_printf("Writing the configuration failed, nothing changed.");
	}

	mempools_free_appconf(appconf);
	commands_printf(" ");
}

static void pas_preset_usage(void) {
	commands_printf("Usage: pas_preset <name> [store]");
	commands_printf("Prints the values the preset would set. Add \"store\" to write them.");
	commands_printf(" ");
	commands_printf("  pedelec  speed cutoff and power cap, and a hard pedal stop");
	commands_printf("  erider   the eRider GTL-T17-73 torque bottom bracket");
	commands_printf(" ");
	commands_printf("They are independent: one sets limits, the other sets what the sensor");
	commands_printf("is, and applying both keeps both.");
	commands_printf(" ");
}

static void terminal_pas_preset(int argc, const char **argv) {
	if (argc < 2) {
		pas_preset_usage();
		return;
	}

	const bool store = (argc >= 3 && strcmp(argv[2], "store") == 0);

	if (strcmp(argv[1], "pedelec") == 0) {
		commands_printf("Pedelec preset:");
		commands_printf("  Speed cutoff      : %.0f km/h (taper start and end equal)",
				(double)PEDELEC_CUTOFF_KMH);
		commands_printf("  Power cap         : %.0f W", (double)PEDELEC_POWER_W);
		commands_printf("  Pedal stop        : hard cut");
		commands_printf(" ");
		commands_printf("These are the well known numbers, not a statement that an installation");
		commands_printf("is compliant. That depends on the whole vehicle and how it is used.");
		commands_printf(" ");
		commands_printf("Note that the speed cutoff is only as good as the speed reading, which");
		commands_printf("comes from the motor RPM and the configured pole count, wheel diameter");
		commands_printf("and gear ratio unless the hardware has a wheel speed sensor.");
		commands_printf(" ");
		commands_printf("There is no walk assist mode, so the preset sets no walk speed limit.");
		commands_printf(" ");

		if (!store) {
			commands_printf("Not written. Add \"store\" to apply and save.");
			commands_printf(" ");
			return;
		}

		pas_preset_write(pas_preset_apply_pedelec);
		return;
	}

	if (strcmp(argv[1], "erider") == 0) {
		commands_printf("eRider GTL-T17-73 preset:");
		commands_printf("  Sensor type       : quadrature");
		commands_printf("  Magnets           : %d  (18 square waves per rotation, two channels)",
				ERIDER_MAGNETS);
		commands_printf("  Torque source     : ADC");
		commands_printf("  Torque zero       : %.2f V", (double)ERIDER_ZERO_V);
		commands_printf("  Torque scale      : %.2f Nm/V  (14.7 mV/Nm)",
				(double)ERIDER_NM_PER_V);
		commands_printf("  Torque max        : %.0f Nm  (1.5 V of span at that scale)",
				(double)ERIDER_MAX_NM);
		commands_printf("  Torque averaging  : %d pulses, one crank revolution",
				ERIDER_MAGNETS);
		commands_printf("  Control type      : power");
		commands_printf(" ");
		commands_printf("The control type is included because a torque sensor with a cadence");
		commands_printf("control type ignores the torque entirely, which is a confusing way to");
		commands_printf("start. Change it afterwards if you want something else.");
		commands_printf(" ");
		commands_printf("NOT set, because only the wiring knows: the torque ADC channel. Set");
		commands_printf("PAS Torque ADC Channel to whichever input the torque wire is on, then");
		commands_printf("run \"pas_status\" to confirm the channel exists and reads a voltage.");
		commands_printf(" ");
		commands_printf("Then run \"pas_torque zero\" with no force on the cranks. The datasheet");
		commands_printf("zero is %.2f V +/- %.2f, and %.2f V is %.1f Nm at this scale -- more than",
				(double)ERIDER_ZERO_V, (double)ERIDER_ZERO_TOL_V,
				(double)ERIDER_ZERO_TOL_V,
				(double)(ERIDER_ZERO_TOL_V * ERIDER_NM_PER_V));
		commands_printf("the default deadband, so a unit at the edge of tolerance gives either");
		commands_printf("phantom assist at rest or a dead zone. Measuring it takes a second.");
		commands_printf(" ");
		commands_printf("The speed outputs are specified as square waves, so check their high");
		commands_printf("level before wiring them to a pin: this sensor runs on 5 to 15 V and a");
		commands_printf("5 V output needs a divider to reach a 3.3 V input safely.");
		commands_printf(" ");

		if (!store) {
			commands_printf("Not written. Add \"store\" to apply and save.");
			commands_printf(" ");
			return;
		}

		pas_preset_write(pas_preset_apply_erider);
		return;
	}

	pas_preset_usage();
}

static void terminal_pas_torque(int argc, const char **argv) {
	if (config.torque_source != PAS_TORQUE_SRC_ADC) {
		commands_printf("The torque source is not set to ADC, so there is nothing to read.");
		commands_printf(" ");
		return;
	}

	int adc_ch = pas_adc_index(config.torque_adc_ch);
	if (adc_ch < 0) {
		commands_printf("ADC channel %d is not available on this hardware.",
				(int)config.torque_adc_ch + 1);
		commands_printf(" ");
		return;
	}

	if (argc >= 2 && strcmp(argv[1], "zero") == 0) {
		// Average with the cranks at rest to find the zero point. Kept to about
		// a second because this runs on the thread that serves the terminal.
		const int samples = 100;
		float sum = 0.0;
		float min_v = 10.0;
		float max_v = 0.0;

		commands_printf("Measuring with no force on the cranks, about one second.");

		for (int i = 0; i < samples; i++) {
			float v = ADC_VOLTS(adc_ch);
			sum += v;
			if (v < min_v) {
				min_v = v;
			}
			if (v > max_v) {
				max_v = v;
			}
			chThdSleepMilliseconds(10);
		}

		float mean = sum / (float)samples;
		commands_printf("Mean    : %.4f V", (double)mean);
		commands_printf("Spread  : %.4f V (min %.4f, max %.4f)", (double)(max_v - min_v),
				(double)min_v, (double)max_v);
		commands_printf("Currently configured zero: %.4f V", (double)config.torque_zero_v);
		commands_printf(" ");
		commands_printf("Set PAS Torque Zero Voltage to the mean above if the cranks were");
		commands_printf("at rest. This does not write the configuration.");
	} else {
		float volts = ADC_VOLTS(adc_ch);
		float nm = (volts - config.torque_zero_v) * config.torque_nm_per_v;

		commands_printf("ADC channel   : %d", adc_ch);
		commands_printf("Voltage       : %.4f V  (reference %.2f V)", (double)volts,
				(double)V_REG);
		commands_printf("Filtered      : %.4f V", (double)torque_filter_v);
		commands_printf("Raw torque    : %.2f Nm (before deadband and limits)", (double)nm);
		commands_printf("Reported      : %.2f Nm", (double)torque_nm);
		if (volts >= (V_REG * 0.99)) {
			commands_printf("SATURATED: at or above the ADC reference.");
		}
		commands_printf(" ");
		commands_printf("Run \"pas_torque zero\" with no force on the cranks to measure the zero point.");
		commands_printf("To measure the scale: brake the rear wheel, hang a known weight on");
		commands_printf("the forward pedal, take the voltage change, then");
		commands_printf("  Nm/V = weight_lb * 4.44 * crank_m / volts_change");
	}

	commands_printf(" ");
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
	commands_printf("Start period      : %.3f s%s", (double)start_period,
			config.start_timeout_s > 0.001 ? "" : " (derived)");
	commands_printf("Stop period       : %.3f s%s", (double)stop_period,
			config.stop_timeout_s > 0.001 ? "" : " (derived)");
	if (stop_period > 0.35) {
		commands_printf("  The stop period is long, so assist will linger after the cranks");
		commands_printf("  stop. Grin suggest 0.15 to 0.30 s on a Cycle Analyst.");
	}
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

	static const char *src_names[] = {"none", "hardware", "adc"};
	commands_printf("Torque source     : %s",
			config.torque_source <= PAS_TORQUE_SRC_ADC ?
					src_names[config.torque_source] : "invalid");
	commands_printf("Torque            : %.2f Nm (ratio %.3f)", (double)torque_nm,
			(double)torque_ratio);

	if (config.torque_source == PAS_TORQUE_SRC_ADC) {
		commands_printf("Torque ADC ch     : %d%s", (int)config.torque_adc_ch + 1,
				torque_ch_invalid ? "  NOT AVAILABLE on this hardware" : "");
		commands_printf("Torque voltage    : %.3f V (zero %.3f V, %.1f Nm/V)",
				(double)torque_volts, (double)config.torque_zero_v,
				(double)config.torque_nm_per_v);
		commands_printf("ADC reference     : %.2f V", (double)V_REG);
		if (torque_saturated) {
			commands_printf("  SATURATED: the sensor output is at or above the ADC reference,");
			commands_printf("  so torque above this point cannot be measured. Fit a divider.");
		}
	}

	if (torque_src_unsupported) {
		commands_printf("  The hardware torque source is selected, but this board does not");
		commands_printf("  implement one. Use the ADC source instead.");
	}

	if ((config.ctrl_type == PAS_CTRL_TYPE_TORQUE ||
			config.ctrl_type == PAS_CTRL_TYPE_TORQUE_WITH_CADENCE_TIMEOUT) &&
			config.torque_source == PAS_TORQUE_SRC_NONE) {
		commands_printf("  A torque control type is selected with no torque source, so");
		commands_printf("  there is no assist. Set the torque source.");
	}

	if (config.ctrl_type == PAS_CTRL_TYPE_POWER) {
		commands_printf("Assist gain       : %.2f motor W per rider W",
				(double)config.assist_gain);
		commands_printf("Rider power       : %.1f W (actual cadence)", (double)rider_power_w);
		if (config.cadence_floor_rpm > 0.01) {
			commands_printf("Assist basis      : %.1f W (cadence floored at %.0f rpm)",
				(double)assist_basis_w, (double)config.cadence_floor_rpm);
		} else {
			commands_printf("Assist basis      : %.1f W (no cadence floor)",
				(double)assist_basis_w);
		}
		commands_printf("Motor power target: %.1f W", (double)motor_power_w);
		commands_printf("Input voltage     : %.1f V",
				(double)mc_interface_get_input_voltage_filtered());
		if (config.power_ctrl_mode == PAS_POWER_CLOSED_LOOP) {
			commands_printf("Power control     : closed loop, gain %.2f /s",
				(double)config.power_gain);
			commands_printf("Measured power    : %.1f W (trim %+.3f)",
				(double)measured_power_w, (double)power_integ);
			if (!primary_output) {
				commands_printf("  Not closed: a throttle shares the output, so measured power");
				commands_printf("  is not attributable to this request. Running open loop.");
			}
		} else {
			commands_printf("Power control     : open loop");
		}
	}

	if (config.torque_avg_pulses > 0) {
		commands_printf("Torque averaging  : %d pulses (%d filled), %.2f Nm",
				(int)config.torque_avg_pulses, (int)torque_ring_fill,
				(double)torque_avg_nm);
		if (config.torque_avg_pulses != config.magnets) {
			commands_printf("  Note: set this to the magnet count (%d) for exactly one",
					(int)config.magnets);
			commands_printf("  crank revolution.");
		}
	} else {
		commands_printf("Torque averaging  : off");
	}

	if (config.taper_start_kmh > 0.01 || config.taper_end_kmh > 0.01) {
		if (config.taper_end_kmh <= (config.taper_start_kmh + 0.01)) {
			commands_printf("Speed limit       : hard cutoff at %.1f km/h",
					(double)config.taper_start_kmh);
		} else {
			commands_printf("Speed limit       : full to %.1f km/h, zero at %.1f km/h",
					(double)config.taper_start_kmh, (double)config.taper_end_kmh);
		}
		commands_printf("Speed             : %.1f km/h (taper %.2f)",
				(double)(mc_interface_get_speed() * 3.6), (double)speed_taper);
	} else {
		commands_printf("Speed limit       : off");
	}

	if (config.power_max_w > 0.1) {
		commands_printf("Power cap         : %.0f W", (double)config.power_max_w);
	} else {
		commands_printf("Power cap         : off");
	}

	commands_printf("Pedal stop        : %s",
			config.pedal_stop_hard ? "hard cut" : "ramp down");

	if (config.brake_source == PAS_BRAKE_SRC_ADC) {
		int brake_ch = pas_adc_index(config.brake_adc_ch);
		commands_printf("Brake             : ADC %d%s, threshold %.2f V%s, %s",
				(int)config.brake_adc_ch + 1,
				brake_ch < 0 ? " NOT AVAILABLE" : "",
				(double)config.brake_threshold_v,
				config.brake_invert ? " inverted" : "",
				brake_engaged ? "APPLIED" : "released");
		if (brake_ch >= 0) {
			commands_printf("Brake voltage     : %.3f V", (double)ADC_VOLTS(brake_ch));
		}
	} else {
		commands_printf("Brake             : off");
	}

	if (config.walk_source != PAS_WALK_SRC_NONE) {
		commands_printf("Walk assist       : %s, %.0f%% current to %.1f km/h%s",
			config.walk_source == PAS_WALK_SRC_LISP ? "script" : "ADC",
			(double)(config.walk_current * 100.0), (double)config.walk_max_kmh,
			walk_active ? "  ACTIVE" : "");
		commands_printf("  Cranks            : %s",
			config.walk_require_pedal ? "must be turning" : "not required");
		if (config.walk_source == PAS_WALK_SRC_ADC) {
			commands_printf("  ADC ch            : %d%s, threshold %.2f V%s",
				(int)config.walk_adc_ch + 1,
				walk_ch_invalid ? " NOT AVAILABLE" : "",
				(double)config.walk_threshold_v,
				config.walk_invert ? " inverted" : "");
		}
		if (config.walk_source == PAS_WALK_SRC_LISP) {
			commands_printf("  A script must call app-pas-walk-set repeatedly while held;");
			commands_printf("  the request expires after %.1f s.", (double)WALK_LISP_TIMEOUT_S);
		}
	} else {
		commands_printf("Walk assist       : off");
	}

	commands_printf("Throttle mix      : %s",
			config.throttle_mode == PAS_THROTTLE_PRIORITY ?
				"throttle takes priority" : "whichever asks for more");
	if (config.throttle_no_pedal_kmh > 0.01) {
		commands_printf("Throttle needs pedalling above %.1f km/h",
			(double)config.throttle_no_pedal_kmh);
	}
	if (config.assist_start_w > 0.01) {
		commands_printf("Assist start      : %.0f W of rider power before assist",
			(double)config.assist_start_w);
	}
	if (config.torque_source == PAS_TORQUE_SRC_ADC && config.torque_bipolar) {
		commands_printf("Torque sensor     : bipolar, magnitude either side of zero");
	}

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

	terminal_register_command_callback(
			"pas_preset",
			"Apply a PAS preset: \"pedelec\" limits, \"erider\" sensor calibration.",
			"<pedelec|erider> [store]",
			terminal_pas_preset);

	terminal_register_command_callback(
			"pas_torque",
			"Read the analog PAS torque sensor. Add \"zero\" to measure the zero point.",
			"[zero]",
			terminal_pas_torque);
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
	terminal_unregister_callback(terminal_pas_torque);
	terminal_unregister_callback(terminal_pas_preset);

	// Cleared in both modes. It was only cleared when PAS was not the primary
	// output, and now that the throttle app reads it for every current control
	// type a stale value could survive into a later throttle-only session.
	output_current_rel = 0.0;

	if (primary_output == true) {
		mc_interface_set_current_rel(0.0);
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

float app_pas_get_torque_nm(void) {
	return torque_nm;
}

float app_pas_get_torque_ratio(void) {
	return torque_ratio;
}

bool app_pas_torque_saturated(void) {
	return torque_saturated;
}

bool app_pas_torque_ch_invalid(void) {
	return torque_ch_invalid;
}

float app_pas_get_rider_power(void) {
	return rider_power_w;
}

float app_pas_get_assist_basis_power(void) {
	return assist_basis_w;
}

/**
 * Conditions worth surfacing, as a bitfield, so that a display or a script can
 * report them without needing an accessor for each one.
 */
int app_pas_get_flags(void) {
	int flags = 0;

	if (torque_saturated) {
		flags |= PAS_FLAG_TORQUE_SATURATED;
	}
	if (torque_ch_invalid) {
		flags |= PAS_FLAG_TORQUE_CH_INVALID;
	}
	if (brake_ch_invalid) {
		flags |= PAS_FLAG_BRAKE_CH_INVALID;
	}
	if (brake_engaged) {
		flags |= PAS_FLAG_BRAKE_ENGAGED;
	}
	if (walk_active) {
		flags |= PAS_FLAG_WALK_ACTIVE;
	}
	if (walk_ch_invalid) {
		flags |= PAS_FLAG_WALK_CH_INVALID;
	}
	if (pins_unavailable) {
		flags |= PAS_FLAG_PINS_UNAVAILABLE;
	}
	if (sensor_type_unsupported) {
		flags |= PAS_FLAG_SENSOR_UNSUPPORTED;
	}
	if (torque_src_unsupported) {
		flags |= PAS_FLAG_TORQUE_SRC_UNSUPPORTED;
	}
	if (speed_taper < 0.999) {
		flags |= PAS_FLAG_SPEED_LIMITED;
	}

	return flags;
}

float app_pas_get_speed_taper(void) {
	return speed_taper;
}

float app_pas_get_motor_power_target(void) {
	return motor_power_w;
}

float app_pas_get_measured_power(void) {
	return measured_power_w;
}

/**
 * Decode a quadrature (two-wire) pedal sensor.
 *
 * Called at the app update rate. Updates pedal_rpm, or sets it to zero when the
 * cranks have been still for longer than the stop period.
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
		// The 2 that QEM yields for an illegal double transition, and the 0 for
		// no movement. Both deliberately do nothing, and saying so is what
		// separates that from a missing case.
		default: break;
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
		// gap is compared against the start period, which is a gap between single
		// pulses, so the age is used here rather than the full revolution period
		// computed below.
		if (!dec_have_reference || pulse_age > start_period) {
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
		dec_pulse_count++;
	} else {
		// If no pedal activity, set RPM as zero
		if (UTILS_AGE_S(dec_last_pulse_time) > stop_period) {
			pedal_rpm = 0.0;
			dec_period_filtered = 0.0;
		}
	}
#endif
}

/**
 * Decode a single-wire pedal sensor.
 *
 * One edge per magnet and no direction information, so invert_pedal_direction
 * has no effect here and back-pedalling is indistinguishable from pedalling
 * forwards. min_pedal_period doubles as the contact debounce. Follows the
 * single-wire decoder in hwconf/itr/hw_itr_x1_core.c.
 */
static void pas_decode_single_wire(void) {
#ifdef HW_PAS1_PORT
	uint8_t level = palReadPad(HW_PAS1_PORT, HW_PAS1_PIN);
	uint8_t prev = dec_old_state;
	dec_old_state = level;

	if (level && !prev) {
		float pulse_age = UTILS_AGE_S(dec_last_pulse_time);
		dec_last_pulse_time = chVTGetSystemTimeX();

		// As for quadrature, the first pulse after a start or after an idle
		// period is only a reference for the next one.
		if (!dec_have_reference || pulse_age > start_period) {
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
			dec_period_filtered = period;
		}

		pedal_rpm = 60.0 / dec_period_filtered;
		dec_pulse_count++;
	} else {
		// If no pedal activity, set RPM as zero
		if (UTILS_AGE_S(dec_last_pulse_time) > stop_period) {
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

		case PAS_SENSOR_TYPE_SINGLE_WIRE:
			pas_decode_single_wire();
			break;

		default:
			// An unsupported sensor type must not silently behave like a supported
			// one, so produce no cadence and let the thread report it.
			pedal_rpm = 0.0;
			sensor_type_unsupported = true;
			break;
	}
}

/**
 * Combine a throttle request with the PAS output.
 *
 * Called by the throttle app when both are running, so that the policy lives
 * with the configuration that selects it. Returns the throttle request
 * unchanged when PAS is not running.
 *
 * Two policies, because both are defensible. Taking whichever asks for more is
 * what this firmware has always done, and means a throttle can only ever add.
 * Giving the throttle priority is what a Cycle Analyst does, where "if throttle
 * is applied even a small amount while pedaling, PAS assist is ignored and the
 * throttle alone determines the output". That is more predictable, at the cost
 * that a throttle brushed by a knee can reduce assist.
 */
float pas_mix_throttle(float throttle_rel, float pas_rel) {
	// Above a configured road speed the throttle only works while pedalling,
	// which is the Cycle Analyst MxThrotSpd behaviour. Zero disables the check
	// here, rather than meaning "always require pedalling" as it does on a Cycle
	// Analyst, so that the default leaves an existing throttle working.
	if (config.throttle_no_pedal_kmh > 0.01 &&
			(mc_interface_get_speed() * 3.6) > config.throttle_no_pedal_kmh &&
			pedal_rpm < 0.01) {
		throttle_rel = 0.0;
	}

	switch (config.throttle_mode) {
		case PAS_THROTTLE_PRIORITY:
			// A small amount of throttle is enough to take over, so the handover
			// does not depend on how far the lever has travelled.
			return fabsf(throttle_rel) > 0.01 ? throttle_rel : pas_rel;

		default:
			return utils_max_abs(throttle_rel, pas_rel);
	}
}

float app_pas_apply_to_throttle(float throttle_rel) {
	if (!is_running) {
		return throttle_rel;
	}

	return pas_mix_throttle(throttle_rel, output_current_rel);
}

/**
 * Compute the relative current output for the configured control type.
 *
 * Separated from the thread so that the assist law is reachable without one,
 * and so that the thread is left holding only the loop and the safety
 * interlocks. dt_ms is the interval since the previous call.
 */
float pas_compute_output(float dt_ms) {
	float output = 0.0;

	// Walk assist replaces the control type entirely rather than adding to it,
	// and bypasses the ramp so that releasing the trigger stops the motor at
	// once rather than over ramp_time_neg. The brake still overrides it.
	walk_active = pas_walk_active();

	if (walk_active) {
		walk_was_active = true;
		output = pas_walk_output();
		out_ramp = output;

		brake_engaged = pas_brake_active();
		if (brake_engaged) {
			out_ramp = 0.0;
			output = 0.0;
		}

		return output;
	}

	// Leaving walk assist must not ramp down from the walk current.
	if (walk_was_active) {
		walk_was_active = false;
		out_ramp = 0.0;
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
				output = utils_map(pedal_rpm, config.pedal_rpm_start, config.pedal_rpm_end, 0.0, pas_output_ceiling());
				utils_truncate_number(&output, 0.0, pas_output_ceiling());
			} else {
				if (pedal_rpm > config.pedal_rpm_end) {
					output = pas_output_ceiling();
				} else {
					output = 0.0;
				}
			}
			break;

		case PAS_CTRL_TYPE_TORQUE:
		case PAS_CTRL_TYPE_TORQUE_WITH_CADENCE_TIMEOUT:
		{
			// Both torque types sample the sensor. Previously only the first case
			// did, and the second relied on falling into it, so selecting the
			// second one directly never computed an output at all.
			//
			// The reading comes from whichever source is configured, so this is
			// no longer restricted to boards that implement hw_get_PAS_torque().
			torque_nm = pas_read_torque_nm();
			pas_torque_track(torque_nm);

			float ratio = config.torque_max_nm > 0.01 ?
					torque_nm / config.torque_max_nm : 0.0;
			utils_truncate_number(&ratio, 0.0, 1.0);
			torque_ratio = ratio;

			output = ratio * pas_output_ceiling();
			utils_truncate_number(&output, 0.0, pas_output_ceiling());

			// The cadence checks below are applied to both types. They were added
			// as a safety fix and reached the plain torque type through the
			// fall-through, so they are kept there rather than dropped.

			// disable assistance if torque has been sensed for >5sec without any pedal movement. Prevents
			// motor overtemps when the rider is just resting on the pedals
			if(output == 0.0 || pedal_rpm > 0) {
				out_ms_without_cadence_or_torque = 0.0;
			} else {
				out_ms_without_cadence_or_torque += dt_ms;
				if(out_ms_without_cadence_or_torque > MAX_MS_WITHOUT_CADENCE_OR_TORQUE) {
					output = 0.0;
				}
			}
			// if cranks are not moving, there should not be any output. This covers the case of a torque sensor
			// stuck with a non-zero signal.
			if(pedal_rpm < 0.01) {
				out_ms_without_cadence += dt_ms;
				if(out_ms_without_cadence > MAX_MS_WITHOUT_CADENCE) {
					output = 0.0;
				}
			} else {
				out_ms_without_cadence = 0.0;
			}
		}
		break;

		case PAS_CTRL_TYPE_POWER:
		{
			// Motor power proportional to rider power. Rider power is zero
			// whenever the cranks are stopped, so this needs no separate
			// interlock for a torque sensor stuck at a non-zero reading.
			torque_nm = pas_read_torque_nm();
			pas_torque_track(torque_nm);

			const float control_nm = pas_torque_for_control(torque_nm);

			// Reported from the real cadence, assisted from the floored one.
			rider_power_w = pas_power_w(control_nm, pedal_rpm);
			assist_basis_w = pas_power_w(control_nm, pas_assist_cadence());
			// A minimum rider effort before the motor contributes anything,
			// subtracted from the basis before the gain, which is how a Cycle
			// Analyst applies its start level. A power threshold rather than the
			// torque deadband, so it demands actual effort rather than tripping
			// on a constant force at any cadence.
			float basis = assist_basis_w - config.assist_start_w;
			if (basis < 0.0) {
				basis = 0.0;
			}
			motor_power_w = basis * config.assist_gain;

			// The ceiling the trim must not wind up past is the tighter of the
			// assist current limit and the power cap, both applied after this.
			float ceiling = pas_output_ceiling();
			const float cap_rel = pas_power_cap_rel();
			if (cap_rel < ceiling) {
				ceiling = cap_rel;
			}

			output = pas_power_to_current_rel(motor_power_w, ceiling, dt_ms);
			utils_truncate_number(&output, 0.0, pas_output_ceiling());
		}
		break;

		default:
			break;
	}
	// Road speed taper, before the ramp so that the ramp smooths it.
	speed_taper = pas_speed_taper();
	output *= speed_taper;

	// Power cap, on the relative current so that it limits every control type.
	output = pas_apply_power_cap(output);

	// Ramping.
	float ramp_time = fabsf(output) > fabsf(out_ramp) ?
			config.ramp_time_pos : config.ramp_time_neg;

	if (ramp_time > 0.01) {
		const float ramp_step = (dt_ms / 1000.0) / ramp_time;
		utils_step_towards(&out_ramp, output, ramp_step);
		utils_truncate_number(&out_ramp, 0.0, pas_output_ceiling());
		output = out_ramp;
	}

	// A hard pedal stop bypasses the ramp, for setups that want assist to end
	// with the pedalling rather than fade out over ramp_time_neg.
	if (config.pedal_stop_hard && pedal_rpm < 0.01) {
		out_ramp = 0.0;
		output = 0.0;
	}

	// The brake cuts assist immediately, after the ramp rather than through it.
	brake_engaged = pas_brake_active();
	if (brake_engaged) {
		out_ramp = 0.0;
		output = 0.0;
	}

	return output;
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

		// The elapsed time is taken every iteration. Taking it only where it was
		// used let it grow without bound whenever that was skipped, so the first
		// enabled iteration took one unbounded step.
		//
		// Scaled in floating point rather than through ST2MS, which divides as
		// integers and rounds up: it quantises the interval to whole
		// milliseconds. At the default 500 Hz that happens to be exact, but the
		// update rate is configurable, and at 3 kHz a true 0.3 ms interval came
		// back as 1 ms -- three times too long, into the ramp and the power
		// loop's integral. clang-tidy's bugprone-integer-division found it.
		const float dt_ms = (float)chVTTimeElapsedSinceX(out_last_time) *
				(1000.0 / (float)CH_CFG_ST_FREQUENCY);
		out_last_time = chVTGetSystemTimeX();

		// Computed fresh every iteration: a control type that produces nothing
		// must not leave a previous value latched to be applied.
		const float output = pas_compute_output(dt_ms);

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

		// Recorded in both modes so that it can be read back as telemetry. It was
		// previously only written when PAS was not the primary output, so the
		// getter returned zero in the mode where PAS drives the motor.
		output_current_rel = output;

		if (primary_output == true) {
			mc_interface_set_current_rel(output);
		}
	}
}
