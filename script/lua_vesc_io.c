/*
	Copyright 2025 Benjamin Vedder	benjamin@vedder.se

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

/*
 * Input bindings: the ADC pins, the RC input, the encoder, and taking over
 * the throttle from the app that normally owns it.
 *
 * The override calls are the reason this set exists. A script that wants to
 * drive the motor from its own logic has to stop the ADC or PPM app from
 * driving it too, and app_disable_output is how a bench setup is made safe
 * before anything spins.
 */

#include "lua_vesc_io.h"

#include "lauxlib.h"

#include <math.h>

#include "conf_general.h"
#include "mc_interface.h"   // for the ADC_Value array ADC_VOLTS reads
#include "app.h"
#include "servo_dec.h"
#include "pwm_servo.h"
#include "encoder.h"
#include "timeout.h"
#include "utils.h"

/* ----------------------------------------------------------------- adc --- */

/*
 * An ADC channel's pin voltage.
 *
 * Channels follow lisp's numbering so a script ports unchanged: 0..2 are
 * EXT..EXT3, 3 is the motor temperature pin, 4..8 are EXT4..EXT8.
 *
 * Unlike lisp, a channel the board does not have returns nil instead of a
 * number, and that is the whole point of this function. hw.h aliases
 * ADC_IND_EXT2 through ADC_IND_EXT8 to ADC_IND_EXT when a board does not
 * define them:
 *
 *     #ifndef ADC_IND_EXT4
 *     #define ADC_IND_EXT4 ADC_IND_EXT
 *     #endif
 *
 * So on a board with one external pin, reading channel 4 silently returns the
 * voltage on channel 0 -- which on most setups is the throttle. A script
 * reading a sensor it thinks is on EXT4 gets a plausible, moving, completely
 * wrong number. lisp has this behaviour; it is not worth copying.
 *
 * The test is that an index above 0 resolved to the same index as channel 0,
 * which means it came from the alias. A board that genuinely wired a second
 * pin to the same ADC index would be reported as absent, which is the safe
 * direction to be wrong in.
 */
static int adc_index(int channel, bool *aliased) {
	int ind;

	*aliased = false;
	switch (channel) {
	case 0: return ADC_IND_EXT;
	case 1: ind = ADC_IND_EXT2; break;
	case 2: ind = ADC_IND_EXT3; break;
	case 3: return ADC_IND_TEMP_MOTOR;
	case 4: ind = ADC_IND_EXT4; break;
	case 5: ind = ADC_IND_EXT5; break;
	case 6: ind = ADC_IND_EXT6; break;
	case 7: ind = ADC_IND_EXT7; break;
	case 8: ind = ADC_IND_EXT8; break;
	default: return -1;
	}

	if (ind == ADC_IND_EXT) {
		*aliased = true;
	}
	return ind;
}

static int l_get_adc(lua_State *L) {
	int channel = (int)luaL_optinteger(L, 1, 0);
	bool aliased = false;
	int ind = adc_index(channel, &aliased);

	if (ind < 0) {
		return luaL_argerror(L, 1, "adc channel must be 0..8");
	}
	if (aliased) {
		// The board has no such pin; see the note above.
		lua_pushnil(L);
		return 1;
	}

	lua_pushnumber(L, (lua_Number)ADC_VOLTS(ind));
	return 1;
}

/*
 * The ADC app's decoded throttle, 0..1 for channel 1 and channel 2.
 *
 * This is what the app itself acts on, after its own mapping, deadband and
 * ramping -- a different number from the pin voltage, and the one a script
 * wants if it is reproducing or replacing the app's behaviour.
 */
static int l_get_adc_decoded(lua_State *L) {
	int channel = (int)luaL_optinteger(L, 1, 1);

	switch (channel) {
	case 1:
		lua_pushnumber(L, (lua_Number)app_adc_get_decoded_level());
		break;
	case 2:
		lua_pushnumber(L, (lua_Number)app_adc_get_decoded_level2());
		break;
	default:
		return luaL_argerror(L, 1, "decoded adc channel must be 1 or 2");
	}
	return 1;
}

/* ----------------------------------------------------------------- ppm --- */

/*
 * The RC input, -1..1, mapped through the PPM app's configured pulse range.
 *
 * Starts the decoder if it is not already running, which is what lisp does
 * and what makes this work when the PPM app is not the selected one. That
 * side effect is worth knowing about: it stops the servo output, since the
 * two share the timer.
 *
 * The mapping is about the configured centre rather than the midpoint of the
 * range, so an asymmetric throttle reads 0 where the transmitter says centre.
 */
static int l_get_ppm(lua_State *L) {
	if (!servodec_is_running()) {
		pwm_servo_stop();
		servodec_init(0);
	}

	const ppm_config *cfg = &(app_get_configuration()->app_ppm_conf);

	servodec_set_pulse_options(cfg->pulse_start, cfg->pulse_end,
			cfg->median_filter);

	float val = servodec_get_servo(0);
	float ms = utils_map(val, -1.0f, 1.0f, cfg->pulse_start, cfg->pulse_end);

	if (ms < cfg->pulse_center) {
		val = utils_map(ms, cfg->pulse_start, cfg->pulse_center, -1.0f, 0.0f);
	} else {
		val = utils_map(ms, cfg->pulse_center, cfg->pulse_end, 0.0f, 1.0f);
	}

	lua_pushnumber(L, (lua_Number)val);
	return 1;
}

/*
 * Seconds since the last RC pulse.
 *
 * A script that drives the motor from the RC input must check this. The
 * decoder holds its last value, so a transmitter that is switched off, out of
 * range, or unplugged reads as whatever it last commanded -- full throttle
 * included -- and nothing about get_ppm says so.
 */
static int l_get_ppm_age(lua_State *L) {
	lua_pushnumber(L,
			(lua_Number)((float)servodec_get_time_since_update() / 1000.0f));
	return 1;
}

static int l_set_servo(lua_State *L) {
	float val = (float)luaL_checknumber(L, 1);

	pwm_servo_set_servo_out(val);
	return 0;
}

/* ------------------------------------------------------------- encoder --- */

static int l_get_encoder(lua_State *L) {
	lua_pushnumber(L, (lua_Number)encoder_read_deg());
	return 1;
}

/* ---------------------------------------------- taking over the throttle - */

/*
 * Stop the apps driving the motor for this many milliseconds, or ask whether
 * they are stopped.
 *
 * With no argument it reports rather than acts, so `if
 * vesc.app_disable_output() then` is a question and
 * `vesc.app_disable_output(500)` is an instruction. A negative time disables
 * indefinitely, which is what the app layer already means by it.
 */
static int l_app_disable_output(lua_State *L) {
	if (lua_isnoneornil(L, 1)) {
		lua_pushboolean(L, app_is_output_disabled() ? 1 : 0);
		return 1;
	}

	app_disable_output((int)luaL_checkinteger(L, 1));
	return 0;
}

/*
 * Detach the ADC app's input from the motor, so a script can feed it instead.
 *
 * 0 reattaches, 1 detaches the throttle, 2 detaches the buttons, 3 both --
 * lisp's numbering. app_adc_override then supplies the value the app would
 * have read.
 */
static int l_app_adc_detach(lua_State *L) {
	int mode = (int)luaL_checkinteger(L, 1);

	luaL_argcheck(L, (mode >= 0) && (mode <= 3), 1,
			"detach mode must be 0..3");

	app_adc_detach_adc((mode & 1) != 0 ? 1 : 0);
	app_adc_detach_buttons((mode & 2) != 0);
	return 0;
}

static int l_app_adc_override(lua_State *L) {
	float val = (float)luaL_checknumber(L, 1);

	app_adc_adc1_override(val);
	return 0;
}

static int l_app_ppm_detach(lua_State *L) {
	app_ppm_detach(lua_toboolean(L, 1) != 0);
	return 0;
}

static int l_app_ppm_override(lua_State *L) {
	float val = (float)luaL_checknumber(L, 1);

	app_ppm_override(val);
	return 0;
}

/* ---------------------------------------------------------------- table -- */

static const luaL_Reg io_fns[] = {
	{"get_adc", l_get_adc},
	{"get_adc_decoded", l_get_adc_decoded},

	{"get_ppm", l_get_ppm},
	{"get_ppm_age", l_get_ppm_age},
	{"set_servo", l_set_servo},

	{"get_encoder", l_get_encoder},

	{"app_disable_output", l_app_disable_output},
	{"app_adc_detach", l_app_adc_detach},
	{"app_adc_override", l_app_adc_override},
	{"app_ppm_detach", l_app_ppm_detach},
	{"app_ppm_override", l_app_ppm_override},

	{NULL, NULL}
};

const luaL_Reg *lua_vesc_io_fns(void) {
	return io_fns;
}
