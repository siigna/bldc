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
 * Motor bindings: the `vesc.*` functions a script uses to read the controller
 * and drive it.
 *
 * These mirror the lisp extensions in lispBM/lispif_vesc_extensions.c, units
 * and all, so a script ported between the two engines means the same thing.
 * Where lisp uses hyphens, Lua uses underscores -- get-rpm becomes
 * vesc.get_rpm -- because a hyphen is not valid in a Lua identifier. Note
 * the motor temperature is get_temp_mot, matching lisp's get-temp-mot rather
 * than reading better; a name that is nearly the same as the other engine's
 * is worse than one that is either identical or obviously different.
 *
 * Two have no lisp counterpart: get_tacho and get_tacho_abs. mc_interface
 * exposes both and lisp simply never bound them.
 *
 * Nothing here is reached for: the table is handed to script_lua_register by
 * the adapter, so the engine itself stays independent of the board.
 */

#include "lua_vesc_mc.h"

#include "lauxlib.h"

#include "mc_interface.h"
#include "timeout.h"

/* --------------------------------------------------------------- reading -- */

static int l_get_rpm(lua_State *L) {
	lua_pushnumber(L, (lua_Number)mc_interface_get_rpm());
	return 1;
}

static int l_get_current(lua_State *L) {
	lua_pushnumber(L, (lua_Number)mc_interface_get_tot_current_filtered());
	return 1;
}

static int l_get_current_dir(lua_State *L) {
	lua_pushnumber(L,
			(lua_Number)mc_interface_get_tot_current_directional_filtered());
	return 1;
}

static int l_get_current_in(lua_State *L) {
	lua_pushnumber(L, (lua_Number)mc_interface_get_tot_current_in_filtered());
	return 1;
}

static int l_get_duty(lua_State *L) {
	lua_pushnumber(L, (lua_Number)mc_interface_get_duty_cycle_now());
	return 1;
}

static int l_get_vin(lua_State *L) {
	lua_pushnumber(L, (lua_Number)mc_interface_get_input_voltage_filtered());
	return 1;
}

static int l_get_temp_fet(lua_State *L) {
	lua_pushnumber(L, (lua_Number)mc_interface_temp_fet_filtered());
	return 1;
}

static int l_get_temp_mot(lua_State *L) {
	lua_pushnumber(L, (lua_Number)mc_interface_temp_motor_filtered());
	return 1;
}

static int l_get_speed(lua_State *L) {
	lua_pushnumber(L, (lua_Number)mc_interface_get_speed());
	return 1;
}

static int l_get_dist(lua_State *L) {
	lua_pushnumber(L, (lua_Number)mc_interface_get_distance());
	return 1;
}

static int l_get_dist_abs(lua_State *L) {
	lua_pushnumber(L, (lua_Number)mc_interface_get_distance_abs());
	return 1;
}

/*
 * The counters take a reset flag, and it defaults to false.
 *
 * Defaulting the other way would make a plain reading destructive, so a
 * script that polls amp-hours for a display would silently zero the trip
 * meter -- and the same call in lisp does not.
 */
static int l_get_ah(lua_State *L) {
	bool reset = lua_toboolean(L, 1) != 0;
	lua_pushnumber(L, (lua_Number)mc_interface_get_amp_hours(reset));
	return 1;
}

static int l_get_ah_chg(lua_State *L) {
	bool reset = lua_toboolean(L, 1) != 0;
	lua_pushnumber(L, (lua_Number)mc_interface_get_amp_hours_charged(reset));
	return 1;
}

static int l_get_wh(lua_State *L) {
	bool reset = lua_toboolean(L, 1) != 0;
	lua_pushnumber(L, (lua_Number)mc_interface_get_watt_hours(reset));
	return 1;
}

static int l_get_wh_chg(lua_State *L) {
	bool reset = lua_toboolean(L, 1) != 0;
	lua_pushnumber(L, (lua_Number)mc_interface_get_watt_hours_charged(reset));
	return 1;
}

static int l_get_tacho(lua_State *L) {
	bool reset = lua_toboolean(L, 1) != 0;
	lua_pushinteger(L, (lua_Integer)mc_interface_get_tachometer_value(reset));
	return 1;
}

static int l_get_tacho_abs(lua_State *L) {
	bool reset = lua_toboolean(L, 1) != 0;
	lua_pushinteger(L,
			(lua_Integer)mc_interface_get_tachometer_abs_value(reset));
	return 1;
}

static int l_get_fault(lua_State *L) {
	lua_pushinteger(L, (lua_Integer)mc_interface_get_fault());
	return 1;
}

/* --------------------------------------------------------------- driving -- */

/*
 * Every setter refreshes the timeout first.
 *
 * This is not a formality. The motor timeout stops output if nothing has
 * refreshed it recently, which is what makes a script that crashes or stalls
 * safe -- the motor coasts instead of holding whatever it was last told. A
 * setter that skips it still works, right up until the first time a script
 * sets a current and then takes longer than the timeout to set the next one,
 * at which point the output drops out for no visible reason. The lisp
 * extensions do the same, in the same order.
 */

static int l_set_current(lua_State *L) {
	float current = (float)luaL_checknumber(L, 1);

	timeout_reset();
	if (!lua_isnoneornil(L, 2)) {
		mc_interface_set_current_off_delay((float)luaL_checknumber(L, 2));
	}
	mc_interface_set_current(current);
	return 0;
}

static int l_set_current_rel(lua_State *L) {
	float rel = (float)luaL_checknumber(L, 1);

	timeout_reset();
	if (!lua_isnoneornil(L, 2)) {
		mc_interface_set_current_off_delay((float)luaL_checknumber(L, 2));
	}
	mc_interface_set_current_rel(rel);
	return 0;
}

static int l_set_duty(lua_State *L) {
	float duty = (float)luaL_checknumber(L, 1);

	timeout_reset();
	mc_interface_set_duty(duty);
	return 0;
}

static int l_set_rpm(lua_State *L) {
	float rpm = (float)luaL_checknumber(L, 1);

	timeout_reset();
	mc_interface_set_pid_speed(rpm);
	return 0;
}

static int l_set_pos(lua_State *L) {
	float pos = (float)luaL_checknumber(L, 1);

	timeout_reset();
	mc_interface_set_pid_pos(pos);
	return 0;
}

static int l_set_brake(lua_State *L) {
	float current = (float)luaL_checknumber(L, 1);

	timeout_reset();
	mc_interface_set_brake_current(current);
	return 0;
}

static int l_set_brake_rel(lua_State *L) {
	float rel = (float)luaL_checknumber(L, 1);

	timeout_reset();
	mc_interface_set_brake_current_rel(rel);
	return 0;
}

static int l_set_handbrake(lua_State *L) {
	float current = (float)luaL_checknumber(L, 1);

	timeout_reset();
	mc_interface_set_handbrake(current);
	return 0;
}

static int l_set_handbrake_rel(lua_State *L) {
	float rel = (float)luaL_checknumber(L, 1);

	timeout_reset();
	mc_interface_set_handbrake_rel(rel);
	return 0;
}

static int l_release_motor(lua_State *L) {
	(void)L;
	timeout_reset();
	mc_interface_release_motor();
	return 0;
}

static int l_timeout_reset(lua_State *L) {
	(void)L;
	timeout_reset();
	return 0;
}

/* ---------------------------------------------------------------- table --- */

static const luaL_Reg mc_fns[] = {
	{"get_rpm", l_get_rpm},
	{"get_current", l_get_current},
	{"get_current_dir", l_get_current_dir},
	{"get_current_in", l_get_current_in},
	{"get_duty", l_get_duty},
	{"get_vin", l_get_vin},
	{"get_temp_fet", l_get_temp_fet},
	{"get_temp_mot", l_get_temp_mot},
	{"get_speed", l_get_speed},
	{"get_dist", l_get_dist},
	{"get_dist_abs", l_get_dist_abs},
	{"get_ah", l_get_ah},
	{"get_ah_chg", l_get_ah_chg},
	{"get_wh", l_get_wh},
	{"get_wh_chg", l_get_wh_chg},
	{"get_tacho", l_get_tacho},
	{"get_tacho_abs", l_get_tacho_abs},
	{"get_fault", l_get_fault},

	{"set_current", l_set_current},
	{"set_current_rel", l_set_current_rel},
	{"set_duty", l_set_duty},
	{"set_rpm", l_set_rpm},
	{"set_pos", l_set_pos},
	{"set_brake", l_set_brake},
	{"set_brake_rel", l_set_brake_rel},
	{"set_handbrake", l_set_handbrake},
	{"set_handbrake_rel", l_set_handbrake_rel},
	{"release_motor", l_release_motor},
	{"timeout_reset", l_timeout_reset},

	{NULL, NULL}
};

const luaL_Reg *lua_vesc_mc_fns(void) {
	return mc_fns;
}
