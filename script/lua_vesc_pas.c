/*
	Copyright 2026 Stephen Bouche

	This file is part of the ESCargot firmware.

	The ESCargot firmware is free software: you can redistribute it and/or modify
	it under the terms of the GNU General Public License as published by
	the Free Software Foundation, either version 3 of the License, or
	(at your option) any later version.

	The ESCargot firmware is distributed in the hope that it will be useful,
	but WITHOUT ANY WARRANTY; without even the implied warranty of
	MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
	GNU General Public License for more details.

	You should have received a copy of the GNU General Public License
	along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

/*
 * Pedal assist, for Lua. One-for-one with the app-pas-* extensions LispBM
 * has had since the PAS work landed -- the same C getters in applications/
 * app.h, the same names with hyphens turned into underscores.
 *
 * Without these a Lua script could set eleven PAS parameters through
 * vesc.conf_set and then read nothing back: not cadence, not torque, not what
 * the assist was doing. Anything that had to react to pedalling could only be
 * written in the other language.
 */

#include "lua_vesc_pas.h"

#include "app.h"

static int l_pas_get_rpm(lua_State *L) {
	lua_pushnumber(L, (lua_Number)app_pas_get_pedal_rpm());
	return 1;
}

static int l_pas_get_torque(lua_State *L) {
	lua_pushnumber(L, (lua_Number)app_pas_get_torque_nm());
	return 1;
}

static int l_pas_get_torque_ratio(lua_State *L) {
	lua_pushnumber(L, (lua_Number)app_pas_get_torque_ratio());
	return 1;
}

static int l_pas_get_rider_power(lua_State *L) {
	lua_pushnumber(L, (lua_Number)app_pas_get_rider_power());
	return 1;
}

/*
 * Named for what it returns, which is the motor power target. The lisp name
 * for this is app-pas-get-assist-power; app_pas_get_assist_basis_power is a
 * different number and is bound separately below.
 */
static int l_pas_get_assist_power(lua_State *L) {
	lua_pushnumber(L, (lua_Number)app_pas_get_motor_power_target());
	return 1;
}

static int l_pas_get_assist_basis_power(lua_State *L) {
	lua_pushnumber(L, (lua_Number)app_pas_get_assist_basis_power());
	return 1;
}

static int l_pas_get_measured_power(lua_State *L) {
	lua_pushnumber(L, (lua_Number)app_pas_get_measured_power());
	return 1;
}

static int l_pas_get_output(lua_State *L) {
	lua_pushnumber(L, (lua_Number)app_pas_get_current_target_rel());
	return 1;
}

static int l_pas_get_speed_taper(lua_State *L) {
	lua_pushnumber(L, (lua_Number)app_pas_get_speed_taper());
	return 1;
}

static int l_pas_get_flags(lua_State *L) {
	lua_pushinteger(L, (lua_Integer)app_pas_get_flags());
	return 1;
}

static int l_pas_torque_saturated(lua_State *L) {
	lua_pushboolean(L, app_pas_torque_saturated() ? 1 : 0);
	return 1;
}

/*
 * Walk assist on or off.
 *
 * A keepalive, not a latch: it expires after half a second, so it has to be
 * called repeatedly for as long as the button is held. A script that stops
 * running therefore releases the motor instead of leaving it driving. See the
 * comment on app_pas_walk_set.
 */
static int l_pas_walk_set(lua_State *L) {
	app_pas_walk_set(lua_toboolean(L, 1) != 0);
	return 0;
}

static const luaL_Reg m_fns[] = {
	{"pas_get_rpm",                l_pas_get_rpm},
	{"pas_get_torque",             l_pas_get_torque},
	{"pas_get_torque_ratio",       l_pas_get_torque_ratio},
	{"pas_get_rider_power",        l_pas_get_rider_power},
	{"pas_get_assist_power",       l_pas_get_assist_power},
	{"pas_get_assist_basis_power", l_pas_get_assist_basis_power},
	{"pas_get_measured_power",     l_pas_get_measured_power},
	{"pas_get_output",             l_pas_get_output},
	{"pas_get_speed_taper",        l_pas_get_speed_taper},
	{"pas_get_flags",              l_pas_get_flags},
	{"pas_torque_saturated",       l_pas_torque_saturated},
	{"pas_walk_set",               l_pas_walk_set},
	{NULL, NULL}
};

const luaL_Reg *lua_vesc_pas_fns(void) {
	return m_fns;
}
