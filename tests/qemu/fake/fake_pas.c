/*
 * Stand-in pedal-assist bindings.
 *
 * script/lua_vesc_pas.c cannot be linked into these images for the same
 * reason as lua_vesc_io.c: it includes app.h, and the real app_pas.c behind
 * it needs the HAL this harness deliberately does not build.
 *
 * These have the same names and shapes and read values a test sets, which is
 * enough to exercise a script's logic end to end -- a script that reacts to
 * cadence, or that drives the walk-assist keepalive, can be run here with no
 * board. What it does not cover is app_pas.c itself; that is tests/app_pas,
 * which compiles the real file against stubs.
 *
 * Test scaffolding; never built into firmware.
 */

#include "lua_vesc_pas.h"
#include "fake_pas.h"

fake_pas_t fake_pas;

void fake_pas_reset(void) {
	fake_pas.rpm = 0.0f;
	fake_pas.torque_nm = 0.0f;
	fake_pas.torque_ratio = 0.0f;
	fake_pas.rider_power = 0.0f;
	fake_pas.assist_power = 0.0f;
	fake_pas.assist_basis_power = 0.0f;
	fake_pas.measured_power = 0.0f;
	fake_pas.output = 0.0f;
	fake_pas.speed_taper = 1.0f;
	fake_pas.flags = 0;
	fake_pas.torque_saturated = false;
	fake_pas.walk_requested = false;
	fake_pas.walk_calls = 0;
}

#define GETTER(name, field)                               \
	static int name(lua_State *L) {                       \
		lua_pushnumber(L, (lua_Number)fake_pas.field);    \
		return 1;                                         \
	}

GETTER(l_rpm, rpm)
GETTER(l_torque, torque_nm)
GETTER(l_torque_ratio, torque_ratio)
GETTER(l_rider_power, rider_power)
GETTER(l_assist_power, assist_power)
GETTER(l_assist_basis_power, assist_basis_power)
GETTER(l_measured_power, measured_power)
GETTER(l_output, output)
GETTER(l_speed_taper, speed_taper)

static int l_flags(lua_State *L) {
	lua_pushinteger(L, (lua_Integer)fake_pas.flags);
	return 1;
}

static int l_torque_saturated(lua_State *L) {
	lua_pushboolean(L, fake_pas.torque_saturated ? 1 : 0);
	return 1;
}

static int l_walk_set(lua_State *L) {
	fake_pas.walk_requested = lua_toboolean(L, 1) != 0;
	fake_pas.walk_calls++;
	return 0;
}

static const luaL_Reg pas_fns[] = {
	{"pas_get_rpm",                l_rpm},
	{"pas_get_torque",             l_torque},
	{"pas_get_torque_ratio",       l_torque_ratio},
	{"pas_get_rider_power",        l_rider_power},
	{"pas_get_assist_power",       l_assist_power},
	{"pas_get_assist_basis_power", l_assist_basis_power},
	{"pas_get_measured_power",     l_measured_power},
	{"pas_get_output",             l_output},
	{"pas_get_speed_taper",        l_speed_taper},
	{"pas_get_flags",              l_flags},
	{"pas_torque_saturated",       l_torque_saturated},
	{"pas_walk_set",               l_walk_set},
	{NULL, NULL}
};

const luaL_Reg *lua_vesc_pas_fns(void) {
	return pas_fns;
}
