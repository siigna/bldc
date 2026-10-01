/*
 * Stand-in input bindings.
 *
 * script/lua_vesc_io.c cannot be linked into these images: it includes
 * conf_general.h for ADC_VOLTS, which pulls in a board header and from there
 * the whole ChibiOS HAL -- hal_lld.h, then pal_lld.h, then one per driver --
 * and this harness is kernel-only by design.
 *
 * So these are *not* the real bindings. They have the same names and shapes
 * and read values a test sets, which is enough to exercise a script's logic
 * end to end. What that does not cover is the bindings themselves; the one
 * part of them with logic rather than a wrapper is the ADC channel mapping,
 * and tests/lua_adc cuts that out of the real source.
 *
 * Test scaffolding; never built into firmware.
 */

#include "lua_vesc_io.h"
#include "fake_io.h"

fake_io_t fake_io;

void fake_io_reset(void) {
	fake_io.ppm = 0.0f;
	fake_io.ppm_age = 0.0f;
	fake_io.ppm_age_is_nil = false;
	fake_io.adc_volts = 0.0f;
	fake_io.adc_is_nil = false;
	fake_io.servo_out = 0.0f;
	fake_io.encoder_deg = 0.0f;
	fake_io.disable_output_ms = -1;
	fake_io.disable_output_calls = 0;
	fake_io.output_disabled = false;
}

static int l_get_ppm(lua_State *L) {
	lua_pushnumber(L, (lua_Number)fake_io.ppm);
	return 1;
}

static int l_get_ppm_age(lua_State *L) {
	if (fake_io.ppm_age_is_nil) {
		lua_pushnil(L);
	} else {
		lua_pushnumber(L, (lua_Number)fake_io.ppm_age);
	}
	return 1;
}

/* nil for a channel the board does not have, as the real one does */
static int l_get_adc(lua_State *L) {
	(void)luaL_optinteger(L, 1, 0);
	if (fake_io.adc_is_nil) {
		lua_pushnil(L);
	} else {
		lua_pushnumber(L, (lua_Number)fake_io.adc_volts);
	}
	return 1;
}

static int l_get_adc_decoded(lua_State *L) {
	(void)luaL_optinteger(L, 1, 1);
	lua_pushnumber(L, 0.0);
	return 1;
}

static int l_set_servo(lua_State *L) {
	fake_io.servo_out = (float)luaL_checknumber(L, 1);
	return 0;
}

static int l_get_encoder(lua_State *L) {
	lua_pushnumber(L, (lua_Number)fake_io.encoder_deg);
	return 1;
}

static int l_app_disable_output(lua_State *L) {
	if (lua_isnoneornil(L, 1)) {
		lua_pushboolean(L, fake_io.output_disabled ? 1 : 0);
		return 1;
	}
	fake_io.disable_output_ms = (int)luaL_checkinteger(L, 1);
	fake_io.disable_output_calls++;
	fake_io.output_disabled = true;
	return 0;
}

static int l_noop(lua_State *L) {
	(void)L;
	return 0;
}

static const luaL_Reg io_fns[] = {
	{"get_adc", l_get_adc},
	{"get_adc_decoded", l_get_adc_decoded},
	{"get_ppm", l_get_ppm},
	{"get_ppm_age", l_get_ppm_age},
	{"set_servo", l_set_servo},
	{"get_encoder", l_get_encoder},
	{"app_disable_output", l_app_disable_output},
	{"app_adc_detach", l_noop},
	{"app_adc_override", l_noop},
	{"app_ppm_detach", l_noop},
	{"app_ppm_override", l_noop},
	{NULL, NULL}
};

const luaL_Reg *lua_vesc_io_fns(void) {
	return io_fns;
}
