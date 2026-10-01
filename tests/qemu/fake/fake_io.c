/*
 * A stub for the input bindings.
 *
 * script/lua_vesc_io.c cannot be linked into these images: it includes
 * conf_general.h for ADC_VOLTS, which pulls in a board header and from there
 * the whole ChibiOS HAL, and this harness is kernel-only by design -- there
 * is no HAL and no STM32 LLD in any of these test images.
 *
 * So test_luaif registers an empty table where the real one would go. What
 * that costs is honest to state: the input bindings are not exercised here at
 * all. The one piece of them with logic rather than a one-line wrapper is the
 * ADC channel mapping, and that is tested in tests/lua_adc, which cuts the
 * function out of the real source so it cannot drift.
 *
 * Test scaffolding; never built into firmware.
 */

#include "lua_vesc_io.h"

static const luaL_Reg none[] = {
	{NULL, NULL}
};

const luaL_Reg *lua_vesc_io_fns(void) {
	return none;
}
