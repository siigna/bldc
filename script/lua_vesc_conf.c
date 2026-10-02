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
 * vesc.conf_get and vesc.conf_set: the motor and app configuration, by name.
 *
 * The lisp equivalents are a pair of 500-line if-else chains, one for reading
 * and one for writing. This is a single table instead, which is both smaller
 * and the only way the two directions cannot drift apart -- and the table is
 * generated from those chains rather than retyped, because a parameter wired
 * to the field next to the right one reads and writes perfectly consistently
 * and is invisible to any round-trip test.
 *
 * Names match lisp with hyphens replaced by underscores: conf-get 'l-current-max
 * becomes vesc.conf_get("l_current_max").
 */

#include "lua_vesc_conf.h"

#include "lauxlib.h"

#include <stddef.h>
#include <math.h>
#include <string.h>

#include "datatypes.h"
#include "mc_interface.h"
#include "app.h"
#include "commands.h"
#include "mempools.h"

/* Flags, derived from the lisp conf-set arm. See lua_vesc_conf_table.h. */
#define CONF_PLAIN		0x00
#define CONF_NEG_ABS	0x01
#define CONF_APPLY		0x02

#include "lua_vesc_conf_table.h"

/*
 * Reading and writing go through generated switches that name the struct
 * member, so the compiler chooses the load and the store.
 *
 * Two earlier designs were wrong, in different ways:
 *
 * A table of byte offsets plus a width classification HardFaulted with
 * UFSR.UNALIGNED. uint8_t and uint16_t members were classified as int and
 * read four bytes wide -- both unaligned and overlapping the neighbours --
 * and a float read through a cast compiles to VLDR, which has no unaligned
 * form on a Cortex-M4. arm-none-eabi also defaults to -fshort-enums, so an
 * enum's width is not knowable from its declaration, which makes offsets
 * unfixable rather than merely buggy.
 *
 * Naming the member fixed that but was then written as a strcmp chain per
 * operation -- read, write and flags lookup, each for both configurations.
 * Six chains over 136 names inlined a strcmp call site per name per chain and
 * came to 16,410 bytes, the third largest object in the firmware, larger than
 * mc_interface.o. So the name is resolved once to an index by a single loop,
 * and everything after that is a switch.
 */

/*
 * One index space, motor parameters first, matching the order the name and
 * flag arrays are built in. No marker enumerator between the two lists: an
 * earlier draft had one assigned CONF_MC_PARAM_COUNT, which restarts the
 * implicit numbering and pushed every app index one past its slot in
 * m_names, leaving one parameter reading zero and one switch case
 * unreachable.
 */
enum {
#define X(nm, fl, field) CONF_IDX_##nm,
	CONF_MC_PARAMS(X)
	CONF_APP_PARAMS(X)
#undef X
	CONF_IDX_COUNT
};

_Static_assert(CONF_IDX_COUNT == CONF_MC_PARAM_COUNT + CONF_APP_PARAM_COUNT,
		"the index space and the generated counts disagree");

static const char *const m_names[] = {
#define X(nm, fl, field) #nm,
	CONF_MC_PARAMS(X)
	CONF_APP_PARAMS(X)
#undef X
};

static const uint8_t m_flags[] = {
#define X(nm, fl, field) (fl),
	CONF_MC_PARAMS(X)
	CONF_APP_PARAMS(X)
#undef X
};

_Static_assert(sizeof(m_names) / sizeof(m_names[0])
		== (CONF_MC_PARAM_COUNT + CONF_APP_PARAM_COUNT),
		"the name array and the generated counts disagree");
_Static_assert(sizeof(m_flags) / sizeof(m_flags[0])
		== (CONF_MC_PARAM_COUNT + CONF_APP_PARAM_COUNT),
		"the flag array and the generated counts disagree");

/*
 * A linear scan, not a binary search. One loop of 136 strcmps is a few dozen
 * bytes of code; a sorted-index array would be faster and bigger, and
 * conf_get is on no hot path. Returns -1 for an unknown name.
 */
static int index_of(const char *name) {
	for (int i = 0; i < (int)(sizeof(m_names) / sizeof(m_names[0])); i++) {
		if (strcmp(m_names[i], name) == 0) {
			return i;
		}
	}
	return -1;
}

static bool is_mc_index(int idx) {
	return idx < CONF_MC_PARAM_COUNT;
}

static float read_index(int idx, const mc_configuration *mc,
		const app_configuration *app) {
	switch (idx) {
#define X(nm, fl, field) case CONF_IDX_##nm: return (float)mc->field;
	CONF_MC_PARAMS(X)
#undef X
#define X(nm, fl, field) case CONF_IDX_##nm: return (float)app->field;
	CONF_APP_PARAMS(X)
#undef X
	default:
		return 0.0f;
	}
}

static void write_index(int idx, mc_configuration *mc,
		app_configuration *app, float v) {
	if ((m_flags[idx] & CONF_NEG_ABS) != 0) {
		v = -fabsf(v);
	}

	switch (idx) {
#define X(nm, fl, field) \
	case CONF_IDX_##nm: mc->field = (__typeof__(mc->field))v; return;
	CONF_MC_PARAMS(X)
#undef X
#define X(nm, fl, field) \
	case CONF_IDX_##nm: app->field = (__typeof__(app->field))v; return;
	CONF_APP_PARAMS(X)
#undef X
	default:
		return;
	}
}

/* ------------------------------------------------------------- specials -- */

/*
 * The seven parameters lisp scales or computes rather than reading out of the
 * struct. By hand because each is its own conversion, and kept out of the
 * generated lists so the generator can refuse anything it does not recognise
 * instead of guessing at it.
 */

static float speed_factor(const mc_configuration *mc) {
	return ((mc->si_motor_poles / 2.0f) * 60.0f * mc->si_gear_ratio)
			/ (mc->si_wheel_diameter * (float)M_PI);
}

/*
 * The five FOC constants lisp scales. Named members rather than offsets, for
 * the same reason as the generated lists.
 */
#define CONF_SCALED(X)                                                       \
	X("foc_motor_l", 1e-6f, foc_motor_l)                                     \
	X("foc_motor_ld_lq_diff", 1e-6f, foc_motor_ld_lq_diff)                   \
	X("foc_motor_r", 1e-3f, foc_motor_r)                                     \
	X("foc_motor_flux_linkage", 1e-3f, foc_motor_flux_linkage)               \
	X("foc_observer_gain", 1e6f, foc_observer_gain)

static bool conf_scaled_read(const mc_configuration *mc, const char *name,
		float *out) {
#define X(nm, sc, field) \
	if (strcmp(name, nm) == 0) { *out = mc->field / (sc); return true; }
	CONF_SCALED(X)
#undef X
	return false;
}

static bool conf_scaled_write(mc_configuration *mc, const char *name,
		float v) {
#define X(nm, sc, field) \
	if (strcmp(name, nm) == 0) { mc->field = v * (sc); return true; }
	CONF_SCALED(X)
#undef X
	return false;
}

static bool is_speed_name(const char *name) {
	return (strcmp(name, "min_speed") == 0)
			|| (strcmp(name, "max_speed") == 0);
}

/* ------------------------------------------------------------- bindings -- */

static int l_conf_get(lua_State *L) {
	const char *name = luaL_checkstring(L, 1);
	const mc_configuration *mc =
			(const mc_configuration *)mc_interface_get_configuration();
	float value = 0.0f;
	int idx = index_of(name);

	if (idx >= 0) {
		lua_pushnumber(L, (lua_Number)read_index(idx, mc,
				app_get_configuration()));
		return 1;
	}

	if (conf_scaled_read(mc, name, &value)) {
		lua_pushnumber(L, (lua_Number)value);
		return 1;
	}

	if (strcmp(name, "min_speed") == 0) {
		lua_pushnumber(L, (lua_Number)(mc->l_min_erpm / speed_factor(mc)));
		return 1;
	}
	if (strcmp(name, "max_speed") == 0) {
		lua_pushnumber(L, (lua_Number)(mc->l_max_erpm / speed_factor(mc)));
		return 1;
	}

	/*
	 * nil rather than an error, and nil rather than 0.
	 *
	 * An unknown name is usually a typo, and 0 would read as a real setting
	 * -- a script checking `if vesc.conf_get("l_current_mx") > 50` would
	 * simply take the other branch forever. nil makes the arithmetic fail
	 * where the typo is.
	 */
	lua_pushnil(L);
	return 1;
}

/* Perform the write, whichever kind of parameter it is. */
static void do_write(int idx, mc_configuration *mc, app_configuration *app,
		const char *name, float v) {
	if (idx >= 0) {
		write_index(idx, mc, app, v);
		return;
	}
	if (conf_scaled_write(mc, name, v)) {
		return;
	}
	if (strcmp(name, "min_speed") == 0) {
		mc->l_min_erpm = -fabsf(v) * speed_factor(mc);
		return;
	}
	mc->l_max_erpm = v * speed_factor(mc);
}

static int l_conf_set(lua_State *L) {
	const char *name = luaL_checkstring(L, 1);
	float value = (float)luaL_checknumber(L, 2);

	int idx = index_of(name);
	bool is_mc;
	bool needs_apply;

	if (idx >= 0) {
		is_mc = is_mc_index(idx);
		needs_apply = (m_flags[idx] & CONF_APPLY) != 0;
	} else {
		const mc_configuration *probe =
				(const mc_configuration *)mc_interface_get_configuration();
		float ignored;

		is_mc = true;
		if (conf_scaled_read(probe, name, &ignored)) {
			/* every scaled constant is a FOC parameter, and lisp applies all
			 * of those through a reconfigure */
			needs_apply = true;
		} else if (is_speed_name(name)) {
			/* these end up in l_min_erpm and l_max_erpm, which lisp writes
			 * live rather than reconfiguring for */
			needs_apply = false;
		} else {
			lua_pushboolean(L, 0);
			return 1;
		}
	}

	/*
	 * Two paths, exactly as lisp has them. Most parameters take effect on a
	 * write to the live configuration; 82 of these need a full reconfigure,
	 * and writing one of those through the fast path leaves it looking set
	 * without having taken effect.
	 */
	if (!needs_apply) {
		if (is_mc) {
			mc_configuration *mc =
					(mc_configuration *)mc_interface_get_configuration();
			do_write(idx, mc, NULL, name, value);
			commands_apply_mcconf_hw_limits(mc);
		} else {
			app_configuration *app =
					(app_configuration *)app_get_configuration();
			do_write(idx, NULL, app, name, value);
			commands_apply_appconf_hw_limits(app);
		}
		lua_pushboolean(L, 1);
		return 1;
	}

	if (is_mc) {
		mc_configuration *mc = mempools_alloc_mcconf();

		if (mc == NULL) {
			lua_pushboolean(L, 0);
			return 1;
		}
		*mc = *mc_interface_get_configuration();
		do_write(idx, mc, NULL, name, value);
		commands_apply_mcconf_hw_limits(mc);
		mc_interface_set_configuration(mc);
		mempools_free_mcconf(mc);
	} else {
		app_configuration *app = mempools_alloc_appconf();

		if (app == NULL) {
			lua_pushboolean(L, 0);
			return 1;
		}
		*app = *app_get_configuration();
		do_write(idx, NULL, app, name, value);
		commands_apply_appconf_hw_limits(app);
		app_set_configuration(app);
		mempools_free_appconf(app);
	}

	lua_pushboolean(L, 1);
	return 1;
}

/*
 * Every name the two above accept, so a script can discover them and a test
 * can walk all of them rather than sampling.
 */
static int l_conf_names(lua_State *L) {
	int n = 1;

	lua_newtable(L);
	for (size_t i = 0; i < sizeof(m_names) / sizeof(m_names[0]); i++) {
		lua_pushstring(L, m_names[i]);
		lua_rawseti(L, -2, n++);
	}

#define X(nm, sc, field) \
	lua_pushstring(L, nm); \
	lua_rawseti(L, -2, n++);
	CONF_SCALED(X)
#undef X

	lua_pushstring(L, "min_speed");
	lua_rawseti(L, -2, n++);
	lua_pushstring(L, "max_speed");
	lua_rawseti(L, -2, n++);
	return 1;
}

static const luaL_Reg conf_fns[] = {
	{"conf_get", l_conf_get},
	{"conf_set", l_conf_set},
	{"conf_names", l_conf_names},
	{NULL, NULL}
};

const luaL_Reg *lua_vesc_conf_fns(void) {
	return conf_fns;
}
