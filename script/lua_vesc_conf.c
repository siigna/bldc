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
 * Reading and writing go through generated code that names the struct member,
 * so the compiler chooses the load and the store.
 *
 * The first version of this used a table of byte offsets plus a width
 * classification, and got the widths wrong in a way that only a target could
 * show: uint8_t and uint16_t members were read four bytes wide, which is both
 * unaligned -- a HardFault with UFSR.UNALIGNED on a Cortex-M4, since a float
 * load uses VLDR and that has no unaligned form -- and overlapping the
 * neighbouring fields. arm-none-eabi also defaults to -fshort-enums, so an
 * enum's width is not knowable from its declaration, which makes the whole
 * approach unfixable rather than merely buggy.
 *
 * The cost is a strcmp chain, which is what the lisp extensions do as well,
 * and conf_get is not on any hot path.
 */

static bool conf_read(const char *name, float *out) {
	const mc_configuration *mc =
			(const mc_configuration *)mc_interface_get_configuration();
	const app_configuration *app = app_get_configuration();

#define X(nm, fl, field) \
	if (strcmp(name, nm) == 0) { *out = (float)mc->field; return true; }
	CONF_MC_PARAMS(X)
#undef X

#define X(nm, fl, field) \
	if (strcmp(name, nm) == 0) { *out = (float)app->field; return true; }
	CONF_APP_PARAMS(X)
#undef X

	return false;
}

static void conf_write_mc(mc_configuration *mc, const char *name, float v) {
#define X(nm, fl, field)                                                     \
	if (strcmp(name, nm) == 0) {                                             \
		mc->field = (((fl) & CONF_NEG_ABS) != 0)                             \
				? (__typeof__(mc->field)) - fabsf(v)                         \
				: (__typeof__(mc->field))v;                                  \
		return;                                                              \
	}
	CONF_MC_PARAMS(X)
#undef X
}

static void conf_write_app(app_configuration *app, const char *name, float v) {
#define X(nm, fl, field)                                                     \
	if (strcmp(name, nm) == 0) {                                             \
		app->field = (((fl) & CONF_NEG_ABS) != 0)                            \
				? (__typeof__(app->field)) - fabsf(v)                        \
				: (__typeof__(app->field))v;                                 \
		return;                                                              \
	}
	CONF_APP_PARAMS(X)
#undef X
}

/* Which configuration a name belongs to, and how it has to be applied. */
static bool conf_lookup(const char *name, bool *is_mc, uint8_t *flags) {
#define X(nm, fl, field)                                                     \
	if (strcmp(name, nm) == 0) { *is_mc = true; *flags = (fl); return true; }
	CONF_MC_PARAMS(X)
#undef X

#define X(nm, fl, field)                                                     \
	if (strcmp(name, nm) == 0) { *is_mc = false; *flags = (fl); return true; }
	CONF_APP_PARAMS(X)
#undef X

	return false;
}

/* Every name, for conf_names and for a test to walk. */
static void conf_each_name(void (*fn)(const char *, void *), void *ud) {
#define X(nm, fl, field) fn(nm, ud);
	CONF_MC_PARAMS(X)
	CONF_APP_PARAMS(X)
#undef X
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

	if (conf_read(name, &value) || conf_scaled_read(mc, name, &value)) {
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
static void do_write_mc(mc_configuration *mc, const char *name, float v) {
	if (conf_scaled_write(mc, name, v)) {
		return;
	}
	if (strcmp(name, "min_speed") == 0) {
		mc->l_min_erpm = -fabsf(v) * speed_factor(mc);
		return;
	}
	if (strcmp(name, "max_speed") == 0) {
		mc->l_max_erpm = v * speed_factor(mc);
		return;
	}
	conf_write_mc(mc, name, v);
}

static int l_conf_set(lua_State *L) {
	const char *name = luaL_checkstring(L, 1);
	float value = (float)luaL_checknumber(L, 2);

	bool is_mc = true;
	uint8_t flags = CONF_PLAIN;
	bool known = conf_lookup(name, &is_mc, &flags);

	if (!known) {
		const mc_configuration *probe =
				(const mc_configuration *)mc_interface_get_configuration();
		float ignored;

		if (conf_scaled_read(probe, name, &ignored)) {
			/* every scaled constant is a FOC parameter, and lisp applies
			 * all of those through a reconfigure */
			known = true;
			is_mc = true;
			flags = CONF_APPLY;
		} else if (is_speed_name(name)) {
			/* these end up in l_min_erpm and l_max_erpm, which lisp writes
			 * live rather than reconfiguring for */
			known = true;
			is_mc = true;
			flags = CONF_PLAIN;
		}
	}

	if (!known) {
		lua_pushboolean(L, 0);
		return 1;
	}

	/*
	 * Two paths, exactly as lisp has them. Most parameters take effect on a
	 * write to the live configuration; 82 of these need a full reconfigure,
	 * and writing one of those through the fast path leaves it looking set
	 * without having taken effect.
	 */
	if ((flags & CONF_APPLY) == 0) {
		if (is_mc) {
			mc_configuration *mc =
					(mc_configuration *)mc_interface_get_configuration();
			do_write_mc(mc, name, value);
			commands_apply_mcconf_hw_limits(mc);
		} else {
			app_configuration *app =
					(app_configuration *)app_get_configuration();
			conf_write_app(app, name, value);
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
		do_write_mc(mc, name, value);
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
		conf_write_app(app, name, value);
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
typedef struct {
	lua_State *L;
	int n;
} name_push_t;

static void push_name(const char *name, void *ud) {
	name_push_t *p = (name_push_t *)ud;

	lua_pushstring(p->L, name);
	lua_rawseti(p->L, -2, p->n++);
}

static int l_conf_names(lua_State *L) {
	name_push_t p = {L, 1};

	lua_newtable(L);
	conf_each_name(push_name, &p);

#define X(nm, sc, field) push_name(nm, &p);
	CONF_SCALED(X)
#undef X

	push_name("min_speed", &p);
	push_name("max_speed", &p);
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
