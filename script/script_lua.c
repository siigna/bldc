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

#include "script_lua.h"

#include "lauxlib.h"
#include "lualib.h"

#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include <stddef.h>

struct script_lua {
	lua_State *L;
	script_lua_cfg_t cfg;
	size_t mem_used;
	size_t mem_peak;
	uint32_t timer_period_ms;

	/*
	 * Plain flags mirroring which handlers are registered.
	 *
	 * script_lua_wants is called from the CAN and comms tasks to decide
	 * whether an event is worth queueing, and it must not touch the
	 * interpreter to answer: lua_getfield manipulates the Lua stack, and
	 * doing that from another task while the engine task is inside a pcall
	 * corrupts it. These are written only from the engine task, while it is
	 * inside the registering call, and read from anywhere.
	 *
	 * volatile, not atomic, and that is sufficient here: each is a single
	 * byte written with a constant and read independently, so a reader sees
	 * either the old or the new value and both are safe -- a stale false
	 * drops one event, a stale true queues one nobody reads.
	 */
	volatile uint8_t have_can;
	volatile uint8_t have_app_data;
	volatile uint8_t have_timer;
	volatile uint8_t have_touch;
};

/*
 * Allocator with a ceiling.
 *
 * Lua routes every allocation through here and treats a NULL return as an
 * out-of-memory error it can raise and a pcall can catch. That is the whole
 * mechanism for containing a runaway script: it hits its own limit and dies
 * with a traceback, rather than exhausting the heap that the comms stack and
 * the rest of the firmware are sharing.
 *
 * The accounting is exact rather than sampled because it is also what the
 * GET_STATS reply reports, and a script author tuning memory wants a number
 * that means something.
 */
static void *l_alloc(void *ud, void *ptr, size_t osize, size_t nsize) {
	script_lua_t *s = (script_lua_t *)ud;

	if (nsize == 0) {
		if (s->cfg.alloc) {
			s->cfg.alloc(s->cfg.alloc_ud, ptr, osize, 0);
		} else {
			free(ptr);
		}
		// Clamped rather than wrapped: an underflowed total would read as
		// enormous and wedge the ceiling shut for the rest of the run.
		s->mem_used = (s->mem_used >= osize) ? (s->mem_used - osize) : 0;
		return NULL;
	}

	if (s->cfg.mem_limit > 0) {
		size_t after = s->mem_used + nsize - osize;
		// Refuse growth past the ceiling. Shrinking is always allowed, and
		// checking `nsize > osize` first keeps a shrink from tripping the
		// limit when usage is already over it after a limit change.
		if (nsize > osize && after > s->cfg.mem_limit) {
			return NULL;
		}
	}

	void *np = s->cfg.alloc
			? s->cfg.alloc(s->cfg.alloc_ud, ptr, osize, nsize)
			: realloc(ptr, nsize);
	if (!np) {
		return NULL;
	}

	s->mem_used = s->mem_used + nsize - osize;
	if (s->mem_used > s->mem_peak) {
		s->mem_peak = s->mem_used;
	}
	return np;
}

/*
 * Instruction hook. Runs every cfg.hook_count VM instructions.
 *
 * This is the equivalent of lbm_set_eval_step_quota in the lisp engine, and
 * it is what makes a script interruptible at all: without it, `while true do
 * end` is unkillable short of a watchdog reset, and stopping a script from
 * VESC Tool would not work.
 *
 * luaL_error raises, which unwinds to the enclosing pcall in script_lua_run.
 * That is deliberate -- killing the task instead would leak whatever the
 * script held and leave the interpreter unusable without a full restart.
 */
static void l_hook(lua_State *L, lua_Debug *ar) {
	(void)ar;
	script_lua_t *s = NULL;
	lua_getallocf(L, (void **)&s);
	if (!s) {
		return;
	}

	if (s->cfg.on_tick) {
		s->cfg.on_tick();
	}

	if (s->cfg.should_stop && s->cfg.should_stop()) {
		luaL_error(L, "script stopped");
	}
}

// print(), routed to the host instead of stdout. Mirrors Lua's own print:
// tostring each argument, tab separated.
static int l_print(lua_State *L) {
	script_lua_t *s = NULL;
	lua_getallocf(L, (void **)&s);

	int n = lua_gettop(L);
	luaL_Buffer b;
	luaL_buffinit(L, &b);

	for (int i = 1; i <= n; i++) {
		size_t len = 0;
		const char *piece = luaL_tolstring(L, i, &len);
		if (i > 1) {
			luaL_addchar(&b, '\t');
		}
		luaL_addlstring(&b, piece, len);
		lua_pop(L, 1);	// luaL_tolstring pushes its result
	}

	luaL_pushresult(&b);
	const char *msg = lua_tostring(L, -1);
	if (s && s->cfg.print && msg) {
		s->cfg.print(msg);
	}
	lua_pop(L, 1);
	return 0;
}

/*
 * require(), resolving only against the import table in the flashed
 * container.
 *
 * Lua's own searchers look at the filesystem and at loaded C libraries.
 * Neither exists here, and both would be a way out of the sandbox, so this
 * replaces require() outright rather than adding a searcher to package.path
 * -- there is no package library in this build to add one to.
 *
 * This is the counterpart of VESC Tool's lisp import bundling: the host-side
 * packer walks require() calls and appends each module's source to the
 * container, and this finds them by name at runtime.
 */
static int l_require(lua_State *L) {
	const char *name = luaL_checkstring(L, 1);

	script_lua_t *s = NULL;
	lua_getallocf(L, (void **)&s);
	if (!s || !s->cfg.blob) {
		return luaL_error(L, "require '%s': no bundled modules in this script", name);
	}

	// Already loaded? Return the cached value, as real require does, so a
	// module required from two places runs once.
	lua_getfield(L, LUA_REGISTRYINDEX, "_SCRIPT_LOADED");
	lua_getfield(L, -1, name);
	if (!lua_isnil(L, -1)) {
		return 1;
	}
	lua_pop(L, 1);

	const uint8_t *data = NULL;
	int32_t data_len = 0;
	if (!script_pack_import(s->cfg.blob, name, &data, &data_len)) {
		return luaL_error(L, "require '%s': not bundled in this script", name);
	}

	char chunkname[64];
	snprintf(chunkname, sizeof(chunkname), "@%s", name);

	/*
	 * Drop the terminator the packer appends.
	 *
	 * Every payload in the container carries a trailing NUL -- VESC Tool's
	 * "pad with 0 in case it is a text file" -- because an imported lisp file
	 * is handed to a reader that needs one. Lua's lexer does not: it treats
	 * the NUL as a stray byte and refuses the chunk. The byte belongs to the
	 * container format rather than to the module, so it comes off here.
	 */
	size_t src_len = (size_t)data_len;
	if (src_len > 0 && data[src_len - 1] == '\0') {
		src_len--;
	}

	if (luaL_loadbuffer(L, (const char *)data, src_len, chunkname) != LUA_OK) {
		return luaL_error(L, "require '%s': %s", name, lua_tostring(L, -1));
	}

	lua_pushstring(L, name);
	lua_call(L, 1, 1);	// Errors propagate to the caller's pcall.

	// A module returning nothing is recorded as true, again matching require.
	if (lua_isnil(L, -1)) {
		lua_pop(L, 1);
		lua_pushboolean(L, 1);
	}

	lua_pushvalue(L, -1);
	lua_setfield(L, -3, name);	// _SCRIPT_LOADED[name] = value
	return 1;
}

/*
 * vesc.asset(name) -> string, or nil when not bundled.
 *
 * The same import table require() uses, but handed back as bytes rather than
 * compiled as Lua. Prepared fonts, icons and lookup tables are binary, and
 * there is nowhere else to put them: the board has no filesystem and the
 * container is the only thing that travels with a script.
 */
static int l_asset(lua_State *L) {
	const char *name = luaL_checkstring(L, 1);

	script_lua_t *s = NULL;
	lua_getallocf(L, (void **)&s);
	if (!s || !s->cfg.blob) {
		lua_pushnil(L);
		return 1;
	}

	const uint8_t *data = NULL;
	int32_t len = 0;
	if (!script_pack_import(s->cfg.blob, name, &data, &len)) {
		lua_pushnil(L);
		return 1;
	}

	// Pushed as a counted string, so a NUL inside a font or an icon is
	// carried through rather than truncating it.
	lua_pushlstring(L, (const char *)data, (size_t)len);
	return 1;
}

/*
 * The sandbox.
 *
 * Opened: base (minus the dangerous entries), coroutine, string, table, math.
 * Never opened: io and os, which reach the filesystem and the clock and are
 * not compiled into this build at all; package, which loads C libraries;
 * debug, which can reach around everything else; utf8, merely unused.
 *
 * Removed from base afterwards: dofile and loadfile, which want a
 * filesystem; load, because a script that can compile a string at runtime
 * can defeat any static review of what got flashed; and collectgarbage,
 * which lets a script disable the GC on a device whose memory ceiling is the
 * only thing protecting the rest of the firmware.
 */
static void install_sandbox(lua_State *L) {
	static const luaL_Reg libs[] = {
		{LUA_GNAME, luaopen_base},
		{LUA_COLIBNAME, luaopen_coroutine},
		{LUA_STRLIBNAME, luaopen_string},
		{LUA_TABLIBNAME, luaopen_table},
		{LUA_MATHLIBNAME, luaopen_math},
		{NULL, NULL},
	};

	for (const luaL_Reg *lib = libs; lib->func; lib++) {
		luaL_requiref(L, lib->name, lib->func, 1);
		lua_pop(L, 1);
	}

	static const char *const strip[] = {
		"dofile", "loadfile", "load", "collectgarbage", NULL,
	};
	for (const char *const *n = strip; *n; n++) {
		lua_pushnil(L);
		lua_setglobal(L, *n);
	}

	lua_pushcfunction(L, l_print);
	lua_setglobal(L, "print");

	lua_pushcfunction(L, l_require);
	lua_setglobal(L, "require");

	// Module cache for require, kept in the registry where scripts cannot
	// reach it.
	lua_newtable(L);
	lua_setfield(L, LUA_REGISTRYINDEX, "_SCRIPT_LOADED");

	// The table bindings attach to.
	lua_newtable(L);
	lua_pushcfunction(L, l_asset);
	lua_setfield(L, -2, "asset");
	lua_setglobal(L, "vesc");
}

script_lua_t *script_lua_open(const script_lua_cfg_t *cfg) {
	if (!cfg) {
		return NULL;
	}

	script_lua_t *s = calloc(1, sizeof(script_lua_t));
	if (!s) {
		return NULL;
	}
	s->cfg = *cfg;

	s->L = lua_newstate(l_alloc, s);
	if (!s->L) {
		free(s);
		return NULL;
	}

	install_sandbox(s->L);

	/*
	 * Generational collection. The incremental collector's pauses scale with
	 * the size of the live set; generational mode keeps the common case to
	 * young objects, which is what a control loop allocating small tables
	 * every iteration produces. This board also runs comms and a display,
	 * so a long stop-the-world pause is visible.
	 */
	lua_gc(s->L, LUA_GCGEN, 0, 0);

	int count = cfg->hook_count > 0 ? cfg->hook_count : 1000;
	lua_sethook(s->L, l_hook, LUA_MASKCOUNT, count);

	return s;
}

void script_lua_close(script_lua_t *s) {
	if (!s) {
		return;
	}
	if (s->L) {
		lua_close(s->L);
	}
	free(s);
}

/*
 * Not taking a const pointer, despite the obvious static-analysis note: the
 * whole purpose is to hand out a state the caller will mutate, by registering
 * bindings or running the collector. A const parameter returning a mutable
 * interior pointer would be a worse lie than the warning.
 */
// cppcheck-suppress constParameterPointer
lua_State *script_lua_state(script_lua_t *s) {
	return s ? s->L : NULL;
}

void script_lua_register(script_lua_t *s, const luaL_Reg *fns) {
	if (!s || !s->L || !fns) {
		return;
	}

	lua_getglobal(s->L, "vesc");
	for (const luaL_Reg *f = fns; f->name; f++) {
		lua_pushcfunction(s->L, f->func);
		lua_setfield(s->L, -2, f->name);
	}
	lua_pop(s->L, 1);
}

size_t script_lua_mem_used(const script_lua_t *s) {
	return s ? s->mem_used : 0;
}

size_t script_lua_mem_peak(const script_lua_t *s) {
	return s ? s->mem_peak : 0;
}

bool script_lua_run(script_lua_t *s, const char *src, int32_t len,
		const char *chunkname, char *err, size_t err_len) {
	if (err && err_len > 0) {
		err[0] = '\0';
	}
	if (!s || !s->L || !src) {
		return false;
	}

	/*
	 * A negative length means NUL-terminated. Without this a caller passing
	 * -1 -- the usual convention, and what Lua's own API accepts -- casts to
	 * SIZE_MAX and the parser reads off the end of the buffer. That is a
	 * fault on a board and silent corruption on a host, from a mistake the
	 * signature invites.
	 */
	size_t srclen = (len < 0) ? strlen(src) : (size_t)len;

	if (luaL_loadbuffer(s->L, src, srclen,
			chunkname ? chunkname : "=script") != LUA_OK) {
		const char *msg = lua_tostring(s->L, -1);
		if (err && err_len > 0 && msg) {
			snprintf(err, err_len, "%s", msg);
		}
		lua_pop(s->L, 1);
		return false;
	}

	if (lua_pcall(s->L, 0, 0, 0) != LUA_OK) {
		const char *msg = lua_tostring(s->L, -1);
		if (err && err_len > 0 && msg) {
			snprintf(err, err_len, "%s", msg);
		}
		lua_pop(s->L, 1);
		return false;
	}

	return true;
}

// ---------------------------------------------------------------- events ---

/*
 * Handlers live in the registry under fixed keys rather than in a Lua table
 * the script can see. A script that could reach the handler table could
 * replace another module's handler, and more practically it would show up in
 * the globals listing that GET_STATS walks.
 */
static const char *const EV_KEYS[] = {
	[SCRIPT_EV_NONE] = NULL,
	[SCRIPT_EV_CAN_SID] = "_ev_can",
	[SCRIPT_EV_CAN_EID] = "_ev_can",
	[SCRIPT_EV_CAN2_SID] = "_ev_can",
	[SCRIPT_EV_CAN2_EID] = "_ev_can",
	[SCRIPT_EV_APP_DATA] = "_ev_app_data",
	[SCRIPT_EV_TIMER] = "_ev_timer",
	[SCRIPT_EV_TOUCH] = "_ev_touch",
};

static const char *ev_key(int type) {
	if (type <= SCRIPT_EV_NONE || type > SCRIPT_EV_TOUCH) {
		return NULL;
	}
	return EV_KEYS[type];
}

static int set_handler(lua_State *L, const char *key, size_t flag_offset) {
	if (!lua_isnil(L, 1)) {
		luaL_checktype(L, 1, LUA_TFUNCTION);
	}
	bool present = !lua_isnil(L, 1);

	lua_settop(L, 1);
	lua_setfield(L, LUA_REGISTRYINDEX, key);

	script_lua_t *s = NULL;
	lua_getallocf(L, (void **)&s);
	if (s) {
		*((volatile uint8_t *)((char *)s + flag_offset)) = present ? 1 : 0;
	}
	return 0;
}

// vesc.on_can(function(id, data, is_ext, bus) end) -- nil to unregister.
static int l_on_can(lua_State *L) {
	return set_handler(L, "_ev_can", offsetof(struct script_lua, have_can));
}

/*
 * vesc.on_touch(function(pressed, x, y, strength, track_id) end)
 *
 * Fires on a press, a release, and movement past a few pixels while held --
 * the touch core decides, and rate limits it. The coordinates are zero on a
 * release, so a handler that only cares where the finger went can test
 * pressed first.
 */
static int l_on_touch(lua_State *L) {
	return set_handler(L, "_ev_touch", offsetof(struct script_lua, have_touch));
}

// vesc.on_app_data(function(data) end)
static int l_on_app_data(lua_State *L) {
	return set_handler(L, "_ev_app_data",
			offsetof(struct script_lua, have_app_data));
}

/*
 * vesc.on_timer(period_ms, function() end)
 *
 * One timer, not many: a script wanting several periods can divide down from
 * one, and a single timer keeps the engine task's scheduling obvious. The
 * period is clamped rather than rejected, because a script asking for 0 means
 * "as fast as possible" and spinning the engine task at that rate would
 * starve everything else on the core.
 */
static int l_on_timer(lua_State *L) {
	lua_Integer period = luaL_checkinteger(L, 1);
	if (period < 1) {
		period = 1;
	}

	script_lua_t *s = NULL;
	lua_getallocf(L, (void **)&s);
	if (s) {
		s->timer_period_ms = (uint32_t)period;
	}

	lua_settop(L, 2);
	if (!lua_isnil(L, 2)) {
		luaL_checktype(L, 2, LUA_TFUNCTION);
	}
	bool present = !lua_isnil(L, 2);
	lua_setfield(L, LUA_REGISTRYINDEX, "_ev_timer");
	if (s) {
		s->have_timer = present ? 1 : 0;
	}
	return 0;
}

void script_lua_install_events(script_lua_t *s) {
	if (!s || !s->L) {
		return;
	}

	static const luaL_Reg ev_fns[] = {
		{"on_can", l_on_can},
		{"on_app_data", l_on_app_data},
		{"on_touch", l_on_touch},
		{"on_timer", l_on_timer},
		{NULL, NULL},
	};
	script_lua_register(s, ev_fns);
}

/*
 * Answered from the mirrored flags, never by asking Lua.
 *
 * This is called from producer tasks, so it cannot use the Lua API at all --
 * an earlier version of this function did a lua_getfield here, which races
 * with the engine task being inside a pcall and would corrupt the Lua stack
 * rather than fail visibly.
 */
bool script_lua_wants(const script_lua_t *s, int type) {
	if (!s) {
		return false;
	}

	switch (type) {
	case SCRIPT_EV_CAN_SID:
	case SCRIPT_EV_CAN_EID:
	case SCRIPT_EV_CAN2_SID:
	case SCRIPT_EV_CAN2_EID:
		return s->have_can != 0;
	case SCRIPT_EV_APP_DATA:
		return s->have_app_data != 0;
	case SCRIPT_EV_TIMER:
		return s->have_timer != 0;
	case SCRIPT_EV_TOUCH:
		return s->have_touch != 0;
	default:
		return false;
	}
}

uint32_t script_lua_timer_period(const script_lua_t *s) {
	return s ? s->timer_period_ms : 0;
}

bool script_lua_dispatch(script_lua_t *s, const script_event_t *ev,
		char *err, size_t err_len) {
	if (err && err_len > 0) {
		err[0] = '\0';
	}
	if (!s || !s->L || !ev) {
		return true;
	}

	const char *key = ev_key(ev->type);
	if (!key) {
		return true;
	}

	lua_State *L = s->L;
	lua_getfield(L, LUA_REGISTRYINDEX, key);
	if (!lua_isfunction(L, -1)) {
		lua_pop(L, 1);
		return true;	// Nothing registered is not a failure.
	}

	int nargs = 0;
	switch (ev->type) {
	case SCRIPT_EV_CAN_SID:
	case SCRIPT_EV_CAN_EID:
	case SCRIPT_EV_CAN2_SID:
	case SCRIPT_EV_CAN2_EID:
		lua_pushinteger(L, (lua_Integer)ev->id);
		// The payload is a string, not a table: it is already a counted byte
		// buffer, string.unpack reads it in whatever layout the sender used,
		// and it allocates nothing per byte in a handler that may run at a
		// few hundred hertz.
		lua_pushlstring(L, (const char *)ev->data, ev->len);
		lua_pushboolean(L, ev->type == SCRIPT_EV_CAN_EID ||
				ev->type == SCRIPT_EV_CAN2_EID);
		lua_pushinteger(L, (ev->type == SCRIPT_EV_CAN2_SID ||
				ev->type == SCRIPT_EV_CAN2_EID) ? 2 : 1);
		nargs = 4;
		break;

	case SCRIPT_EV_APP_DATA:
		lua_pushlstring(L, (const char *)ev->data, ev->len);
		nargs = 1;
		break;

	case SCRIPT_EV_TIMER:
		nargs = 0;
		break;

	case SCRIPT_EV_TOUCH: {
		// Short payload means a producer that did not fill the struct, which
		// is a bug rather than something to guess at.
		if (ev->len < sizeof(script_event_touch_t)) {
			lua_pop(L, 1);
			return true;
		}

		script_event_touch_t t;
		memcpy(&t, ev->data, sizeof(t));
		lua_pushboolean(L, t.pressed);
		lua_pushinteger(L, t.x);
		lua_pushinteger(L, t.y);
		lua_pushinteger(L, t.strength);
		lua_pushinteger(L, t.track_id);
		nargs = 5;
		break;
	}

	default:
		lua_pop(L, 1);
		return true;
	}

	if (lua_pcall(L, nargs, 0, 0) != LUA_OK) {
		const char *msg = lua_tostring(L, -1);
		if (err && err_len > 0 && msg) {
			snprintf(err, err_len, "%s", msg);
		}
		lua_pop(L, 1);
		return false;
	}

	return true;
}
