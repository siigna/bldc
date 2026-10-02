/*
	Copyright 2025 Stephen Bouche

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

#ifndef MAIN_SCRIPT_SCRIPT_LUA_H_
#define MAIN_SCRIPT_SCRIPT_LUA_H_

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

#include "script_pack.h"
#include "script_event.h"
#include "lua.h"
#include "lauxlib.h"

/*
 * The Lua engine, with no firmware dependencies.
 *
 * Everything the engine needs from its host arrives through the config below
 * as a function pointer, so the whole thing -- sandbox, memory ceiling,
 * interrupt hook, require resolution -- runs and is tested on a host. On a
 * board that matters more than it sounds: a script engine that can only be
 * exercised by flashing is a script engine whose failure modes get discovered
 * on a vehicle.
 *
 * The firmware adapter is luaif.c, which implements the same eight entry
 * points lispif.c does. Those two files are the whole of the build-time
 * choice between engines.
 */

typedef struct {
	/*
	 * Hard ceiling on bytes the script may hold at once, enforced in the
	 * allocator. Lua asks for memory and copes with refusal by raising a
	 * catchable error, so a runaway script fails its own allocation instead
	 * of starving the rest of the firmware. Zero means no ceiling, which is
	 * only sensible in tests.
	 */
	size_t mem_limit;

	// Where print() and runtime errors go. Never NULL in practice.
	void (*print)(const char *msg);

	/*
	 * Polled from the instruction hook. Returning true unwinds the running
	 * script with an error rather than killing the task, so finalisers run
	 * and the interpreter is left reusable.
	 */
	bool (*should_stop)(void);

	/*
	 * Called from the instruction hook roughly every `hook_count` VM
	 * instructions, for yielding to the scheduler. May be NULL.
	 */
	void (*on_tick)(void);
	int hook_count;

	/*
	 * The loaded script container, used to resolve require(). May be NULL, in
	 * which case require() finds nothing and says so.
	 */
	const script_blob_t *blob;

	/*
	 * Backing allocator, in Lua's own realloc shape, with alloc_ud passed
	 * through as its first argument. NULL means the C library's realloc,
	 * which is what a host test and any part with a general-purpose malloc
	 * should use.
	 *
	 * A target whose interpreter must live somewhere specific passes its own
	 * here instead: on an F405 there is no room for a Lua heap in main RAM,
	 * so bldc passes script_alloc and serves the interpreter out of a fixed
	 * arena in CCM.
	 *
	 * The ceiling in mem_limit is enforced above this, so it still applies
	 * whatever the allocator is -- and a backing allocator may also have a
	 * hard limit of its own, which a script reaches only if mem_limit is
	 * higher or zero.
	 */
	void *(*alloc)(void *ud, void *ptr, size_t osize, size_t nsize);
	void *alloc_ud;
} script_lua_cfg_t;

typedef struct script_lua script_lua_t;

/*
 * Create an interpreter with the sandbox installed. Returns NULL if the
 * ceiling is too low to build one, which is a legitimate outcome rather than
 * a fault.
 */
script_lua_t *script_lua_open(const script_lua_cfg_t *cfg);

void script_lua_close(script_lua_t *s);

/*
 * Compile and run a chunk. Returns false on a compile or runtime error and
 * copies the message into err, which may be NULL. Errors never propagate out
 * of this call: everything runs under lua_pcall, because a longjmp escaping
 * into firmware that is driving hardware is not recoverable.
 *
 * A negative len means src is NUL-terminated.
 */
bool script_lua_run(script_lua_t *s, const char *src, int32_t len,
		const char *chunkname, char *err, size_t err_len);

// Bytes currently held by the script, as counted by the allocator.
size_t script_lua_mem_used(const script_lua_t *s);

// Peak of the above since the interpreter was opened.
size_t script_lua_mem_peak(const script_lua_t *s);

/*
 * The underlying state, for registering bindings. Returns NULL for a NULL
 * engine so callers can chain without checking twice.
 */
lua_State *script_lua_state(script_lua_t *s);

/*
 * Install vesc.on_can, vesc.on_app_data and vesc.on_timer, which a script
 * uses to register handlers. The handlers are kept in the registry, out of
 * reach of the script itself.
 */
void script_lua_install_events(script_lua_t *s);

/*
 * Deliver one event to the script's handler for it.
 *
 * Returns false if the handler raised, with the message in err. A handler
 * that fails is reported and kept: one bad frame should not silently
 * unsubscribe a vehicle from its own CAN traffic. Returns true with no
 * handler registered, which is not an error -- most scripts want only some
 * of these.
 */
bool script_lua_dispatch(script_lua_t *s, const script_event_t *ev,
		char *err, size_t err_len);

// True if the script registered a handler for this event type. Lets the
// producer side skip queueing work nothing will consume.
bool script_lua_wants(const script_lua_t *s, int type);

/*
 * Timer period the script asked for via vesc.on_timer, in milliseconds, or 0
 * if it did not ask for one.
 */
uint32_t script_lua_timer_period(const script_lua_t *s);

/*
 * Register a table of C functions as fields of the global `vesc` table.
 * Bindings live in lua_vesc_ext.c and are passed in rather than reached for,
 * so the engine itself stays independent of the board.
 */
void script_lua_register(script_lua_t *s, const luaL_Reg *fns);

#endif /* MAIN_SCRIPT_SCRIPT_LUA_H_ */
