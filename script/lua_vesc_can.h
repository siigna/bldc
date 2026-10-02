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

#ifndef SCRIPT_LUA_VESC_CAN_H_
#define SCRIPT_LUA_VESC_CAN_H_

#include "lua.h"
#include "lauxlib.h"

/*
 * The CAN bindings, as a table for script_lua_register.
 *
 * Reading an absent controller gives 0, matching lisp; can_msg_age returns
 * nil when nothing has been heard, and is how a script tells "no data" from
 * "data". See the note at the top of lua_vesc_can.c -- this is the one
 * binding set whose mistakes leave the board.
 */
const luaL_Reg *lua_vesc_can_fns(void);

#endif /* SCRIPT_LUA_VESC_CAN_H_ */
