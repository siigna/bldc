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

#ifndef LUA_VESC_PAS_H_
#define LUA_VESC_PAS_H_

#include "lua.h"
#include "lauxlib.h"

/*
 * Pedal assist, for Lua.
 *
 * LispBM has had these since the PAS work landed; Lua had none of them, so a
 * Lua script could write eleven PAS settings through conf_set and then not
 * read a single thing back -- not cadence, not torque, not what the assist
 * was actually doing. Anything wanting to react to pedalling had to be
 * written in the other language.
 */
const luaL_Reg *lua_vesc_pas_fns(void);

#endif /* LUA_VESC_PAS_H_ */
