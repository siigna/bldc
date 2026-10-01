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

#ifndef SCRIPT_LUA_VESC_CONF_H_
#define SCRIPT_LUA_VESC_CONF_H_

#include "lua.h"
#include "lauxlib.h"

/*
 * conf_get, conf_set and conf_names, as a table for script_lua_register.
 *
 * conf_get returns nil for an unknown name rather than 0, so a typo fails
 * the arithmetic where it is rather than reading as a plausible setting.
 * conf_set returns a boolean.
 */
const luaL_Reg *lua_vesc_conf_fns(void);

#endif /* SCRIPT_LUA_VESC_CONF_H_ */
