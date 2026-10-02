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
 * CAN bindings: reading other controllers, commanding them, and putting raw
 * frames on the bus.
 *
 * This is the first binding set where a mistake leaves the board. Everything
 * else a script can get wrong affects this controller; these reach every
 * device on the bus, so two decisions are worth stating rather than leaving
 * to be inferred.
 *
 * READING AN ABSENT CONTROLLER GIVES ZERO, NOT NIL.
 *
 * comm_can keeps a cache of the status frames other controllers broadcast,
 * and canget_* reads it. When nothing has ever been heard from an id, lisp
 * returns 0.0, and so does this -- a script ported between the engines must
 * not change behaviour. But zero is a plausible reading: canget_rpm on a
 * controller that is not there is indistinguishable from one that is stopped,
 * and the cache also keeps the last frame forever, so a controller that
 * dropped off the bus mid-ride still reads whatever it last said.
 *
 * can_msg_age is how a script tells those apart, and it returns nil rather
 * than a number when nothing has been heard. A script that acts on another
 * controller's telemetry should check it.
 *
 * can_cmd IS DELIBERATELY ABSENT.
 *
 * The lisp extension of that name hands a command packet to another
 * controller's protocol layer, which is remote control of that board rather
 * than a CAN message. It has no business being reachable from a script here
 * until there is a reason for it that is worth the exposure.
 */

#include "lua_vesc_can.h"

#include "lauxlib.h"

#include <string.h>
#include <math.h>

#include "datatypes.h"
#include "comm_can.h"
#include "app.h"
#include "utils.h"

/*
 * Controller ids run 0..253; 254 and 255 are reserved by the protocol. An id
 * outside that is a programming error rather than a missing device, so it
 * raises instead of quietly addressing nothing.
 */
#define CAN_ID_MAX	253

static int check_id(lua_State *L, int arg) {
	lua_Integer id = luaL_checkinteger(L, arg);

	luaL_argcheck(L, (id >= 0) && (id <= CAN_ID_MAX), arg,
			"controller id must be 0..253");
	return (int)id;
}

/* ------------------------------------------------------------- reading --- */

/*
 * Each status frame carries a different group of values, so which cache a
 * reader looks in is part of the binding. SIGN() on the duty is how lisp
 * recovers a direction for the directional current.
 */

#define CANGET_1(fname, expr)                                                \
	static int fname(lua_State *L) {                                         \
		can_status_msg *s = comm_can_get_status_msg_id(check_id(L, 1));      \
		lua_pushnumber(L, (lua_Number)(s ? (expr) : 0.0f));                  \
		return 1;                                                            \
	}

CANGET_1(l_canget_rpm, s->rpm)
CANGET_1(l_canget_current, s->current)
CANGET_1(l_canget_current_dir, s->current *SIGN(s->duty))
CANGET_1(l_canget_duty, s->duty)

#define CANGET_N(fname, type, getter, expr)                                  \
	static int fname(lua_State *L) {                                         \
		type *s = getter(check_id(L, 1));                                    \
		lua_pushnumber(L, (lua_Number)(s ? (expr) : 0.0f));                  \
		return 1;                                                            \
	}

CANGET_N(l_canget_ah, can_status_msg_2, comm_can_get_status_msg_2_id,
		s->amp_hours)
CANGET_N(l_canget_ah_chg, can_status_msg_2, comm_can_get_status_msg_2_id,
		s->amp_hours_charged)
CANGET_N(l_canget_wh, can_status_msg_3, comm_can_get_status_msg_3_id,
		s->watt_hours)
CANGET_N(l_canget_wh_chg, can_status_msg_3, comm_can_get_status_msg_3_id,
		s->watt_hours_charged)
CANGET_N(l_canget_temp_fet, can_status_msg_4, comm_can_get_status_msg_4_id,
		s->temp_fet)
CANGET_N(l_canget_temp_mot, can_status_msg_4, comm_can_get_status_msg_4_id,
		s->temp_motor)
CANGET_N(l_canget_current_in, can_status_msg_4, comm_can_get_status_msg_4_id,
		s->current_in)
CANGET_N(l_canget_pid_pos, can_status_msg_4, comm_can_get_status_msg_4_id,
		s->pid_pos_now)
CANGET_N(l_canget_vin, can_status_msg_5, comm_can_get_status_msg_5_id,
		s->v_in)
CANGET_N(l_canget_tacho, can_status_msg_5, comm_can_get_status_msg_5_id,
		(float)s->tacho_value)

/*
 * Seconds since the given status frame was last heard from that controller,
 * or nil if it never has been.
 *
 * nil rather than a large number, because this is the one binding whose whole
 * job is to distinguish "no data" from "data". A sentinel would have to be
 * compared against correctly to be any use, and a script that forgets reads
 * it as a plausible age.
 *
 * UTILS_AGE_S rather than dividing a tick count: a tick is 100 us here and
 * the divisor belongs in one place.
 */
static int l_can_msg_age(lua_State *L) {
	int id = check_id(L, 1);
	int msg = (int)luaL_optinteger(L, 2, 1);
	systime_t rx;

	switch (msg) {
	case 1: {
		can_status_msg *s = comm_can_get_status_msg_id(id);
		if (s == NULL) { lua_pushnil(L); return 1; }
		rx = s->rx_time;
	} break;
	case 2: {
		can_status_msg_2 *s = comm_can_get_status_msg_2_id(id);
		if (s == NULL) { lua_pushnil(L); return 1; }
		rx = s->rx_time;
	} break;
	case 3: {
		can_status_msg_3 *s = comm_can_get_status_msg_3_id(id);
		if (s == NULL) { lua_pushnil(L); return 1; }
		rx = s->rx_time;
	} break;
	case 4: {
		can_status_msg_4 *s = comm_can_get_status_msg_4_id(id);
		if (s == NULL) { lua_pushnil(L); return 1; }
		rx = s->rx_time;
	} break;
	case 5: {
		can_status_msg_5 *s = comm_can_get_status_msg_5_id(id);
		if (s == NULL) { lua_pushnil(L); return 1; }
		rx = s->rx_time;
	} break;
	default:
		return luaL_argerror(L, 2, "status frame must be 1..5");
	}

	lua_pushnumber(L, (lua_Number)UTILS_AGE_S(rx));
	return 1;
}

/* ----------------------------------------------------------- discovery --- */

static int l_can_local_id(lua_State *L) {
	lua_pushinteger(L, (lua_Integer)app_get_configuration()->controller_id);
	return 1;
}

/*
 * Ids this controller has heard a status frame 1 from, as a table.
 *
 * Built from the cache rather than by probing the bus, so it costs nothing
 * and tells the truth about what has actually been seen -- which is a
 * different question from what is physically connected. can_ping asks.
 */
static int l_can_scan(lua_State *L) {
	int n = 1;

	lua_newtable(L);
	for (int i = 0; i < CAN_STATUS_MSGS_TO_STORE; i++) {
		can_status_msg *s = comm_can_get_status_msg_index(i);

		if ((s != NULL) && (s->id >= 0)) {
			lua_pushinteger(L, (lua_Integer)s->id);
			lua_rawseti(L, -2, n++);
		}
	}
	return 1;
}

static int l_can_ping(lua_State *L) {
	int id = check_id(L, 1);
	HW_TYPE hw = HW_TYPE_VESC;

	if (comm_can_ping((uint8_t)id, &hw)) {
		lua_pushinteger(L, (lua_Integer)hw);
	} else {
		lua_pushnil(L);
	}
	return 1;
}

/* ------------------------------------------------------- commanding ------ */

/*
 * These drive another controller's motor.
 *
 * They deliberately do not touch timeout_reset. The local motor timeout is
 * about this board's output; the controller being commanded runs its own, and
 * refreshing ours here would be meaningless at best. Adding it by analogy
 * with the motor bindings would be the obvious mistake, so: not an oversight.
 */

static int l_canset_current(lua_State *L) {
	int id = check_id(L, 1);
	float current = (float)luaL_checknumber(L, 2);

	if (lua_isnoneornil(L, 3)) {
		comm_can_set_current((uint8_t)id, current);
	} else {
		comm_can_set_current_off_delay((uint8_t)id, current,
				(float)luaL_checknumber(L, 3));
	}
	return 0;
}

static int l_canset_current_rel(lua_State *L) {
	int id = check_id(L, 1);
	float rel = (float)luaL_checknumber(L, 2);

	if (lua_isnoneornil(L, 3)) {
		comm_can_set_current_rel((uint8_t)id, rel);
	} else {
		comm_can_set_current_rel_off_delay((uint8_t)id, rel,
				(float)luaL_checknumber(L, 3));
	}
	return 0;
}

static int l_canset_duty(lua_State *L) {
	int id = check_id(L, 1);

	comm_can_set_duty((uint8_t)id, (float)luaL_checknumber(L, 2));
	return 0;
}

static int l_canset_rpm(lua_State *L) {
	int id = check_id(L, 1);

	comm_can_set_rpm((uint8_t)id, (float)luaL_checknumber(L, 2));
	return 0;
}

static int l_canset_pos(lua_State *L) {
	int id = check_id(L, 1);

	comm_can_set_pos((uint8_t)id, (float)luaL_checknumber(L, 2));
	return 0;
}

static int l_canset_brake(lua_State *L) {
	int id = check_id(L, 1);

	comm_can_set_current_brake((uint8_t)id, (float)luaL_checknumber(L, 2));
	return 0;
}

static int l_canset_brake_rel(lua_State *L) {
	int id = check_id(L, 1);

	comm_can_set_current_brake_rel((uint8_t)id,
			(float)luaL_checknumber(L, 2));
	return 0;
}

/* ----------------------------------------------------------- raw frames -- */

/*
 * An arbitrary frame on the bus, for talking to a BMS or anything else that
 * is not a VESC.
 *
 * The payload is a Lua string rather than a table, matching how an incoming
 * frame is delivered to vesc.on_can -- a script that echoes what it received
 * should not have to convert between two shapes.
 *
 * Length is checked against 8 rather than truncated: a frame silently cut
 * short is a different message, and the receiver has no way to know.
 */
static int can_send(lua_State *L, bool extended) {
	lua_Integer id = luaL_checkinteger(L, 1);
	size_t len = 0;
	const char *data = luaL_checklstring(L, 2, &len);

	luaL_argcheck(L, id >= 0, 1, "frame id must not be negative");
	luaL_argcheck(L, extended ? (id <= 0x1FFFFFFF) : (id <= 0x7FF), 1,
			extended ? "extended frame id must fit 29 bits"
					: "standard frame id must fit 11 bits");
	luaL_argcheck(L, len <= 8, 2, "a CAN frame carries at most 8 bytes");

	msg_t res = extended
			? comm_can_transmit_eid((uint32_t)id, (const uint8_t *)data,
					(uint8_t)len)
			: comm_can_transmit_sid((uint32_t)id, (const uint8_t *)data,
					(uint8_t)len);

	lua_pushboolean(L, res == MSG_OK);
	return 1;
}

static int l_can_send_sid(lua_State *L) {
	return can_send(L, false);
}

static int l_can_send_eid(lua_State *L) {
	return can_send(L, true);
}

/* ---------------------------------------------------------------- table -- */

static const luaL_Reg can_fns[] = {
	{"canget_rpm", l_canget_rpm},
	{"canget_current", l_canget_current},
	{"canget_current_dir", l_canget_current_dir},
	{"canget_current_in", l_canget_current_in},
	{"canget_duty", l_canget_duty},
	{"canget_ah", l_canget_ah},
	{"canget_ah_chg", l_canget_ah_chg},
	{"canget_wh", l_canget_wh},
	{"canget_wh_chg", l_canget_wh_chg},
	{"canget_temp_fet", l_canget_temp_fet},
	{"canget_temp_mot", l_canget_temp_mot},
	{"canget_pid_pos", l_canget_pid_pos},
	{"canget_vin", l_canget_vin},
	{"canget_tacho", l_canget_tacho},
	{"can_msg_age", l_can_msg_age},

	{"can_local_id", l_can_local_id},
	{"can_scan", l_can_scan},
	{"can_ping", l_can_ping},

	{"canset_current", l_canset_current},
	{"canset_current_rel", l_canset_current_rel},
	{"canset_duty", l_canset_duty},
	{"canset_rpm", l_canset_rpm},
	{"canset_pos", l_canset_pos},
	{"canset_brake", l_canset_brake},
	{"canset_brake_rel", l_canset_brake_rel},

	{"can_send_sid", l_can_send_sid},
	{"can_send_eid", l_can_send_eid},

	{NULL, NULL}
};

const luaL_Reg *lua_vesc_can_fns(void) {
	return can_fns;
}
