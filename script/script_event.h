/*
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

#ifndef MAIN_SCRIPT_SCRIPT_EVENT_H_
#define MAIN_SCRIPT_SCRIPT_EVENT_H_

#include <stdint.h>

/*
 * Events handed from the firmware's own tasks to the script engine.
 *
 * A CAN frame arrives on the CAN task and app data on a comms task, while
 * the interpreter belongs to the engine task. An interpreter is not
 * reentrant, so a producer must never call into it: it copies the event into
 * a queue and returns. Everything in this struct is therefore plain bytes,
 * fixed size and copyable, with no pointers into the producer's buffers --
 * those buffers are reused the moment the producer returns.
 *
 * Keeping the payload fixed also means the queue is allocated once at start
 * up. A producer that had to allocate could fail or block, and both are worse
 * than dropping a frame.
 */

// Chosen so a whole CAN frame always fits with room for app data that is
// worth delivering. Anything longer is truncated, and the truncation is
// reported rather than hidden: see script_lua_events_dropped.
#define SCRIPT_EVENT_PAYLOAD	64

typedef enum {
	SCRIPT_EV_NONE = 0,
	SCRIPT_EV_CAN_SID,	// Standard id, bus 1
	SCRIPT_EV_CAN_EID,	// Extended id, bus 1
	SCRIPT_EV_CAN2_SID,	// Standard id, bus 2
	SCRIPT_EV_CAN2_EID,	// Extended id, bus 2
	SCRIPT_EV_APP_DATA,	// COMM_CUSTOM_APP_DATA
	SCRIPT_EV_TIMER,	// Periodic, posted by the engine task itself
	SCRIPT_EV_TOUCH,	// Contact state or position changed
	/*
	 * A REPL expression is waiting to be evaluated.
	 *
	 * Carries nothing: the text lives in a buffer the adapter owns, because
	 * an expression is up to 512 bytes and the payload here is 64 -- and
	 * sizing the payload for the REPL would cost that much in every queued
	 * CAN frame.
	 *
	 * It is an event at all because the interpreter must not run on the
	 * task that received the packet. The comm task has a 3 KB stack, and
	 * loading and calling a chunk on it after 1.3 KB of local buffers is
	 * what made the Lua REPL silently do nothing while the lisp one, which
	 * hands work to its own evaluator thread, worked fine.
	 */
	SCRIPT_EV_REPL,
} script_event_type_t;

/*
 * Payload of a SCRIPT_EV_TOUCH, carried in script_event_t.data.
 *
 * A struct rather than four packed fields in id, because the touch core
 * already has the point in this shape and the queue copies bytes either way.
 * pressed is the state being reported; the coordinates are meaningless when
 * it is zero, and the core zeroes them there.
 */
typedef struct {
	uint8_t pressed;
	uint8_t track_id;
	uint16_t x;
	uint16_t y;
	uint16_t strength;
} script_event_touch_t;

typedef struct {
	uint8_t type;		// script_event_type_t
	uint32_t id;		// CAN id, unused otherwise
	uint16_t len;		// Bytes valid in data
	uint8_t truncated;	// Non-zero if the source was longer than data
	uint8_t data[SCRIPT_EVENT_PAYLOAD];
} script_event_t;

#endif /* MAIN_SCRIPT_SCRIPT_EVENT_H_ */
