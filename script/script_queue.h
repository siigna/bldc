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
 * A bounded queue of script events, written by producers and drained by the
 * engine thread.
 *
 * Why this exists rather than a ChibiOS mailbox: a mailbox carries a msg_t,
 * which is pointer sized, and a script_event_t is the better part of a
 * hundred bytes. The usual answer is a memory pool of events plus a mailbox
 * of pointers to them, which is two allocators and a lifetime to get wrong
 * on a path that runs per CAN frame. A fixed ring copies the event once and
 * has no lifetime at all.
 *
 * The rules it has to keep, because the producers are interrupt-adjacent:
 *
 *   A post never blocks. A full queue drops the newest event and counts it.
 *   Dropping is the right failure for this: the alternative is stalling the
 *   CAN path behind a script that is busy.
 *
 *   The drop is counted rather than silent. A script that misses frames and
 *   a script that is never sent any look identical from inside, and the
 *   count is the only thing that separates them.
 *
 *   Order is preserved. A script reading a multi-frame protocol cannot
 *   reassemble it from a queue that reorders.
 */

#ifndef SCRIPT_SCRIPT_QUEUE_H_
#define SCRIPT_SCRIPT_QUEUE_H_

#include "script_event.h"

#include <stdint.h>
#include <stdbool.h>

// Events held. Sized for a burst rather than a backlog: the engine drains at
// its own rate and anything older than a few frames is stale anyway.
#define SCRIPT_QUEUE_LEN		16

typedef struct {
	script_event_t ev[SCRIPT_QUEUE_LEN];
	uint8_t head;			// where the next post goes
	uint8_t tail;			// where the next fetch comes from
	uint8_t count;
	uint32_t dropped;
} script_queue_t;

void script_queue_init(script_queue_t *q);

/*
 * Copy one event in. Returns false when the queue was full, in which case the
 * event is dropped and the dropped count is incremented.
 *
 * Safe from any context that can take the system lock. Never blocks.
 */
bool script_queue_post(script_queue_t *q, const script_event_t *ev);

// Copy the oldest event out. Returns false when empty.
bool script_queue_fetch(script_queue_t *q, script_event_t *out);

int script_queue_count(const script_queue_t *q);
uint32_t script_queue_dropped(const script_queue_t *q);

#endif /* SCRIPT_SCRIPT_QUEUE_H_ */
