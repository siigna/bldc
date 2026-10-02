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

#include "script_queue.h"

#include <string.h>

/*
 * The lock. A ChibiOS system lock rather than a mutex, because a post can
 * come from a context that must not sleep, and because the critical section
 * is a handful of byte copies.
 *
 * SCRIPT_QUEUE_HOST_TEST builds the same code without it, so the ring
 * arithmetic can be tested off-target. The test is single-threaded, which is
 * the one thing that makes dropping the lock sound -- a host test that
 * stubbed out a lock it actually needed would be worse than no test.
 */
#ifndef SCRIPT_QUEUE_HOST_TEST
#include "ch.h"
#define Q_LOCK()	chSysLock()
#define Q_UNLOCK()	chSysUnlock()
#else
#define Q_LOCK()	do {} while (0)
#define Q_UNLOCK()	do {} while (0)
#endif

void script_queue_init(script_queue_t *q) {
	if (!q) {
		return;
	}

	Q_LOCK();
	q->head = 0;
	q->tail = 0;
	q->count = 0;
	q->dropped = 0;
	Q_UNLOCK();
}

bool script_queue_post(script_queue_t *q, const script_event_t *ev) {
	if (!q || !ev) {
		return false;
	}

	Q_LOCK();

	if (q->count >= SCRIPT_QUEUE_LEN) {
		/*
		 * Full: drop the newest rather than overwrite the oldest. The oldest
		 * is the one the engine is about to read, and a protocol being
		 * reassembled from the queue survives losing its tail better than
		 * losing its head.
		 */
		q->dropped++;
		Q_UNLOCK();
		return false;
	}

	memcpy(&q->ev[q->head], ev, sizeof(*ev));
	q->head = (uint8_t)((q->head + 1u) % SCRIPT_QUEUE_LEN);
	q->count++;

	Q_UNLOCK();
	return true;
}

bool script_queue_fetch(script_queue_t *q, script_event_t *out) {
	if (!q || !out) {
		return false;
	}

	Q_LOCK();

	if (q->count == 0) {
		Q_UNLOCK();
		return false;
	}

	memcpy(out, &q->ev[q->tail], sizeof(*out));
	q->tail = (uint8_t)((q->tail + 1u) % SCRIPT_QUEUE_LEN);
	q->count--;

	Q_UNLOCK();
	return true;
}

int script_queue_count(const script_queue_t *q) {
	if (!q) {
		return 0;
	}
	return (int)q->count;
}

uint32_t script_queue_dropped(const script_queue_t *q) {
	if (!q) {
		return 0;
	}
	return q->dropped;
}
