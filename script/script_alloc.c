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

#include "script_alloc.h"

#include <string.h>

/*
 * The arena's allocator. ChibiOS on target; malloc on the host, so the
 * realloc arithmetic above it can be tested where a failure is a stack trace
 * rather than a hard fault.
 *
 * What the host build does NOT test is the arena: malloc has no ceiling of
 * its own, so exhaustion there comes only from the accounting below. That is
 * the part worth testing anyway -- the ceiling is what a script hits first,
 * by design.
 */
#ifndef SCRIPT_ALLOC_HOST_TEST
#include "ch.h"

static memory_heap_t m_heap;

static void arena_init(void *arena, size_t size) {
	chHeapObjectInit(&m_heap, arena, size);
}

static void *arena_alloc(size_t size) {
	return chHeapAlloc(&m_heap, size);
}

static void arena_free(void *p) {
	chHeapFree(p);
}
#else
#include <stdlib.h>

static void arena_init(void *arena, size_t size) {
	(void)arena;
	(void)size;
}

/*
 * Test-only: make the next n arena allocations fail.
 *
 * Without this the arena never fails under the host build -- malloc obliges
 * -- so the branch that handles a failed allocation is unreachable, and with
 * it the rule that a failed resize leaves the original block alive. That rule
 * matters precisely when a script is recovering from an out-of-memory, which
 * is the worst moment to be handed a dangling pointer.
 *
 * Inside the host-test guard, so it is not in a firmware build.
 */
static int m_fail_next;

void script_alloc_test_fail_next(int n) {
	m_fail_next = n;
}

static void *arena_alloc(size_t size) {
	if (m_fail_next > 0) {
		m_fail_next--;
		return NULL;
	}
	return malloc(size);
}

static void arena_free(void *p) {
	free(p);
}
#endif

static size_t m_limit;
static size_t m_used;
static size_t m_peak;
static bool m_ready;

void script_alloc_init(void *arena, size_t arena_size, size_t limit) {
	if (!arena || arena_size == 0) {
		m_ready = false;
		return;
	}

	// A ceiling above the arena would be a promise the arena cannot keep.
	if (limit == 0 || limit > arena_size) {
		limit = arena_size;
	}

	arena_init(arena, arena_size);

	m_limit = limit;
	m_used = 0;
	m_peak = 0;
	m_ready = true;
}

void *script_alloc(void *ud, void *ptr, size_t osize, size_t nsize) {
	(void)ud;

	if (!m_ready) {
		return NULL;
	}

	if (nsize == 0) {
		if (ptr) {
			arena_free(ptr);
			// Guarded because a free of something never counted would
			// underflow this to an enormous number and wedge the ceiling
			// shut for the rest of the run.
			m_used = (m_used >= osize) ? (m_used - osize) : 0;
		}
		return NULL;
	}

	/*
	 * Refuse growth past the ceiling. Shrinking is always allowed, and
	 * testing nsize > osize first keeps a shrink from being refused when
	 * usage is already over the ceiling -- which it can be after init
	 * lowers it.
	 */
	if (nsize > osize) {
		size_t after = m_used + (nsize - osize);
		if (after > m_limit) {
			return NULL;
		}
	}

	void *np = arena_alloc(nsize);
	if (!np) {
		return NULL;
	}

	if (ptr) {
		// Copy what both blocks have in common, then let the old one go. A
		// shrink keeps the head of the block, which is what realloc means.
		size_t copy = (osize < nsize) ? osize : nsize;
		memcpy(np, ptr, copy);
		arena_free(ptr);
		m_used = (m_used >= osize) ? (m_used - osize) : 0;
	}

	m_used += nsize;
	if (m_used > m_peak) {
		m_peak = m_used;
	}

	return np;
}

size_t script_alloc_used(void) {
	return m_used;
}

size_t script_alloc_peak(void) {
	return m_peak;
}
