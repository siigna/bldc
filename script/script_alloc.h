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
 * The script heap: a fixed arena, with realloc semantics on top of it.
 *
 * Lua wants a realloc. ChibiOS gives an alloc and a free over a heap it can
 * build on a buffer you hand it, and no realloc -- so that is what this
 * supplies, using the old size Lua passes in to know how much to copy.
 *
 * The arena is in CCM rather than main RAM, which is where the interpreter
 * heap has to live on an F405: with the script engine out, main RAM is
 * already 107 KB of the 128 KB it has, while CCM is 62 KB holding nothing
 * but LispBM's own heap -- and that is gone in a build that wants this one.
 * CCM is not DMA-capable, which costs an interpreter heap nothing.
 *
 * Not newlib's malloc. That lands in the main RAM heap behind _sbrk, where
 * there are about 21 KB and no say in the matter, and it would put the
 * interpreter in contention with everything else that allocates.
 *
 * A ceiling is enforced on top of the arena size. The arena is the hard
 * limit; the ceiling is the one a script hits first, so a runaway fails with
 * a Lua error and a traceback rather than by exhausting the heap under
 * whatever allocates next.
 */

#ifndef SCRIPT_SCRIPT_ALLOC_H_
#define SCRIPT_SCRIPT_ALLOC_H_

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>

/*
 * Hand the allocator its arena. Call once before anything allocates; calling
 * it again resets the accounting and abandons whatever was allocated, which
 * is only correct when the interpreter it belonged to is gone.
 *
 * limit caps the total handed out, and is clamped to the arena size: a
 * ceiling above the arena would be a promise the arena cannot keep.
 */
void script_alloc_init(void *arena, size_t arena_size, size_t limit);

/*
 * The Lua allocator, in Lua's own shape.
 *
 *   nsize == 0            free ptr, return NULL
 *   ptr == NULL           allocate nsize
 *   otherwise             resize, copying min(osize, nsize) bytes
 *
 * Returns NULL when the request would pass the ceiling or the arena is too
 * fragmented to serve it. Lua treats that as an out-of-memory it can raise
 * and a pcall can catch, which is the whole mechanism for containing a
 * runaway script.
 *
 * A failed resize leaves the original block untouched and still owned by the
 * caller. Freeing it on failure would hand Lua a dangling pointer at exactly
 * the moment it is trying to recover.
 */
void *script_alloc(void *ud, void *ptr, size_t osize, size_t nsize);

// Bytes currently handed out, and the high-water mark since init.
size_t script_alloc_used(void);
size_t script_alloc_peak(void);

#endif /* SCRIPT_SCRIPT_ALLOC_H_ */
