/*
	Copyright Benjamin Vedder
	Copyright 2026 Stephen Bouche

	Parts of this file were moved here from util/mempools.h, utils.h;
	git blame -C records 3 lines as Benjamin Vedder's.

	A stand-in for the firmware's util/mempools.h and utils.h, built only for the host
	tests. It reproduces the declarations the code under test needs, so the
	parts that come from upstream are upstream's -- a stub for a header cannot
	avoid being one.

	This file is part of the ESCargot firmware, a fork of the VESC firmware.

	SPDX-License-Identifier: GPL-3.0-or-later
 */

/*
 * Stands in for the two board-specific things luaif.c needs: the flash the
 * script is read from, and the terminal its output goes to. Everything else
 * -- the kernel, the engine, the arena, the queue -- is the real code.
 *
 * Test scaffolding; never compiled into firmware.
 */

#ifndef LUAIF_HOST_H_
#define LUAIF_HOST_H_

#include <stdint.h>

// Replaces flash_helper_code_data/_size. Returns NULL for "no script".
const uint8_t *luaif_host_code(int32_t *len_out);

// Replaces commands_printf_lisp. Captured so tests can assert on output.
int commands_printf_lisp(const char *format, ...);

// Everything the engine printed since the last reset, NUL terminated.
const char *luaif_host_output(void);
void luaif_host_output_reset(void);

/*
 * Replaces mempools' shared packet buffer. The real one is handed out from a
 * pool shared with the comms path; a single static buffer is equivalent for
 * one caller at a time, which is all luaif_process_cmd ever is.
 */
uint8_t *mempools_get_packet_buffer(void);
void mempools_free_packet_buffer(uint8_t *buffer);

/*
 * Replaces utils.h's nestable scheduler lock. chSysLock is not nestable and
 * the real helpers count, which is why luaif.c uses them rather than locking
 * directly.
 */
void utils_sys_lock_cnt(void);
void utils_sys_unlock_cnt(void);

#endif /* LUAIF_HOST_H_ */
