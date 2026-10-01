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

#endif /* LUAIF_HOST_H_ */
