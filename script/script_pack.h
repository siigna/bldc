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
 * NOTE ON PROVENANCE
 *
 * This file is a copy of vesc_express/main/script/script_pack.h. The two are
 * kept identical on purpose: the container format is a wire contract between
 * four implementations -- the packer in vesc_express/tools/luapack.py, the
 * builder in vesc_tool's codeloader, and the parser in each firmware -- and a
 * format that drifts between them presents as a board that erases its script
 * slot and then runs nothing.
 *
 * tests/script_pack anchors that: alongside the unit tests it parses
 * container.bin, which is a container luapack.py actually produced, so a
 * change to the format here fails against a real artifact rather than only
 * against this file's own idea of one.
 *
 * If you change the format, change it in all four and refresh container.bin.
 */

#ifndef MAIN_SCRIPT_SCRIPT_PACK_H_
#define MAIN_SCRIPT_SCRIPT_PACK_H_

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

/*
 * The on-flash script container, shared by both script engines.
 *
 * This is the layout VESC Tool has always written for LispBM, and nothing
 * about it is lisp-specific: the firmware reads the source as bytes and the
 * import table by name. Keeping it byte-for-byte means the existing upload,
 * erase, run/stop and REPL commands (COMM_LISP_*, 130-139 and 152) carry Lua
 * unchanged, and one package store serves both engines.
 *
 *   offset 0   uint32  size of everything after the crc field
 *   offset 4   uint16  crc16 over everything after the crc field
 *   offset 6   uint16  flags        <- see SCRIPT_FLAG_* below
 *   offset 8   char[]  source, NUL terminated
 *              uint16  number of imports
 *              per import:
 *                 char[]  name, NUL terminated
 *                 int32   offset of the payload, relative to offset 8
 *                 int32   length of the payload
 *
 * All multi-byte fields are big endian, as everywhere else in the protocol.
 *
 * The flags field is the one piece of slack in the original design: VESC Tool
 * writes zero and the firmware has never read it. That makes it the natural
 * place to record which language the source is, so a mixed package store can
 * be told apart without a new command id or a change to the container.
 */

#define SCRIPT_HEADER_SIZE	8

// Bit 0 of the flags word. Zero means LispBM, which is what every package
// built before this existed already says.
#define SCRIPT_FLAG_LANG_MASK	0x0001
#define SCRIPT_FLAG_LANG_LISP	0x0000
#define SCRIPT_FLAG_LANG_LUA	0x0001

typedef enum {
	SCRIPT_LANG_LISP = 0,
	SCRIPT_LANG_LUA = 1,
} script_lang_t;

typedef struct {
	const char *src;	// Source text, NUL terminated. NULL if the blob is unusable.
	int32_t src_len;	// Length of src in bytes, excluding the terminator.
	const uint8_t *base;	// Start of the payload area, i.e. blob + 8.
	int32_t base_len;	// Bytes available from base.
	script_lang_t lang;
	uint16_t flags;
	uint16_t num_imports;	// Zero when there is no import table.
} script_blob_t;

/*
 * Parse a container. `blob` points at offset 0 and `len` is what the flash
 * holds, which may be larger than the script. Returns false and leaves *out
 * zeroed if the blob cannot be trusted.
 */
bool script_pack_parse(const uint8_t *blob, int32_t len, script_blob_t *out);

/*
 * Look an import up by name. On success sets *data and *data_len to point
 * inside the blob and returns true. Bounds are checked against the blob, so a
 * truncated or hostile table cannot produce an out-of-range pointer.
 */
bool script_pack_import(const script_blob_t *b, const char *name,
		const uint8_t **data, int32_t *data_len);

/*
 * Iterate the import table. Index is zero based; returns false once past the
 * end. `name` is a pointer into the blob and is NUL terminated.
 */
bool script_pack_import_at(const script_blob_t *b, int index,
		const char **name, const uint8_t **data, int32_t *data_len);

#endif /* MAIN_SCRIPT_SCRIPT_PACK_H_ */
