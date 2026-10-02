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

/*
 * NOTE ON PROVENANCE
 *
 * This file is a copy of vesc_express/main/script/script_pack.c. The two are
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

/*
 * Parsing of the on-flash script container. No firmware dependencies on
 * purpose: this is the one part of the script engine that is pure arithmetic
 * over untrusted bytes, so it is the part worth testing exhaustively on a
 * host. See main/script/test.
 *
 * Everything here treats the blob as hostile. It arrives over the packet
 * protocol from whatever is on the other end of a USB or BLE link, it is
 * stored in flash across reboots, and a truncated upload leaves a partial one
 * behind. An offset or length taken on trust is an out-of-bounds read at
 * best.
 */

#include "script_pack.h"

#include <string.h>

/*
 * Bounded strlen, written out here rather than calling the POSIX one so this
 * file needs no feature macros and compiles identically on the target and in
 * the host tests. Returns max when no terminator is found within max bytes.
 */
static int32_t bounded_strlen(const char *s, int32_t max) {
	int32_t i = 0;
	while (i < max && s[i] != '\0') {
		i++;
	}
	return i;
}

/*
 * Both of these assemble in unsigned arithmetic and convert at the end. The
 * obvious version shifts the top byte left 24 places as a signed int, which
 * is undefined once that byte reaches 0x80 -- UBSan caught exactly that here,
 * on the first run of the tests below.
 */
/*
 * CRC16-CCITT, the polynomial the packet protocol uses, written out here so
 * this file keeps no dependencies and stays host-testable.
 */
static uint16_t pack_crc16(const uint8_t *data, int32_t len) {
	uint16_t crc = 0;
	for (int32_t i = 0; i < len; i++) {
		crc ^= (uint16_t)((uint16_t)data[i] << 8);
		for (int b = 0; b < 8; b++) {
			crc = (crc & 0x8000u) ? (uint16_t)((crc << 1) ^ 0x1021u)
					: (uint16_t)(crc << 1);
		}
	}
	return crc;
}

static int32_t get_u16(const uint8_t *b, int32_t *ind) {
	uint32_t v = ((uint32_t)b[*ind] << 8) | (uint32_t)b[*ind + 1];
	*ind += 2;
	return (int32_t)v;
}

static int32_t get_i32(const uint8_t *b, int32_t *ind) {
	uint32_t v = ((uint32_t)b[*ind] << 24) | ((uint32_t)b[*ind + 1] << 16) |
			((uint32_t)b[*ind + 2] << 8) | (uint32_t)b[*ind + 3];
	*ind += 4;
	return (int32_t)v;
}

bool script_pack_parse(const uint8_t *blob, int32_t len, script_blob_t *out) {
	if (!out) {
		return false;
	}
	memset(out, 0, sizeof(*out));

	if (!blob || len <= SCRIPT_HEADER_SIZE) {
		return false;
	}

	/*
	 * Check the stored length and checksum before trusting anything else.
	 *
	 * This was skipped originally -- the partition size was used and these
	 * two fields ignored -- and that cost twice. A container written with the
	 * size field two bytes out loaded here and failed only on the lisp
	 * engine, which does check; and a script written into flash that had not
	 * really been erased parsed as source and surfaced as "unexpected symbol
	 * near '<238>'" rather than as the bad checksum it was.
	 *
	 * The stored size counts from the flags word and is two less than that
	 * region's length, which is what VESC Tool writes.
	 */
	int32_t ind = 0;
	int32_t stored_size = get_i32(blob, &ind);
	uint16_t stored_crc = (uint16_t)get_u16(blob, &ind);

	if (stored_size < 2) {
		return false;
	}

	int64_t body_len = (int64_t)stored_size + 2;
	if (body_len > (int64_t)len - 6) {
		return false;	// longer than the flash holds: truncated or garbage
	}

	if (pack_crc16(blob + 6, (int32_t)body_len) != stored_crc) {
		return false;
	}

	uint16_t flags = (uint16_t)get_u16(blob, &ind);

	const uint8_t *base = blob + SCRIPT_HEADER_SIZE;
	/*
	 * Bounded by the container's own length rather than by the partition.
	 * Anything past the checksummed region is whatever was in flash before
	 * and must not be reachable as an import payload.
	 */
	int32_t base_len = (int32_t)(body_len - 2);

	/*
	 * The source has to be NUL terminated inside the blob. strnlen returning
	 * the full length means the terminator is missing, which is what a
	 * truncated upload looks like, and going on would read past the end.
	 */
	int32_t src_len = bounded_strlen((const char *)base, base_len);
	if (src_len >= base_len) {
		return false;
	}

	out->src = (const char *)base;
	out->src_len = src_len;
	out->base = base;
	out->base_len = base_len;
	out->flags = flags;
	out->lang = (flags & SCRIPT_FLAG_LANG_MASK) == SCRIPT_FLAG_LANG_LUA ?
			SCRIPT_LANG_LUA : SCRIPT_LANG_LISP;

	// An import table is optional. Two bytes for the count have to fit after
	// the source and its terminator, or there simply is not one.
	int32_t tbl = src_len + 1;
	if (base_len - tbl >= 2) {
		int32_t t = tbl;
		int32_t n = get_u16(base, &t);
		// 500 is the cap the lisp import extension has always applied. Keep
		// it: a plausible table is small, and a huge count is either
		// corruption or an attempt to walk us off the end.
		if (n > 0 && n < 500) {
			out->num_imports = (uint16_t)n;
		}
	}

	return true;
}

bool script_pack_import_at(const script_blob_t *b, int index,
		const char **name, const uint8_t **data, int32_t *data_len) {
	if (!b || !b->base || index < 0 || index >= (int)b->num_imports) {
		return false;
	}

	int32_t ind = b->src_len + 1 + 2;

	for (int i = 0; i <= index; i++) {
		if (ind >= b->base_len) {
			return false;
		}

		const char *entry = (const char *)(b->base + ind);
		int32_t name_len = bounded_strlen(entry, b->base_len - ind);
		if (ind + name_len >= b->base_len) {
			return false;	// Unterminated name.
		}
		ind += name_len + 1;

		// Two int32 fields must fit before they are read.
		if (b->base_len - ind < 8) {
			return false;
		}
		int32_t offset = get_i32(b->base, &ind);
		int32_t payload_len = get_i32(b->base, &ind);

		if (i != index) {
			continue;
		}

		/*
		 * Both fields are signed on the wire and both come from outside, so
		 * check for negatives as well as for the end of the blob, and do the
		 * addition in 64 bits so it cannot wrap.
		 */
		if (offset < 0 || payload_len < 0 ||
				(int64_t)offset + (int64_t)payload_len > (int64_t)b->base_len) {
			return false;
		}

		if (name) {
			*name = entry;
		}
		if (data) {
			*data = b->base + offset;
		}
		if (data_len) {
			*data_len = payload_len;
		}
		return true;
	}

	return false;
}

bool script_pack_import(const script_blob_t *b, const char *name,
		const uint8_t **data, int32_t *data_len) {
	if (!b || !name) {
		return false;
	}

	for (int i = 0; i < (int)b->num_imports; i++) {
		const char *entry = NULL;
		const uint8_t *d = NULL;
		int32_t dl = 0;
		if (!script_pack_import_at(b, i, &entry, &d, &dl)) {
			return false;
		}
		if (entry && strcmp(entry, name) == 0) {
			if (data) {
				*data = d;
			}
			if (data_len) {
				*data_len = dl;
			}
			return true;
		}
	}

	return false;
}
