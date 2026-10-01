/*
 * Host tests for the on-flash script container parser.
 *
 * The blob this parses is untrusted: it arrives over USB or BLE, survives
 * reboots in flash, and a cancelled upload leaves a partial one behind. Every
 * offset and length in the import table is attacker-controlled, so the tests
 * below spend most of their effort on malformed input rather than on the
 * happy path.
 *
 *   make -C main/script/test
 */

#include "../../script/script_pack.h"

#include <stdio.h>
#include <string.h>
#include <stdlib.h>

static int checks = 0;
static int failures = 0;

static void expect(const char *what, long got, long want) {
	checks++;
	if (got != want) {
		failures++;
		printf("FAIL %-52s got %ld, want %ld\n", what, got, want);
	}
}

static void expect_str(const char *what, const char *got, const char *want) {
	checks++;
	if (!got || strcmp(got, want) != 0) {
		failures++;
		printf("FAIL %-52s got %s, want %s\n", what, got ? got : "(null)", want);
	}
}

// Builder mirroring what tools/luapack.py writes, so the two cannot drift
// without a test noticing.
typedef struct {
	uint8_t buf[4096];
	int32_t len;
} blob_t;

static void put_u16(blob_t *b, uint32_t v) {
	b->buf[b->len++] = (uint8_t)(v >> 8);
	b->buf[b->len++] = (uint8_t)v;
}

static void put_i32(blob_t *b, int32_t v) {
	b->buf[b->len++] = (uint8_t)(v >> 24);
	b->buf[b->len++] = (uint8_t)(v >> 16);
	b->buf[b->len++] = (uint8_t)(v >> 8);
	b->buf[b->len++] = (uint8_t)v;
}

static void blob_start(blob_t *b, uint16_t flags, const char *src) {
	memset(b, 0, sizeof(*b));
	b->len = 0;
	put_i32(b, 0);		// size, filled in by blob_finish
	put_u16(b, 0);		// crc, likewise
	put_u16(b, flags);
	memcpy(b->buf + b->len, src, strlen(src) + 1);
	b->len += (int32_t)strlen(src) + 1;
}

static uint16_t t_crc16(const uint8_t *d, int32_t n) {
	uint16_t crc = 0;
	for (int32_t i = 0; i < n; i++) {
		crc ^= (uint16_t)((uint16_t)d[i] << 8);
		for (int k = 0; k < 8; k++) {
			crc = (crc & 0x8000u) ? (uint16_t)((crc << 1) ^ 0x1021u)
					: (uint16_t)(crc << 1);
		}
	}
	return crc;
}

/*
 * Fill in size and crc the way VESC Tool does: the counted region starts at
 * the flags word and the stored figure is two less than its length.
 */
static void blob_finish(blob_t *b) {
	int32_t body = b->len - 6;
	b->buf[0] = (uint8_t)((body - 2) >> 24);
	b->buf[1] = (uint8_t)((body - 2) >> 16);
	b->buf[2] = (uint8_t)((body - 2) >> 8);
	b->buf[3] = (uint8_t)(body - 2);
	uint16_t crc = t_crc16(b->buf + 6, body);
	b->buf[4] = (uint8_t)(crc >> 8);
	b->buf[5] = (uint8_t)crc;
}


/*
 * The cross-repo anchor.
 *
 * Everything above builds its own containers, which checks the parser against
 * this file's idea of the format -- and would go on passing if the format
 * itself drifted. container.bin is a container
 * vesc_express/tools/luapack.py actually emitted, committed as bytes, so a
 * format change has to break against a real artifact.
 *
 * The expected values are what that packer put in it: a Lua script of 385
 * source bytes with two imports. If this fails after a deliberate format
 * change, refresh container.bin from the packer -- do not relax the test.
 */
static void anchor_tests(void) {
	FILE *f = fopen("container.bin", "rb");
	if (!f) {
		printf("FAIL cannot open container.bin (run from tests/script_pack)\n");
		failures++;
		checks++;
		return;
	}

	static uint8_t blob[64 * 1024];
	size_t n = fread(blob, 1, sizeof(blob), f);
	fclose(f);

	script_blob_t p;
	expect("anchor parses", script_pack_parse(blob, (int32_t)n, &p), 1);
	expect("anchor is flagged Lua", p.lang, SCRIPT_LANG_LUA);
	expect("anchor source length", p.src_len, 385);
	expect("anchor import count", p.num_imports, 2);

	/*
	 * The imports have to be reachable, not merely counted: the offsets are
	 * absolute into the blob, and that is exactly where an off-by-a-header
	 * shows up -- the same class of mistake that put a container inside a
	 * container when vesc_tool learned to embed one.
	 */
	for (int i = 0;i < p.num_imports;i++) {
		const char *nm = NULL;
		const uint8_t *d = NULL;
		int32_t dl = 0;
		char what[64];
		snprintf(what, sizeof(what), "anchor import %d reachable", i);
		expect(what, script_pack_import_at(&p, i, &nm, &d, &dl) &&
				nm != NULL && d != NULL && dl > 0, 1);
	}
}

int main(void) {
	// Happy path, no imports.
	{
		blob_t b;
		blob_start(&b, SCRIPT_FLAG_LANG_LUA, "return 1");
		blob_finish(&b);
		script_blob_t p;
		expect("plain lua blob parses", script_pack_parse(b.buf, b.len, &p), 1);
		expect_str("source", p.src, "return 1");
		expect("source length", p.src_len, 8);
		expect("language is lua", p.lang, SCRIPT_LANG_LUA);
		expect("no imports", p.num_imports, 0);
	}

	// A blob with flags zero is lisp, which is what every package built
	// before the flag existed says.
	{
		blob_t b;
		blob_start(&b, 0, "(print 1)");
		blob_finish(&b);
		script_blob_t p;
		expect("flags zero parses", script_pack_parse(b.buf, b.len, &p), 1);
		expect("language is lisp", p.lang, SCRIPT_LANG_LISP);
	}

	// Import table, two entries.
	{
		blob_t b;
		blob_start(&b, SCRIPT_FLAG_LANG_LUA, "require 'a'");
		int32_t table_at = b.len;
		put_u16(&b, 2);
		// Entries first, payloads appended afterwards, so offsets are known.
		int32_t hdr = table_at + 2 + (int32_t)strlen("a") + 1 + 8 +
				(int32_t)strlen("bee") + 1 + 8;
		int32_t off_a = hdr - SCRIPT_HEADER_SIZE;
		int32_t off_b = off_a + 5;
		memcpy(b.buf + b.len, "a", 2); b.len += 2;
		put_i32(&b, off_a);
		put_i32(&b, 5);
		memcpy(b.buf + b.len, "bee", 4); b.len += 4;
		put_i32(&b, off_b);
		put_i32(&b, 3);
		memcpy(b.buf + b.len, "AAAAA", 5); b.len += 5;
		memcpy(b.buf + b.len, "BBB", 3); b.len += 3;

		blob_finish(&b);
		script_blob_t p;
		expect("blob with imports parses", script_pack_parse(b.buf, b.len, &p), 1);
		expect("two imports", p.num_imports, 2);

		const uint8_t *d = NULL;
		int32_t dl = 0;
		expect("import a found", script_pack_import(&p, "a", &d, &dl), 1);
		expect("import a length", dl, 5);
		checks++;
		if (!d || memcmp(d, "AAAAA", 5) != 0) {
			failures++;
			printf("FAIL import a payload\n");
		}

		expect("import bee found", script_pack_import(&p, "bee", &d, &dl), 1);
		expect("import bee length", dl, 3);
		checks++;
		if (!d || memcmp(d, "BBB", 3) != 0) {
			failures++;
			printf("FAIL import bee payload\n");
		}

		expect("missing import not found",
				script_pack_import(&p, "nope", &d, &dl), 0);

		const char *nm = NULL;
		expect("iterate entry 0", script_pack_import_at(&p, 0, &nm, &d, &dl), 1);
		expect_str("entry 0 name", nm, "a");
		expect("iterate past end", script_pack_import_at(&p, 2, &nm, &d, &dl), 0);
		expect("negative index refused",
				script_pack_import_at(&p, -1, &nm, &d, &dl), 0);
	}

	// Malformed input. None of these may read out of bounds; run this file
	// under ASan in CI so that "did not crash" means something.
	{
		script_blob_t p;
		expect("null blob refused", script_pack_parse(NULL, 100, &p), 0);
		expect("zero length refused", script_pack_parse((uint8_t *)"", 0, &p), 0);

		uint8_t tiny[SCRIPT_HEADER_SIZE] = {0};
		expect("header only refused",
				script_pack_parse(tiny, SCRIPT_HEADER_SIZE, &p), 0);

		// Source with no terminator: the whole payload is source bytes.
		uint8_t unterm[16];
		memset(unterm, 'x', sizeof(unterm));
		expect("unterminated source refused",
				script_pack_parse(unterm, sizeof(unterm), &p), 0);

		// Import count claiming more entries than the blob can hold.
		blob_t b;
		blob_start(&b, SCRIPT_FLAG_LANG_LUA, "x");
		put_u16(&b, 400);
		blob_finish(&b);
		expect("oversized count still parses the source",
				script_pack_parse(b.buf, b.len, &p), 1);
		const uint8_t *d = NULL;
		int32_t dl = 0;
		expect("truncated table yields nothing",
				script_pack_import(&p, "a", &d, &dl), 0);

		// Count above the 500 cap is treated as absent.
		blob_start(&b, SCRIPT_FLAG_LANG_LUA, "x");
		put_u16(&b, 500);
		blob_finish(&b);
		expect("count at the cap parses", script_pack_parse(b.buf, b.len, &p), 1);
		expect("count at the cap means no imports", p.num_imports, 0);

		// An entry whose offset and length point past the end.
		blob_start(&b, SCRIPT_FLAG_LANG_LUA, "x");
		put_u16(&b, 1);
		memcpy(b.buf + b.len, "a", 2); b.len += 2;
		put_i32(&b, 0);
		put_i32(&b, 100000);
		blob_finish(&b);
		expect("out of range length parses", script_pack_parse(b.buf, b.len, &p), 1);
		expect("out of range import refused",
				script_pack_import(&p, "a", &d, &dl), 0);

		// Negative offset, which is representable because the field is signed.
		blob_start(&b, SCRIPT_FLAG_LANG_LUA, "x");
		put_u16(&b, 1);
		memcpy(b.buf + b.len, "a", 2); b.len += 2;
		put_i32(&b, -8);
		put_i32(&b, 4);
		blob_finish(&b);
		expect("negative offset parses", script_pack_parse(b.buf, b.len, &p), 1);
		expect("negative offset refused",
				script_pack_import(&p, "a", &d, &dl), 0);

		// Offset plus length chosen to overflow a 32-bit addition.
		blob_start(&b, SCRIPT_FLAG_LANG_LUA, "x");
		put_u16(&b, 1);
		memcpy(b.buf + b.len, "a", 2); b.len += 2;
		put_i32(&b, 0x7FFFFFFF);
		put_i32(&b, 0x7FFFFFFF);
		blob_finish(&b);
		expect("overflowing pair parses", script_pack_parse(b.buf, b.len, &p), 1);
		expect("overflowing pair refused",
				script_pack_import(&p, "a", &d, &dl), 0);

		// Entry name with no terminator before the end of the blob.
		blob_start(&b, SCRIPT_FLAG_LANG_LUA, "x");
		put_u16(&b, 1);
		memset(b.buf + b.len, 'n', 8);
		b.len += 8;
		blob_finish(&b);
		expect("unterminated entry name parses", script_pack_parse(b.buf, b.len, &p), 1);
		expect("unterminated entry name refused",
				script_pack_import(&p, "nnnnnnnn", &d, &dl), 0);
	}

	// Size and checksum validation. Both fields were ignored originally,
	// which let a container with the size field two bytes out load here and
	// fail only on the other engine, and let a half-erased flash region parse
	// as source.
	{
		blob_t b;
		script_blob_t p;

		blob_start(&b, SCRIPT_FLAG_LANG_LUA, "return 1");
		blob_finish(&b);
		expect("valid container accepted", script_pack_parse(b.buf, b.len, &p), 1);

		// One flipped byte in the payload.
		blob_start(&b, SCRIPT_FLAG_LANG_LUA, "return 1");
		blob_finish(&b);
		b.buf[9] ^= 0x40;
		expect("corrupted payload refused", script_pack_parse(b.buf, b.len, &p), 0);

		// The exact mistake the packer made: size written without the -2.
		blob_start(&b, SCRIPT_FLAG_LANG_LUA, "return 1");
		blob_finish(&b);
		int32_t body = b.len - 6;
		b.buf[0] = (uint8_t)(body >> 24);
		b.buf[1] = (uint8_t)(body >> 16);
		b.buf[2] = (uint8_t)(body >> 8);
		b.buf[3] = (uint8_t)body;
		expect("size field off by two refused",
				script_pack_parse(b.buf, b.len, &p), 0);

		// A size claiming more than the buffer holds.
		blob_start(&b, SCRIPT_FLAG_LANG_LUA, "return 1");
		blob_finish(&b);
		b.buf[0] = 0x7F;
		expect("oversized size field refused",
				script_pack_parse(b.buf, b.len, &p), 0);

		// Trailing bytes past the checksummed region must not be reachable:
		// that is what a partially erased partition looks like.
		blob_start(&b, SCRIPT_FLAG_LANG_LUA, "x");
		blob_finish(&b);
		int32_t real_len = b.len;
		memset(b.buf + b.len, 0xEE, 64);
		b.len += 64;
		expect("trailing flash content ignored",
				script_pack_parse(b.buf, b.len, &p), 1);
		expect("payload bounded by the container, not the partition",
				p.base_len, real_len - SCRIPT_HEADER_SIZE);
	}

	anchor_tests();

	printf("\n%d checks, %d failures\n", checks, failures);
	return failures ? 1 : 0;
}
