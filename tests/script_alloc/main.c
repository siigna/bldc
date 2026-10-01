/*
 * Host tests for the script heap's realloc semantics.
 *
 * Compiled from script/script_alloc.c with SCRIPT_ALLOC_HOST_TEST, which
 * backs the arena with malloc instead of a ChibiOS heap over CCM. So this
 * tests the realloc arithmetic and the ceiling, not the arena -- and the
 * ceiling is the part that matters, because it is what a runaway script hits
 * first by design.
 *
 * Lua treats a NULL return as an out-of-memory it can raise and a pcall can
 * catch. Every rule below exists because getting it wrong turns that into
 * something worse than an error: a silent truncation, a dangling pointer
 * handed back during recovery, or a ceiling that wedges shut.
 */

#include "../../script/script_alloc.h"

// Test-only hook, defined in script_alloc.c under SCRIPT_ALLOC_HOST_TEST.
void script_alloc_test_fail_next(int n);

#include <stdio.h>
#include <string.h>
#include <stdlib.h>

static int checks;
static int failures;

static void expect(const char *what, long got, long want) {
	checks++;
	if (got != want) {
		failures++;
		printf("FAIL %-52s got %ld, want %ld\n", what, got, want);
	}
}

static void ok(const char *what, int cond) {
	checks++;
	if (!cond) {
		failures++;
		printf("FAIL %s\n", what);
	}
}

static uint8_t arena[64 * 1024];

int main(void) {
	// --- before init, nothing is served ---
	ok("no alloc before init", script_alloc(NULL, NULL, 0, 16) == NULL);

	script_alloc_init(arena, sizeof(arena), 1024);
	expect("used starts at zero", (long)script_alloc_used(), 0);
	expect("peak starts at zero", (long)script_alloc_peak(), 0);

	// --- allocate, account, free ---
	void *a = script_alloc(NULL, NULL, 0, 100);
	ok("alloc returns memory", a != NULL);
	expect("used tracks the request", (long)script_alloc_used(), 100);

	void *b = script_alloc(NULL, NULL, 0, 200);
	ok("second alloc", b != NULL);
	expect("used accumulates", (long)script_alloc_used(), 300);
	expect("peak follows",     (long)script_alloc_peak(), 300);

	ok("free returns NULL", script_alloc(NULL, a, 100, 0) == NULL);
	expect("used drops on free", (long)script_alloc_used(), 200);
	expect("peak is a high-water mark", (long)script_alloc_peak(), 300);

	// --- the contents survive a grow, and a shrink keeps the head ---
	unsigned char *p = script_alloc(NULL, NULL, 0, 16);
	for (int i = 0;i < 16;i++) {
		p[i] = (unsigned char)(i + 1);
	}
	p = script_alloc(NULL, p, 16, 64);
	ok("grow succeeds", p != NULL);
	expect("grow copied the old bytes", memcmp(p, "\1\2\3\4\5\6\7\10\11\12\13\14\15\16\17\20", 16), 0);
	expect("used reflects the new size", (long)script_alloc_used(), 200 + 64);

	p = script_alloc(NULL, p, 64, 8);
	ok("shrink succeeds", p != NULL);
	expect("shrink kept the head", memcmp(p, "\1\2\3\4\5\6\7\10", 8), 0);
	expect("used reflects the shrink", (long)script_alloc_used(), 200 + 8);

	script_alloc(NULL, p, 8, 0);
	script_alloc(NULL, b, 200, 0);
	expect("all freed", (long)script_alloc_used(), 0);

	// --- the ceiling ---
	script_alloc_init(arena, sizeof(arena), 1000);
	void *big = script_alloc(NULL, NULL, 0, 1000);
	ok("exactly the ceiling is allowed", big != NULL);
	ok("one byte past is refused", script_alloc(NULL, NULL, 0, 1) == NULL);
	expect("a refusal does not account", (long)script_alloc_used(), 1000);

	// A failed grow must leave the original alive: freeing it on failure
	// hands Lua a dangling pointer at the moment it is trying to recover.
	unsigned char *keep = script_alloc(NULL, big, 1000, 16);
	ok("shrink to make room", keep != NULL);
	memcpy(keep, "survive", 8);
	void *grown = script_alloc(NULL, keep, 16, 2000);
	ok("grow past the ceiling is refused", grown == NULL);
	expect("and the original is intact", memcmp(keep, "survive", 8), 0);
	expect("and still accounted",        (long)script_alloc_used(), 16);
	script_alloc(NULL, keep, 16, 0);

	// A shrink is allowed even when usage is already over the ceiling, which
	// it can be when init lowers it. Refusing it would wedge the ceiling
	// shut: the only way down is through a shrink.
	script_alloc_init(arena, sizeof(arena), 4096);
	void *held = script_alloc(NULL, NULL, 0, 4096);
	ok("fill to the ceiling", held != NULL);
	void *smaller = script_alloc(NULL, held, 4096, 64);
	ok("shrink at the ceiling is allowed", smaller != NULL);
	expect("and frees the difference", (long)script_alloc_used(), 64);
	script_alloc(NULL, smaller, 64, 0);

	// --- a ceiling above the arena is clamped to it ---
	script_alloc_init(arena, 2048, 1024 * 1024);
	ok("an impossible ceiling is clamped",
			script_alloc(NULL, NULL, 0, 4096) == NULL);

	// --- defensive arguments ---
	script_alloc_init(NULL, 1024, 512);
	ok("init with no arena disables the allocator",
			script_alloc(NULL, NULL, 0, 16) == NULL);
	script_alloc_init(arena, 0, 512);
	ok("init with a zero arena disables it too",
			script_alloc(NULL, NULL, 0, 16) == NULL);

	// A free of something never counted must not underflow the accounting:
	// an underflowed used would wedge the ceiling shut for the rest of the
	// run.
	script_alloc_init(arena, sizeof(arena), 1024);
	void *one = script_alloc(NULL, NULL, 0, 8);
	script_alloc(NULL, one, 9999, 0);
	expect("an oversized free clamps to zero", (long)script_alloc_used(), 0);
	void *again = script_alloc(NULL, NULL, 0, 8);
	ok("and the allocator still works", again != NULL);
	script_alloc(NULL, again, 8, 0);

	// --- a failed arena allocation leaves the original alive ---
	//
	// The ceiling refuses before the arena is asked, so arena exhaustion is
	// a separate path and one the host cannot reach on its own: malloc
	// obliges. The hook makes it reachable, because the rule it guards --
	// a failed resize does not free what it failed to replace -- matters
	// exactly when a script is recovering from an out-of-memory.
	script_alloc_init(arena, sizeof(arena), 64 * 1024);
	unsigned char *live = script_alloc(NULL, NULL, 0, 32);
	memcpy(live, "original", 9);

	script_alloc_test_fail_next(1);
	void *failed = script_alloc(NULL, live, 32, 64);
	ok("a failed arena grow returns NULL", failed == NULL);
	expect("and the original is not freed", memcmp(live, "original", 9), 0);
	expect("and is still accounted for",    (long)script_alloc_used(), 32);

	// Still usable afterwards: a failure must not poison the allocator.
	void *after = script_alloc(NULL, live, 32, 64);
	ok("and the allocator recovers", after != NULL);
	expect("recovered grow kept the bytes", memcmp(after, "original", 9), 0);
	script_alloc(NULL, after, 64, 0);

	// A fresh allocation that fails must not account for anything.
	script_alloc_test_fail_next(1);
	ok("a failed fresh alloc returns NULL",
			script_alloc(NULL, NULL, 0, 16) == NULL);
	expect("and accounts for nothing", (long)script_alloc_used(), 0);

	printf("\n%d checks, %d failures\n", checks, failures);
	fflush(stdout);
	return failures ? 1 : 0;
}
