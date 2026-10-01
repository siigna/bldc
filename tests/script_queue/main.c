/*
 * Host tests for the script event queue.
 *
 * Compiled from script/script_queue.c with SCRIPT_QUEUE_HOST_TEST, which
 * builds the same ring arithmetic without the ChibiOS system lock. The test
 * is single-threaded, which is the one thing that makes dropping the lock
 * sound; it tests the arithmetic, not the mutual exclusion.
 *
 * The three rules worth pinning are the ones the producers depend on, and
 * none is visible from the type: a post never blocks, a full queue drops the
 * newest and counts it, and order is preserved. A script reassembling a
 * multi-frame protocol cannot do it from a queue that reorders, and a script
 * that misses frames looks identical from inside to one that is never sent
 * any -- the dropped count is the only thing that separates them.
 */

#include "../../script/script_queue.h"

#include <stdio.h>
#include <string.h>

static int checks;
static int failures;

static void expect(const char *what, long got, long want) {
	checks++;
	if (got != want) {
		failures++;
		printf("FAIL %-50s got %ld, want %ld\n", what, got, want);
	}
}

// An event carrying a recognisable id, so order can be asserted.
static script_event_t mk(uint32_t id) {
	script_event_t e;
	memset(&e, 0, sizeof(e));
	e.type = SCRIPT_EV_CAN_SID;
	e.id = id;
	e.len = 1;
	e.data[0] = (uint8_t)id;
	return e;
}

int main(void) {
	script_queue_t q;
	script_event_t e;

	// --- empty ---
	script_queue_init(&q);
	expect("empty count", script_queue_count(&q), 0);
	expect("empty fetch fails", script_queue_fetch(&q, &e), 0);
	expect("nothing dropped", (long)script_queue_dropped(&q), 0);

	// --- order, which a multi-frame protocol depends on ---
	script_queue_init(&q);
	for (uint32_t i = 1;i <= 5;i++) {
		script_event_t in = mk(i);
		expect("post accepted", script_queue_post(&q, &in), 1);
	}
	expect("five held", script_queue_count(&q), 5);
	for (uint32_t i = 1;i <= 5;i++) {
		char what[48];
		snprintf(what, sizeof(what), "fetch %u in order", i);
		expect(what, script_queue_fetch(&q, &e) ? (long)e.id : -1, (long)i);
	}
	expect("drained", script_queue_count(&q), 0);
	expect("fetch past the end fails", script_queue_fetch(&q, &e), 0);

	// --- the payload survives the copy, not just the id ---
	script_queue_init(&q);
	script_event_t big;
	memset(&big, 0, sizeof(big));
	big.type = SCRIPT_EV_APP_DATA;
	big.id = 0xDEADBEEF;
	big.len = SCRIPT_EVENT_PAYLOAD;
	big.truncated = 1;
	for (int i = 0;i < SCRIPT_EVENT_PAYLOAD;i++) {
		big.data[i] = (uint8_t)(i * 7 + 1);
	}
	script_queue_post(&q, &big);
	script_queue_fetch(&q, &e);
	expect("type survives",      e.type, SCRIPT_EV_APP_DATA);
	expect("id survives",        (long)e.id, (long)0xDEADBEEF);
	expect("len survives",       e.len, SCRIPT_EVENT_PAYLOAD);
	expect("truncated survives", e.truncated, 1);
	expect("payload survives",   memcmp(e.data, big.data, SCRIPT_EVENT_PAYLOAD), 0);

	// --- full: drop the newest, count it, keep the oldest ---
	script_queue_init(&q);
	for (uint32_t i = 1;i <= SCRIPT_QUEUE_LEN;i++) {
		script_event_t in = mk(i);
		expect("fills to capacity", script_queue_post(&q, &in), 1);
	}
	expect("full", script_queue_count(&q), SCRIPT_QUEUE_LEN);

	script_event_t over = mk(999);
	expect("post past full is refused", script_queue_post(&q, &over), 0);
	expect("and counted",               (long)script_queue_dropped(&q), 1);
	expect("and did not grow",          script_queue_count(&q), SCRIPT_QUEUE_LEN);

	for (int i = 0;i < 3;i++) {
		script_queue_post(&q, &over);
	}
	expect("every drop counts", (long)script_queue_dropped(&q), 4);

	// The oldest is still the first one posted: a full queue must not have
	// overwritten the event the engine is about to read.
	expect("oldest preserved", script_queue_fetch(&q, &e) ? (long)e.id : -1, 1);
	expect("room again",       script_queue_post(&q, &over), 1);

	// --- wrap: the ring has to keep working past its own length ---
	script_queue_init(&q);
	uint32_t next_in = 1, next_out = 1;
	for (int round = 0;round < SCRIPT_QUEUE_LEN * 4;round++) {
		script_event_t in = mk(next_in++);
		script_queue_post(&q, &in);
		if (!script_queue_fetch(&q, &e) || e.id != next_out++) {
			failures++;
			printf("FAIL wrap round %d\n", round);
		}
		checks++;
	}
	expect("wrap left it empty",   script_queue_count(&q), 0);
	expect("and dropped nothing",  (long)script_queue_dropped(&q), 0);

	// --- defensive arguments, all reachable from a producer ---
	expect("null queue post",   script_queue_post(NULL, &big), 0);
	expect("null event post",   script_queue_post(&q, NULL), 0);
	expect("null queue fetch",  script_queue_fetch(NULL, &e), 0);
	expect("null out fetch",    script_queue_fetch(&q, NULL), 0);
	expect("null count",        script_queue_count(NULL), 0);
	expect("null dropped",      (long)script_queue_dropped(NULL), 0);
	script_queue_init(NULL);

	printf("\n%d checks, %d failures\n", checks, failures);
	return failures ? 1 : 0;
}
