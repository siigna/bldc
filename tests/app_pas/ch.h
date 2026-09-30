// Minimal ChibiOS stubs for the host test. The virtual timer is driven by the
// test so that pedal timing is deterministic and uptime can be fast-forwarded.
#ifndef CH_H
#define CH_H

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

#define CH_CFG_ST_FREQUENCY	10000
#define NORMALPRIO			64

typedef uint32_t systime_t;
typedef struct { uint32_t *p_stklimit; } thread_t;

// Provided by the test.
extern systime_t test_now;

static inline systime_t chVTGetSystemTimeX(void) { return test_now; }
static inline systime_t chVTTimeElapsedSinceX(systime_t t) { return test_now - t; }

#define ST2MS(n)	(((n) * 1000UL + CH_CFG_ST_FREQUENCY - 1UL) / CH_CFG_ST_FREQUENCY)
#define MS2ST(m)	((systime_t)(((m) * CH_CFG_ST_FREQUENCY + 999UL) / 1000UL))

// Sleeping advances the virtual clock, which is what lets the app's own thread
// be run here: the loop reads the elapsed time every iteration, so a sleep that
// did nothing would give it a dt of zero forever.
//
// test_on_sleep is how a test ends the loop. The thread runs until stop_now is
// set, and only the app itself can set that, so a test counts sleeps and calls
// app_pas_stop from the hook. Null by default, which is every other test.
extern void (*test_on_sleep)(void);

// Only advances the clock while a test has asked for it, by installing the
// hook. Advancing unconditionally broke every existing test: app_pas_stop
// sleeps while it waits, so calling it moved the virtual clock, and the tests
// that drive pedal timing tick by tick depend on owning that clock entirely.
static inline void chThdSleep(systime_t t) {
	if (test_on_sleep) {
		test_now += t;
		test_on_sleep();
	}
}
static inline void chThdSleepMilliseconds(uint32_t t) {
	chThdSleep((systime_t)t * CH_CFG_ST_FREQUENCY / 1000);
}
static inline void chRegSetThreadName(const char *n) { (void)n; }

#define THD_WORKING_AREA(name, size)	uint32_t name[(size) / 4]
#define THD_FUNCTION(name, arg)			void name(void *arg)
// The thread entry point is recorded rather than started. pas_thread is static
// inside app_pas.c, so this is the only seam through which a test can reach it,
// and it needs no change to the firmware.
extern void (*test_thread_fn)(void *);

static inline thread_t *chThdCreateStatic(void *wa, size_t size, int prio,
		void (*fn)(void *), void *arg) {
	(void)wa; (void)size; (void)prio; (void)arg;
	test_thread_fn = fn;
	return 0;
}

#endif
