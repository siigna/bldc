// Minimal ChibiOS stubs for the host test. The virtual timer is driven by the
// test so that pedal timing is deterministic and uptime can be fast-forwarded.
#ifndef CH_H
#define CH_H

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

#include "sched.h"

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

// Sleeping is where control changes hands, which is what makes the app's own
// thread runnable here. See sched.h.
//
// From inside the thread it advances the virtual clock by the time slept and
// returns to the test. From the test it runs the thread to its next sleep,
// which is what lets app_pas_stop's wait loop actually complete.
//
// The clock only moves while a thread exists. Advancing it unconditionally
// broke every test that drives pedal timing tick by tick, because app_pas_stop
// sleeps while it waits and those tests depend on owning the clock.
static inline void chThdSleep(systime_t t) {
	if (sched_in_thread()) {
		test_now += t;
	}
	sched_switch();
}
static inline void chThdSleepMilliseconds(uint32_t t) {
	chThdSleep((systime_t)t * CH_CFG_ST_FREQUENCY / 1000);
}
static inline void chRegSetThreadName(const char *n) { (void)n; }

#define THD_WORKING_AREA(name, size)	uint32_t name[(size) / 4]
#define THD_FUNCTION(name, arg)			void name(void *arg)
// The thread is handed to the cooperative scheduler rather than started.
// pas_thread is static inside app_pas.c, so this is the only seam through which
// a test can reach it, and it needs no change to the firmware.
static inline thread_t *chThdCreateStatic(void *wa, size_t size, int prio,
		void (*fn)(void *), void *arg) {
	(void)wa; (void)size; (void)prio;
	sched_create(fn, arg);
	return 0;
}

#endif
