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

static inline void chThdSleep(systime_t t) { (void)t; }
static inline void chThdSleepMilliseconds(uint32_t t) { (void)t; }
static inline void chRegSetThreadName(const char *n) { (void)n; }

#define THD_WORKING_AREA(name, size)	uint32_t name[(size) / 4]
#define THD_FUNCTION(name, arg)			void name(void *arg)
static inline thread_t *chThdCreateStatic(void *wa, size_t size, int prio,
		void (*fn)(void *), void *arg) {
	(void)wa; (void)size; (void)prio; (void)fn; (void)arg;
	return 0;
}

#endif
