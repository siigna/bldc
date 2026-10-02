// Minimal ChibiOS HAL stubs. Pad levels are supplied by the test.
#ifndef HAL_H
#define HAL_H

#include "ch.h"

#define PAL_MODE_INPUT_PULLUP	1

typedef int ioportid_t;
typedef uint32_t iomode_t;

// Provided by the test: the level of each of the two pedal sensor pads.
extern uint8_t test_pad[2];

static inline uint8_t palReadPad(ioportid_t port, uint32_t pin) {
	(void)port;
	return test_pad[pin & 1];
}
static inline void palSetPadMode(ioportid_t port, uint32_t pin, iomode_t mode) {
	(void)port; (void)pin; (void)mode;
}

#endif
