/*
 * A stand-in for ChibiOS's hal.h.
 *
 * LispBM's platform_timestamp.h includes <hal.h>, but nothing behind it uses
 * a HAL symbol -- the platform layer calls chMtx*, chThd* and the virtual
 * timer, all of which are kernel. This harness is kernel-only by design, and
 * the real hal.h reaches a board header and from there every peripheral LLD.
 *
 * Test scaffolding; never built into firmware.
 */

#ifndef FAKE_HAL_H_
#define FAKE_HAL_H_

#include "ch.h"

#endif /* FAKE_HAL_H_ */
