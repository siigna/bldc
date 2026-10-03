/*
 * A stand-in for the firmware's conf_general.h.
 *
 * imu/ahrs.h includes it, and all it needs from there is ATTITUDE_INFO, which
 * datatypes.h defines -- and datatypes.h on its own needs nothing but fixed
 * width integers and bool. The real conf_general.h reaches the board headers
 * and from there every peripheral, which is why including it is the one thing
 * standing between this function and a host test.
 *
 * Test scaffolding; never built into firmware.
 */

#ifndef FAKE_CONF_GENERAL_H_
#define FAKE_CONF_GENERAL_H_

#include "datatypes.h"

#endif /* FAKE_CONF_GENERAL_H_ */
