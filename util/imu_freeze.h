/*
	Copyright 2026 Lukas Hrazky
	Copyright 2026 Stephen Bouche

	This file is part of the ESCargot firmware, a fork of the
	VESC firmware.

	The ESCargot firmware is free software: you can redistribute it and/or
	modify it under the terms of the GNU General Public License as published
	by the Free Software Foundation, either version 3 of the License, or
	(at your option) any later version.

	The ESCargot firmware is distributed in the hope that it will be useful,
	but WITHOUT ANY WARRANTY; without even the implied warranty of
	MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
	GNU General Public License for more details.

	You should have received a copy of the GNU General Public License
	along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef IMU_FREEZE_H_
#define IMU_FREEZE_H_

#include <stdint.h>
#include <stdbool.h>

/*
 * A dead-bus detector for an IMU, lifted out of imu_thread.c so that it can
 * be tested. A read on a dead bus can succeed and return the same bytes
 * forever, so a run of bit-identical samples is the signal -- a live MEMS part
 * always has noise on the low bits.
 *
 * What it protects: with no detector a frozen sensor feeds a constant
 * attitude to whatever is balancing on it, and nothing reports a fault. What
 * it risks in the other direction is a false positive cutting the callback on
 * a vehicle that is upright, so the threshold and the reset conditions both
 * matter, which is the argument for being able to check them off-target.
 */

// Consecutive identical samples before the bus is called dead. Enough
// headroom over the short duplicate runs a poll produces when it transiently
// outruns the sensor's output data rate.
#define IMU_FROZEN_SAMPLES_LIMIT 32

typedef struct {
	uint32_t streak;
	float prev[6];
} imu_freeze_state_t;

void imu_freeze_reset(imu_freeze_state_t *st);

/*
 * Feeds one sample in and says whether the bus should be considered dead.
 * The magnetometer is deliberately not part of the comparison: it has its own
 * data rate and can legitimately repeat while accel and gyro are live.
 */
bool imu_freeze_update(imu_freeze_state_t *st, const float *accel,
		const float *gyro);

#endif /* IMU_FREEZE_H_ */
