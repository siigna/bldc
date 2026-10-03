/*
	Copyright 2026 Stephen Bouche

	This file is part of the ESCargot firmware.

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

#include "imu_freeze.h"

#include <string.h>

void imu_freeze_reset(imu_freeze_state_t *st) {
	st->streak = 0;
	memset(st->prev, 0, sizeof(st->prev));
}

bool imu_freeze_update(imu_freeze_state_t *st, const float *accel,
		const float *gyro) {
	if (memcmp(st->prev, accel, sizeof(float) * 3) == 0 &&
			memcmp(st->prev + 3, gyro, sizeof(float) * 3) == 0) {
		// Saturated rather than wrapping: this runs at the sensor's data rate
		// for as long as the board is powered.
		if (st->streak < IMU_FROZEN_SAMPLES_LIMIT) {
			st->streak++;
		}
	} else {
		st->streak = 0;
		memcpy(st->prev, accel, sizeof(float) * 3);
		memcpy(st->prev + 3, gyro, sizeof(float) * 3);
	}

	return st->streak >= IMU_FROZEN_SAMPLES_LIMIT;
}
