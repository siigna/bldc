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

#ifndef MC_LIMITS_H_
#define MC_LIMITS_H_

#include "datatypes.h"

/*
 * The input current limits, as arithmetic over the configuration and the
 * measured input voltage. Lifted out of update_override_limits in
 * mc_interface.c, unchanged, so that it can be tested: that function is static
 * in a three-thousand-line file that reaches the ADC macros, the FOC state and
 * the board headers, and this part of it referenced none of those.
 *
 * Everything here is a limit the rider feels and some of it is a limit that
 * protects the pack, so being able to check the ramps against arithmetic
 * rather than against a bench is worth the indirection.
 */

typedef struct {
	// l_in_current_{min,max} with their scale factors applied. The caller
	// needs these separately: the BMS is allowed to limit within them, and
	// the final clamp is against them rather than against the ramped values.
	float in_min_base;
	float in_max_base;

	// After the battery cutoff ramp, the regen overvoltage ramp and the
	// wattage limits, and before the BMS has had its say.
	float in_min;
	float in_max;
} mc_in_limits_t;

void mc_limits_input_current(const volatile mc_configuration *conf,
		float v_in, mc_in_limits_t *out);

#endif /* MC_LIMITS_H_ */
