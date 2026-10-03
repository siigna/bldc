/*
	Copyright 2016 - 2022 Benjamin Vedder	benjamin@vedder.se
	Copyright 2026 Stephen Bouche

	The input current limit arithmetic in this file was moved here from
	mc_interface.c unchanged; it is Benjamin Vedder's work, with the input
	current scale factors added upstream after that. What is new here is the
	function boundary, the struct and the comments.

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

#include "mc_limits.h"
#include "utils_math.h"

void mc_limits_input_current(const volatile mc_configuration *conf,
		float v_in, mc_in_limits_t *out) {
	/*
	 * The scale factors are upstream's addition. They multiply the configured
	 * limits rather than replacing them, so a scale of 1 is the old
	 * behaviour and the ramps below act on the scaled values -- which means a
	 * scale of 0.5 halves the cutoff ramp's starting point too, not just the
	 * ceiling.
	 */
	const float l_in_current_min_tmp = conf->l_in_current_min * conf->l_in_current_min_scale;
	const float l_in_current_max_tmp = conf->l_in_current_max * conf->l_in_current_max_scale;

	// Battery cutoff
	float lo_in_max_batt = 0.0;
	if (v_in > (conf->l_battery_cut_start - 0.1)) {
		lo_in_max_batt = l_in_current_max_tmp;
	} else if (v_in < (conf->l_battery_cut_end + 0.1)) {
		lo_in_max_batt = 0.0;
	} else {
		lo_in_max_batt = utils_map(v_in, conf->l_battery_cut_start,
				conf->l_battery_cut_end, l_in_current_max_tmp, 0.0);
	}

	// Regen overvoltage cutoff
	float lo_in_min_batt = 0.0;
	if (v_in < (conf->l_battery_regen_cut_start + 0.1)) {
		lo_in_min_batt = l_in_current_min_tmp;
	} else if (v_in > (conf->l_battery_regen_cut_end - 0.1)) {
		lo_in_min_batt = 0.0;
	} else {
		lo_in_min_batt = utils_map(v_in, conf->l_battery_regen_cut_start,
				conf->l_battery_regen_cut_end, l_in_current_min_tmp, 0.0);
	}

	// Wattage limits
	const float lo_in_max_watt = conf->l_watt_max / v_in;
	const float lo_in_min_watt = conf->l_watt_min / v_in;

	out->in_min_base = l_in_current_min_tmp;
	out->in_max_base = l_in_current_max_tmp;
	out->in_max = utils_min_abs(lo_in_max_watt, lo_in_max_batt);
	out->in_min = utils_min_abs(lo_in_min_watt, lo_in_min_batt);
}
