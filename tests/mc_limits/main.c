/*
 * The input current limits: how much the pack is allowed to give and take.
 *
 * Upstream added l_in_current_max_scale and l_in_current_min_scale, and these
 * limits are the ones that protect the battery -- the cutoff ramp is what
 * stops a nearly empty pack being pulled to collapse, and the regen ramp is
 * what stops a full one being pushed over voltage. None of it was tested,
 * before the merge or after.
 *
 * The arithmetic is in util/mc_limits.c, lifted unchanged out of
 * update_override_limits so that it could be reached at all; see the note
 * there.
 *
 * Values are chosen clear of the 0.1 V guard bands in the comparisons, except
 * where a case is deliberately about one.
 */

#include <stdio.h>
#include <string.h>
#include <math.h>

#include "mc_limits.h"

static int checks;
static int failures;

static void near(const char *what, float got, float want, float tol) {
	checks++;
	if (fabsf(got - want) <= tol) {
		printf("  ok   %s (%.3f)\n", what, (double)got);
	} else {
		failures++;
		printf("  FAIL %s: got %.3f, want %.3f +/- %.3f\n",
				what, (double)got, (double)want, (double)tol);
	}
}

/*
 * A 12S pack with limits well apart, so each ramp can be probed without the
 * others or the wattage limit interfering. Wattage is set far out of the way
 * and brought in only by the cases about it.
 */
static mc_configuration base_conf(void) {
	mc_configuration c;
	memset(&c, 0, sizeof(c));

	c.l_in_current_max = 60.0f;
	c.l_in_current_min = -20.0f;
	c.l_in_current_max_scale = 1.0f;
	c.l_in_current_min_scale = 1.0f;

	c.l_battery_cut_start = 36.0f;
	c.l_battery_cut_end = 30.0f;
	c.l_battery_regen_cut_start = 50.0f;
	c.l_battery_regen_cut_end = 54.0f;

	c.l_watt_max = 1.0e6f;
	c.l_watt_min = -1.0e6f;

	return c;
}

static mc_in_limits_t at(mc_configuration c, float v_in) {
	mc_in_limits_t out;
	mc_limits_input_current(&c, v_in, &out);
	return out;
}

int main(void) {
	printf("mc_limits_input_current\n");

	const float v_mid = 44.0f;	/* clear of every ramp */

	/* --- the scale factors, which is what upstream added --------------- */

	mc_configuration c = base_conf();
	mc_in_limits_t r = at(c, v_mid);
	near("scale 1.0: draw limit is the configured one", r.in_max_base, 60.0f, 1e-3f);
	near("scale 1.0: regen limit is the configured one", r.in_min_base, -20.0f, 1e-3f);

	c = base_conf();
	c.l_in_current_max_scale = 0.5f;
	c.l_in_current_min_scale = 0.25f;
	r = at(c, v_mid);
	near("draw scaled by half", r.in_max_base, 30.0f, 1e-3f);
	near("regen scaled by a quarter", r.in_min_base, -5.0f, 1e-3f);
	near("scaled draw reaches the output", r.in_max, 30.0f, 1e-3f);
	near("scaled regen reaches the output", r.in_min, -5.0f, 1e-3f);

	c = base_conf();
	c.l_in_current_max_scale = 0.0f;
	r = at(c, v_mid);
	near("a scale of zero means no draw at all", r.in_max, 0.0f, 1e-6f);

	/* --- the battery cutoff ramp -------------------------------------- */

	c = base_conf();
	near("well above the cutoff: full draw", at(c, v_mid).in_max, 60.0f, 1e-3f);
	near("below the cutoff end: no draw", at(c, 29.0f).in_max, 0.0f, 1e-6f);

	/*
	 * Halfway down the ramp, so half the draw. 33 V sits midway between
	 * start 36 and end 30.
	 */
	near("halfway down the ramp: half the draw",
			at(c, 33.0f).in_max, 30.0f, 0.2f);

	/*
	 * The guard bands. The comparison is `v_in > start - 0.1`, so a volt
	 * exactly at the start is still full draw rather than the first step of
	 * the ramp -- worth pinning, because a change to that bound would be
	 * invisible at every other voltage.
	 */
	near("exactly at the cutoff start: still full draw",
			at(c, 36.0f).in_max, 60.0f, 1e-3f);

	/* --- the scale moves the ramp, not just the ceiling --------------- */

	c = base_conf();
	c.l_in_current_max_scale = 0.5f;
	near("halfway down the ramp, scaled: half of the halved draw",
			at(c, 33.0f).in_max, 15.0f, 0.2f);

	/* --- the regen overvoltage ramp ----------------------------------- */

	c = base_conf();
	near("well below the regen cutoff: full regen",
			at(c, v_mid).in_min, -20.0f, 1e-3f);
	near("above the regen cutoff end: no regen",
			at(c, 55.0f).in_min, 0.0f, 1e-6f);
	near("halfway up the regen ramp: half the regen",
			at(c, 52.0f).in_min, -10.0f, 0.2f);

	/* --- the wattage limits ------------------------------------------- */

	c = base_conf();
	c.l_watt_max = 440.0f;	/* 10 A at 44 V, below the 60 A current limit */
	near("a watt limit below the current limit wins",
			at(c, v_mid).in_max, 10.0f, 1e-2f);

	c = base_conf();
	c.l_watt_max = 4400.0f;	/* 100 A at 44 V, above the current limit */
	near("a watt limit above the current limit does not",
			at(c, v_mid).in_max, 60.0f, 1e-2f);

	/*
	 * The same wattage at a lower voltage is more current, which is the
	 * reason this limit is a wattage: 440 W is 10 A at 44 V and 20 A at 22 V.
	 *
	 * The cutoff thresholds move down for this case. The first version left
	 * them at the 12S values and read 22 V, which is below a cut_end of 30 --
	 * so the ramp had already taken the draw to zero and the wattage limit
	 * never got a say. The test failed and was right to: the case as written
	 * was not about what it claimed.
	 */
	c = base_conf();
	c.l_watt_max = 440.0f;
	c.l_battery_cut_start = 18.0f;
	c.l_battery_cut_end = 12.0f;
	near("the watt limit is amps per volt, not a fixed current",
			at(c, 22.0f).in_max, 20.0f, 1e-2f);

	c = base_conf();
	c.l_watt_min = -220.0f;	/* 5 A at 44 V, inside the 20 A regen limit */
	near("a regen watt limit below the current limit wins",
			at(c, v_mid).in_min, -5.0f, 1e-2f);

	/* --- the base values are not themselves ramped -------------------- */

	/*
	 * The caller clamps against these after the BMS has spoken, so they have
	 * to stay the configured ceiling even at a voltage where the ramp has
	 * taken the usable limit to zero. Collapsing them here would let a BMS
	 * raise the limit back up on a flat pack.
	 */
	c = base_conf();
	r = at(c, 29.0f);
	near("the draw ceiling survives the cutoff", r.in_max_base, 60.0f, 1e-3f);
	near("but the usable draw does not", r.in_max, 0.0f, 1e-6f);

	/* --- zero and near-zero input voltage ----------------------------- */

	/*
	 * The wattage limit divides by v_in, so a reading of zero makes it
	 * infinite. utils_min_abs then discards it in favour of the battery
	 * limit, which is the only reason this is safe -- an infinity surviving
	 * into the output would mean no limit at all, which is the worst possible
	 * direction for it to fail.
	 *
	 * Not hypothetical: before upstream seeded the input voltage filters at
	 * init, consumers saw them converge from zero after boot, so this is a
	 * state the limits really were asked about.
	 */
	c = base_conf();
	c.l_watt_max = 1500.0f;
	c.l_watt_min = -1500.0f;
	r = at(c, 0.0f);
	checks++;
	if (isfinite(r.in_max) && isfinite(r.in_min)) {
		printf("  ok   zero volts: both limits stay finite\n");
	} else {
		failures++;
		printf("  FAIL zero volts: in_max=%g in_min=%g\n",
				(double)r.in_max, (double)r.in_min);
	}

	/* And the cutoff still refuses to draw from a pack reading zero. */
	near("zero volts: no draw", r.in_max, 0.0f, 1e-6f);

	/*
	 * Just above zero the wattage limit is a huge but finite current, and
	 * must still lose to the battery limit rather than becoming the binding
	 * one.
	 */
	r = at(c, 0.5f);
	checks++;
	if (isfinite(r.in_max) && fabsf(r.in_max) <= 60.0f) {
		printf("  ok   half a volt: the watt limit does not become the limit\n");
	} else {
		failures++;
		printf("  FAIL half a volt: in_max=%g\n", (double)r.in_max);
	}

	printf("\n%d checks, %d failures\n", checks, failures);
	return failures ? 1 : 0;
}
