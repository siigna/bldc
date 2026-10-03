/*
 * ahrs_update_initial_orientation: the attitude a board boots with.
 *
 * Upstream made this load-bearing. The AHRS used to start from identity and
 * converge, which took a visible second or two; it now seeds the quaternion
 * from the gravity vector on the first settled sample, so whatever this
 * function returns *is* the starting attitude. A sign or an axis wrong here
 * is a board that believes it is tilted when it is level, and balance
 * applications act on that before a rider can.
 *
 * Nothing tested it, before or after that change.
 *
 * The checks are properties rather than recorded outputs. Two of them hold
 * whatever Euler convention the function uses, which matters because a test
 * that merely records today's numbers would pass a sign flip tomorrow:
 *
 *   - the quaternion is a unit quaternion, or it is not a rotation at all;
 *   - rotating the body-frame gravity direction by that quaternion has to
 *     reproduce the measured acceleration. That is the entire claim the
 *     function makes, and it is convention-independent.
 *
 * The roll and pitch checks then pin the convention itself, read back through
 * the same accessors the firmware uses.
 */

#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#include "ahrs.h"

static int checks;
static int failures;

static void ok(const char *what, int cond) {
	checks++;
	if (cond) {
		printf("  ok   %s\n", what);
	} else {
		failures++;
		printf("  FAIL %s\n", what);
	}
}

static void near(const char *what, float got, float want, float tol) {
	checks++;
	if (fabsf(got - want) <= tol) {
		printf("  ok   %s (%.4f)\n", what, (double)got);
	} else {
		failures++;
		printf("  FAIL %s: got %.4f, want %.4f +/- %.4f\n",
				what, (double)got, (double)want, (double)tol);
	}
}

/* Rotate a vector from the body frame into the earth frame by q. */
static void rotate_by_q(const ATTITUDE_INFO *att, const float *v, float *out) {
	const float w = att->q0, x = att->q1, y = att->q2, z = att->q3;

	out[0] = v[0]*(w*w + x*x - y*y - z*z) + v[1]*2.0f*(x*y - w*z)
			+ v[2]*2.0f*(x*z + w*y);
	out[1] = v[0]*2.0f*(x*y + w*z) + v[1]*(w*w - x*x + y*y - z*z)
			+ v[2]*2.0f*(y*z - w*x);
	out[2] = v[0]*2.0f*(x*z - w*y) + v[1]*2.0f*(y*z + w*x)
			+ v[2]*(w*w - x*x - y*y + z*z);
}

static ATTITUDE_INFO seed(float ax, float ay, float az) {
	ATTITUDE_INFO att;
	ahrs_init_attitude_info(&att);

	const float accel[3] = { ax, ay, az };
	/* No magnetometer on most of these boards; this is what imu.c passes. */
	const float mag[3] = { 1.0f, 0.0f, 0.0f };

	ahrs_update_initial_orientation(accel, mag, &att);
	return att;
}

static void check_unit_and_roundtrip(const char *name,
		float ax, float ay, float az) {
	char buf[128];
	ATTITUDE_INFO att = seed(ax, ay, az);

	const float norm = sqrtf(att.q0*att.q0 + att.q1*att.q1
			+ att.q2*att.q2 + att.q3*att.q3);
	snprintf(buf, sizeof(buf), "%s: quaternion is a unit quaternion", name);
	near(buf, norm, 1.0f, 1e-4f);

	/*
	 * The seeded attitude says where the board is. Taking the direction
	 * gravity points in the earth frame and rotating it back into the body
	 * frame must give the accelerometer reading it was derived from.
	 */
	const float g_earth[3] = { 0.0f, 0.0f, 1.0f };
	float body[3];

	/* Inverse rotation: conjugate the quaternion. */
	ATTITUDE_INFO inv = att;
	inv.q1 = -att.q1;
	inv.q2 = -att.q2;
	inv.q3 = -att.q3;
	rotate_by_q(&inv, g_earth, body);

	const float mag_in = sqrtf(ax*ax + ay*ay + az*az);
	snprintf(buf, sizeof(buf), "%s: gravity rotates back onto the reading", name);
	ok(buf, fabsf(body[0] - ax/mag_in) < 2e-3f
		&& fabsf(body[1] - ay/mag_in) < 2e-3f
		&& fabsf(body[2] - az/mag_in) < 2e-3f);
}

int main(void) {
	const float d2r = (float)M_PI / 180.0f;
	const float s45 = sqrtf(0.5f);

	printf("ahrs_update_initial_orientation\n");

	/* --- the convention, through the firmware's own accessors ---------- */

	/* Level, z up: no rotation to report. */
	ATTITUDE_INFO level = seed(0.0f, 0.0f, 1.0f);
	near("level: roll is zero", ahrs_get_roll(&level), 0.0f, 0.5f * d2r);
	near("level: pitch is zero", ahrs_get_pitch(&level), 0.0f, 0.5f * d2r);

	/* Rolled: gravity moves onto the y axis. */
	ATTITUDE_INFO rolled = seed(0.0f, s45, s45);
	near("rolled 45 deg: roll follows", ahrs_get_roll(&rolled), -45.0f * d2r,
			1.0f * d2r);
	near("rolled 45 deg: pitch stays level", ahrs_get_pitch(&rolled), 0.0f,
			1.0f * d2r);

	/* Pitched: gravity moves onto the x axis. */
	ATTITUDE_INFO pitched = seed(s45, 0.0f, s45);
	near("pitched 45 deg: pitch follows", ahrs_get_pitch(&pitched), -45.0f * d2r,
			1.0f * d2r);
	near("pitched 45 deg: roll stays level", ahrs_get_roll(&pitched), 0.0f,
			1.0f * d2r);

	/*
	 * Nose straight down. Worth its own case because pitch passes through
	 * the singularity of the atan form the function uses, and the result is
	 * still required to be a valid rotation.
	 */
	ATTITUDE_INFO nose_down = seed(1.0f, 0.0f, 0.0f);
	near("nose down: pitch is a right angle",
			fabsf(ahrs_get_pitch(&nose_down)), 90.0f * d2r, 2.0f * d2r);

	/* --- the convention-independent properties ------------------------ */

	check_unit_and_roundtrip("level", 0.0f, 0.0f, 1.0f);
	check_unit_and_roundtrip("rolled", 0.0f, s45, s45);
	check_unit_and_roundtrip("pitched", s45, 0.0f, s45);
	check_unit_and_roundtrip("rolled the other way", 0.0f, -s45, s45);
	check_unit_and_roundtrip("pitched the other way", -s45, 0.0f, s45);
	check_unit_and_roundtrip("tilted on both axes", 0.3f, 0.4f, 0.866f);

	/*
	 * The magnitude gate upstream added admits 0.7 g to 1.3 g, so a reading
	 * inside that window but not exactly 1 g still has to produce a sane
	 * attitude -- the gate exists to reject settling and acceleration, not to
	 * guarantee a normalised vector.
	 */
	check_unit_and_roundtrip("0.8 g, within the gate", 0.0f, 0.0f, 0.8f);
	check_unit_and_roundtrip("1.25 g, within the gate", 0.0f, 0.0f, 1.25f);

	printf("\n%d checks, %d failures\n", checks, failures);
	return failures ? 1 : 0;
}
