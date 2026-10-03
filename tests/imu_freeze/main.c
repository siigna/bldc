/*
 * The IMU dead-bus detector.
 *
 * A read on a dead bus can succeed and return the same bytes forever, so a run
 * of bit-identical samples is the signal. Upstream added this and it arrived
 * without a test, which is worth fixing in both directions: with no detector a
 * frozen sensor feeds a constant attitude to whatever is balancing on it and
 * nothing reports a fault, and with a detector that fires early the callback
 * stops on a vehicle that is upright.
 *
 * So the threshold is pinned exactly, and so is every way the streak resets.
 * The arithmetic is in util/imu_freeze.c, lifted unchanged out of
 * imu_thread.c's sample loop; see the note there.
 */

#include <stdio.h>
#include <string.h>

#include "imu_freeze.h"

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

/* Feeds the same sample n times and returns whether it ended up frozen. */
static bool feed_same(imu_freeze_state_t *st, int n) {
	const float accel[3] = { 0.01f, -0.02f, 0.98f };
	const float gyro[3] = { 0.5f, -0.25f, 0.125f };
	bool frozen = false;

	for (int i = 0; i < n; i++) {
		frozen = imu_freeze_update(st, accel, gyro);
	}

	return frozen;
}

int main(void) {
	imu_freeze_state_t st;
	const float a0[3] = { 0.01f, -0.02f, 0.98f };
	const float g0[3] = { 0.5f, -0.25f, 0.125f };

	printf("imu_freeze_update\n");

	/* --- the threshold, exactly ---------------------------------------- */

	/*
	 * The first sample cannot be a repeat of anything, and must not be
	 * treated as one -- a detector that counted it would fire one sample
	 * early forever.
	 */
	imu_freeze_reset(&st);
	ok("the first sample is never frozen", imu_freeze_update(&st, a0, g0) == false);

	/*
	 * The limit counts identical samples, and the first of a run establishes
	 * the value rather than repeating it -- so the boundary is where an
	 * off-by-one would sit. One short of the limit is still live.
	 */
	imu_freeze_reset(&st);
	ok("from a reset, limit calls is one repeat short",
			feed_same(&st, IMU_FROZEN_SAMPLES_LIMIT) == false);

	imu_freeze_reset(&st);
	ok("from a reset, one more call freezes",
			feed_same(&st, IMU_FROZEN_SAMPLES_LIMIT + 1) == true);

	/*
	 * And it stays frozen rather than flapping, while the data stays
	 * identical.
	 */
	ok("it stays frozen while nothing changes", feed_same(&st, 50) == true);

	/*
	 * The streak saturates. This runs at the sensor's data rate for as long
	 * as the board is powered, so a counter that wrapped would un-freeze a
	 * dead bus after about half an hour at 1 kHz.
	 */
	ok("the streak saturates rather than wrapping",
			st.streak == IMU_FROZEN_SAMPLES_LIMIT);

	/* --- every way it has to reset ------------------------------------- */

	/* A change in one accel axis is enough. */
	imu_freeze_reset(&st);
	feed_same(&st, IMU_FROZEN_SAMPLES_LIMIT + 5);
	const float a_moved[3] = { 0.02f, -0.02f, 0.98f };
	ok("a change in accel x clears it",
			imu_freeze_update(&st, a_moved, g0) == false);

	imu_freeze_reset(&st);
	feed_same(&st, IMU_FROZEN_SAMPLES_LIMIT + 5);
	const float a_moved_z[3] = { 0.01f, -0.02f, 0.9801f };
	ok("a change in accel z clears it",
			imu_freeze_update(&st, a_moved_z, g0) == false);

	/*
	 * And a change in gyro alone, with accel bit-identical. A detector that
	 * compared only accel would miss a stationary board with a live gyro,
	 * and -- worse -- would call a perfectly still board on a bench dead.
	 */
	imu_freeze_reset(&st);
	feed_same(&st, IMU_FROZEN_SAMPLES_LIMIT + 5);
	const float g_moved[3] = { 0.5f, -0.25f, 0.126f };
	ok("a change in gyro alone clears it",
			imu_freeze_update(&st, a0, g_moved) == false);

	/*
	 * After a change the count starts again -- but from one repeat fewer than
	 * a fresh reset needs, and the difference is worth stating because it
	 * caught this test out.
	 *
	 * The streak counts *repeats*, so freezing takes IMU_FROZEN_SAMPLES_LIMIT
	 * of them. From a fresh reset the first sample is spent establishing the
	 * baseline, so 33 calls are needed. After a change, the changing sample
	 * already established the baseline, so 32 more identical ones freeze. The
	 * first version of this case assumed the fresh-reset count applied here
	 * too and called a correct implementation early.
	 */
	imu_freeze_reset(&st);
	feed_same(&st, IMU_FROZEN_SAMPLES_LIMIT + 5);
	imu_freeze_update(&st, a_moved, g0);	/* the change, sets the baseline */

	{
		bool frozen = false;

		for (int i = 0; i < IMU_FROZEN_SAMPLES_LIMIT - 1; i++) {
			frozen = imu_freeze_update(&st, a_moved, g0);
		}

		ok("after a change, one repeat short is still live", frozen == false);
		ok("after a change, the next repeat freezes again",
				imu_freeze_update(&st, a_moved, g0) == true);
	}

	/* --- the magnetometer is deliberately not part of it -------------- */

	/*
	 * imu_freeze_update takes accel and gyro and nothing else, which is the
	 * decision rather than an omission: the magnetometer has its own data
	 * rate and legitimately repeats while the other two are live. This check
	 * is here so that adding mag to the comparison has to be deliberate --
	 * it would make a live board look dead between mag updates.
	 */
	imu_freeze_reset(&st);
	ok("a run of identical accel and gyro freezes regardless of mag",
			feed_same(&st, IMU_FROZEN_SAMPLES_LIMIT + 1) == true);

	/* --- reset clears the stored sample too --------------------------- */

	/*
	 * Not just the count. imu_thread.c calls the reset when the thread
	 * starts, and a stale previous sample would let the first real sample
	 * count as a repeat.
	 */
	imu_freeze_reset(&st);
	ok("reset clears the remembered sample", st.streak == 0
		&& st.prev[0] == 0.0f && st.prev[5] == 0.0f);

	printf("\n%d checks, %d failures\n", checks, failures);
	return failures ? 1 : 0;
}
