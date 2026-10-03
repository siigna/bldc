/*
	Copyright 2026 Lukas Hrazky
	Copyright 2026 Stephen Bouche

	The reference implementation below is Lukas Hrazky's, reproduced from
	imu_thread.c as it stood before the detector was moved into
	util/imu_freeze.c. It is here to be proven equivalent to, not to be built.

	This file is part of the ESCargot firmware, a fork of the VESC firmware.

	SPDX-License-Identifier: GPL-3.0-or-later
 */

/*
 * Proof: moving the dead-bus detector out of imu_thread.c did not change what
 * it does.
 *
 * The commit that moved it said "behaviour unchanged", and the evidence was
 * that the lines were character-identical and a test of twelve cases passed.
 * Neither is a proof. This is: CBMC explores every sequence the model admits
 * and reports a counterexample if the two ever disagree.
 *
 * By induction over the state, not by exploring sequences. The first attempt
 * ran 34 steps with a free choice of sample at each -- 2^34 paths, which did
 * not finish -- and needed samples drawn from a two-element alphabet to keep
 * the state space down, so it would have proven something about the
 * abstraction rather than about the code.
 *
 * One step from an arbitrary state is cheaper and claims more. Equal states
 * in, one step each, equal states out; together with the base case that a
 * reset leaves them equal, that is equivalence for every sequence of every
 * length, with no bound and no alphabet. Samples and the stored sample are
 * free 32-bit floats here, compared bytewise exactly as the implementations
 * compare them.
 *
 * What this does not cover. NaN is admitted and handled as the code handles
 * it -- bytewise, so two identical payloads compare equal, which is the
 * behaviour under proof rather than a gap. And it proves the C under CBMC's
 * semantics with no undefined behaviour reported; it says nothing about the
 * compiler or the hardware.
 */

#include <stdint.h>
#include <stdbool.h>
#include <string.h>

#include "imu_freeze.h"

/*
 * Declared, not left implicit. Without these CBMC's nondeterministic choices
 * come back as int: the first run reported an overflow converting one to
 * uint32_t, and -- the part that actually matters -- every float below would
 * have been an int widened to float, so the proof would have covered only
 * sample values that happen to be integers while claiming to cover all of
 * them. A proof that quietly proves less than it says is worse than no proof.
 */
uint32_t nondet_uint32_t(void);
float nondet_float(void);

/* ---- the original, before the move -------------------------------------
 *
 * Reproduced exactly, including the saturating increment and the order of the
 * memcmp/memcpy, with the two file statics as locals so two copies can run
 * side by side.
 */

#define FROZEN_SAMPLES_LIMIT 32

typedef struct {
	uint32_t m_frozen_streak;
	float m_prev_sample[6];
} ref_state_t;

static bool ref_update(ref_state_t *st, const float *accel, const float *gyro) {
	if (memcmp(st->m_prev_sample, accel, sizeof(float) * 3) == 0 &&
			memcmp(st->m_prev_sample + 3, gyro, sizeof(float) * 3) == 0) {
		if (st->m_frozen_streak < FROZEN_SAMPLES_LIMIT) {
			st->m_frozen_streak++;
		}
	} else {
		st->m_frozen_streak = 0;
		memcpy(st->m_prev_sample, accel, sizeof(float) * 3);
		memcpy(st->m_prev_sample + 3, gyro, sizeof(float) * 3);
	}

	return st->m_frozen_streak >= FROZEN_SAMPLES_LIMIT;
}

/* ---- the proof ---------------------------------------------------------- */

int main(void) {
	/* --- base case: a reset puts both in the same state ---------------- */

	ref_state_t ref;
	imu_freeze_state_t ours;

	ref.m_frozen_streak = 0;
	memset(ref.m_prev_sample, 0, sizeof(ref.m_prev_sample));
	imu_freeze_reset(&ours);

	__CPROVER_assert(ours.streak == ref.m_frozen_streak,
			"base case: reset leaves the counters equal");
	__CPROVER_assert(memcmp(ours.prev, ref.m_prev_sample,
			sizeof(ours.prev)) == 0,
			"base case: reset leaves the stored samples equal");

	/* --- step case: from any equal state, one step keeps them equal ---- */

	uint32_t streak = nondet_uint32_t();

	/*
	 * The invariant the induction carries. Both implementations saturate, so
	 * no reachable state has a streak above the limit -- and the step proof
	 * below re-establishes it, which is what makes assuming it here sound
	 * rather than circular.
	 */
	__CPROVER_assume(streak <= FROZEN_SAMPLES_LIMIT);

	float prev[6];
	float accel[3];
	float gyro[3];

	for (int i = 0; i < 6; i++) {
		prev[i] = nondet_float();
	}
	for (int i = 0; i < 3; i++) {
		accel[i] = nondet_float();
		gyro[i] = nondet_float();
	}

	ref.m_frozen_streak = streak;
	memcpy(ref.m_prev_sample, prev, sizeof(prev));
	ours.streak = streak;
	memcpy(ours.prev, prev, sizeof(prev));

	const bool want = ref_update(&ref, accel, gyro);
	const bool got = imu_freeze_update(&ours, accel, gyro);

	__CPROVER_assert(got == want,
			"step: the extracted detector agrees with the original");
	__CPROVER_assert(ours.streak == ref.m_frozen_streak,
			"step: the counters stay equal");
	__CPROVER_assert(memcmp(ours.prev, ref.m_prev_sample,
			sizeof(ours.prev)) == 0,
			"step: the stored samples stay equal");

	/* --- the properties, as theorems rather than samples -------------- */

	__CPROVER_assert(ours.streak <= IMU_FROZEN_SAMPLES_LIMIT,
			"the streak never exceeds the limit, so it cannot wrap");
	__CPROVER_assert(got == (ours.streak == IMU_FROZEN_SAMPLES_LIMIT),
			"frozen is exactly the streak being at the limit");

	/*
	 * A sample that differs always clears the streak. This is the direction
	 * that matters on a board in use: a live sensor must never be called dead.
	 */
	if (memcmp(prev, accel, sizeof(float) * 3) != 0 ||
			memcmp(prev + 3, gyro, sizeof(float) * 3) != 0) {
		__CPROVER_assert(ours.streak == 0 && got == false,
				"a sample that differs always clears the streak");
	}

	return 0;
}
