/*
 * Settable pedal-assist state for the stand-in PAS bindings. Test
 * scaffolding; never built into firmware. See fake_pas.c.
 */

#ifndef FAKE_PAS_H_
#define FAKE_PAS_H_

#include <stdbool.h>

typedef struct {
	float rpm;				// cadence
	float torque_nm;
	float torque_ratio;
	float rider_power;
	float assist_power;		// the motor power target
	float assist_basis_power;
	float measured_power;
	float output;			// relative current PAS is asking for
	float speed_taper;
	int flags;
	bool torque_saturated;

	// What the script asked for, so a test can assert on the keepalive.
	bool walk_requested;
	int walk_calls;
} fake_pas_t;

extern fake_pas_t fake_pas;

void fake_pas_reset(void);

#endif /* FAKE_PAS_H_ */
