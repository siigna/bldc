/*
 * Settable inputs for the stand-in io bindings. Test scaffolding; never built
 * into firmware. See fake_io.c for what these do and do not cover.
 */

#ifndef FAKE_IO_H_
#define FAKE_IO_H_

#include <stdbool.h>

typedef struct {
	float ppm;
	float ppm_age;
	bool ppm_age_is_nil;		// as if the decoder had never run

	float adc_volts;
	bool adc_is_nil;			// as if the board had no such pin

	float servo_out;
	float encoder_deg;

	int disable_output_ms;
	int disable_output_calls;
	bool output_disabled;
} fake_io_t;

extern fake_io_t fake_io;

void fake_io_reset(void);

#endif /* FAKE_IO_H_ */
