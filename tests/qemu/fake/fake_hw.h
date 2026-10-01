/*
 * A minimal board header, standing in for a real hwconf one.
 *
 * Deliberately defines ADC_IND_EXT, ADC_IND_EXT2 and ADC_IND_TEMP_MOTOR and
 * leaves ADC_IND_EXT3 through ADC_IND_EXT8 undefined, so that the real
 * hwconf/hw.h aliases them to ADC_IND_EXT exactly as it does on a board with
 * only two external pins. That aliasing is what lua_vesc_io.c's get_adc has
 * to detect, so the test has to exercise the real thing rather than a copy of
 * it.
 *
 * Test scaffolding; never built into firmware.
 */

#ifndef FAKE_HW_H_
#define FAKE_HW_H_

#define HW_NAME					"FAKE"

#define ADC_IND_EXT				0
#define ADC_IND_EXT2			1
#define ADC_IND_TEMP_MOTOR		2

#define FAKE_ADC_CHANNELS		8
extern float fake_adc_volts[FAKE_ADC_CHANNELS];

#define V_REG					3.3
#define ADC_VOLTS(ch)			(fake_adc_volts[(ch)])

#endif /* FAKE_HW_H_ */
