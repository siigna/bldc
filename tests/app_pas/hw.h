// Fake board. The pedal sensor pin and torque sensor configurations are chosen
// by -D flags from the Makefile so that each combination can be built.
#ifndef HW_H
#define HW_H

#ifndef TEST_NO_PAS_PINS
#define HW_PAS1_PORT	0
#define HW_PAS1_PIN		0
#define HW_PAS2_PORT	0
#define HW_PAS2_PIN		1
#endif

#ifdef TEST_PAS_PINS_SHARED
#define HW_PAS_PINS_SHARED_WITH_UART
#endif

#ifdef TEST_TORQUE_SENSOR
#define HW_HAS_PAS_TORQUE_SENSOR
float hw_get_PAS_torque(void);
#endif

// ADC. The channel numbers are arbitrary but distinct, as on a real board.
// EXT1 to EXT3 exist here and EXT4 to EXT8 do not, which is the common case.
#define V_REG					3.3
#define ADC_IND_EXT				6
#define ADC_IND_EXT2			7
#define ADC_IND_EXT3			10

// Mirrors the aliasing in hwconf/hw.h, which is what makes testing a channel
// with #ifdef useless and is why app_pas.c compares the values.
#ifndef ADC_IND_EXT2
#define ADC_IND_EXT2			ADC_IND_EXT
#endif
#ifndef ADC_IND_EXT3
#define ADC_IND_EXT3			ADC_IND_EXT
#endif
#ifndef ADC_IND_EXT4
#define ADC_IND_EXT4			ADC_IND_EXT
#endif
#ifndef ADC_IND_EXT5
#define ADC_IND_EXT5			ADC_IND_EXT
#endif
#ifndef ADC_IND_EXT6
#define ADC_IND_EXT6			ADC_IND_EXT
#endif
#ifndef ADC_IND_EXT7
#define ADC_IND_EXT7			ADC_IND_EXT
#endif
#ifndef ADC_IND_EXT8
#define ADC_IND_EXT8			ADC_IND_EXT
#endif

#define ADC_VOLTS(ch)			((float)ADC_Value[ch] / 4096.0 * V_REG)

#endif
