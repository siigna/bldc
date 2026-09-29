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

#endif
