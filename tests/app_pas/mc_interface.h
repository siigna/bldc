#ifndef MC_INTERFACE_H
#define MC_INTERFACE_H
#include "datatypes.h"

// The real declaration lives in motor/mc_interface.h.
extern volatile uint16_t ADC_Value[16];
// Provided by the test.
extern mc_fault_code test_fault;
extern float test_current_rel;
static inline mc_fault_code mc_interface_get_fault(void) { return test_fault; }
static inline void mc_interface_set_current_rel(float v) { test_current_rel = v; }
#endif
