// The stub firmware app_pas.c is compiled against, shared by the unit tests in
// main.c and the property fuzzer in fuzz.c. It lived in main.c until the
// fuzzer needed the same stubs and could not link a second main().
#include <stdarg.h>
#include <stdbool.h>
#include <stdint.h>

#include "ch.h"
#include "hal.h"
#include "datatypes.h"
#include "app.h"

// Stub state referenced by the stub headers, and written by the tests.
systime_t test_now = 0;
uint8_t test_pad[2] = {0, 0};
mc_fault_code test_fault = FAULT_CODE_NONE;
float test_current_rel = 0.0;
volatile uint16_t ADC_Value[16] = {0};
float test_v_in = 50.0;
float test_speed = 0.0;
float test_current_in = 0.0;
mc_configuration test_mcconf;

// Stubs for the rest of the firmware that app_pas.c calls.
app_configuration test_appconf;
const app_configuration *app_get_configuration(void) { return &test_appconf; }
bool app_is_output_disabled(void) { return false; }
int commands_printf(const char *format, ...) { (void)format; return 0; }
void terminal_register_command_callback(const char *command, const char *help,
		const char *arg_names, void(*cbf)(int argc, const char **argv)) {
	(void)command; (void)help; (void)arg_names; (void)cbf;
}
void terminal_unregister_callback(void(*cbf)(int argc, const char **argv)) { (void)cbf; }

// The preset command writes the configuration. Recorded rather than performed.
static app_configuration test_appconf_scratch;
app_configuration *mempools_alloc_appconf(void) { return &test_appconf_scratch; }
void mempools_free_appconf(app_configuration *p) { (void)p; }
// cppcheck-suppress constParameterPointer ; these two match the firmware's
// own prototypes, which app_pas.c is compiled against.
bool conf_general_store_app_configuration(app_configuration *c) {
	test_appconf = *c;
	return true;
}
// cppcheck-suppress constParameterPointer
void app_set_configuration(app_configuration *c) { test_appconf = *c; }

// Defined unconditionally now. The board override is only compiled into
// app_pas.c under HW_HAS_PAS_TORQUE_SENSOR, so an unused definition here costs
// nothing and saves the fuzzer a second build variant.
float test_torque = 0.0;
float hw_get_PAS_torque(void) { return test_torque; }
