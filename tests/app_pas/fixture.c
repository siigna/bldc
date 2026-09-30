// The stub firmware app_pas.c is compiled against, shared by the unit tests in
// main.c and the property fuzzer in fuzz.c. It lived in main.c until the
// fuzzer needed the same stubs and could not link a second main().
#include <stdarg.h>
#include <stdio.h>
#include <string.h>
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

// Terminal commands are recorded, not registered, so the tests can call them.
// There are three and the array is checked against that, rather than sized
// generously: a fourth should fail here and be noticed.
#define TEST_MAX_CMDS 8
const char *test_cmd_name[TEST_MAX_CMDS];
void (*test_cmd_fn[TEST_MAX_CMDS])(int argc, const char **argv);
int test_cmd_count = 0;

// What commands_printf produced, so a command's output can be asserted on
// rather than only its not crashing. Truncated rather than grown: a command
// that printed more than this has already told us what we need.
char test_out[8192];
int test_out_len = 0;

void test_out_clear(void) {
	test_out_len = 0;
	test_out[0] = 0;
}

// Stubs for the rest of the firmware that app_pas.c calls.
app_configuration test_appconf;
const app_configuration *app_get_configuration(void) { return &test_appconf; }
bool app_is_output_disabled(void) { return false; }
int commands_printf(const char *format, ...) {
	va_list ap;
	va_start(ap, format);
	int room = (int)sizeof(test_out) - test_out_len - 2;
	if (room > 0) {
		int n = vsnprintf(test_out + test_out_len, (size_t)room, format, ap);
		if (n > 0) {
			test_out_len += n < room ? n : room;
			test_out[test_out_len++] = '\n';
			test_out[test_out_len] = 0;
		}
	}
	va_end(ap);
	return 0;
}

// Replaces by name rather than appending. The suite starts and stops the app
// many times over, and an appending version filled the table early and then
// silently dropped every later registration -- which looked like the terminal
// commands not being registered at all.
void terminal_register_command_callback(const char *command, const char *help,
		const char *arg_names, void(*cbf)(int argc, const char **argv)) {
	(void)help; (void)arg_names;
	for (int i = 0; i < test_cmd_count; i++) {
		if (test_cmd_name[i] && strcmp(test_cmd_name[i], command) == 0) {
			test_cmd_fn[i] = cbf;
			return;
		}
	}
	if (test_cmd_count < TEST_MAX_CMDS) {
		test_cmd_name[test_cmd_count] = command;
		test_cmd_fn[test_cmd_count] = cbf;
		test_cmd_count++;
	}
}

// Clears the callback but keeps the name, so a re-registration lands in the
// same slot and the table stays the size of the command set.
void terminal_unregister_callback(void(*cbf)(int argc, const char **argv)) {
	for (int i = 0; i < test_cmd_count; i++) {
		if (test_cmd_fn[i] == cbf) {
			test_cmd_fn[i] = 0;
		}
	}
}

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
