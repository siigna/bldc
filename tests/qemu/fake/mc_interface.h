/*
 * A fake mc_interface, placed earlier on the include path than the real one
 * so script/lua_vesc_mc.c compiles against it unchanged. The bindings carry
 * no test seam of their own as a result.
 *
 * Every getter returns a value the test sets, and every setter records what
 * it was given, so a binding that reads the wrong field or drops a unit
 * conversion is visible. Test scaffolding; never built into firmware.
 */

#ifndef FAKE_MC_INTERFACE_H_
#define FAKE_MC_INTERFACE_H_

#include <stdbool.h>
#include <stdint.h>

/*
 * The real fault enum, not a copy of it. datatypes.h is the firmware's own
 * and costs nothing to include here, so the codes a test asserts on cannot
 * drift away from the ones the firmware reports.
 */
#include "datatypes.h"

typedef struct {
	// what the getters return
	float rpm, current, current_dir, current_in, duty, vin;
	float temp_fet, temp_motor, speed, dist, dist_abs;
	float ah, ah_chg, wh, wh_chg;
	int tacho, tacho_abs;
	mc_fault_code fault;

	// what the setters were given, and how many times
	float set_current, set_current_rel, set_off_delay, set_duty;
	// counted, not just recorded: set_current_off_delay(0) is not the same
	// as never calling it -- zero clears a delay set earlier.
	int off_delay_calls;
	float set_rpm, set_pos, set_brake, set_brake_rel;
	float set_handbrake, set_handbrake_rel;
	int released;
	int set_calls;

	// configuration, and how many times a full reconfigure was asked for
	mc_configuration mcconf;
	int mcconf_applied;

	// reset flags the counter getters were passed
	bool ah_reset, ah_chg_reset, wh_reset, wh_chg_reset;
	bool tacho_reset, tacho_abs_reset;
} fake_mc_t;

extern fake_mc_t fake_mc;

/* getters */
float mc_interface_get_rpm(void);
float mc_interface_get_tot_current_filtered(void);
float mc_interface_get_tot_current_directional_filtered(void);
float mc_interface_get_tot_current_in_filtered(void);
float mc_interface_get_duty_cycle_now(void);
float mc_interface_get_input_voltage_filtered(void);
float mc_interface_temp_fet_filtered(void);
float mc_interface_temp_motor_filtered(void);
float mc_interface_get_speed(void);
float mc_interface_get_distance(void);
float mc_interface_get_distance_abs(void);
float mc_interface_get_amp_hours(bool reset);
float mc_interface_get_amp_hours_charged(bool reset);
float mc_interface_get_watt_hours(bool reset);
float mc_interface_get_watt_hours_charged(bool reset);
int mc_interface_get_tachometer_value(bool reset);
int mc_interface_get_tachometer_abs_value(bool reset);
mc_fault_code mc_interface_get_fault(void);

/* configuration */
const volatile mc_configuration *mc_interface_get_configuration(void);
void mc_interface_set_configuration(mc_configuration *configuration);

/* setters */
void mc_interface_set_current(float current);
void mc_interface_set_current_rel(float val);
void mc_interface_set_current_off_delay(float delay_sec);
void mc_interface_set_duty(float dutyCycle);
void mc_interface_set_pid_speed(float rpm);
void mc_interface_set_pid_pos(float pos);
void mc_interface_set_brake_current(float current);
void mc_interface_set_brake_current_rel(float val);
void mc_interface_set_handbrake(float current);
void mc_interface_set_handbrake_rel(float val);
void mc_interface_release_motor(void);

#endif /* FAKE_MC_INTERFACE_H_ */
