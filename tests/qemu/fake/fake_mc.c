#include "mc_interface.h"
#include "timeout.h"

fake_mc_t fake_mc;
int fake_timeout_resets;

void timeout_reset(void) { fake_timeout_resets++; }

float mc_interface_get_rpm(void) { return fake_mc.rpm; }
float mc_interface_get_tot_current_filtered(void) { return fake_mc.current; }
float mc_interface_get_tot_current_directional_filtered(void) { return fake_mc.current_dir; }
float mc_interface_get_tot_current_in_filtered(void) { return fake_mc.current_in; }
float mc_interface_get_duty_cycle_now(void) { return fake_mc.duty; }
float mc_interface_get_input_voltage_filtered(void) { return fake_mc.vin; }
float mc_interface_temp_fet_filtered(void) { return fake_mc.temp_fet; }
float mc_interface_temp_motor_filtered(void) { return fake_mc.temp_motor; }
float mc_interface_get_speed(void) { return fake_mc.speed; }
float mc_interface_get_distance(void) { return fake_mc.dist; }
float mc_interface_get_distance_abs(void) { return fake_mc.dist_abs; }
mc_fault_code mc_interface_get_fault(void) { return fake_mc.fault; }

float mc_interface_get_amp_hours(bool reset) {
	fake_mc.ah_reset = reset;
	return fake_mc.ah;
}
float mc_interface_get_amp_hours_charged(bool reset) {
	fake_mc.ah_chg_reset = reset;
	return fake_mc.ah_chg;
}
float mc_interface_get_watt_hours(bool reset) {
	fake_mc.wh_reset = reset;
	return fake_mc.wh;
}
float mc_interface_get_watt_hours_charged(bool reset) {
	fake_mc.wh_chg_reset = reset;
	return fake_mc.wh_chg;
}
int mc_interface_get_tachometer_value(bool reset) {
	fake_mc.tacho_reset = reset;
	return fake_mc.tacho;
}
int mc_interface_get_tachometer_abs_value(bool reset) {
	fake_mc.tacho_abs_reset = reset;
	return fake_mc.tacho_abs;
}

void mc_interface_set_current(float c) { fake_mc.set_current = c; fake_mc.set_calls++; }
void mc_interface_set_current_rel(float v) { fake_mc.set_current_rel = v; fake_mc.set_calls++; }
void mc_interface_set_current_off_delay(float d) {
	fake_mc.set_off_delay = d;
	fake_mc.off_delay_calls++;
}
void mc_interface_set_duty(float d) { fake_mc.set_duty = d; fake_mc.set_calls++; }
void mc_interface_set_pid_speed(float r) { fake_mc.set_rpm = r; fake_mc.set_calls++; }
void mc_interface_set_pid_pos(float p) { fake_mc.set_pos = p; fake_mc.set_calls++; }
void mc_interface_set_brake_current(float c) { fake_mc.set_brake = c; fake_mc.set_calls++; }
void mc_interface_set_brake_current_rel(float v) { fake_mc.set_brake_rel = v; fake_mc.set_calls++; }
void mc_interface_set_handbrake(float c) { fake_mc.set_handbrake = c; fake_mc.set_calls++; }
void mc_interface_set_handbrake_rel(float v) { fake_mc.set_handbrake_rel = v; fake_mc.set_calls++; }
void mc_interface_release_motor(void) { fake_mc.released++; fake_mc.set_calls++; }
