#include "comm_can.h"
#include <string.h>

fake_can_t fake_can;

void fake_can_reset(void) {
	memset(&fake_can, 0, sizeof(fake_can));
	for (int i = 0; i < CAN_STATUS_MSGS_TO_STORE; i++) {
		fake_can.m1[i].id = -1;
		fake_can.m2[i].id = -1;
		fake_can.m3[i].id = -1;
		fake_can.m4[i].id = -1;
		fake_can.m5[i].id = -1;
	}
	fake_can.ping_answers = -1;
}

void fake_can_publish(int id, float rpm, float current, float duty,
		float temp_fet, float v_in) {
	for (int i = 0; i < CAN_STATUS_MSGS_TO_STORE; i++) {
		if (fake_can.m1[i].id >= 0) {
			continue;
		}
		systime_t now = chVTGetSystemTimeX();

		fake_can.m1[i].id = id;
		fake_can.m1[i].rx_time = now;
		fake_can.m1[i].rpm = rpm;
		fake_can.m1[i].current = current;
		fake_can.m1[i].duty = duty;

		fake_can.m2[i].id = id;
		fake_can.m2[i].rx_time = now;
		fake_can.m2[i].amp_hours = 2.0f;
		fake_can.m2[i].amp_hours_charged = 3.0f;

		fake_can.m3[i].id = id;
		fake_can.m3[i].rx_time = now;
		fake_can.m3[i].watt_hours = 4.0f;
		fake_can.m3[i].watt_hours_charged = 5.0f;

		fake_can.m4[i].id = id;
		fake_can.m4[i].rx_time = now;
		fake_can.m4[i].temp_fet = temp_fet;
		fake_can.m4[i].temp_motor = 55.0f;
		fake_can.m4[i].current_in = 6.0f;
		fake_can.m4[i].pid_pos_now = 7.0f;

		fake_can.m5[i].id = id;
		fake_can.m5[i].rx_time = now;
		fake_can.m5[i].v_in = v_in;
		fake_can.m5[i].tacho_value = 9999;
		return;
	}
}

#define FIND(arr, type)                                                      \
	for (int i = 0; i < CAN_STATUS_MSGS_TO_STORE; i++) {                     \
		if (fake_can.arr[i].id == id) {                                      \
			return &fake_can.arr[i];                                         \
		}                                                                    \
	}                                                                        \
	return NULL;

can_status_msg *comm_can_get_status_msg_id(int id) { FIND(m1, can_status_msg) }
can_status_msg_2 *comm_can_get_status_msg_2_id(int id) { FIND(m2, can_status_msg_2) }
can_status_msg_3 *comm_can_get_status_msg_3_id(int id) { FIND(m3, can_status_msg_3) }
can_status_msg_4 *comm_can_get_status_msg_4_id(int id) { FIND(m4, can_status_msg_4) }
can_status_msg_5 *comm_can_get_status_msg_5_id(int id) { FIND(m5, can_status_msg_5) }

can_status_msg *comm_can_get_status_msg_index(int index) {
	if ((index < 0) || (index >= CAN_STATUS_MSGS_TO_STORE)) {
		return NULL;
	}
	return &fake_can.m1[index];
}

bool comm_can_ping(uint8_t controller_id, HW_TYPE *hw_type) {
	fake_can.ping_calls++;
	if ((int)controller_id == fake_can.ping_answers) {
		*hw_type = HW_TYPE_VESC;
		return true;
	}
	return false;
}

#define RECORD(nm, v)                                                        \
	fake_can.last_id = (int)id;                                              \
	fake_can.last_value = (v);                                               \
	fake_can.last_call = (nm);                                               \
	fake_can.set_calls++;

void comm_can_set_current(uint8_t id, float c) { RECORD("current", c) }
void comm_can_set_current_rel(uint8_t id, float r) { RECORD("current_rel", r) }
void comm_can_set_duty(uint8_t id, float d) { RECORD("duty", d) }
void comm_can_set_rpm(uint8_t id, float r) { RECORD("rpm", r) }
void comm_can_set_pos(uint8_t id, float p) { RECORD("pos", p) }
void comm_can_set_current_brake(uint8_t id, float c) { RECORD("brake", c) }
void comm_can_set_current_brake_rel(uint8_t id, float r) { RECORD("brake_rel", r) }

void comm_can_set_current_off_delay(uint8_t id, float c, float d) {
	RECORD("current", c)
	fake_can.last_off_delay = d;
	fake_can.off_delay_calls++;
}

void comm_can_set_current_rel_off_delay(uint8_t id, float r, float d) {
	RECORD("current_rel", r)
	fake_can.last_off_delay = d;
	fake_can.off_delay_calls++;
}

static msg_t transmit(uint32_t id, const uint8_t *data, uint8_t len) {
	fake_can.tx_id = id;
	fake_can.tx_len = len;
	memset(fake_can.tx_data, 0, sizeof(fake_can.tx_data));
	if (len > 0) {
		memcpy(fake_can.tx_data, data,
				(len <= sizeof(fake_can.tx_data)) ? len
						: sizeof(fake_can.tx_data));
	}
	return fake_can.tx_fail ? MSG_TIMEOUT : MSG_OK;
}

msg_t comm_can_transmit_sid(uint32_t id, const uint8_t *data, uint8_t len) {
	fake_can.tx_sid_calls++;
	return transmit(id, data, len);
}

msg_t comm_can_transmit_eid(uint32_t id, const uint8_t *data, uint8_t len) {
	fake_can.tx_eid_calls++;
	return transmit(id, data, len);
}
