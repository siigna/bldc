/*
 * Fake comm_can: settable status caches and recorded transmissions, so every
 * CAN binding can be checked against what it actually read or sent.
 *
 * The status structs and HW_TYPE come from the firmware's datatypes.h, not
 * copies, so a field a binding reads has to be a field that exists.
 *
 * Test scaffolding; never built into firmware.
 */

#ifndef FAKE_COMM_CAN_H_
#define FAKE_COMM_CAN_H_

#include "datatypes.h"
#include "ch.h"

#ifndef CAN_STATUS_MSGS_TO_STORE
#define CAN_STATUS_MSGS_TO_STORE 10
#endif

typedef struct {
	// id of -1 in a slot means "nothing heard"
	can_status_msg m1[CAN_STATUS_MSGS_TO_STORE];
	can_status_msg_2 m2[CAN_STATUS_MSGS_TO_STORE];
	can_status_msg_3 m3[CAN_STATUS_MSGS_TO_STORE];
	can_status_msg_4 m4[CAN_STATUS_MSGS_TO_STORE];
	can_status_msg_5 m5[CAN_STATUS_MSGS_TO_STORE];

	// what the commanding bindings were given
	int last_id;
	float last_value;
	float last_off_delay;
	int off_delay_calls;
	int set_calls;
	const char *last_call;

	// what the raw senders put on the bus
	uint32_t tx_id;
	uint8_t tx_data[8];
	uint8_t tx_len;
	int tx_sid_calls;
	int tx_eid_calls;
	int tx_fail;			// non-zero makes the next transmit fail

	int ping_calls;
	int ping_answers;		// id that answers; -1 for none
} fake_can_t;

extern fake_can_t fake_can;

void fake_can_reset(void);
// Publish a status frame as if it had just arrived from that controller.
void fake_can_publish(int id, float rpm, float current, float duty,
		float temp_fet, float v_in);

can_status_msg *comm_can_get_status_msg_index(int index);
can_status_msg *comm_can_get_status_msg_id(int id);
can_status_msg_2 *comm_can_get_status_msg_2_id(int id);
can_status_msg_3 *comm_can_get_status_msg_3_id(int id);
can_status_msg_4 *comm_can_get_status_msg_4_id(int id);
can_status_msg_5 *comm_can_get_status_msg_5_id(int id);

bool comm_can_ping(uint8_t controller_id, HW_TYPE *hw_type);

void comm_can_set_current(uint8_t id, float current);
void comm_can_set_current_off_delay(uint8_t id, float current, float d);
void comm_can_set_current_rel(uint8_t id, float rel);
void comm_can_set_current_rel_off_delay(uint8_t id, float rel, float d);
void comm_can_set_duty(uint8_t id, float duty);
void comm_can_set_rpm(uint8_t id, float rpm);
void comm_can_set_pos(uint8_t id, float pos);
void comm_can_set_current_brake(uint8_t id, float current);
void comm_can_set_current_brake_rel(uint8_t id, float rel);

msg_t comm_can_transmit_sid(uint32_t id, const uint8_t *data, uint8_t len);
msg_t comm_can_transmit_eid(uint32_t id, const uint8_t *data, uint8_t len);

#endif /* FAKE_COMM_CAN_H_ */
