/*
	Copyright 2026 Stephen Bouche

	This file is part of the VESC firmware.

	The VESC firmware is free software: you can redistribute it and/or modify
	it under the terms of the GNU General Public License as published by
	the Free Software Foundation, either version 3 of the License, or
	(at your option) any later version.

	The VESC firmware is distributed in the hope that it will be useful,
	but WITHOUT ANY WARRANTY; without even the implied warranty of
	MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
	GNU General Public License for more details.

	You should have received a copy of the GNU General Public License
	along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

/*
 * A controller, over TCP, with no controller.
 *
 * Serves enough of the VESC protocol that ESCargot Tool considers itself
 * connected and will poll real-time values. The point is to make the Tool's
 * Android and desktop paths testable with no hardware: a connection is the
 * gate in front of the log settings, the real-time pages and everything
 * reached from them.
 *
 * Why this lives in the firmware tree rather than the Tool's: the protocol is
 * defined here. It uses this repository's own comm/packet.c and util/crc.c,
 * so the framing, the length encoding and the CRC are the firmware's
 * implementation and not a second one written from the specification. That is
 * the half of a protocol test that is worth having -- an independent
 * reimplementation of framing mostly tests the reimplementation.
 *
 * What is NOT the firmware's: the replies. Those are assembled here, from the
 * layouts in comm/commands.c, and a drift between the two is exactly what
 * this cannot catch. The response layout is therefore kept next to a comment
 * naming the case in commands.c it mirrors.
 *
 * Deliberately not attempted: anything that needs the configuration blobs.
 * COMM_GET_MCCONF and COMM_GET_APPCONF carry a confgenerator serialisation
 * whose signature the Tool checks against its own XML, and getting that wrong
 * means the Tool rejects the whole blob. Serving those properly means linking
 * confgenerator.c and the hardware headers behind it; until then the Tool
 * connects, reads values and logs, and configuration pages stay empty.
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <unistd.h>
#include <errno.h>
#include <signal.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <arpa/inet.h>

#include "packet.h"

// From conf_general.h, which cannot be included here: it pulls in the
// hardware configuration and ChibiOS. Checked against it by the Makefile.
#ifndef SIM_FW_VERSION_MAJOR
#define SIM_FW_VERSION_MAJOR 7
#endif
#ifndef SIM_FW_VERSION_MINOR
#define SIM_FW_VERSION_MINOR 2
#endif
#ifndef SIM_FW_TEST_VERSION_NUMBER
#define SIM_FW_TEST_VERSION_NUMBER 1
#endif

#define SIM_HW_NAME "SIM"
#define SIM_FW_NAME ""

// datatypes.h, COMM_PACKET_ID.
#define COMM_FW_VERSION      0
#define COMM_GET_VALUES      4
#define COMM_SET_MCCONF      13
#define COMM_GET_MCCONF      14
#define COMM_GET_APPCONF     17
#define COMM_TERMINAL_CMD    20
#define COMM_GET_VALUES_SETUP 47
#define COMM_PING_CAN        62
#define COMM_GET_IMU_DATA    65

#define HW_TYPE_VESC 0

static int client_fd = -1;
static PACKET_STATE_t packet_state;
static int verbose = 0;
static unsigned long packets_in = 0;
static unsigned long packets_out = 0;

static void buf_u32(uint8_t *b, int32_t *i, uint32_t v)
{
	b[(*i)++] = v >> 24;
	b[(*i)++] = v >> 16;
	b[(*i)++] = v >> 8;
	b[(*i)++] = v;
}

static void buf_i16(uint8_t *b, int32_t *i, int16_t v)
{
	b[(*i)++] = (uint16_t)v >> 8;
	b[(*i)++] = (uint16_t)v;
}

static void buf_u16(uint8_t *b, int32_t *i, uint16_t v)
{
	b[(*i)++] = v >> 8;
	b[(*i)++] = v;
}

static void buf_i32(uint8_t *b, int32_t *i, int32_t v)
{
	buf_u32(b, i, (uint32_t)v);
}

/* Scaled integers, the way buffer_append_float* does it. */
static void buf_f16(uint8_t *b, int32_t *i, float v, float scale)
{
	buf_i16(b, i, (int16_t)(v * scale));
}

static void buf_f32(uint8_t *b, int32_t *i, float v, float scale)
{
	buf_i32(b, i, (int32_t)(v * scale));
}

/* Handed to packet.c as its transmit function. */
static void send_raw(unsigned char *data, unsigned int len)
{
	if (client_fd < 0) {
		return;
	}

	size_t off = 0;
	while (off < len) {
		ssize_t n = write(client_fd, data + off, len - off);
		if (n <= 0) {
			if (errno == EINTR) {
				continue;
			}
			return;
		}
		off += (size_t)n;
	}
}

static void reply(uint8_t *buf, int32_t len)
{
	packets_out++;
	packet_send_packet(buf, (unsigned int)len, &packet_state);
}

/* Mirrors `case COMM_FW_VERSION` in comm/commands.c. */
static void send_fw_version(void)
{
	uint8_t b[128];
	int32_t ind = 0;

	b[ind++] = COMM_FW_VERSION;
	b[ind++] = SIM_FW_VERSION_MAJOR;
	b[ind++] = SIM_FW_VERSION_MINOR;

	strcpy((char *)(b + ind), SIM_HW_NAME);
	ind += (int32_t)strlen(SIM_HW_NAME) + 1;

	/* STM32_UUID_8, 12 bytes. Fixed, so a test can assert on it. */
	for (int i = 0; i < 12; i++) {
		b[ind++] = (uint8_t)(0xA0 + i);
	}

	b[ind++] = 0;                             /* pairing_done */
	b[ind++] = SIM_FW_TEST_VERSION_NUMBER;
	b[ind++] = HW_TYPE_VESC;
	b[ind++] = 0;                             /* conf_custom_cfg_num */
	b[ind++] = 0;                             /* phase filters */
	b[ind++] = 0;                             /* qmlui hw */
	b[ind++] = 0;                             /* qmlui app */
	b[ind++] = 0;                             /* nrf_flags */

	strcpy((char *)(b + ind), SIM_FW_NAME);
	ind += (int32_t)strlen(SIM_FW_NAME) + 1;

	buf_u32(b, &ind, 0);                      /* main_calc_hw_crc */

	reply(b, ind);
}

/*
 * Mirrors `case COMM_GET_VALUES` in comm/commands.c.
 *
 * For a plain COMM_GET_VALUES the mask is all ones and is NOT appended, so
 * every field goes out in bit order. One wrong field or scale shifts
 * everything after it, and the Tool shows plausible nonsense rather than
 * failing -- so this list was extracted from commands.c rather than written
 * from memory, and `make check-layout` re-extracts it and fails if the two
 * have drifted.
 *
 * Note bit 22: the PAS fields are this fork's own extension to
 * COMM_GET_VALUES. A simulator that omitted them would leave the Tool
 * reading six fields of nothing, and that coupling between the two
 * repositories is among the things worth having a test for.
 *
 * The values move slowly and deterministically, so a log written against
 * this has more than one distinct row. A logging test that cannot tell a
 * working logger from a stuck one is not worth running.
 */
static void send_values(void)
{
	static int tick = 0;
	tick++;

	float ramp = (float)(tick % 100) / 100.0f;

	uint8_t b[256];
	int32_t ind = 0;

	b[ind++] = COMM_GET_VALUES;

	buf_f16(b, &ind, 25.0f + ramp * 5.0f, 1e1f);    /*  0 temp fet */
	buf_f16(b, &ind, 30.0f + ramp * 5.0f, 1e1f);    /*  1 temp motor */
	buf_f32(b, &ind, ramp * 10.0f, 1e2f);           /*  2 current motor */
	buf_f32(b, &ind, ramp * 4.0f, 1e2f);            /*  3 current in */
	buf_f32(b, &ind, 0.0f, 1e2f);                   /*  4 id */
	buf_f32(b, &ind, ramp * 10.0f, 1e2f);           /*  5 iq */
	buf_f16(b, &ind, ramp, 1e3f);                   /*  6 duty */
	buf_f32(b, &ind, ramp * 5000.0f, 1e0f);         /*  7 rpm */
	buf_f16(b, &ind, 48.0f - ramp, 1e1f);           /*  8 v in */
	buf_f32(b, &ind, ramp, 1e4f);                   /*  9 amp hours */
	buf_f32(b, &ind, ramp * 0.1f, 1e4f);            /* 10 amp hours charged */
	buf_f32(b, &ind, ramp * 40.0f, 1e4f);           /* 11 watt hours */
	buf_f32(b, &ind, ramp * 4.0f, 1e4f);            /* 12 watt hours charged */
	buf_i32(b, &ind, tick * 10);                    /* 13 tachometer */
	buf_i32(b, &ind, tick * 10);                    /* 14 tachometer abs */
	b[ind++] = 0;                                   /* 15 fault code */
	buf_f32(b, &ind, 0.0f, 1e6f);                   /* 16 pid pos */
	b[ind++] = 1;                                   /* 17 controller id */
	buf_f16(b, &ind, 26.0f, 1e1f);                  /* 18 temp mos 1 */
	buf_f16(b, &ind, 27.0f, 1e1f);                  /* 18 temp mos 2 */
	buf_f16(b, &ind, 28.0f, 1e1f);                  /* 18 temp mos 3 */
	buf_f32(b, &ind, 0.0f, 1e3f);                   /* 19 vd */
	buf_f32(b, &ind, 0.0f, 1e3f);                   /* 20 vq */
	b[ind++] = 0;                                   /* 21 status */

	/* 22, this fork's PAS block. */
	buf_f16(b, &ind, ramp * 60.0f, 1e1f);           /*    pedal rpm */
	buf_f16(b, &ind, ramp * 20.0f, 1e1f);           /*    torque Nm */
	buf_f16(b, &ind, ramp * 120.0f, 1e0f);          /*    rider power */
	buf_f16(b, &ind, ramp * 250.0f, 1e0f);          /*    motor power target */
	buf_f16(b, &ind, ramp, 1e3f);                   /*    current target rel */
	buf_u16(b, &ind, 0);                            /*    flags */

	reply(b, ind);
}

static void handle_packet(unsigned char *data, unsigned int len)
{
	if (len < 1) {
		return;
	}

	packets_in++;
	uint8_t id = data[0];

	if (verbose) {
		fprintf(stderr, "vescsim: rx id %u len %u\n", id, len);
	}

	switch (id) {
	case COMM_FW_VERSION:
		send_fw_version();
		break;

	case COMM_GET_VALUES:
		send_values();
		break;

	/*
	 * Answered with silence on purpose, not with an empty packet.
	 *
	 * COMM_GET_MCCONF and COMM_GET_APPCONF carry a confgenerator blob whose
	 * signature the Tool checks against its own XML; a wrong one is rejected
	 * wholesale, and a truncated one is worse than no reply because the Tool
	 * cannot tell it from corruption. The Tool treats no reply as a timeout
	 * and carries on, which is the honest outcome until confgenerator.c is
	 * linked in here.
	 */
	case COMM_GET_MCCONF:
	case COMM_GET_APPCONF:
	default:
		if (verbose) {
			fprintf(stderr, "vescsim: id %u not served\n", id);
		}
		break;
	}
}

int main(int argc, char **argv)
{
	int port = 65102;

	for (int i = 1; i < argc; i++) {
		if (!strcmp(argv[i], "-v")) {
			verbose = 1;
		} else if (!strcmp(argv[i], "-p") && i + 1 < argc) {
			port = atoi(argv[++i]);
		} else {
			fprintf(stderr,
				"usage: %s [-p port] [-v]\n"
				"\n"
				"Serves the VESC protocol over TCP using this repository's\n"
				"own comm/packet.c, so a tool can be tested without a\n"
				"controller. Default port %d.\n", argv[0], port);
			return 2;
		}
	}

	/* A client that goes away mid-write must not kill the process. */
	signal(SIGPIPE, SIG_IGN);

	int srv = socket(AF_INET, SOCK_STREAM, 0);
	if (srv < 0) {
		perror("socket");
		return 1;
	}

	int one = 1;
	setsockopt(srv, SOL_SOCKET, SO_REUSEADDR, &one, sizeof(one));

	struct sockaddr_in addr;
	memset(&addr, 0, sizeof(addr));
	addr.sin_family = AF_INET;
	/* Any interface: an Android emulator reaches the host as 10.0.2.2. */
	addr.sin_addr.s_addr = htonl(INADDR_ANY);
	addr.sin_port = htons((uint16_t)port);

	if (bind(srv, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
		perror("bind");
		return 1;
	}

	if (listen(srv, 1) < 0) {
		perror("listen");
		return 1;
	}

	/* Line buffered, so a harness waiting on this sees it immediately. */
	setvbuf(stdout, NULL, _IOLBF, 0);
	printf("vescsim: listening on port %d\n", port);

	for (;;) {
		client_fd = accept(srv, NULL, NULL);
		if (client_fd < 0) {
			if (errno == EINTR) {
				continue;
			}
			perror("accept");
			break;
		}

		setsockopt(client_fd, IPPROTO_TCP, TCP_NODELAY, &one, sizeof(one));
		printf("vescsim: client connected\n");

		/* Fresh framing per client; a half-read packet must not carry over. */
		packet_init(send_raw, handle_packet, &packet_state);
		packet_reset(&packet_state);

		uint8_t buf[512];
		for (;;) {
			ssize_t n = read(client_fd, buf, sizeof(buf));
			if (n <= 0) {
				if (n < 0 && errno == EINTR) {
					continue;
				}
				break;
			}

			for (ssize_t i = 0; i < n; i++) {
				packet_process_byte(buf[i], &packet_state);
			}
		}

		printf("vescsim: client gone (%lu in, %lu out)\n",
		       packets_in, packets_out);
		close(client_fd);
		client_fd = -1;
	}

	close(srv);
	return 0;
}
