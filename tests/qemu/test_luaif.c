/*
 * The firmware adapter -- luaif.c -- on the simulated STM32F405.
 *
 * This is the piece that was supposed to need a bench: the engine thread, the
 * lifecycle, and the event path from a producer thread into a running script.
 * All of it runs here against the real ChibiOS kernel, the real engine and the
 * real arena, with only the flash and the terminal stubbed (luaif_host.h).
 *
 * Still not covered, and still needing hardware: the COMM_LISP_* protocol,
 * which luaif.c does not implement yet, and whether flash_helper's pointer
 * really does go stale under an erase.
 */

#include "qrt.h"
#include "luaif.h"
#include "luaif_host.h"
#include "script_alloc.h"
#include "script_pack.h"

#include <stdarg.h>
#include <stdio.h>
#include <string.h>

/* ------------------------------------------------------- the fake flash -- */

static uint8_t m_code[1024];
static int32_t m_code_len;

const uint8_t *luaif_host_code(int32_t *len_out) {
	*len_out = m_code_len;
	return (m_code_len > 0) ? m_code : NULL;
}

/*
 * CRC-16/XMODEM, matching script_pack.c's own pack_crc16 bit for bit.
 * Deliberately a copy rather than a call into util/crc.c: if the two ever
 * disagree the parser is the authority, and a test that borrows the parser's
 * helper could not detect that disagreement at all.
 */
static uint16_t test_crc16(const uint8_t *data, int32_t len) {
	uint16_t crc = 0;

	for (int32_t i = 0; i < len; i++) {
		crc ^= (uint16_t)((uint16_t)data[i] << 8);
		for (int b = 0; b < 8; b++) {
			crc = (crc & 0x8000u) ? (uint16_t)((crc << 1) ^ 0x1021u)
					: (uint16_t)(crc << 1);
		}
	}
	return crc;
}

/*
 * Build a container the real parser will accept, so the test exercises
 * script_pack_parse rather than side-stepping it. Layout per script_pack.h:
 * big-endian uint32 size, uint16 crc, uint16 flags, NUL-terminated source,
 * then uint16 import count.
 *
 * The size field is *two less* than the length of the region it covers --
 * what VESC Tool writes, and what script_pack.c compensates for. Getting that
 * wrong is how this test first failed, with every container refused.
 */
static void flash_write_script(const char *src, bool lua) {
	size_t srclen = strlen(src) + 1u;          /* including the terminator */
	size_t body = 2u + srclen + 2u;            /* flags + source + imports */
	uint8_t *p = m_code;

	size_t stored = body - 2u;                 /* see the note above */

	p[0] = (uint8_t)((stored >> 24) & 0xffu);
	p[1] = (uint8_t)((stored >> 16) & 0xffu);
	p[2] = (uint8_t)((stored >> 8) & 0xffu);
	p[3] = (uint8_t)(stored & 0xffu);

	uint8_t *b = p + 6;                        /* flags onward */
	b[0] = 0x00u;
	b[1] = lua ? 0x01u : 0x00u;                /* SCRIPT_FLAG_LANG_LUA */
	memcpy(b + 2, src, srclen);
	b[2 + srclen] = 0x00u;                     /* no imports */
	b[3 + srclen] = 0x00u;

	uint16_t ck = test_crc16(b, (int32_t)body);
	p[4] = (uint8_t)((ck >> 8) & 0xffu);
	p[5] = (uint8_t)(ck & 0xffu);

	m_code_len = (int32_t)(body + 6u);
}

static void flash_erase_script(void) {
	m_code_len = 0;
}

/* ----------------------------------------------------- captured terminal -- */

static char m_out[1024];
static size_t m_out_len;

int commands_printf_lisp(const char *format, ...) {
	char line[256];
	va_list ap;

	va_start(ap, format);
	int n = vsnprintf(line, sizeof(line), format, ap);
	va_end(ap);

	if (n > 0) {
		size_t add = strlen(line);
		if (m_out_len + add + 1u < sizeof(m_out)) {
			memcpy(m_out + m_out_len, line, add);
			m_out_len += add;
			m_out[m_out_len] = '\0';
		}
		/*
		 * Captured, not echoed. The engine runs on its own thread, so
		 * printing here races main's own output and shreds both lines. Tests
		 * dump the capture at a point where they control the timing.
		 */
	}
	return n;
}

const char *luaif_host_output(void) {
	return m_out;
}

void luaif_host_output_reset(void) {
	m_out_len = 0;
	m_out[0] = '\0';
}

/* ------------------------------------------------------------------ tests -- */

int main(void) {
	qrt_console_init();
	chSysInit();
	qrt_systick_init();

	qrt_puts("luaif test\r\n");

#define STAGE(s) do { qrt_puts("  .. "); qrt_puts(s); qrt_puts("\r\n"); } while (0)

	/* --- an empty code slot is a normal state, not a failure ----------- */
	STAGE("empty slot");
	flash_erase_script();
	luaif_init();
	chThdSleepMilliseconds(100);
	qrt_expect_ok("survives an empty code slot", luaif_get_restart_cnt() >= 1);

	/* --- a script that runs, and whose output comes back -------------- */
	STAGE("run a script");
	luaif_host_output_reset();
	flash_write_script(
			"local n = 0\n"
			"for i = 1, 10 do n = n + i end\n"
			"print('total=' .. n)\n", true);
	qrt_expect_ok("restart runs the script", luaif_restart(true, true));
	qrt_expect_ok("script output reached the terminal",
			strstr(luaif_host_output(), "total=55") != NULL);
	qrt_expect_ok("arena in use while running", script_alloc_used() > 0u);

	/* --- a CAN frame gets from a producer thread into a handler -------- */
	luaif_host_output_reset();
	STAGE("can handler");
	flash_write_script(
			"vesc.on_can(function(id, data)\n"
			"  print('can id=' .. id .. ' n=' .. #data)\n"
			"end)\n", true);
	qrt_expect_ok("restart with a can handler", luaif_restart(true, true));
	{
		uint8_t frame[4] = {1, 2, 3, 4};
		luaif_process_can(0x123u, frame, sizeof(frame), false);
		chThdSleepMilliseconds(50);
	}
	qrt_expect_ok("can frame reached the handler",
			strstr(luaif_host_output(), "can id=291 n=4") != NULL);

	/* --- app data takes the same path --------------------------------- */
	luaif_host_output_reset();
	STAGE("app data");
	flash_write_script(
			"vesc.on_app_data(function(data)\n"
			"  print('app n=' .. #data)\n"
			"end)\n", true);
	qrt_expect_ok("restart with an app-data handler", luaif_restart(true, true));
	{
		unsigned char payload[6] = {9, 8, 7, 6, 5, 4};
		luaif_process_custom_app_data(payload, sizeof(payload));
		chThdSleepMilliseconds(50);
	}
	qrt_expect_ok("app data reached the handler",
			strstr(luaif_host_output(), "app n=6") != NULL);

	/* --- a script that raises is reported and does not take the thread - */
	luaif_host_output_reset();
	STAGE("raising script");
	flash_write_script("error('deliberate')\n", true);
	qrt_expect_ok("a raising script is not a successful start",
			!luaif_restart(true, true));
	qrt_expect_ok("the error was reported",
			strstr(luaif_host_output(), "deliberate") != NULL);

	/* --- a corrupt container is refused, not run ---------------------- */
	luaif_host_output_reset();
	STAGE("corrupt container");
	flash_write_script("print('never')\n", true);
	m_code[20] ^= 0xffu;                       /* break the crc */
	qrt_expect_ok("corrupt container refused", !luaif_restart(true, true));
	qrt_expect_ok("corruption was reported",
			strstr(luaif_host_output(), "not valid") != NULL);

	/*
	 * --- stop releases the interpreter --------------------------------
	 * This is the one flash_helper depends on: it erases the pages the
	 * source is read from, so the engine must have let go before the call
	 * returns, not merely been asked to.
	 */
	luaif_host_output_reset();
	STAGE("stop releases");
	flash_write_script("print('running')\n", true);
	qrt_expect_ok("restart before stop", luaif_restart(true, true));
	luaif_stop();
	qrt_expect("stop freed the arena", (unsigned)script_alloc_used(), 0u);

	/* --- and the engine comes back after a stop ----------------------- */
	luaif_host_output_reset();
	STAGE("restart after stop");
	flash_write_script("print('again=1')\n", true);
	qrt_expect_ok("restart after stop", luaif_restart(true, true));
	qrt_expect_ok("ran again after a stop",
			strstr(luaif_host_output(), "again=1") != NULL);

	/*
	 * --- a long script runs to completion ------------------------------
	 *
	 * Long enough to cross the instruction hook, which the other scripts
	 * here are not: they finish in fewer instructions than cfg.hook_count, so
	 * they never ask should_stop anything. That left the whole stop handshake
	 * untestable in the direction that matters most -- a stop request that is
	 * never cleared makes should_stop permanently true, which kills every
	 * script after the first restart, and every check still passed.
	 */
	luaif_host_output_reset();
	STAGE("long script");
	flash_write_script(
			"local n = 0\n"
			"for i = 1, 20000 do n = n + 1 end\n"
			"print('big=' .. n)\n", true);
	qrt_expect_ok("a long script is not cut short", luaif_restart(true, true));
	qrt_expect_ok("the long script finished",
			strstr(luaif_host_output(), "big=20000") != NULL);

	/*
	 * --- a handler stuck in a loop can still be stopped ---------------
	 *
	 * This is the one that makes should_stop worth having, and the reason
	 * flash_helper can trust luaif_stop. A script is not obliged to return:
	 * the stop has to reach it through the instruction hook and unwind it.
	 *
	 * Timed, because the failure mode is not a wrong answer but a slow one --
	 * with the hook ignored, the stop still "works" by timing out and tearing
	 * the engine down from under the caller a full second later. Without a
	 * deadline this check passes either way, which is how it was written the
	 * first time.
	 */
	luaif_host_output_reset();
	STAGE("looping handler");
	flash_write_script(
			"vesc.on_can(function(id, data)\n"
			"  while true do end\n"
			"end)\n", true);
	qrt_expect_ok("restart with a looping handler", luaif_restart(true, true));
	{
		uint8_t frame[1] = {0};
		systime_t t0;
		unsigned ms;

		luaif_process_can(0x1u, frame, sizeof(frame), false);
		chThdSleepMilliseconds(50);          /* let it get into the loop */

		t0 = chVTGetSystemTimeX();
		luaif_stop();
		ms = (unsigned)(chVTGetSystemTimeX() - t0)
				/ (CH_CFG_ST_FREQUENCY / 1000u);

		qrt_puts("stop_of_looping_handler_ms=");
		qrt_putu(ms);
		qrt_puts("\r\n");

		qrt_expect_ok("a looping handler was interrupted, not waited out",
				ms < 300u);
		qrt_expect("looping handler released the arena",
				(unsigned)script_alloc_used(), 0u);
	}

	/* --- the thread is still alive and its stack is sane -------------- */
	qrt_expect_ok("restarts were counted", luaif_get_restart_cnt() >= 8);

	qrt_report();
	return 0;
}
