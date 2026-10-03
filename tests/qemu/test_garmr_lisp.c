/*
	Copyright Benjamin Vedder
	Copyright 2026 Stephen Bouche

	Parts of this file were moved here from examples/esp32c3-packet/main/main.c, examples/esp32c3/main/main.c, lispBM/lispif.c;
	git blame -C records 8 lines as Benjamin Vedder's.

	This file is part of the ESCargot firmware.

	The ESCargot firmware is free software: you can redistribute it and/or
	modify it under the terms of the GNU General Public License as published
	by the Free Software Foundation, either version 3 of the License, or
	(at your option) any later version.

	The ESCargot firmware is distributed in the hope that it will be useful,
	but WITHOUT ANY WARRANTY; without even the implied warranty of
	MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
	GNU General Public License for more details.

	You should have received a copy of the GNU General Public License
	along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

/*
 * The LispBM Garmr's engage logic on the simulated STM32F405 -- the
 * counterpart to test_garmr.c, which does the same for the Lua version.
 *
 * This closes an asymmetry: the Lua prototype's engage logic ran in CI on a
 * simulated board while the LispBM one ran only in the vesc_express repl,
 * which CI does not have. The two scripts are the same logic, and the one
 * most bikes can run was the one CI never saw.
 *
 * The stubs are LispBM, not C. garmr.lisp calls six firmware extensions and
 * this image registers none of them, so stubs.lisp defines them as ordinary
 * functions -- which is why no C faking was needed here at all. The three
 * sources are evaluated in the order the repl is given them, and are embedded
 * from the package tree rather than copied, so the test cannot drift from what
 * ships.
 *
 * LispBM ships a ChibiOS platform layer of its own, so the core links into
 * this kernel-only image the same way the Lua core does: a heap and an arena
 * in CCM, an evaluator on its own thread, and callbacks for print and sleep.
 * No HAL, no flash driver, no board.
 *
 * Sizes are the firmware's where they fit. The one difference worth naming is
 * the evaluator's working area: LispBM asks for 2 KB where the Lua engine
 * asks for 12 KB, because LispBM evaluates on its own heap-allocated
 * continuation stack rather than on the C stack.
 *
 *   make TEST=demo_lisp run
 */

#include "hal.h"

#include "qrt.h"
#include "lispbm.h"
#include "array_extensions.h"
#include "math_extensions.h"
#include "string_extensions.h"
#include "extensions/lbm_dyn_lib.h"

#include <string.h>
#include <stdio.h>
#include <stdarg.h>

/* --- memory, all of it in CCM ------------------------------------------- */

/*
 * lispif.c's numbers, not a comfortable set chosen for the test: the firmware
 * gives LispBM 2428 cons cells and a 28K arena, so running the script in that
 * much is itself the check that it fits on a board. A larger heap here would
 * pass while the real thing ran out.
 */
#define EXTENSION_STORAGE_SIZE	64

/* lispBM has no 28K size of its own; lispif.c defines these two, so does this. */
#define LBM_MEMORY_SIZE_28K			LBM_MEMORY_SIZE_64BYTES_TIMES_X(448)
#define LBM_MEMORY_BITMAP_SIZE_28K	LBM_MEMORY_BITMAP_SIZE(448)

#define HEAP_SIZE				2428		/* cons cells, as lispif.c */
#define LISP_MEM_SIZE			LBM_MEMORY_SIZE_28K
#define LISP_MEM_BITMAP_SIZE	LBM_MEMORY_BITMAP_SIZE_28K
#define GC_STACK_SIZE			160
#define PRINT_STACK_SIZE		128

__attribute__((section(".ram4"))) static lbm_cons_t heap[HEAP_SIZE]
		__attribute__((aligned(8)));
__attribute__((section(".ram4"))) static uint32_t memory_array[LISP_MEM_SIZE];
__attribute__((section(".ram4"))) static uint32_t bitmap_array[LISP_MEM_BITMAP_SIZE];
static lbm_extension_t extension_storage[EXTENSION_STORAGE_SIZE];

/* The evaluator's own thread. 2 KB is what the firmware gives it. */
__attribute__((section(".ram4"))) static THD_WORKING_AREA(eval_wa, 2048);

/*
 * The image. LispBM keeps its symbol table and constant heap here, and on a
 * board it is a region of flash written through a driver. That is why the
 * first version of this demo could evaluate (+ 1 2) but not (define poles 14):
 * arithmetic on built-in symbols needs no image, and interning a new symbol
 * does.
 *
 * Backed by RAM here, with a write function that just stores. Main RAM rather
 * than CCM because CCM is already carrying the heap, the arena and two thread
 * stacks. 8 KB against the firmware's 128 KB, which is plenty for a REPL
 * session and would not be for a program with a large constant heap.
 */
#define IMAGE_WORDS (8 * 1024 / 4)
static uint32_t m_image[IMAGE_WORDS];

static bool image_write(uint32_t w, int32_t ix, bool const_heap) {
	(void)const_heap;
	if ((ix < 0) || (ix >= (int32_t)IMAGE_WORDS)) {
		return false;
	}
	m_image[ix] = w;
	return true;
}

/* --- what the engine needs from its host -------------------------------- */

static void sleep_callback(uint32_t us) {
	chThdSleepMicroseconds(us < 1u ? 1u : us);
}

static void watch_line(const char *line);

static int print_callback(const char *fmt, ...) {
	char buf[192];
	va_list ap;

	va_start(ap, fmt);
	vsnprintf(buf, sizeof(buf), fmt, ap);
	va_end(ap);

	watch_line(buf);

	qrt_puts("  lisp> ");
	qrt_puts(buf);
	qrt_puts("\r\n");
	return 0;
}

static THD_FUNCTION(eval_thread, arg) {
	(void)arg;
	chRegSetThreadName("lbm");
	lbm_run_eval();
}


/*
 * print is a VESC extension, not part of LispBM or the three extension sets
 * this image registers -- so without it the test loaded, ran, and reported
 * nothing but "UNDEFINED: print". The firmware's version routes to
 * commands_printf_lisp; this one routes to the console and the line watcher,
 * which is the whole mechanism by which the lisp checks reach the verdict.
 *
 * lbm_print_value quotes a string, and the checks print assembled strings, so
 * a string argument is taken as-is the way a terminal would show it.
 */
static lbm_value ext_print(lbm_value *args, lbm_uint argn) {
	char line[256];
	size_t used = 0;

	line[0] = '\0';

	for (lbm_uint i = 0; i < argn; i++) {
		char piece[192];

		if (lbm_is_array_r(args[i])) {
			const char *str = lbm_dec_str(args[i]);
			snprintf(piece, sizeof(piece), "%s", str ? str : "");
		} else {
			lbm_print_value(piece, sizeof(piece), args[i]);
		}

		int n = snprintf(line + used, sizeof(line) - used, "%s", piece);

		if (n < 0) {
			break;
		}

		used += (size_t)n;

		if (used >= sizeof(line) - 1) {
			break;
		}
	}

	watch_line(line);
	qrt_puts("  lisp> ");
	qrt_puts(line);
	qrt_puts("\r\n");

	return ENC_SYM_TRUE;
}

/*
 * The timing extensions, with the firmware's semantics: systime is a tick
 * count and secs-since divides the elapsed ticks by the kernel frequency.
 * These are host services rather than stand-ins for PAS behaviour, which is
 * why they are here in C and not in stubs.lisp -- the debounce is measured
 * against a real clock, and a lisp fake would be measuring itself.
 */
static lbm_value ext_systime(lbm_value *args, lbm_uint argn) {
	(void)args; (void)argn;
	return lbm_enc_u32(chVTGetSystemTimeX());
}

static lbm_value ext_secs_since(lbm_value *args, lbm_uint argn) {
	LBM_CHECK_ARGN_NUMBER(1);
	return lbm_enc_float((float)chVTTimeElapsedSinceX(lbm_dec_as_u32(args[0]))
			/ (float)CH_CFG_ST_FREQUENCY);
}

static lbm_value ext_sleep(lbm_value *args, lbm_uint argn) {
	LBM_CHECK_ARGN_NUMBER(1);
	float secs = lbm_dec_as_float(args[0]);

	if (secs > 0.0f) {
		chThdSleepMilliseconds((uint32_t)(secs * 1000.0f));
	} else {
		chThdYield();
	}

	return ENC_SYM_TRUE;
}

static void say(const char *s) {
	qrt_puts(s);
	qrt_puts("\r\n");
}

/* --- the three sources, embedded from the package tree ------------------ */

static const char *const src_stubs =
#include "garmr_stubs.inc"
;
static const char *const src_garmr =
#include "garmr_lisp.inc"
;
static const char *const src_test =
#include "garmr_test.inc"
;

/* --- watching what the test prints -------------------------------------- */

/*
 * The checks live in the lisp, so the verdict is in its output: each check
 * prints "  ok   " or "  FAIL ", and the last line is "N checks, M fails".
 * This mirrors every line to the console and keeps the counts, so a failure
 * names itself rather than arriving as a number.
 */
static int m_fail_lines;
static int m_ok_lines;
static bool m_saw_summary;
static bool m_summary_clean;

static void watch_line(const char *line) {
	if (strstr(line, "FAIL") != NULL) {
		m_fail_lines++;
	} else if (strstr(line, "  ok   ") != NULL) {
		m_ok_lines++;
	}

	if (strstr(line, " checks, ") != NULL) {
		m_saw_summary = true;
		m_summary_clean = (strstr(line, ", 0 fails") != NULL);
	}
}

/* --- a whole program, evaluated and waited on --------------------------- */

static volatile lbm_cid m_want = -1;
static volatile bool m_done;
static volatile bool m_error;

static void ctx_done_callback(eval_context_t *ctx) {
	if (ctx->id == m_want) {
		char buf[80];
		lbm_print_value(buf, sizeof(buf), ctx->r);

		/*
		 * A program that raises leaves an error symbol as its result, and the
		 * evaluator reports done either way -- so without looking at the
		 * result a broken script would present as a test that simply printed
		 * nothing.
		 */
		if (strstr(buf, "error") != NULL) {
			qrt_puts("    result: ");
			qrt_puts(buf);
			qrt_puts("\r\n");
			m_error = true;
		}

		m_done = true;
	}
}

static lbm_string_channel_state_t m_st[3];
static lbm_char_channel_t m_chan[3];
static int m_slot;

/*
 * One channel per program and never reused: the reader keeps a reference for
 * the life of the context, so a shared set works for the first program and
 * gives read_error for the rest.
 */
static bool run_program(const char *name, const char *code) {
	if (m_slot >= 3) {
		return false;
	}

	lbm_string_channel_state_t *st = &m_st[m_slot];
	lbm_char_channel_t *chan = &m_chan[m_slot];
	m_slot++;

	/* Cast away const: the channel only reads, but takes a char*. */
	lbm_create_string_char_channel(st, chan, (char *)code);

	m_done = false;
	m_error = false;

	lbm_cid cid = lbm_load_and_eval_program(chan, (char *)name);

	if (cid <= 0) {
		qrt_expect_ok("program started", 0);
		return false;
	}

	m_want = cid;

	/* The debounce checks sleep, so this waits well past the test's own. */
	for (int i = 0; (i < 20000) && !m_done; i++) {
		chThdSleepMilliseconds(1);
	}

	return m_done && !m_error;
}

int main(void) {
	chSysInit();
	qrt_systick_init();

	say("");
	say("  Garmr (LispBM) -- engage logic on a simulated STM32F405");
	say("");

	if (!lbm_init(heap, HEAP_SIZE,
			memory_array, LISP_MEM_SIZE,
			bitmap_array, LISP_MEM_BITMAP_SIZE,
			GC_STACK_SIZE, PRINT_STACK_SIZE,
			extension_storage, EXTENSION_STORAGE_SIZE)) {
		say("  lbm_init failed");
		qrt_exit(1);
	}

	lbm_set_usleep_callback(sleep_callback);
	lbm_set_printf_callback(print_callback);
	lbm_set_ctx_done_callback(ctx_done_callback);

	lbm_image_init(m_image, IMAGE_WORDS, image_write);
	if (!lbm_image_exists()) {
		lbm_image_create("qemu-garmr");
	}
	lbm_image_boot();
	lbm_add_eval_symbols();
	lbm_eval_init_events(20);

	lbm_array_extensions_init();
	lbm_math_extensions_init();
	lbm_string_extensions_init();

	/*
	 * str-merge and to-str are C extensions, but defun, var and loopwhile are
	 * lisp definitions the engine loads on demand through this callback. The
	 * firmware wires it up in lispif_vesc_extensions.c; demo_lisp.c does not,
	 * which is fine for arithmetic at a REPL and not for a program.
	 */
	lbm_dyn_lib_init();
	lbm_set_dynamic_load_callback(lbm_dyn_lib_find);

	lbm_add_extension("print", ext_print);
	lbm_add_extension("systime", ext_systime);
	lbm_add_extension("secs-since", ext_secs_since);
	lbm_add_extension("sleep", ext_sleep);

	chThdCreateStatic(eval_wa, sizeof(eval_wa), NORMALPRIO - 1,
			eval_thread, NULL);
	chThdSleepMilliseconds(50);

	qrt_expect_ok("stubs evaluated", run_program("stubs", src_stubs));
	qrt_expect_ok("garmr.lisp evaluated", run_program("garmr", src_garmr));
	qrt_expect_ok("engage test ran", run_program("test", src_test));

	qrt_expect_ok("the test printed its summary", m_saw_summary);
	qrt_expect_ok("no check failed", m_fail_lines == 0);
	qrt_expect_ok("the summary agrees nothing failed", m_summary_clean);

	/*
	 * A test that evaluated nothing would satisfy every check above: no FAIL
	 * lines, and a summary it never reached. So the count has to be there too,
	 * and it has to be the count the repl run produces.
	 */
	qrt_expect("checks reported", (unsigned)m_ok_lines, 31u);

	qrt_puts("heap_cells=");
	qrt_putu(HEAP_SIZE);
	qrt_puts(" free_after=");
	qrt_putu((unsigned)lbm_heap_num_free());
	qrt_puts("\r\n");

	qrt_report();
	return 0;
}
