/*
	Copyright Benjamin Vedder
	Copyright 2026 Stephen Bouche

	Parts of this file were moved here from examples/esp32c3-packet/main/main.c, examples/esp32c3/main/main.c, lispBM/lispif.c;
	git blame -C records 3 lines as Benjamin Vedder's.

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
 * The LispBM engine on the simulated STM32F405 -- the counterpart to demo.c,
 * which does the same for Lua.
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

#include "qrt.h"
#include "lispbm.h"
#include "array_extensions.h"
#include "math_extensions.h"
#include "string_extensions.h"

#include <string.h>
#include <stdio.h>
#include <stdarg.h>

/* --- memory, all of it in CCM ------------------------------------------- */

#define EXTENSION_STORAGE_SIZE	64
#define HEAP_SIZE				1536		/* cons cells */
#define LISP_MEM_SIZE			LBM_MEMORY_SIZE_8K
#define LISP_MEM_BITMAP_SIZE	LBM_MEMORY_BITMAP_SIZE_8K
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

static int print_callback(const char *fmt, ...) {
	char buf[192];
	va_list ap;

	va_start(ap, fmt);
	vsnprintf(buf, sizeof(buf), fmt, ap);
	va_end(ap);

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

/* --- a REPL line, evaluated and printed --------------------------------- */

static void say(const char *s) {
	qrt_puts(s);
	qrt_puts("\r\n");
}

/*
 * The result arrives through a done callback rather than a blocking wait,
 * which is how lispif.c reads a REPL result too: the evaluator runs on its own
 * thread and reports when a context finishes, matched by id.
 */
static volatile lbm_cid m_want = -1;
static volatile bool m_done;
static char m_result[160];

static void ctx_done_callback(eval_context_t *ctx) {
	if (ctx->id == m_want) {
		lbm_print_value(m_result, sizeof(m_result), ctx->r);
		m_done = true;
	}
}

/*
 * One channel, state and buffer per call rather than a shared set.
 *
 * The reader keeps a reference to the channel for the life of the context, so
 * reusing a single static set worked for the first expression and gave
 * read_error for every one after it -- the done callback fires before the
 * context is torn down, so the next call was overwriting a channel still in
 * use.
 */
#define EVAL_SLOTS 8
static lbm_string_channel_state_t m_st[EVAL_SLOTS];
static lbm_char_channel_t m_chan[EVAL_SLOTS];
static char m_buf[EVAL_SLOTS][192];
static int m_slot;

static void eval(const char *expr) {
	if (m_slot >= EVAL_SLOTS) {
		say("    (out of channel slots)");
		return;
	}
	lbm_string_channel_state_t *st = &m_st[m_slot];
	lbm_char_channel_t *chan = &m_chan[m_slot];
	char *buf = m_buf[m_slot];
	m_slot++;

	qrt_puts("  > ");
	say(expr);

	snprintf(buf, sizeof(m_buf[0]), "%s", expr);
	lbm_create_string_char_channel(st, chan, buf);

	m_done = false;
	m_result[0] = '\0';

	lbm_cid cid = lbm_load_and_eval_expression(chan);
	if (cid <= 0) {
		say("    (could not start a context)");
		return;
	}
	m_want = cid;

	for (int i = 0; (i < 2000) && !m_done; i++) {
		chThdSleepMilliseconds(1);
	}
	m_want = -1;

	if (!m_done) {
		say("    (timed out)");
		return;
	}

	qrt_puts("    ");
	say(m_result);
}

int main(void) {
	qrt_console_init();
	chSysInit();
	qrt_systick_init();

	say("");
	say("  ESCargot firmware -- script engine: LispBM  (USE_LISPBM=1)");
	say("  target: STM32F405, ChibiOS 3.0.5          [QEMU, not a board]");
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

	/*
	 * The rest of the sequence lispif.c performs. Without it the engine runs
	 * but cannot intern a symbol, which presents as read_error on anything
	 * that defines a name.
	 */
	lbm_image_init(m_image, IMAGE_WORDS, image_write);
	if (!lbm_image_exists()) {
		lbm_image_create("qemu-demo");
	}
	lbm_image_boot();
	lbm_add_eval_symbols();
	lbm_eval_init_events(20);

	/*
	 * The three extension sets the firmware registers that need no board.
	 * Note this is not everything a script sees: str-merge and friends come
	 * from LispBM's dynamic library, loaded on demand through a callback the
	 * firmware supplies, and that is not wired up here.
	 */
	lbm_array_extensions_init();
	lbm_math_extensions_init();
	lbm_string_extensions_init();

	qrt_puts("  [  OK  ] heap ");
	qrt_putu(HEAP_SIZE);
	qrt_puts(" cells, ");
	qrt_putu((unsigned)(sizeof(heap) + sizeof(memory_array)
			+ sizeof(bitmap_array)));
	say(" bytes of CCM");
	qrt_puts("  [  OK  ] evaluator thread, ");
	qrt_putu((unsigned)sizeof(eval_wa));
	say(" byte stack in CCM");
	say("");

	chThdCreateStatic(eval_wa, sizeof(eval_wa), NORMALPRIO - 1,
			eval_thread, NULL);
	chThdSleepMilliseconds(50);

	eval("(+ 1 2 3 4 5)");
	eval("(define poles 14)");
	eval("(define kv 1800)");
	eval("(* kv 16.8 (/ poles 2))");
	eval("(map (lambda (x) (* x x)) (list 1 2 3 4 5))");
	/* the check the bench script's notes ask you to make before spinning a
	 * 1800kv outrunner on 4s: 14 poles means 7 pole pairs, and the result is
	 * far above a default FOC configuration */
	eval("(define l-max-erpm 150000)");
	eval("(< (* kv 16.8 (/ poles 2)) l-max-erpm)");
	say("");

	lbm_heap_state_t h = lbm_heap_state;
	qrt_puts("  measured, this session:");
	say("");
	qrt_puts("    cons cells used      ");
	qrt_putu((unsigned)(HEAP_SIZE - lbm_heap_num_free()));
	qrt_puts(" of ");
	qrt_putu(HEAP_SIZE);
	say("");
	qrt_puts("    gc passes            ");
	qrt_putu((unsigned)h.gc_num);
	say("");
	qrt_puts("    arena words free     ");
	qrt_putu((unsigned)lbm_memory_num_free());
	qrt_puts(" of ");
	qrt_putu((unsigned)lbm_memory_num_words());
	say("");
	qrt_puts("    evaluator stack      ");
	qrt_putu(qrt_stack_used(eval_wa, sizeof(eval_wa)));
	qrt_puts(" of ");
	qrt_putu((unsigned)qrt_stack_total(sizeof(eval_wa)));
	say(" bytes");
	say("");

	qrt_exit(0);
	return 0;
}
