/*
 * The ADC channel mapping in script/lua_vesc_io.c.
 *
 * adc_index() is extracted from the real source by the Makefile rather than
 * copied, so this cannot drift from what the firmware does. The rest of
 * lua_vesc_io.c is one-line wrappers around app and driver calls; this is the
 * only part with logic of its own, and it is the part with a trap in it.
 *
 * hwconf/hw.h aliases ADC_IND_EXT2 through ADC_IND_EXT8 to ADC_IND_EXT when a
 * board does not define them:
 *
 *     #ifndef ADC_IND_EXT4
 *     #define ADC_IND_EXT4 ADC_IND_EXT
 *     #endif
 *
 * So on a board with two external pins, reading channel 4 returns the voltage
 * on channel 0, which is usually the throttle. lisp does exactly that. This
 * checks that the Lua binding reports the channel as absent instead.
 *
 * Two boards are simulated by defining the macros this file controls, with
 * the same aliasing rule hw.h applies. That is the one thing not taken from
 * the real source, and it is four lines.
 */

#include <stdio.h>
#include <stdbool.h>

static int failures;
static int checks;

static void expect(const char *what, int got, int want) {
	checks++;
	if (got != want) {
		failures++;
		printf("FAIL %s: got %d want %d\n", what, got, want);
	}
}

/* ---- board A: every external pin wired, nothing aliased ---------------- */

#define ADC_IND_EXT				0
#define ADC_IND_EXT2			1
#define ADC_IND_EXT3			2
#define ADC_IND_TEMP_MOTOR		3
#define ADC_IND_EXT4			4
#define ADC_IND_EXT5			5
#define ADC_IND_EXT6			6
#define ADC_IND_EXT7			7
#define ADC_IND_EXT8			8

#define adc_index adc_index_full
#include "adc_index.inc"
#undef adc_index

#undef ADC_IND_EXT
#undef ADC_IND_EXT2
#undef ADC_IND_EXT3
#undef ADC_IND_TEMP_MOTOR
#undef ADC_IND_EXT4
#undef ADC_IND_EXT5
#undef ADC_IND_EXT6
#undef ADC_IND_EXT7
#undef ADC_IND_EXT8

/* ---- board B: two external pins, the rest aliased by hw.h -------------- */

#define ADC_IND_EXT				0
#define ADC_IND_EXT2			1
#define ADC_IND_TEMP_MOTOR		2

/* the aliasing rule from hwconf/hw.h, applied the same way */
#ifndef ADC_IND_EXT3
#define ADC_IND_EXT3			ADC_IND_EXT
#endif
#ifndef ADC_IND_EXT4
#define ADC_IND_EXT4			ADC_IND_EXT
#endif
#ifndef ADC_IND_EXT5
#define ADC_IND_EXT5			ADC_IND_EXT
#endif
#ifndef ADC_IND_EXT6
#define ADC_IND_EXT6			ADC_IND_EXT
#endif
#ifndef ADC_IND_EXT7
#define ADC_IND_EXT7			ADC_IND_EXT
#endif
#ifndef ADC_IND_EXT8
#define ADC_IND_EXT8			ADC_IND_EXT
#endif

#define adc_index adc_index_sparse
#include "adc_index.inc"
#undef adc_index

int main(void) {
	bool aliased;

	/* --- a board with everything wired resolves every channel --------- */
	for (int ch = 0; ch <= 8; ch++) {
		char what[64];
		int ind = adc_index_full(ch, &aliased);

		snprintf(what, sizeof(what), "full board: channel %d resolves", ch);
		expect(what, ind, ch);
		snprintf(what, sizeof(what), "full board: channel %d not aliased", ch);
		expect(what, aliased ? 1 : 0, 0);
	}

	/* --- a sparse board resolves what it has -------------------------- */
	expect("sparse: channel 0 is EXT", adc_index_sparse(0, &aliased), 0);
	expect("sparse: channel 0 not aliased", aliased ? 1 : 0, 0);

	expect("sparse: channel 1 is EXT2", adc_index_sparse(1, &aliased), 1);
	expect("sparse: channel 1 not aliased", aliased ? 1 : 0, 0);

	expect("sparse: channel 3 is the motor temp pin",
			adc_index_sparse(3, &aliased), 2);
	expect("sparse: channel 3 not aliased", aliased ? 1 : 0, 0);

	/*
	 * --- and reports the rest as absent rather than as channel 0 ------
	 *
	 * This is the whole point. Without the check, every one of these reads
	 * the throttle pin and a script cannot tell.
	 */
	{
		const int absent[] = {2, 4, 5, 6, 7, 8};

		for (unsigned i = 0; i < sizeof(absent) / sizeof(absent[0]); i++) {
			char what[80];

			(void)adc_index_sparse(absent[i], &aliased);
			snprintf(what, sizeof(what),
					"sparse: channel %d reported absent", absent[i]);
			expect(what, aliased ? 1 : 0, 1);
		}
	}

	/* --- an out-of-range channel is rejected, on both boards ---------- */
	expect("channel 9 rejected", adc_index_full(9, &aliased), -1);
	expect("channel -1 rejected", adc_index_full(-1, &aliased), -1);
	expect("channel 9 rejected (sparse)", adc_index_sparse(9, &aliased), -1);

	printf("\n%d checks, %d failures\n", checks, failures);
	fflush(stdout);
	return failures == 0 ? 0 : 1;
}
