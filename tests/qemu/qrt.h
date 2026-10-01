/*
 * Minimal test runtime for ChibiOS images running under
 * qemu-system-arm -M olimex-stm32-h405 (an STM32F405, the same part the
 * firmware targets).
 *
 * This is test scaffolding only -- nothing here is built into firmware.
 */

#ifndef QRT_H_
#define QRT_H_

#include "ch.h"

/* USART1 TX, polled. QEMU does not model the baud divider, so there is no
 * clock setup to get wrong. */
void qrt_console_init(void);
void qrt_puts(const char *s);
void qrt_putu(unsigned v);
void qrt_puthex(unsigned v);

/* Starts the periodic system tick. The HAL's st_lld would normally do this;
 * we drive SysTick directly so the image needs no STM32 LLD at all. */
void qrt_systick_init(void);

/* Exits QEMU through semihosting with this status, so a harness can read an
 * exit code rather than parse output. */
void qrt_exit(int code);

/* Check bookkeeping. qrt_report() prints the totals and exits: 0 when every
 * check passed, 1 otherwise. */
void qrt_expect(const char *what, unsigned got, unsigned want);
void qrt_expect_ok(const char *what, int cond);
int  qrt_failures(void);
void qrt_report(void);

/*
 * Touched bytes of a thread working area's stack.
 *
 * chThdCreateStatic lays out the working area as thread_t at the BASE, filled
 * with CH_DBG_THREAD_FILL_VALUE, then the stack above it filled with
 * CH_DBG_STACK_FILL_VALUE and grown downward from the top. The run of
 * remaining fill bytes just above the thread struct is therefore the headroom.
 * Requires CH_DBG_FILL_THREADS, which bldc's chconf.h enables.
 *
 * Verified to be live and exact: a 400-byte local in the thread moves the
 * result by exactly 400.
 */
unsigned qrt_stack_used(const void *wa, size_t wa_size);
size_t   qrt_stack_total(size_t wa_size);

#endif /* QRT_H_ */
