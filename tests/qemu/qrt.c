#include "qrt.h"

/* ------------------------------------------------------------------ console */

#define U1    0x40011000u
#define U_DR  (*(volatile unsigned *)(U1 + 0x04))
#define U_CR1 (*(volatile unsigned *)(U1 + 0x0C))

void qrt_console_init(void) {
  U_CR1 = (1u << 13) | (1u << 3);      /* UE | TE */
}

static void putc_(char c) {
  U_DR = (unsigned)c;
}

void qrt_puts(const char *s) {
  while (*s != '\0') {
    putc_(*s++);
  }
}

void qrt_putu(unsigned v) {
  char b[11];
  int i = 0;

  if (v == 0u) {
    putc_('0');
    return;
  }
  while (v != 0u) {
    b[i++] = (char)('0' + (v % 10u));
    v /= 10u;
  }
  while (i-- > 0) {
    putc_(b[i]);
  }
}

void qrt_puthex(unsigned v) {
  const char *d = "0123456789abcdef";
  int i;

  qrt_puts("0x");
  for (i = 28; i >= 0; i -= 4) {
    putc_(d[(v >> i) & 0xfu]);
  }
}

/* ------------------------------------------------------------------ systick */

#define SYST_CSR (*(volatile unsigned *)0xE000E010u)
#define SYST_RVR (*(volatile unsigned *)0xE000E014u)
#define SYST_CVR (*(volatile unsigned *)0xE000E018u)

/* QEMU's core clock here is not the board's 168 MHz, so wall-clock timing
 * means nothing. What matters is that time advances monotonically at a known
 * tick rate, which is all the kernel needs. */
#define QRT_ASSUMED_CORE_HZ 16000000u

void qrt_systick_init(void) {
  SYST_RVR = (QRT_ASSUMED_CORE_HZ / CH_CFG_ST_FREQUENCY) - 1u;
  SYST_CVR = 0u;
  SYST_CSR = 7u;                       /* ENABLE | TICKINT | CLKSOURCE=core */
}

CH_IRQ_HANDLER(SysTick_Handler) {
  CH_IRQ_PROLOGUE();
  chSysLockFromISR();
  chSysTimerHandlerI();
  chSysUnlockFromISR();
  CH_IRQ_EPILOGUE();
}

/* --------------------------------------------------------------------- exit */

void qrt_exit(int code) {
  volatile unsigned a[2];

  a[0] = 0x20026u;                     /* ADP_Stopped_ApplicationExit */
  a[1] = (unsigned)code;
  {
    register unsigned r0 __asm__("r0") = 0x20u;   /* SYS_EXIT_EXTENDED */
    register unsigned r1 __asm__("r1") = (unsigned)a;
    __asm__ volatile ("bkpt 0xab" :: "r"(r0), "r"(r1) : "memory");
  }
  for (;;) {
  }
}

/* ------------------------------------------------------------------- faults */

/*
 * ChibiOS aliases every fault vector to _unhandled_exception, which spins. A
 * spin is the worst outcome under QEMU because it is indistinguishable from
 * slow progress, so each fault instead prints its status registers and exits.
 *
 * A thread stack overflow is the motivating case: it smashes the thread_t at
 * the base of the working area, and the kernel then wedges with no output at
 * all. See the note in run.sh about why a timeout must be read as a failure.
 */
#define SCB_CFSR  (*(volatile unsigned *)0xE000ED28u)
#define SCB_HFSR  (*(volatile unsigned *)0xE000ED2Cu)
#define SCB_MMFAR (*(volatile unsigned *)0xE000ED34u)
#define SCB_BFAR  (*(volatile unsigned *)0xE000ED38u)

/*
 * The stacked exception frame carries the faulting PC, which is the only
 * diagnostic that turns a fault into an address. Without it a bus fault just
 * reports which address was touched, not what touched it -- which is enough
 * to form a wrong theory and not enough to test one. Feed the PC to
 * `arm-none-eabi-addr2line -e <test>.elf`.
 */
void qrt_fault_report(unsigned *frame, unsigned exc_lr, const char *name);

/*
 * True if an address is in a RAM region we can read without faulting again.
 * The frame lives on the stack that faulted, so on a stack overflow -- the
 * most likely reason to be here -- it points somewhere unmapped. Reading it
 * then takes a second fault inside the handler, which the CPU cannot escalate
 * and which presents as an unexplained lockup with no output. Checking first
 * costs nothing and keeps the report honest about what it could not read.
 */
static int readable(const void *p) {
  unsigned a = (unsigned)(size_t)p;

  return ((a >= 0x20000000u) && (a < 0x20020000u - 32u))   /* SRAM  128K */
      || ((a >= 0x10000000u) && (a < 0x10010000u - 32u));  /* CCM    64K */
}

void qrt_fault_report(unsigned *frame, unsigned exc_lr, const char *name) {
  qrt_puts("\r\nFAULT ");
  qrt_puts(name);
  qrt_puts("\r\n  CFSR=");  qrt_puthex(SCB_CFSR);
  qrt_puts(" HFSR=");        qrt_puthex(SCB_HFSR);
  qrt_puts("\r\n  MMFAR="); qrt_puthex(SCB_MMFAR);
  qrt_puts(" BFAR=");        qrt_puthex(SCB_BFAR);
  qrt_puts("\r\n  SP~=");   qrt_puthex((unsigned)(size_t)frame);
  qrt_puts(" EXC_RET=");     qrt_puthex(exc_lr);
  if (readable(frame)) {
    qrt_puts("\r\n  PC=");  qrt_puthex(frame[6]);
    qrt_puts(" LR=");        qrt_puthex(frame[5]);
    qrt_puts(" PSR=");       qrt_puthex(frame[7]);
  } else {
    /* An unreadable frame is itself the diagnosis: the stack pointer left its
     * stack, which on this part means a thread working area was too small. */
    qrt_puts("\r\n  frame unreadable -- stack pointer is outside RAM,");
    qrt_puts("\r\n  which means a thread overflowed its working area");
  }
  if ((SCB_CFSR & 0x00001000u) != 0u) {
    qrt_puts("\r\n  BFSR.STKERR: fault while stacking (stack overflow)");
  }
  qrt_puts("\r\nRESULT fail\r\n");
  qrt_exit(4);
}

/*
 * Each handler is naked so the frame the CPU pushed is still exactly where it
 * was pushed. Bit 2 of EXC_RETURN says which stack it went on.
 */
#define FAULT_HANDLER(fn, nm)                                                \
  void fn##_c(unsigned *frame, unsigned exc_lr);                             \
  void fn##_c(unsigned *frame, unsigned exc_lr) {                            \
    qrt_fault_report(frame, exc_lr, nm);                                     \
  }                                                                          \
  __attribute__((naked)) void fn(void) {                                     \
    __asm__ volatile (                                                       \
      "tst   lr, #4   \n"                                                    \
      "ite   eq       \n"                                                    \
      "mrseq r0, msp  \n"                                                    \
      "mrsne r0, psp  \n"                                                    \
      "mov   r1, lr   \n"                                                    \
      "b     " #fn "_c\n");                                                  \
  }

FAULT_HANDLER(HardFault_Handler,  "HardFault")
FAULT_HANDLER(MemManage_Handler,  "MemManage")
FAULT_HANDLER(BusFault_Handler,   "BusFault")
FAULT_HANDLER(UsageFault_Handler, "UsageFault")
FAULT_HANDLER(NMI_Handler,        "NMI")

/* bldc's chconf.h routes CH_CFG_SYSTEM_HALT_HOOK here, so a kernel panic
 * reports its reason rather than spinning. */
void main_system_halt(const char *reason) {
  qrt_puts("\r\nPANIC: ");
  qrt_puts((reason != NULL) ? reason : "(null)");
  qrt_puts("\r\nRESULT fail\r\n");
  qrt_exit(3);
}

/* ------------------------------------------------------------------- checks */

static int m_failures;
static int m_checks;

void qrt_expect(const char *what, unsigned got, unsigned want) {
  m_checks++;
  if (got != want) {
    m_failures++;
    qrt_puts("FAIL ");
    qrt_puts(what);
    qrt_puts(": got ");
    qrt_putu(got);
    qrt_puts(" want ");
    qrt_putu(want);
    qrt_puts("\r\n");
  }
}

void qrt_expect_ok(const char *what, int cond) {
  qrt_expect(what, (cond != 0) ? 1u : 0u, 1u);
}

int qrt_failures(void) {
  return m_failures;
}

void qrt_report(void) {
  qrt_putu((unsigned)m_checks);
  qrt_puts(" checks, ");
  qrt_putu((unsigned)m_failures);
  qrt_puts(" failures\r\n");
  qrt_puts((m_failures != 0) ? "RESULT fail\r\n" : "RESULT pass\r\n");
  qrt_exit((m_failures != 0) ? 1 : 0);
}

/* --------------------------------------------------------------- stack usage */

size_t qrt_stack_total(size_t wa_size) {
  return wa_size - sizeof(thread_t);
}

unsigned qrt_stack_used(const void *wa, size_t wa_size) {
  const unsigned char *base = (const unsigned char *)wa + sizeof(thread_t);
  size_t total = qrt_stack_total(wa_size);
  size_t freebytes = 0;

  while ((freebytes < total) && (base[freebytes] == CH_DBG_STACK_FILL_VALUE)) {
    freebytes++;
  }
  return (unsigned)(total - freebytes);
}
