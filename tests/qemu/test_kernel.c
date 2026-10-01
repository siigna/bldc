/*
 * Proves the simulated target is worth testing against: the real ChibiOS
 * kernel, built from the vendored tree with bldc's own chconf.h, boots under
 * QEMU and provides working time, threads, mutexes and CCM-resident stacks.
 *
 * Everything the script engine needs from the OS is exercised here, so a
 * failure in this file means the simulator is wrong, not the engine.
 */

#include "qrt.h"

static MUTEX_DECL(m_lock);
static unsigned m_shared;
static unsigned m_evt_seen;

/* .ram4 is CCM. Putting the working area there is what the engine thread will
 * do, and on this part CCM is the only place with room for it. */
static THD_WORKING_AREA(wa_worker, 512) __attribute__((section(".ram4")));

static event_source_t m_evs;

static THD_FUNCTION(worker, arg) {
  (void)arg;
  chRegSetThreadName("worker");

  for (int i = 0; i < 100; i++) {
    chMtxLock(&m_lock);
    m_shared++;
    chMtxUnlock(&m_lock);
    chThdSleepMilliseconds(1);
  }
  chEvtBroadcast(&m_evs);
}

int main(void) {
  qrt_console_init();

  /* The kernel is the thing under test, so start it before anything else. */
  chSysInit();
  qrt_systick_init();
  chEvtObjectInit(&m_evs);

  qrt_puts("chibios up\r\n");

  /* --- CCM is mapped and writable, which the linker script assumes ------- */
  {
    volatile unsigned *ccm = (volatile unsigned *)0x10000000u;
    const unsigned last = (64u * 1024u / 4u) - 1u;

    ccm[0]    = 0xdeadbeefu;
    ccm[last] = 0xa5a5a5a5u;
    qrt_expect_ok("ccm base writable", ccm[0] == 0xdeadbeefu);
    qrt_expect_ok("ccm top writable",  ccm[last] == 0xa5a5a5a5u);
  }

  /* --- time advances at the configured tick rate ------------------------ */
  {
    systime_t t0 = chVTGetSystemTimeX();
    systime_t t1;
    unsigned ms;

    chThdSleepMilliseconds(50);
    t1 = chVTGetSystemTimeX();
    ms = (unsigned)(t1 - t0) / (CH_CFG_ST_FREQUENCY / 1000u);
    qrt_puts("slept_ms=");
    qrt_putu(ms);
    qrt_puts("\r\n");
    qrt_expect_ok("sleep >= 50ms", ms >= 50u);
    qrt_expect_ok("sleep < 100ms", ms < 100u);
  }

  /* --- threads, mutexes, events, and a working area in CCM -------------- */
  {
    event_listener_t el;
    thread_t *tp;

    chEvtRegister(&m_evs, &el, 0);
    tp = chThdCreateStatic(wa_worker, sizeof(wa_worker),
                           NORMALPRIO + 1, worker, NULL);
    qrt_expect_ok("thread created", tp != NULL);
    qrt_expect_ok("working area is in ccm",
                  ((unsigned)(size_t)wa_worker >> 24) == 0x10u);

    if (chEvtWaitOneTimeout(ALL_EVENTS, MS2ST(2000)) != 0) {
      m_evt_seen = 1u;
    }
    chThdWait(tp);
    chEvtUnregister(&m_evs, &el);

    qrt_expect("mutex-guarded count", m_shared, 100u);
    qrt_expect_ok("event delivered", m_evt_seen != 0u);
  }

  /* --- the stack high-water measurement the engine thread will need ----- */
  {
    unsigned used  = qrt_stack_used(wa_worker, sizeof(wa_worker));
    unsigned total = (unsigned)qrt_stack_total(sizeof(wa_worker));

    qrt_puts("worker_stack_used=");
    qrt_putu(used);
    qrt_puts(" of ");
    qrt_putu(total);
    qrt_puts("\r\n");
    qrt_expect_ok("stack use measured", used > 0u);
    qrt_expect_ok("stack did not overflow", used < total);
  }

  qrt_report();
  return 0;
}
