// A cooperative scheduler for one thread, so the app's own thread function can
// be run from a host test.
//
// The firmware creates its thread with chThdCreateStatic and ends it by setting
// a flag that the thread itself has to observe. Neither is possible with a stub
// that does nothing: is_running is never true, app_pas_stop spins forever
// waiting for an acknowledgement, and the whole exit path is unreachable.
//
// Real OS threads would model it faithfully and would also be wrong here. The
// firmware's shared state is `volatile`, not atomic, which is sound on a
// single-core MCU with a scheduler that switches at known points and is a data
// race on a host with pre-emption. A test built on that would be racy and
// occasionally wrong.
//
// So the thread is a coroutine. Exactly one side runs at a time, switches
// happen only where the firmware sleeps, and the test drives it: a switch into
// the thread runs it up to its next sleep, and a sleep inside the thread
// returns control. app_pas_stop then works for real -- its wait loop sleeps,
// which switches into the thread, which sees the flag and exits.
#ifndef SCHED_H
#define SCHED_H

#include <stdbool.h>

// Records the entry point and prepares its stack. Does not run it.
void sched_create(void (*fn)(void *), void *arg);

// True once created and not yet returned.
bool sched_alive(void);

// True while the thread's own code is executing.
bool sched_in_thread(void);

// Hand control to the other side. From the test this runs the thread to its
// next sleep; from the thread it returns to the test. A call with nothing
// created, or from the test with the thread finished, does nothing.
void sched_switch(void);

// Forget the thread, so a later create starts clean. Does not unwind it: a
// thread abandoned part way through has left the firmware's state as it was,
// which is what a test that stops driving it wants.
void sched_reset(void);

#endif
