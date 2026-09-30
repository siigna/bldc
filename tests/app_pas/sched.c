#include "sched.h"

#include <stddef.h>
#include <string.h>
#include <ucontext.h>

// AddressSanitizer prints one warning the first time swapcontext is used:
// "ASan doesn't fully support makecontext/swapcontext functions and may produce
// false positives in some cases". It is a warning, not a finding, and no false
// positive has appeared from this scheduler -- the sanitizer run is otherwise
// clean over all four board variants.
//
// It does have a fiber annotation API, __sanitizer_start_switch_fiber and
// __sanitizer_finish_switch_fiber, which is meant to remove the warning. An
// attempt at it produced "finishing a fiber switch that has not started",
// because getting the pairing right across both the trampoline's first entry
// and the ordinary two-way switch is subtler than it looks. Rather than ship a
// guess at a sanitizer's internal contract, the warning stands and check.sh
// fails on ERROR and on runtime errors, not on it.

#define SCHED_STACK_SIZE (256 * 1024)

static ucontext_t ctx_main;
static ucontext_t ctx_thread;
static char thread_stack[SCHED_STACK_SIZE];

static void (*thread_fn)(void *);
static void *thread_arg;
static bool thread_created;
static bool thread_done;
static bool in_thread;


static void sched_trampoline(void) {
	thread_fn(thread_arg);

	// Returning means the thread function ran to completion, which for the app
	// means it saw its stop flag and exited. uc_link takes us back.
	thread_done = true;
	in_thread = false;
}

void sched_create(void (*fn)(void *), void *arg) {
	thread_fn = fn;
	thread_arg = arg;
	thread_created = true;
	thread_done = false;
	in_thread = false;

	getcontext(&ctx_thread);
	ctx_thread.uc_stack.ss_sp = thread_stack;
	ctx_thread.uc_stack.ss_size = sizeof(thread_stack);
	ctx_thread.uc_link = &ctx_main;
	makecontext(&ctx_thread, sched_trampoline, 0);
}

bool sched_alive(void) {
	return thread_created && !thread_done;
}

bool sched_in_thread(void) {
	return in_thread;
}

void sched_switch(void) {
	if (!thread_created) {
		return;
	}

	if (in_thread) {
		in_thread = false;
		swapcontext(&ctx_thread, &ctx_main);
		in_thread = true;
		return;
	}

	if (thread_done) {
		return;
	}

	in_thread = true;
	swapcontext(&ctx_main, &ctx_thread);
	in_thread = false;
}

void sched_reset(void) {
	thread_created = false;
	thread_done = false;
	in_thread = false;
}
