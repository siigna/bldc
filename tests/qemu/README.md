# Simulated STM32F405 target

`qemu-system-arm -M olimex-stm32-h405` models an **STM32F405** — the same part
the firmware targets. That makes it possible to run the real ChibiOS kernel,
built from the vendored tree with bldc's own `chconf.h`, without a board.

```sh
nix-shell -p gcc-arm-embedded gnumake qemu --run ./run.sh
```

## What this is for

The script engine needs five things from the OS: a thread, a mutex, an event,
a timed sleep, and somewhere in CCM to put a stack. All five are testable here
against the real kernel rather than a host shim, on real Thumb-2 code, with
real stack sizes. `tests/script_queue` and `tests/script_alloc` stub ChibiOS
out to test logic on the host; this complements them by testing the parts where
the OS itself is the thing that has to behave.

The measurement that matters most is `qrt_stack_used()`, which reads the
`CH_DBG_FILL_THREADS` pattern to report a thread's stack high-water mark.
Sizing the engine thread's working area is otherwise a bench job, and `.ram4`
has very little slack. The figure is exact: a 400-byte local in the thread
moves it by exactly 400.

## The configuration bindings, and a table that cannot drift

`conf_get`/`conf_set` cover 143 parameters. In lisp they are a pair of
500-line if-else chains, one per direction. Here the mapping is **generated
from those chains** by `tests/conf_table/gen_conf_table.py`, and
`tests/conf_table/run.sh` regenerates and diffs it, so a divergence fails a
test rather than shipping.

Generated rather than retyped because a round-trip test cannot catch the
failure that matters: set-then-get agrees whether `l_current_max` is wired to
`l_current_max` or to the field beside it. An independent statement of the
same mapping is the only thing that catches it, and lisp's chains are one that
already exists. The generator refuses anything it does not recognise, so a new
upstream parameter is reported rather than guessed at.

Two properties also had to be derived, not assumed:

- **87 of 143 need a full reconfigure** rather than a write to the live
  struct. Writing one of those through the fast path leaves it looking set
  without taking effect.
- **Four are stored negative** and given as a positive magnitude (`-fabsf` in
  lisp). `conf_get` returns the stored value either way, matching lisp.

### Byte offsets were the wrong design, and only the target said so

The first version was a table of `offsetof` plus a width classification. It
HardFaulted with `UFSR.UNALIGNED`, and the reason is worth recording:

- `uint8_t` and `uint16_t` members were classified as int and read four bytes
  wide — both unaligned and overlapping their neighbours.
- A float read through a cast compiles to `VLDR`, which has **no unaligned
  form** on a Cortex-M4, so a misaligned float access faults rather than
  limping.
- `arm-none-eabi` defaults to `-fshort-enums`, so an enum's width is not
  knowable from its declaration at all. That makes the approach unfixable
  rather than merely buggy.

The generated lists now name the struct member and let the compiler pick the
load. The cost is a `strcmp` chain — which is what lisp does too — and about
11 KB of flash over the offset table. Correctness was worth it; `conf_get` is
on no hot path.

A comment I had written claimed `CONF_INT` was only emitted for int-sized
members. It was false when I wrote it.

### Measured

Walking all 143 names from Lua peaks at **20,174 bytes** of script memory —
essentially the firmware's entire 20 KB ceiling. A script that enumerates the
configuration has no room left for anything else.

## The motor bindings

`test_bindings` compiles `script/lua_vesc_mc.c` against a fake `mc_interface`
in `fake/`, which sits **earlier on the include path** than the real headers.
The bindings therefore carry no test seam of their own -- the production file
is compiled unmodified.

`fake/` must come first in `INCDIR`, before `$(TOP)`, which also has a real
`timeout.h`. And `mc_fault_code` comes from the firmware's own `datatypes.h`
rather than a copy, so the codes a test asserts on cannot drift from the ones
the firmware reports.

The property worth the effort is that **every setter refreshes the motor
timeout**. A setter that forgets works perfectly until the first gap longer
than the timeout, and then the output drops out with nothing to show why. The
setters are therefore driven from a list, so adding one without adding it to
the list is the only way to miss it.

Two mutations needed the test sharpened rather than the code:

- Reading an absent argument with `lua_tonumber` yields 0, and
  `mc_interface_set_current_off_delay(0)` is **not** the same as not calling
  it -- zero clears a delay set earlier. Asserting the recorded value was
  still 0 could not tell those apart. The fake now counts the calls.
- Name parity with lisp was claimed, not checked. Comparing the two name
  lists mechanically found that lisp calls the motor temperature
  `get-temp-mot`, not `get-temp-motor`, and binds no tachometer getter at
  all. A name *nearly* the same as the other engine's is worse than one that
  is identical or obviously different, so the binding was renamed and the two
  additions documented.

`test_luaif` then drives the whole chain: a CAN frame arriving on one thread
ending up as motor current from another, through a script -- plus the case
where the script filters the frame out, without which the first check passes
for a handler that ignores its arguments. One thing it records rather than
asserts: stopping a script does **not** command the motor to zero. It keeps
what it was last told until the firmware's timeout expires. That is the
timeout's job, not the adapter's, and it is worth knowing which.

## What it found in the protocol handler

`test_luaif` also drives the `COMM_LISP_*` packets the way `commands.c` does.
Three things came out of that:

**The REPL must not run on the thread that received the packet.** Building and
calling a chunk needs ~2 KB of stack before the script does anything. Running
it on the caller gave a HardFault with the PC set to the `0x55555555` stack
fill pattern, several frames from the cause. `script_event.h`'s own note about
`SCRIPT_EV_REPL` says exactly this -- the same mistake had already been made
and fixed in vesc_express -- and it was made again here anyway. Evaluation now
goes to the engine thread through a dedicated slot.

**The protocol has to be tested on a comms-sized stack.** Driving it from
`main` was both unfaithful and too small: `commands_process_packet` runs on
`comm_usb`'s `serial_process_thread`, whose working area is 2048 bytes, while
main's process stack is 0x800. The checks faulted intermittently with
`UFSR.INVPC`, three runs in five. Measured on a 2048-byte thread,
`luaif_process_cmd` uses **1,648 of 2,384 bytes** -- and that figure includes
the test's own `vsnprintf`, which the real `commands_printf_lisp` does not do
on the caller's stack.

**One passing run proves nothing about a flaky test.** The variable list is
best-effort by design: `append_globals` takes the lock with `chMtxTryLock`,
because the engine thread holds it while dispatching and waiting would hang
the connection on a busy script. So a single `GET_STATS` can legitimately
return no variables. The first version asserted on one poll and passed by
luck; it now polls until they appear, which is what VESC Tool does anyway.

Eight mutations of the handler, all caught, including the one that reproduces
the REPL stack fault and the one that copies lispif's habit of passing the
read index to `reply_func` instead of the built length.

## What it found: three bugs in the firmware adapter

`test_luaif` runs `script/luaif.c` -- the engine thread, the lifecycle and the
event path -- against the real kernel, with only the flash and the terminal
stubbed. Writing it turned up three defects that no amount of reading had:

**A zero yield parks the thread forever.** `on_tick` called
`chThdSleepMilliseconds(0)`. Zero is `TIME_IMMEDIATE`, not a valid sleep, and
with `CH_DBG_ENABLE_ASSERTS` off -- how the firmware is built -- it does not
complain, it simply never wakes. Any script short enough to finish inside one
`hook_count` never reaches the hook, so only `while true do end` showed it.
Now `chThdYield()`.

**One flag cannot carry the stop handshake.** The thread cleared the restart
request *before* doing the work, so `luaif_restart` stopped waiting and read
the state of a torn-down engine, reporting failure for a script that was about
to run fine. Clearing it afterwards instead would have `should_stop()` abort
the very script it was asked to start. It needs three counters: a request, an
ack for "the old engine is gone" (which is what `should_stop` keys off), and
an ack for "the request is served" (which is what callers wait for).

**A restart timeout can kill a working script.** `request()` is bounded so a
wedged engine cannot take the comms thread with it. But a main chunk that
legitimately runs past the deadline gets force-closed -- the first long script
added here printed its result *and* reported failure.

## A coverage gap that passing tests hid

Mutation testing is the only reason the handshake is tested at all. With
`m_ack_tear` never set, `should_stop()` is permanently true, which kills every
script after the first restart -- and all twenty checks still passed, because
no test script ran long enough to reach the instruction hook. A script that
crosses `hook_count` and must *not* be interrupted was the missing case.

One mutation still survives knowingly: removing `on_tick`'s yield changes
nothing here, because the engine runs below `NORMALPRIO` and everything that
matters preempts it regardless. The yield buys fairness against threads at its
own priority, and the test has none.

## What it found: Lua's C-call limit is unusable at its default

`test_engine` runs the real engine -- the vendored Lua core, the container
parser and the CCM arena -- on a thread whose working area is in `.ram4`,
exactly where the firmware adapter will put it. That turned up a problem no
host test could see, because a host has a stack measured in megabytes.

Each nested `pcall` level costs a measured **464 bytes of thread stack**, over
a 2,088-byte baseline, linear across depths 5 to 30:

| depth | 0 | 5 | 10 | 20 | 30 |
|---|---|---|---|---|---|
| stack used | 2088 | 3632 | 5952 | 10592 | 15232 |

`LUAI_MAXCCALLS` defaults to 200. That is the guard whose whole purpose is to
stop runaway recursion *before* it overflows the C stack, and at 200 it would
need `2088 + 200 * 464` = **94,888 bytes** for one thread. All of CCM is
65,536. The hardware stack dies long before Lua's check fires.

Measured difference at depth 40 with a 12 KB working area:

| `LUAI_MAXCCALLS` | result |
|---|---|
| 200 (default) | `FAULT HardFault`, `BFSR.STKERR` -- the controller is gone |
| 16 | `C stack overflow`, a catchable Lua error; the script dies, firmware continues |

So `script/script.mk` sets `-DLUAI_MAXCCALLS=16`, sized for a 12 KB working
area by `(usable_stack - 2088) / 464` less margin. The two numbers have to move
together: raising the limit means raising the working area, and there is no
free way to get one. `llimits.h` guards the default with `#if !defined`, so a
`-D` is the documented way in.

On a vehicle this is the difference between a script erroring and a motor
controller resetting mid-ride.


## What it does *not* simulate

Nothing board-specific. There is no HAL and no STM32 LLD in the image at all:

- **No peripherals.** No ADC, CAN, UART receive, SPI, I²C, or FOC timers. QEMU
  does not model them, and the motor control path cannot be exercised here.
- **No flash driver.** Reading a script out of `CODE_IND_LISP` needs the real
  `flash_helper`, so a test that loads from flash has to fake the read.
- **No real clock tree.** `halInit()` is never called, and the STM32 clock
  setup would spin on PLL-ready bits QEMU does not implement. (`STM32_NO_INIT`
  in `hwconf/mcuconf.h` is the documented escape hatch if a future test does
  want the HAL.) SysTick is driven directly at `CH_CFG_ST_FREQUENCY`, so tick
  counts are faithful but **wall-clock timing is meaningless** — this cannot
  measure whether something fits in a control-loop deadline.
- **Timing and concurrency are not the hardware's.** QEMU is not cycle
  accurate, so a race that depends on real interrupt latency may not reproduce,
  and one that does reproduce here may have different odds on silicon.

So: a pass here means the logic and the OS usage are sound. It is not evidence
that anything works on a motor controller.

## Why QEMU and not the ChibiOS simulator

`ChibiOS_3.0.5/os/rt/ports/SIMIA32` runs the kernel as a 32-bit x86 Linux
process, which would be faster and debuggable with the usual tools. QEMU was
chosen anyway because the questions worth asking are about the target: Thumb-2
code size, CCM placement, and stack consumption under the real ARM port.
SIMIA32 answers none of those.

## Two traps in the harness itself

Both of these made an earlier version of this directory report success wrongly.

**A hang must not read as a pass.** A thread stack overflow smashes the
`thread_t` at the *base* of its working area and wedges the kernel. ChibiOS
aliases every fault vector to a spin loop, so the original symptom was no
output at all — no panic, no fault, nothing, which is indistinguishable from
slow progress. `qrt.c` now overrides the fault vectors to dump `CFSR`/`HFSR`/
`MMFAR`/`BFAR` and exit, and `run.sh` treats a `timeout` as a failure.

**`make run` collapses every child status to 2**, which hid the difference
between a timeout, a fault and a failed check — every mutation reported the
same misleading verdict. `run.sh` invokes QEMU directly for that reason.

The runner is verified against five deliberate breakages: a wrong expected
value, a stack overflow, a null dereference, an infinite loop, and exiting 0
without reporting. Each produces a distinct, correct verdict. Absence of `FAIL`
lines is not treated as success — a fault exits before printing any, so
`RESULT pass` must be present *and* the exit status zero.
