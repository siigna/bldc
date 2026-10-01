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
