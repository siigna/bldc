# app_pas host tests

Host tests for the pedal decoder in `applications/app_pas.c`.

The real `app_pas.c` is compiled against the stub headers in this directory, so
the tests exercise the shipping decoder rather than a copy of it. Pad levels and
the virtual timer are driven from the test, which makes pedal timing
deterministic and allows uptime to be fast-forwarded.

    make run

The stubs cover only what `app_pas.c` calls: `ch.h`, `hal.h`, `hw.h`,
`mc_interface.h`, `timeout.h`, `commands.h`, `terminal.h` and an empty
`stm32f4xx_conf.h`. `datatypes.h`, `app.h` and `util/utils_math.c` are the real
ones, so `pas_config` and the deadband and filter maths cannot drift from the
firmware.

The stub `hw.h` reproduces the ADC channel aliasing from `hwconf/hw.h`, where
the channels a board does not provide are defined as the first channel. That is
what makes an `#ifdef` test useless and is why `app_pas.c` compares the values,
so the test would be misleading without it.

`hw.h` selects the board configuration from `-D` flags, so the pin and torque
sensor combinations can each be built:

| flag | effect |
|---|---|
| (none) | dedicated pedal sensor pins, no torque sensor |
| `TEST_NO_PAS_PINS` | hardware with no pedal sensor pins at all |
| `TEST_PAS_PINS_SHARED` | pins shared with the COMM UART |
| `TEST_TORQUE_SENSOR` | hardware with a torque sensor |

## What is covered

Cadence: accuracy for both sensor types, independence from uptime, magnet count
scaling, direction gating and inversion, the idle timeout and recovery from it,
rejection of implausibly short periods, that `use_filter` has an effect, and
that an unsupported sensor type decodes nothing.

Torque: the volts to Nm conversion with its zero point and scale, that force
below the zero point reads as no torque rather than negative, the deadband and
its rescaling, the clamp to full scale, saturation reporting, rejection of an
ADC channel the hardware lacks, and each torque source.

Pedalling detection: that the stop threshold alone controls the cutoff delay and
the start threshold does not, that a start threshold below the pulse gap never
establishes a cadence, and that zero derives the old single period including how
badly that scales at a low pole count.

Assist cadence: that the floor lifts the assist basis in proportion, that
reported rider power stays on the real cadence, and that the floor does not
manufacture assist from a stopped crank.

Limits: the speed taper at both ends and half way through, the hard cutoff form,
the power cap including that it applies to cadence control and tracks the input
voltage, the hard pedal stop against the ramping behaviour, the brake including
inversion and channel validation, and the pedelec combination as a whole.

Assist law: rider power as torque times cadence, the gain, the conversion of
watts to a relative current against the input voltage, the input voltage floor,
the assist ceiling, and the torque averaging window including that it smooths
variation through a pedal stroke and that it is discarded when the cranks stop.

One case is worth reading for the hardware implication rather than the
assertion: `test_torque_reference_ceiling`. A 1.5 V to 3.5 V sensor at 70 Nm/V
has 140 Nm of range, but a 3.3 V ADC reference can only reach 126 Nm of it, so
the top of the sensor range is unmeasurable and the assist would flatten out
there rather than at full effort.

## Sampling rate matters in these tests

The firmware thread samples torque at `update_rate_hz`, which at the default
500 Hz and 60 rpm with 24 magnets is about twenty samples per pedal pulse. A
test that pedals and calls the control law once per pulse therefore
misrepresents the sensor low-pass, and will show a residual that the board would
not. `cycle_fwd_with_control` interleaves them at a realistic ratio; use it
rather than driving pulses and the control law in separate loops.

## Beyond `make run`

`tests/check.sh` at the top of the tree runs all of this in one go, and is what
CI runs. Individually:

| target | what |
|---|---|
| `make run-all` | every board variant, and fails if any does |
| `make run-san` | the same under AddressSanitizer and UndefinedBehaviorSanitizer |
| `make coverage` | line and branch coverage of `app_pas.c`, summarised per function |
| `make fuzz-run` | the property fuzzer, `FUZZ_SECS` to set the duration |

The stub firmware lives in `fixture.c` rather than in `main.c` so that the
fuzzer can link the same stubs without a second `main()`.

### Coverage

`make coverage` reports coverage of `app_pas.c`, not of the test file: what
matters is which branches of the firmware the tests reach. `uncovered.py`
groups the unreached lines by function, because a per-line report is too long
to read and says which lines are untested rather than which behaviour is.

Both the thread and the terminal commands **are** covered, through two seams
in the stubs rather than through changes to the firmware:

- `chThdCreateStatic` records the thread entry point instead of starting one.
  `pas_thread` is static inside `app_pas.c`, so this is the only way a test can
  reach it. `run_thread` then calls it directly.
- `terminal_register_command_callback` records the name and callback, so a test
  can invoke a command with its own `argv` and read what it printed back out of
  the buffer `commands_printf` fills.

Leaving the thread loop is the awkward part, and worth knowing about. It exits
when `stop_now` is set, and the only thing that sets it is `app_pas_stop` --
which then spins until the thread clears `is_running`. Called from inside the
thread that is a deadlock. So the sleep hook `longjmp`s out after a set number
of iterations, which leaves `is_running` true, exactly as it is on real
hardware while the thread runs. Nothing may call `app_pas_stop` afterwards,
which is why the thread test runs last. The real exit path needs a scheduler
and stays uncovered.

What the thread tests are actually for is the interlocks, which is the part of
the file that decides whether the motor gets current: safe start, the fault
check, and the output-disabled path. Two of those turn out **not to write the
motor at all** rather than writing zero, which is the stronger guarantee and
what the tests assert -- a sentinel value left untouched, plus a published zero
for the throttle app to read.

Still uncovered, and why: about half of `pas_status`, which is advisory text
behind conditions a bench session would have to construct one at a time; the
`is_running` branch of `app_pas_apply_to_throttle`, for the reason above; and
the handful of `pas_compute_output` branches that need a second motor or a
board this build is not.

### The fuzzer

`fuzz.c` drives the module with sequences no rider would produce: pedal pulses
in illegal orders, time steps of zero, a torque signal above the reference, a
pack voltage of zero or of 120 V, a configuration outside the limits VESC Tool
enforces. Then it asserts the properties that have to hold whatever the input
was -- output finite and within 0..1, pedal rpm within human range, the speed
taper within 0..1, mixing closed over its input range.

The properties are the point. A crash means an invariant broke, not that the
module was fed something silly: it polls real hardware, and real hardware
produces silly things -- a dirty sensor, a loose connector, a brownout that
restarts one MCU and not the other.

It found one real bug in its first second. `current_scaling * sub_scaling` is
used as the `max` argument of `utils_truncate_number`, which tests the maximum
before the minimum, so a **negative** ceiling clamped the output to that
negative value instead of to the zero minimum -- and
`mc_interface_set_current_rel` reads a negative value as braking current. A
negative PAS Max Current made pedalling brake the bike in proportion to
cadence. VESC Tool will not send one, since the parameter's minimum is zero,
but the XML limits are not part of the configuration signature and are not
enforced anywhere in the firmware, so any sender of `COMM_SET_APPCONF` can
write one. Fixed in `pas_output_ceiling`, and kept as
`test_negative_scaling_cannot_brake` -- a fuzzer finding is only a regression
test once it is one.

## clang-tidy

Checks are in `.clang-tidy` at the top of the tree, curated with the reasoning
there. The default sets are unusable on a codebase of this age:
`bugprone-narrowing-conversions` alone fires on every `static float x = 0.0;`
in the firmware, which is the house style, and `misc-include-cleaner` fights
the umbrella headers the tree uses on purpose. Left on: `clang-analyzer`'s
path-sensitive checks, and the `bugprone`, `misc` and `performance` checks that
are not about house style.

It found one real bug. The thread took its interval through `ST2MS`, which
divides as integers **and rounds up**, so `dt_ms` was quantised to whole
milliseconds. At the default 500 Hz update rate that happens to be exact, which
is why nothing noticed; but the rate is configurable, and at 3 kHz a true 0.3 ms
interval came back as 1 ms -- three times too long, feeding the output ramp and
the power loop's integral. It is scaled in floating point now.

Two findings in the tests were mine and fixed: a `memcmp` over `pas_config`,
which has padding whose bytes are unspecified, so the comparison could report a
difference that was not one; and an out-of-range enum cast, which is the point
of that test and carries a `NOLINTNEXTLINE` with the reason.

Three findings are upstream and left alone: `bugprone-macro-parentheses` on
`UTILS_NAN_ZERO` and `UTILS_LP_FAST` in `util/utils_math.h`, where the
assignment target is not parenthesised. Real, and worth reporting, but misusing
either is a compile error rather than silent misbehaviour, and those macros are
used across the whole tree.

Two things about running it that cost an hour between them, both now handled in
`check.sh`:

- **`WarningsAsErrors: '*'` is wrong here.** It counts warnings raised inside
  *system* headers, which clang-tidy neither displays nor lets you fix, so it
  exits non-zero having printed nothing at all. `check.sh` decides instead, by
  failing when a diagnostic is printed for a file that is ours.
- **Include order matters.** The stub directory has to come before `-I.`, or
  `timeout.h` resolves to the real one and nothing parses. The Makefile always
  had it right because it runs from inside this directory.

## Tree-wide cppcheck

`./tests/check.sh --tree` runs cppcheck over `applications`, `util`, `comm`,
`motor` and `driver`. It is **informational and does not gate**, because what it
finds is not this branch's to fix.

At `warning,performance,portability` the whole firmware produces four findings,
and all four are false positives or environmental:

| finding | why it is not a bug |
|---|---|
| `mcpwm.c:540,541` division by zero | `curr_start_samples` is a `volatile` a busy-wait waits for an interrupt to raise; cppcheck cannot see the ISR |
| `mcpwm.c:2333` uninitialised `val_sample` | same, a struct an interrupt fills |
| `lzodefs.h` `#error` | needs LZO's own defines to preprocess |

Adding `style` takes it to 141, of which 77 are const-correctness and 28 are
variable scope. The tail worth reading is five findings across
`digital_filter.c`, `foc_math.c`, `mc_interface.c` and `driver/timer.c`, all
double literals in float expressions or a small shift into a long -- benign.

So the honest value of the tree-wide pass is as a **regression net**: a new
finding in a file this branch touches is worth reading, and the standing four
are not.
