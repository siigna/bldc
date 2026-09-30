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

Around a third of the file is the three terminal commands and the thread
function, which need a terminal and a scheduler and are not reachable from
here. Two getters are also unreachable in principle: `app_pas_is_running` and
`app_pas_get_current_target_rel` report state the **thread** writes, so in a
host test they read as their initial values however much the assist law is
exercised. That is worth knowing rather than working around -- it also means
`app_pas_apply_to_throttle` passes the throttle through unchanged for however
long it takes the thread to be scheduled after a start.

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
