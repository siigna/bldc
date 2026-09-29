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

One case is worth reading for the hardware implication rather than the
assertion: `test_torque_reference_ceiling`. A 1.5 V to 3.5 V sensor at 70 Nm/V
has 140 Nm of range, but a 3.3 V ADC reference can only reach 126 Nm of it, so
the top of the sensor range is unmeasurable and the assist would flatten out
there rather than at full effort.
