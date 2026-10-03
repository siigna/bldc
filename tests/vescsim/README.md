# A controller over TCP, with no controller

```
make && make check && ./vescsim -p 65102
```

Serves enough of the VESC protocol that ESCargot Tool considers itself
connected and will poll real-time values. A connection is the gate in front
of the Tool's log settings, its real-time pages and everything reached from
them, so without something like this none of that is testable without
hardware.

Verified end to end against the real Tool:

```
"VESC Firmware Version 7.2, Hardware: SIM, UUID: A0 A1 ... AB"
Connected
```

## Why it lives here

The protocol is defined in this repository, so the simulator uses this
repository's own `comm/packet.c` and `util/crc.c`. The framing, the length
encoding and the CRC are the firmware's implementation, not a second one
written from the specification — an independent reimplementation of framing
mostly tests the reimplementation. `-DNO_STM32` is what lets `crc.c` build
for a host, the same way `tests/packet_recovery` does it.

What is *not* the firmware's is the replies: those are assembled here from
the layouts in `comm/commands.c`. Drift between the two is the thing this
could get wrong, so:

- `make check-version` compares the version it reports against
  `conf_general.h`. A mismatch makes a tool refuse to talk to it for a reason
  that looks like a protocol bug.
- `make check-layout` re-extracts the `COMM_GET_VALUES` field list from
  `comm/commands.c` and compares types and scales against what the simulator
  sends. Thirty fields at the time of writing.

That second check is the one worth having. For a plain `COMM_GET_VALUES` the
mask is all ones and is not transmitted, so every field goes out in bit order
and one wrong type or scale shifts everything after it — the Tool then shows
plausible nonsense rather than failing. Writing that list from memory got
three fields wrong on the first attempt: `pid_pos` at the wrong scale,
`controller_id` as a float when it is a `uint8`, and `vd`/`vq` at 1e2 instead
of 1e3.

It also pins the coupling that matters across the two repositories: **bit 22
of `COMM_GET_VALUES` is this fork's own PAS extension** — pedal rpm, torque,
rider power, motor power target, current target and flags. A simulator that
omitted them would leave the Tool reading six fields of nothing.

## What it does not serve

`COMM_GET_MCCONF` and `COMM_GET_APPCONF` carry a `confgenerator`
serialisation whose signature the Tool checks against its own XML. A wrong
signature is rejected wholesale, and a truncated blob is worse than no reply
because the Tool cannot tell it from corruption. So those are answered with
silence, which the Tool treats as a timeout and carries on from — it reports
"Could not read the motor configuration" and stays connected.

Serving them means linking `confgenerator.c` and the hardware headers behind
it. That is worth doing: it would make the Tool's XML and the firmware's
serialisation testable against each other, which is what the
`APPCONF_SIGNATURE` work is about. Not done.

## A trap, when the Tool will not connect

If the Tool reports `"fw_version" not found` and "does not seem to have any
supported firmwares", that is not this simulator. `fw_version` comes from the
Tool's info config, and `Utility::configPath` registers
`$AppData/res_config.rcc` over the compiled-in parameter XML — so a stale
archive there makes a working connection look like a protocol failure. Run
the Tool with `XDG_CONFIG_HOME`, `XDG_DATA_HOME` and `HOME` pointed at a
temporary directory, which is what its own test harness does.

## Reaching it from an Android emulator

The emulator reaches the host as `10.0.2.2`, and the simulator binds every
interface, so `10.0.2.2:65102` works from inside. That is the route to
testing the storage access framework picker and background logging, both of
which sit behind a connection.
