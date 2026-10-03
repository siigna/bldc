#!/usr/bin/env python3
"""Checks vescsim's COMM_GET_VALUES layout against comm/commands.c.

Run as `make check-layout`.

The simulator assembles that reply by hand, field by field. For a plain
COMM_GET_VALUES the mask is all ones and is not sent, so every field goes out
in bit order and one wrong type or scale shifts everything after it -- the
Tool then displays plausible nonsense instead of failing, which is the worst
way for this to be wrong.

So the field list is extracted from the firmware here and compared with what
the simulator implements. It compares types and scales, not the values, which
are the simulator's own business.

This catches the case that actually matters: a field added to the firmware's
reply -- as this fork did with the PAS block at bit 22 -- without the
simulator following.
"""

import re
import sys

COMMANDS = "../../comm/commands.c"
SIM = "vescsim.c"


def firmware_fields():
    s = open(COMMANDS).read()
    start = s.index("case COMM_GET_VALUES:\n\tcase COMM_GET_VALUES_SELECTIVE:")
    end = s.index("reply_func(send_buffer, ind);", start)
    seg = s[start:end]

    out = []
    bit = None
    seen_motor2 = False

    for line in seg.splitlines():
        m = re.search(r"1 << (\d+)\)", line)
        if m:
            bit = int(m.group(1))
            continue

        # The MOS temperatures appear twice, once per motor. The simulator is
        # a single motor, so only the first branch is taken.
        if "_M2()" in line:
            seen_motor2 = True
            continue
        if seen_motor2 and "NTC_TEMP_MOS" in line:
            pass

        m = re.search(r"buffer_append_(float16|float32)\(send_buffer, [^,]+, ([0-9e.]+), &ind\)", line)
        if m:
            out.append((bit, m.group(1), m.group(2)))
            continue

        m = re.search(r"buffer_append_(uint16|uint32|int32)\(send_buffer, ([^,]+), &ind\)", line)
        if m:
            # The mask itself is appended only for COMM_GET_VALUES_SELECTIVE.
            # A plain COMM_GET_VALUES, which is what the simulator serves,
            # does not carry it.
            if m.group(2).strip() == "mask":
                continue
            out.append((bit, m.group(1), None))
            continue

        m = re.search(r"send_buffer\[ind\+\+\] = (.+);", line)
        if m and "packet_id" not in m.group(1):
            out.append((bit, "uint8", None))

    return out


def sim_fields():
    s = open(SIM).read()
    start = s.index("static void send_values(void)")
    end = s.index("reply(b, ind);", start)
    seg = s[start:end]

    out = []
    for line in seg.splitlines():
        m = re.search(r"buf_f(16|32)\(b, &ind, [^,]+, ([0-9e.]+)f\)", line)
        if m:
            out.append(("float" + m.group(1), m.group(2)))
            continue
        m = re.search(r"buf_u16\(b, &ind,", line)
        if m:
            out.append(("uint16", None))
            continue
        m = re.search(r"buf_i32\(b, &ind,", line)
        if m:
            out.append(("int32", None))
            continue
        if re.search(r"b\[ind\+\+\] = ", line) and "COMM_GET_VALUES" not in line:
            out.append(("uint8", None))

    return out


def main():
    fw = firmware_fields()
    sim = sim_fields()

    # The firmware emits the three MOS temperatures twice, once per motor
    # branch; a single-motor simulator sends them once.
    fw_norm = []
    mos = 0
    for bit, typ, scale in fw:
        if bit == 18:
            mos += 1
            if mos > 3:
                continue
        fw_norm.append((typ, scale))

    if len(fw_norm) != len(sim):
        print("  FAILED  the firmware sends %d fields, vescsim sends %d"
              % (len(fw_norm), len(sim)))
        print("          comm/commands.c and vescsim.c have drifted; the Tool")
        print("          would read shifted values rather than fail")
        for i in range(max(len(fw_norm), len(sim))):
            a = fw_norm[i] if i < len(fw_norm) else None
            b = sim[i] if i < len(sim) else None
            if a != b:
                print("          first difference at field %d: firmware %s, sim %s"
                      % (i, a, b))
                break
        return 1

    for i, (a, b) in enumerate(zip(fw_norm, sim)):
        if a[0] != b[0]:
            print("  FAILED  field %d is %s in the firmware and %s in vescsim"
                  % (i, a[0], b[0]))
            return 1
        if a[1] and b[1] and float(a[1]) != float(b[1]):
            print("  FAILED  field %d scale is %s in the firmware and %s in vescsim"
                  % (i, a[1], b[1]))
            return 1

    print("  ok      COMM_GET_VALUES layout matches commands.c (%d fields)"
          % len(sim))
    return 0


if __name__ == "__main__":
    sys.exit(main())
