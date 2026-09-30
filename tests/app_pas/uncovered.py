#!/usr/bin/env python3
"""Summarise a gcov annotation by function.

gcov prints one line per source line, which is too much to read and too easy to
ignore. This groups the unreached lines under whichever function they are in,
so the output says which behaviour is untested rather than which lines are.
"""
import re
import sys

path = sys.argv[1] if len(sys.argv) > 1 else "app_pas.c.gcov"

# A definition at column zero that ends in "{", which is how every function in
# this file is written. Close enough, and wrong only in ways that misattribute
# a line rather than miss one.
DEF = re.compile(r"^[A-Za-z_][\w \t\*]*\b(\w+)\s*\([^;]*\)\s*\{\s*$")

func = "(file scope)"
total = {}
missed = {}

try:
    lines = open(path, encoding="utf-8", errors="replace").read().splitlines()
except OSError as e:
    print(f"  no coverage annotation: {e}")
    sys.exit(0)

for raw in lines:
    parts = raw.split(":", 2)
    if len(parts) < 3:
        continue
    count, _, text = parts[0].strip(), parts[1], parts[2]

    m = DEF.match(text)
    if m:
        func = m.group(1)

    if count in ("-", ""):
        continue
    total[func] = total.get(func, 0) + 1
    if count == "#####" or count == "=====":
        missed[func] = missed.get(func, 0) + 1

rows = []
for f, t in total.items():
    m = missed.get(f, 0)
    if m:
        rows.append((m / t, m, t, f))
rows.sort(reverse=True)

if not rows:
    print("  every executable line reached")
else:
    for frac, m, t, f in rows:
        print(f"  {frac*100:5.1f}%  {m:4d}/{t:<4d} unreached  {f}")
