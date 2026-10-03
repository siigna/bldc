#!/usr/bin/env python3
"""Checks that the two script engines expose the same pedal-assist bindings.

    tests/pas_parity/check.py

LispBM and Lua are separate binding tables maintained by hand, and on an
STM32F405 a firmware can carry only one of them: LispBM alone leaves .ram4 at
99.6% of 62 KB, so a Lua build has no LispBM and a LispBM build has no Lua.
A binding that exists in one engine and not the other is therefore not a
cosmetic gap -- it means a package written for one engine cannot be ported to
the other at all, and whoever tries finds out by reading a nil.

This drifted exactly that way once: the Lua table was brought up to the Lisp
one and then grew four bindings past it -- torque ratio, assist basis power,
speed taper and torque saturated -- so Lisp was the one missing pieces, and
nothing said so.

Names differ by convention, and only by convention: Lua uses pas_get_rpm under
a `vesc` table, Lisp uses app-pas-get-rpm with the prefix spelled out. The
mapping below is that transformation and nothing else, so a genuinely new name
on either side fails rather than being quietly paired up.
"""

import os
import re
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
LISP = os.path.join(ROOT, "lispBM", "lispif_vesc_extensions.c")
LUA = os.path.join(ROOT, "script", "lua_vesc_pas.c")


def lisp_names():
    text = open(LISP, encoding="utf-8").read()
    return set(re.findall(r'lbm_add_extension\("(app-pas-[a-z0-9-]+)"', text))


def lua_names():
    text = open(LUA, encoding="utf-8").read()
    # The registration table: {"pas_get_rpm", l_pas_get_rpm}, ...
    return set(re.findall(r'\{\s*"(pas_[a-z0-9_]+)"', text))


def lua_to_lisp(name):
    """pas_get_rpm -> app-pas-get-rpm. The whole of the naming convention."""
    return "app-" + name.replace("_", "-")


def main():
    lisp = lisp_names()
    lua = lua_names()

    if not lisp:
        sys.exit("no app-pas-* extensions found in %s" % LISP)

    if not lua:
        sys.exit("no pas_* bindings found in %s" % LUA)

    expected = {lua_to_lisp(n): n for n in lua}

    missing_in_lisp = sorted(k for k in expected if k not in lisp)
    missing_in_lua = sorted(lisp - set(expected))

    for name in missing_in_lisp:
        print("  Lua has %s, LispBM has no %s" % (expected[name], name))

    for name in missing_in_lua:
        print("  LispBM has %s, Lua has no %s"
              % (name, name[len("app-"):].replace("-", "_")))

    n = len(missing_in_lisp) + len(missing_in_lua)
    print("\n%d PAS binding(s) in one engine and not the other "
          "(%d in Lisp, %d in Lua)" % (n, len(lisp), len(lua)))
    return 1 if n else 0


if __name__ == "__main__":
    sys.exit(main())
