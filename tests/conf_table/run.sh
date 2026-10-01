#!/usr/bin/env sh
#
# Checks script/lua_vesc_conf_table.h against the LispBM extensions it is
# derived from.
#
# This exists because a round-trip test cannot catch a mis-mapping: set then
# get agrees whether "l_current_max" is wired to l_current_max or to the field
# beside it. Lisp's conf-get/conf-set chains are an independent statement of
# the same mapping, already shipped and already exercised, so the table is
# generated from them and this fails if the two drift.
#
# It also fails if the generator meets anything it does not recognise -- a new
# upstream parameter whose conf-set arm is not a plain assignment or a -fabsf
# is reported rather than quietly dropped or guessed at.
#
# Usage: nix-shell -p python3 --run ./run.sh

set -e
cd "$(dirname "$0")"

python3 ./gen_conf_table.py --check
