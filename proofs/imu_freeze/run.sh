#!/usr/bin/env bash
# Copyright 2026 Stephen Bouche
# SPDX-License-Identifier: GPL-3.0-or-later
#
# Proves that moving the dead-bus detector out of imu_thread.c did not change
# what it does, and four properties of it besides. About two seconds.
set -uo pipefail
cd "$(dirname "$0")"

CBMC=${CBMC:-cbmc}

if ! command -v "$CBMC" >/dev/null 2>&1; then
    echo "  skipped: no cbmc" >&2
    exit 0
fi

out=$("$CBMC" equiv.c ../../util/imu_freeze.c -I../../util \
      --bounds-check --pointer-check --conversion-check 2>&1)
st=$?

printf '%s\n' "$out" | grep -E "^\[main\.assertion" | sed 's/^/  /'
printf '%s\n' "$out" | grep -E "^\*\*|VERIFICATION" | sed 's/^/  /'

if ! printf '%s\n' "$out" | grep -q "VERIFICATION SUCCESSFUL"; then
    printf '%s\n' "$out" | tail -30 >&2
    exit 1
fi

exit $st
