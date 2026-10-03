#!/usr/bin/env bash
# Copyright 2026 Stephen Bouche
# SPDX-License-Identifier: GPL-3.0-or-later
#
# Every proof under proofs/*/. Each directory has its own run.sh and skips
# rather than failing when the prover is absent, the same way the host tests
# treat a missing gtest.
#
#   ./proofs/run.sh
set -uo pipefail
cd "$(dirname "$0")"

fail=0

for d in */; do
    [ -x "${d}run.sh" ] || continue
    printf -- '--- %s ---\n' "${d%/}"
    "${d}run.sh" || fail=1
done

exit $fail
