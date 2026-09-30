#!/usr/bin/env bash
# Everything that can be checked without a board.
#
#   ./tests/check.sh            tests, sanitizers, static analysis, coverage
#   ./tests/check.sh --fuzz     and a short fuzzing run
#   FUZZ_SECS=600 ./tests/check.sh --fuzz    a longer one
#
# Each stage prints its own summary and the script exits non-zero if any of
# them failed, so this is usable both by hand and as the one thing CI runs.
set -uo pipefail
cd "$(dirname "$0")/.."

FUZZ=0
[ "${1:-}" = "--fuzz" ] && FUZZ=1

fail=0
stage() { printf '\n=== %s ===\n' "$1"; }
report() {
    if [ "$1" = 0 ]; then echo "  ok"; else echo "  FAILED"; fail=1; fi
}

stage "host tests, every board variant"
make -C tests/app_pas run-all 2>&1 | grep -E "^---|checks,"
report "${PIPESTATUS[0]}"

for t in angles packet_recovery float_serialization overvoltage_fault; do
    if [ -f "tests/$t/Makefile" ]; then
        stage "host tests: $t"
        make -C "tests/$t" run 2>&1 | tail -3
        report $?
    fi
done

stage "sanitizers (address, undefined)"
make -C tests/app_pas run-san 2>&1 | grep -E "^---|checks,|runtime error|ERROR:"
report "${PIPESTATUS[0]}"

stage "configuration signature and field order"
if [ -d ../vesc_tool/res/config/7.02 ]; then
    python3 tests/confsig/checkconf.py . ../vesc_tool/res/config/7.02
    report $?
else
    echo "  skipped: no ../vesc_tool/res/config/7.02 to compare against"
fi

stage "cppcheck"
cppcheck --enable=warning,style,performance,portability --inline-suppr \
    --suppress=missingIncludeSystem --error-exitcode=1 --std=c99 \
    -DNO_STM32 '-DHW_SOURCE="hw.h"' '-DHW_HEADER="hw.h"' \
    -I. -Iutil -Iapplications -Imotor -Icomm -Itests/app_pas \
    --quiet applications/app_pas.c tests/app_pas/*.c
report $?

stage "coverage of the code under test"
make -C tests/app_pas coverage 2>&1 | grep -E "Lines exec|Branches exec|unreached"
report "${PIPESTATUS[0]}"

if [ "$FUZZ" = 1 ]; then
    stage "fuzzing (${FUZZ_SECS:-30}s)"
    make -C tests/app_pas fuzz-run FUZZ_SECS="${FUZZ_SECS:-30}" 2>&1 \
        | grep -E "INVARIANT|Test unit written|stat::number|stat::average|DONE"
    report "${PIPESTATUS[0]}"
fi

printf '\n'
if [ "$fail" = 0 ]; then
    echo "all checks passed"
else
    echo "one or more checks failed"
fi
exit $fail
