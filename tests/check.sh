#!/usr/bin/env bash
# Everything that can be checked without a board.
#
#   ./tests/check.sh            tests, sanitizers, static analysis, coverage
#
# Covers both lines of work: the PAS suites and the script engine's, including
# the QEMU images. tests/run_all.sh, which the script-engine branch grew
# separately, is gone -- this is the one entry point.
#   ./tests/check.sh --fuzz     and a short fuzzing run
#   ./tests/check.sh --tree     and cppcheck over the whole firmware
#   FUZZ_SECS=600 ./tests/check.sh --fuzz    a longer one
#
# Each stage prints its own summary and the script exits non-zero if any of
# them failed, so this is usable both by hand and as the one thing CI runs.
set -uo pipefail
cd "$(dirname "$0")/.."

FUZZ=0
TREE=0
for a in "$@"; do
    [ "$a" = "--fuzz" ] && FUZZ=1
    [ "$a" = "--tree" ] && TREE=1
done

fail=0
stage() { printf '\n=== %s ===\n' "$1"; }
report() {
    if [ "$1" = 0 ]; then echo "  ok"; else echo "  FAILED"; fail=1; fi
}

stage "host tests, every board variant"
make -C tests/app_pas run-all 2>&1 | grep -E "^---|checks,"
report "${PIPESTATUS[0]}"

# Every plain-C suite with its own Makefile. The script-engine ones joined when
# that branch merged; adding a directory here is all a new suite needs.
#
# Built from clean first, deliberately. Without it a failed build leaves the
# previous binary for the run step to test, which this tree has produced in
# four different forms -- including a QEMU image that ran while its own build
# was failing.
# Discovered, not listed. The list used to be written out here, and when
# upstream added tests/uavcan_vesc_frames the test came with the merge and was
# never run -- a hardcoded list silently declines anything new, which is the
# opposite of what you want from a fork that merges someone else's tests.
#
# Excluded are the suites that bring their own runner or are driven elsewhere
# in this script: app_pas has run-all and run-san stages of its own, qemu and
# conf_table have run.sh, confsig and pas_parity are python invoked directly.
for t in $(ls -d tests/*/ 2>/dev/null | sed 's|tests/||; s|/||' \
           | grep -vxE 'app_pas|qemu|conf_table|confsig|pas_parity'); do
    if [ -f "tests/$t/Makefile" ]; then
        stage "host tests: $t"
        if ! make --no-print-directory -C "tests/$t" clean >/dev/null 2>&1 \
             || ! out=$(make --no-print-directory -C "tests/$t" 2>&1); then
            # A missing dependency is a skip; a real compile error is a
            # failure. utils_math needs gtest, which the dev shell does not
            # carry, and reporting that as FAILED hides anything else.
            if printf '%s' "$out" | grep -q "gtest/gtest.h: No such file"; then
                printf '  skipped: no gtest\n'
            elif printf '%s' "$out" | grep -q "gnu/stubs-32.h: No such file"; then
                # uavcan_vesc_frames builds -m32 because libcanard needs a
                # 32-bit host. The dev shell carries gcc_multi for it; without
                # multilib headers this is an absent dependency, not a defect.
                printf '  skipped: no 32-bit headers (needs gcc_multi)\n'
                continue
            fi

            printf '%s\n' "$out" | tail -8
            report 1
            continue
        fi
        make --no-print-directory -C "tests/$t" run 2>&1 | tail -3
        report "${PIPESTATUS[0]}"
    fi
done

# Suites that bring their own runner.
stage "parameter table against the LispBM extensions"
./tests/conf_table/run.sh
report $?

# The two engines are separate hand-maintained binding tables, and on an F405
# a firmware can carry only one of them -- LispBM alone leaves .ram4 at 99.6%
# of 62 KB. So a binding present in one and not the other means a package
# written for one engine cannot be ported to the other, which is how the Lua
# table came to be four bindings ahead of the Lisp one with nothing saying so.
# Machine-checked claims, where a commit message asserted something a test can
# only sample -- see proofs/README.md, including what a passing run does not
# mean. Skipped rather than failed without CBMC, like the other tool stages.
stage "proofs (bounded model checking)"
if ! command -v cbmc >/dev/null 2>&1; then
    printf '  skipped: no cbmc\n'
else
    ./proofs/run.sh 2>&1 | grep -E "^---|VERIFICATION|failed"
    report "${PIPESTATUS[0]}"
fi

stage "pedal-assist bindings, Lisp against Lua"
python3 ./tests/pas_parity/check.py
report $?

stage "script engine on a simulated STM32F405 (QEMU)"
# Both halves are needed, and gating on only the emulator was a trap: with
# qemu present and no cross compiler every image fails to build, and the
# stage reports FAILED over a wall of "arm-none-eabi-gcc: No such file or
# directory" rather than saying what is missing. Both come from the dev
# shell; outside it, nix develop.
if ! command -v qemu-system-arm >/dev/null 2>&1; then
    echo "  skipped: no qemu-system-arm on PATH"
elif ! command -v arm-none-eabi-gcc >/dev/null 2>&1; then
    echo "  skipped: no arm-none-eabi-gcc, so the images cannot be built"
else
    ./tests/qemu/run.sh 2>&1 | grep -E ": ok|FAILED|checks,"
    report "${PIPESTATUS[0]}"
fi

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
if ! command -v cppcheck >/dev/null 2>&1; then
    # Skipped, not failed. A tool that is merely absent must not report the
    # same way as a tool that found something, or a real finding is lost in
    # the noise of an environment that never had it -- which is how two
    # stages were red in the dev shell for weeks.
    printf '  skipped: no cppcheck\n'
else
# --check-level=exhaustive, and not only for the extra thoroughness. At the
# normal level cppcheck 2.18 emits an *informational* message about limiting
# branch analysis, and --error-exitcode turns that advice into a red stage --
# a tool telling you it did less work should not read the same as a tool
# finding a bug. Exhaustive says nothing and takes 1.2 seconds here.
#
# These tools are version-sensitive, which is why the dev shell pins them.
# Ubuntu's cppcheck 2.13 reports a constParameterPointer in
# tests/app_pas/fixture.c that neither 2.18 nor 2.21 does, and 2.21 does not
# emit the branch-limit message at all.
cppcheck --enable=warning,style,performance,portability --inline-suppr \
    --suppress=missingIncludeSystem --error-exitcode=1 --std=c99 \
    --check-level=exhaustive \
    -DNO_STM32 '-DHW_SOURCE="hw.h"' '-DHW_HEADER="hw.h"' \
    -Itests/app_pas -I. -Iutil -Iapplications -Imotor -Icomm \
    --quiet applications/app_pas.c tests/app_pas/*.c
report $?
fi

stage "clang-tidy"
# Checks are in .clang-tidy, curated there with the reasoning. Scoped to the
# files this branch is responsible for: a tree-wide run needs a
# compile_commands.json to get each board's defines right, and without one the
# firmware files that depend on a hwconf do not parse.
if command -v clang-tidy >/dev/null; then
    # Failing on printed diagnostics rather than on the exit code. See the note
    # in .clang-tidy: the exit code also counts warnings from system headers,
    # which are not displayed and not ours.
    out=$(clang-tidy applications/app_pas.c tests/app_pas/*.c -- \
        -DNO_STM32 '-DHW_SOURCE="hw.h"' '-DHW_HEADER="hw.h"' \
        -Itests/app_pas -I. -Iutil -Iapplications -Imotor -Icomm -std=gnu99 2>&1)

    # A clang-tidy that cannot find the standard headers reports every one as a
    # clang-diagnostic-error, which is an environment problem and not a finding.
    # Saying so beats failing the branch over a toolchain that is missing its
    # own include paths.
    if echo "$out" | grep -q "file not found .clang-diagnostic-error"; then
        echo "  skipped: clang-tidy cannot find the standard headers here"
    else
        echo "$out" | grep -E "warning:|error:" && report 1 || report 0
    fi
else
    echo "  skipped: no clang-tidy"
fi

stage "coverage of the code under test"
make -C tests/app_pas coverage 2>&1 | grep -E "Lines exec|Branches exec|unreached"
report "${PIPESTATUS[0]}"

if [ "$FUZZ" = 1 ]; then
    stage "fuzzing (${FUZZ_SECS:-30}s)"
    make -C tests/app_pas fuzz-run FUZZ_SECS="${FUZZ_SECS:-30}" 2>&1 \
        | grep -E "INVARIANT|Test unit written|stat::number|stat::average|DONE"
    report "${PIPESTATUS[0]}"
fi

if [ "$TREE" = 1 ]; then
    # Informational, and deliberately not gating. As of this branch the whole
    # firmware produces four findings at this level, all of them false
    # positives from volatiles an interrupt writes that cppcheck cannot see,
    # plus an LZO header that needs defines to preprocess. With style enabled
    # it is 141, of which 77 are const-correctness and 28 variable scope.
    #
    # So the value here is as a regression net rather than as a bug hunt: a new
    # finding in a file this branch touches is worth reading, and the standing
    # ones are not this branch's to fix.
    stage "cppcheck over the whole firmware (informational)"
    cppcheck --enable=warning,performance,portability --inline-suppr \
        --suppress=missingIncludeSystem --suppress=missingInclude --std=c99 \
        -DNO_STM32 '-DHW_SOURCE="hw.h"' '-DHW_HEADER="hw.h"' \
        -I. -Iutil -Iapplications -Imotor -Icomm -j4 --quiet \
        applications util comm motor driver 2>&1 \
        | grep -E ": (error|warning|performance|portability):" || true
    echo "  (not gating)"
fi

printf '\n'
if [ "$fail" = 0 ]; then
    echo "all checks passed"
else
    echo "one or more checks failed"
fi
exit $fail
