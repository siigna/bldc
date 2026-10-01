#!/usr/bin/env sh
#
# Every test on this branch that does not need hardware.
#
# OVERLAP, READ BEFORE MERGING. The app-pas-fixes branch already has
# tests/check.sh, which opens with the same sentence and does strictly more
# than this: the same style of per-suite pass/fail, plus address and undefined
# sanitizers, cppcheck, clang-tidy, coverage, a property fuzzer and a
# configuration-signature check. This file was written without looking for it,
# which was a mistake -- that branch is on origin.
#
# Neither is a superset of the other, so the two have to be reconciled rather
# than one deleted:
#
#   only in check.sh      app_pas (run-all and run-san), overvoltage_fault,
#                         confsig, and every analysis stage
#   only in this file     script_pack, script_queue, script_alloc, lua_adc,
#                         utils_math, conf_table, qemu
#   in both               angles, packet_recovery, float_serialization
#
# When these branches meet, check.sh is the one to keep -- it is the older
# entry point and has the analysis stages -- and the suites listed above as
# only here should move into it. The three rules below are worth carrying
# across; check.sh does not clean before building, so it can test a stale
# binary.
#
# There are ten suites in three styles, and running them one at a time is how
# a breakage gets missed: twice while the script engine was being built, a
# change broke a suite that was not the one being worked on, and both times it
# surfaced only because something else happened to rebuild it.
#
#   nix-shell -p gcc gnumake python3 gcc-arm-embedded qemu gtest \
#       --run ./tests/run_all.sh
#
# Exit status is zero only if every suite passed. Three rules, each of which
# this tree has been bitten by:
#
#   * A suite that fails to BUILD is a failure, not a skip. A stale binary from
#     a previous run will otherwise be tested instead.
#   * The exit status is what counts, not the absence of the word FAIL. A
#     sanitiser abort and a CPU fault both exit without printing one.
#   * A skipped suite is named, with its reason, every run. A skip nobody sees
#     is indistinguishable from a pass.

set -u
cd "$(dirname "$0")"

# Plain C, built and run by their own Makefile.
MAKE_SUITES="angles float_serialization packet_recovery script_pack
             script_queue script_alloc lua_adc utils_math"

# Suites with a runner of their own.
SH_SUITES="conf_table qemu"

# Not run, and why. Printed every time so the omission stays visible. Empty is
# a result worth seeing too.
SKIPPED=""

passed=0
failed=0
failed_names=""

report() {
    if [ "$2" -eq 0 ]; then
        printf '  %-22s ok\n' "$1"
        passed=$((passed + 1))
    else
        printf '  %-22s FAILED (%s)\n' "$1" "$3"
        failed=$((failed + 1))
        failed_names="$failed_names $1"
    fi
}

printf '== host suites ==\n'
for s in $MAKE_SUITES; do
    [ -d "$s" ] || { report "$s" 1 "directory missing"; continue; }

    # Built from scratch: a failed build otherwise leaves the previous binary
    # for the run step to use.
    if ! out=$(make --no-print-directory -C "$s" clean 2>&1 \
                   && make --no-print-directory -C "$s" 2>&1); then
        report "$s" 1 "build error"
        printf '%s\n' "$out" | tail -12 | sed 's/^/      /'
        continue
    fi

    if out=$(make --no-print-directory -C "$s" run 2>&1); then
        report "$s" 0 ""
    else
        report "$s" 1 "see below"
        # The interesting lines are the failures, not whatever is last: make
        # prints its own trailing noise and a tail window shows that instead.
        printf '%s\n' "$out" \
            | grep -iE 'fail|error|abort|sanitiz|assert|checks,' \
            | head -15 | sed 's/^/      /'
    fi
done

printf '== suites with their own runner ==\n'
for s in $SH_SUITES; do
    [ -x "$s/run.sh" ] || { report "$s" 1 "run.sh missing or not executable"; continue; }

    if out=$("./$s/run.sh" 2>&1); then
        report "$s" 0 ""
    else
        report "$s" 1 "see below"
        printf '%s\n' "$out" \
            | grep -iE 'fail|error|abort|fault|panic|timed out|checks,' \
            | head -20 | sed 's/^/      /'
    fi
done

printf '== not run ==\n'
if [ -z "$SKIPPED" ]; then
    printf '  (nothing)\n'
else
    printf '%s\n' "$SKIPPED" | tr -s ' \n' ' ' | sed 's/^/  /;s/$/\n/'
fi

printf '\n%d suite(s) passed, %d failed\n' "$passed" "$failed"
if [ "$failed" -ne 0 ]; then
    printf 'failed:%s\n' "$failed_names"
    exit 1
fi
exit 0
