#!/usr/bin/env sh
#
# Every test in this tree that does not need hardware.
#
# There are nine suites in three styles, and running them one at a time is how
# a breakage gets missed: twice while the script engine was being built, a
# change broke a suite that was not the one being worked on, and both times it
# surfaced only because something else happened to rebuild it.
#
#   nix-shell -p gcc gnumake python3 gcc-arm-embedded qemu --run ./tests/run_all.sh
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
             script_queue script_alloc lua_adc"

# Suites with a runner of their own.
SH_SUITES="conf_table qemu"

# Not run, and why. Printed every time so the omission stays visible.
SKIPPED="utils_math:wants Google Test sources to build gtest-all.cc and points
         at utils_math.c in its old location; bit-rotted, not yet fixed"

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
printf '%s\n' "$SKIPPED" | tr -s ' \n' ' ' | sed 's/^/  /;s/$/\n/'

printf '\n%d suite(s) passed, %d failed\n' "$passed" "$failed"
if [ "$failed" -ne 0 ]; then
    printf 'failed:%s\n' "$failed_names"
    exit 1
fi
exit 0
