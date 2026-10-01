#!/usr/bin/env sh
#
# Runs the QEMU test images and reports a single status.
#
# Two things this is careful about, both learned the hard way:
#
#   * A hang must not read as a pass. A thread stack overflow smashes the
#     thread_t at the base of its working area and wedges the kernel with no
#     output whatsoever -- no panic, no fault, nothing. `timeout` turns that
#     into exit 124, and this script treats any nonzero exit as a failure.
#
#   * Absence of "FAIL" lines is not success. A fault or panic exits before
#     printing any, so the "RESULT pass" line must be present *as well as* a
#     zero exit status.
#
# Usage: nix-shell -p gcc-arm-embedded gnumake qemu --run ./run.sh

cd "$(dirname "$0")"

TESTS="test_kernel test_engine"
# The engine must be built with the limit the firmware uses, or the deep-
# recursion checks would be testing a configuration nothing ships.
ENGINE_FLAGS="-DLUAI_MAXCCALLS=16"
QEMU="${QEMU:-qemu-system-arm}"
rc=0

for t in $TESTS; do
    printf '=== %s ===\n' "$t"
    # Rebuild from scratch each time. A failed build leaves the previous .elf
    # in place, and QEMU will cheerfully run it -- a stale pass is worse than
    # no result at all.
    rm -f "$t.elf"
    xflags=""
    [ "$t" = "test_engine" ] && xflags="$ENGINE_FLAGS"
    if ! make --no-print-directory TEST="$t" CFLAGS_EXTRA="$xflags" >/dev/null 2>&1; then
        printf '%s: FAILED (build error)\n' "$t"
        make --no-print-directory TEST="$t" CFLAGS_EXTRA="$xflags" 2>&1 | tail -20
        rc=1
        continue
    fi

    # qemu is invoked directly, not through `make run`: make collapses every
    # child status to 2, which would hide the difference between a timeout, a
    # fault and a failed check.
    out=$(timeout 60 "$QEMU" -M olimex-stm32-h405 -cpu cortex-m4 -nographic \
            -semihosting --semihosting-config enable=on,target=native \
            -serial mon:stdio -kernel "$t.elf" 2>&1) && status=0 || status=$?
    printf '%s\n' "$out"

    if [ "$status" -eq 124 ]; then
        printf '%s: FAILED (timed out -- a wedged kernel prints nothing)\n' "$t"
        rc=1
    elif [ "$status" -eq 4 ]; then
        printf '%s: FAILED (CPU fault or kernel panic -- see above)\n' "$t"
        rc=1
    elif [ "$status" -ne 0 ]; then
        printf '%s: FAILED (exit %s)\n' "$t" "$status"
        rc=1
    elif ! printf '%s' "$out" | grep -q 'RESULT pass'; then
        printf '%s: FAILED (no RESULT pass line; exited 0 without reporting)\n' "$t"
        rc=1
    else
        printf '%s: ok\n' "$t"
    fi
done

exit $rc
