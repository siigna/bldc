/*
 * A stand-in for ChibiOS's ch.h, reached only because datatypes.h includes it.
 *
 * Nothing on the path to ATTITUDE_INFO uses a kernel type, so this needs to
 * exist rather than to do anything. app_pas's equivalent is larger because
 * that test drives the virtual timer; this one only needs the include to
 * resolve.
 *
 * Test scaffolding; never built into firmware.
 */

#ifndef CH_H
#define CH_H

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

/*
 * datatypes.h uses these two in structures this test never touches, but C
 * still has to know what they are to parse the file.
 */
typedef uint32_t systime_t;
typedef uint64_t sysinterval_t;

#endif /* CH_H */
