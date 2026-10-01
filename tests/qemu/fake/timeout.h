/*
 * A fake timeout, counting refreshes so a test can prove every motor setter
 * refreshes it. Test scaffolding; never built into firmware.
 */

#ifndef FAKE_TIMEOUT_H_
#define FAKE_TIMEOUT_H_

extern int fake_timeout_resets;

void timeout_reset(void);

#endif /* FAKE_TIMEOUT_H_ */
