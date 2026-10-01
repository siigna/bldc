/*
 * Fake config mempools. The real ones hand out from a shared pool; a single
 * static of each is equivalent for one caller at a time, and the allocation
 * count is recorded so a test can tell a leak from a balanced pair.
 *
 * Test scaffolding; never built into firmware.
 */

#ifndef FAKE_MEMPOOLS_H_
#define FAKE_MEMPOOLS_H_

#include "datatypes.h"

extern int fake_mempool_mcconf_out;
extern int fake_mempool_appconf_out;

mc_configuration *mempools_alloc_mcconf(void);
void mempools_free_mcconf(mc_configuration *conf);
app_configuration *mempools_alloc_appconf(void);
void mempools_free_appconf(app_configuration *conf);

#endif /* FAKE_MEMPOOLS_H_ */
