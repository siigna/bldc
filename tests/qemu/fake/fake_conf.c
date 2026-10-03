/*
	Copyright Benjamin Vedder
	Copyright 2026 Lukáš Hrázký
	Copyright 2026 Stephen Bouche

	Parts of this file were moved here from applications/app.c, commands.c, mempools.c;
	git blame -C records 5 lines as Benjamin Vedder's.

	A stand-in for the firmware's applications/app.c and commands.c, built only for the host
	tests. It reproduces the declarations the code under test needs, so the
	parts that come from upstream are upstream's -- a stub for a header cannot
	avoid being one.

	This file is part of the ESCargot firmware, a fork of the VESC firmware.

	SPDX-License-Identifier: GPL-3.0-or-later
 */

#include "app.h"
#include "mempools.h"
#include "commands.h"

fake_app_t fake_app;
int fake_mc_limits_applied;
int fake_app_limits_applied;
int fake_mempool_mcconf_out;
int fake_mempool_appconf_out;

const app_configuration *app_get_configuration(void) {
	return &fake_app.conf;
}

void app_set_configuration(app_configuration *conf) {
	fake_app.conf = *conf;
	fake_app.applied++;
}

/*
 * The real clamps rewrite values that exceed what the hardware allows. Doing
 * nothing here is deliberate: a test that wants to know a parameter round
 * trips should not have its value quietly changed, and whether the clamps are
 * right is their own business.
 */
void commands_apply_mcconf_hw_limits(mc_configuration *mcconf) {
	(void)mcconf;
	fake_mc_limits_applied++;
}

void commands_apply_appconf_hw_limits(app_configuration *appconf) {
	(void)appconf;
	fake_app_limits_applied++;
}

static mc_configuration m_mcconf;
static app_configuration m_appconf;

mc_configuration *mempools_alloc_mcconf(void) {
	fake_mempool_mcconf_out++;
	return &m_mcconf;
}

void mempools_free_mcconf(mc_configuration *conf) {
	(void)conf;
	fake_mempool_mcconf_out--;
}

app_configuration *mempools_alloc_appconf(void) {
	fake_mempool_appconf_out++;
	return &m_appconf;
}

void mempools_free_appconf(app_configuration *conf) {
	(void)conf;
	fake_mempool_appconf_out--;
}
