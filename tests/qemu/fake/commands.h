/*
 * Fake command helpers: just the two hw-limit clamps and the script print
 * channel. Test scaffolding; never built into firmware.
 */

#ifndef FAKE_COMMANDS_H_
#define FAKE_COMMANDS_H_

#include "datatypes.h"

extern int fake_mc_limits_applied;
extern int fake_app_limits_applied;

void commands_apply_mcconf_hw_limits(mc_configuration *mcconf);
void commands_apply_appconf_hw_limits(app_configuration *appconf);

int commands_printf_lisp(const char *format, ...);

#endif /* FAKE_COMMANDS_H_ */
