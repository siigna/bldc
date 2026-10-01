/*
 * Fake app configuration accessors. Test scaffolding; never built into
 * firmware. app_configuration itself comes from the firmware's datatypes.h.
 */

#ifndef FAKE_APP_H_
#define FAKE_APP_H_

#include "datatypes.h"

typedef struct {
	app_configuration conf;
	int applied;
} fake_app_t;

extern fake_app_t fake_app;

const app_configuration *app_get_configuration(void);
void app_set_configuration(app_configuration *conf);

#endif /* FAKE_APP_H_ */
