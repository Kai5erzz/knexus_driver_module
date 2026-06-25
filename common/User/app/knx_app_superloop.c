/* Superloop entry point: alternative to calling knx_app_init/loop from a task.
 *
 * The active FreeRTOS app task calls knx_app_init() once and knx_app_loop()
 * periodically; this helper remains available for simple superloop builds.
 */

#include "knx_app.h"

void knx_app_main(void)
{
    knx_app_init();

    while (1) {
        knx_app_loop();
    }
}
