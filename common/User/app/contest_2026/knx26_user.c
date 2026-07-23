#include "knx26_user.h"

/* These weak callbacks are the only file-level extension points required by
 * the upper application. A contest solution may override any callback in a
 * separate source file without editing the runtime or drivers. */
__attribute__((weak)) void knx26_user_init(void) {}
__attribute__((weak)) void knx26_user_update(const struct knx26_context *context)
{
    (void)context;
}
__attribute__((weak)) void knx26_user_on_intersection(const knx_intersection_result_t *result)
{
    (void)result;
}
__attribute__((weak)) void knx26_user_on_peer_message(const knx_comm_message_t *message)
{
    (void)message;
}
