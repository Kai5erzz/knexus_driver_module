#include "knexus_mode_backend.h"

/*
 * 用户自定义模式模板。
 * 启用 KNEXUS_MODE_USER 后，可直接在下面六个函数内编写上层逻辑。
 * 函数由弱符号提供，因此也可以在自己的 .c 文件里用同名函数覆盖。
 */

__attribute__((weak)) void knexus_mode_user_init(void)
{
}

__attribute__((weak)) void knexus_mode_user_update(
    const struct knx26_context *context)
{
    (void)context;
}

__attribute__((weak)) void knexus_mode_user_on_intersection(
    const knx_intersection_result_t *result)
{
    (void)result;
}

__attribute__((weak)) void knexus_mode_user_on_peer_message(
    const knx_comm_message_t *message)
{
    (void)message;
}

__attribute__((weak)) void knexus_mode_user_debug_octo(
    Octolinker_Instance_t *octo, uint16_t base_id)
{
    (void)octo;
    (void)base_id;
}

__attribute__((weak)) void knexus_mode_user_debug_control_octo(
    Octolinker_Instance_t *octo, uint16_t base_id)
{
    (void)octo;
    (void)base_id;
}
