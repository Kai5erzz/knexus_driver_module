#include "knexus_mode_common.h"
#include "knx_params.h"
#include "drv8701e.h"
#include "encoder.h"

extern float knx26_test_left_cmd_polarity;
extern float knx26_test_right_cmd_polarity;
extern float knx26_test_left_feedback_polarity;
extern float knx26_test_right_feedback_polarity;

static int8_t polarity_sign(float value)
{
    return (value < 0.0f) ? -1 : 1;
}

void knexus_mode_apply_motor_config(float max_duty)
{
    motor_left.dir_sign = polarity_sign(knx26_test_left_cmd_polarity);
    motor_right.dir_sign = polarity_sign(knx26_test_right_cmd_polarity);
    encoder_left.dir_sign = polarity_sign(knx26_test_left_feedback_polarity);
    encoder_right.dir_sign = polarity_sign(knx26_test_right_feedback_polarity);

    /* 极性已经由硬件抽象层处理，drive 层必须保持正号。 */
    g_knx_params.drive.left_cmd_sign = 1.0f;
    g_knx_params.drive.right_cmd_sign = 1.0f;
    g_knx_params.drive.left_feedback_sign = 1.0f;
    g_knx_params.drive.right_feedback_sign = 1.0f;
    g_knx_params.drive.angular_feedback_sign = 1.0f;

    if (max_duty < 0.0f) max_duty = 0.0f;
    if (max_duty > 1.0f) max_duty = 1.0f;
    uint32_t period = 0U;
    if (motor_left.port != NULL &&
        knx_pwm_get_period((knx_pwm_channel_t *)&motor_left.port->pwm,
                           &period) == KNX_OK && period > 0U) {
        vc_left.out_min = -(float)period * max_duty;
        vc_left.out_max = (float)period * max_duty;
    }
    period = 0U;
    if (motor_right.port != NULL &&
        knx_pwm_get_period((knx_pwm_channel_t *)&motor_right.port->pwm,
                           &period) == KNX_OK && period > 0U) {
        vc_right.out_min = -(float)period * max_duty;
        vc_right.out_max = (float)period * max_duty;
    }
}
