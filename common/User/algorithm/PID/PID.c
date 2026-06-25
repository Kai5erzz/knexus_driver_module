/**
 * @file controller.c
 * @author wanghongxi
 * @author modified by neozng
 * @brief  PID鎺у埗鍣ㄥ畾涔?
 * @version beta
 * @date 2022-11-01
 *
 * @copyrightCopyright (c) 2022 HNU YueLu EC all rights reserved
 */

#include <string.h>
#include <stdlib.h>
#include "PID.h"
#include "knx_time.h"

static inline uint32_t pid_time_us(void)
{
    return knx_micros();
}

static inline float pid_get_delta(uint32_t *stamp_us)
{
    uint32_t now = pid_time_us();
    if (now == 0U) {
        return 0.001f;
    }

    uint32_t delta = now - *stamp_us;
    *stamp_us = now;
    if (delta == 0U) {
        delta = 1000U;
    }

    return (float)delta * 0.000001f;
}

static uint8_t idx = 0; // register idx,鏄鏂囦欢鐨勫叏灞€PID绱㈠紩,鍦ㄦ敞鍐屾椂浣跨敤
/* PID鎺у埗鍣ㄧ殑瀹炰緥,姝ゅ浠呬繚瀛樻寚閽?鍐呭瓨鐨勫垎閰嶅皢閫氳繃瀹炰緥鍒濆鍖栨椂閫氳繃malloc()杩涜 */
static pid_obj_t *pid_obj[PID_NUM_MAX] = {NULL};

/* ----------------------------浠ヤ笅鏄痯id浼樺寲鐜妭鐨勫疄鐜?--------------------------- */

// 姊舰绉垎
static void f_Trapezoid_Intergral(pid_obj_t *pid)
{
    // 璁＄畻姊舰鐨勯潰绉?(涓婂簳+涓嬪簳)*楂?2
    pid->ITerm = pid->Ki * ((pid->Err + pid->Last_Err) / 2) * pid->dt;
}

// 鍙橀€熺Н鍒?璇樊灏忔椂绉垎浣滅敤鏇村己)
static void f_Changing_Integration_Rate(pid_obj_t *pid)
{
    if (pid->Err * pid->Iout > 0)
    {
        // 绉垎鍛堢疮绉秼鍔?
        if (usr_abs(pid->Err) <= pid->CoefB)
            return; // Full integral
        if (usr_abs(pid->Err) <= (pid->CoefA + pid->CoefB))
            pid->ITerm *= (pid->CoefA - usr_abs(pid->Err) + pid->CoefB) / pid->CoefA;
        else // 鏈€澶ч槇鍊?涓嶄娇鐢ㄧН鍒?
            pid->ITerm = 0;
    }
}

static void f_Integral_Limit(pid_obj_t *pid)
{
    static float temp_Output, temp_Iout;
    temp_Iout = pid->Iout + pid->ITerm;
    temp_Output = pid->Pout + pid->Iout + pid->Dout;
    if (usr_abs(temp_Output) > pid->MaxOut)
    {
        if (pid->Err * pid->Iout > 0) // 绉垎鍗磋繕鍦ㄧ疮绉?
        {
            pid->ITerm = 0; // 褰撳墠绉垎椤圭疆闆?
        }
    }

    if (temp_Iout > pid->IntegralLimit)
    {
        pid->ITerm = 0;
        pid->Iout = pid->IntegralLimit;
    }
    if (temp_Iout < -pid->IntegralLimit)
    {
        pid->ITerm = 0;
        pid->Iout = -pid->IntegralLimit;
    }
}

// 寰垎鍏堣(浠呬娇鐢ㄥ弽棣堝€艰€屼笉璁″弬鑰冭緭鍏ョ殑寰垎)
static void f_Derivative_On_Measurement(pid_obj_t *pid)
{
    pid->Dout = pid->Kd * (pid->Last_Measure - pid->Measure) / pid->dt;
}

// 寰垎婊ゆ尝(閲囬泦寰垎鏃?婊ら櫎楂橀鍣０)
static void f_Derivative_Filter(pid_obj_t *pid)
{
    pid->Dout = pid->Dout * pid->dt / (pid->Derivative_LPF_RC + pid->dt) +
                pid->Last_Dout * pid->Derivative_LPF_RC / (pid->Derivative_LPF_RC + pid->dt);
}

// 杈撳嚭婊ゆ尝
static void f_Output_Filter(pid_obj_t *pid)
{
    pid->Output = pid->Output * pid->dt / (pid->Output_LPF_RC + pid->dt) +
                  pid->Last_Output * pid->Output_LPF_RC / (pid->Output_LPF_RC + pid->dt);
}

// 杈撳嚭闄愬箙
static void f_Output_Limit(pid_obj_t *pid)
{
    if (pid->Output > pid->MaxOut)
    {
        pid->Output = pid->MaxOut;
    }
    if (pid->Output < -(pid->MaxOut))
    {
        pid->Output = -(pid->MaxOut);
    }
}

// 鐢垫満鍫佃浆妫€娴?
static void f_PID_ErrorHandle(pid_obj_t *pid)
{
    /*Motor Blocked Handle*/
    if (fabsf(pid->Output) < pid->MaxOut * 0.001f || fabsf(pid->Ref) < 0.0001f)
        return;

    if ((fabsf(pid->Ref - pid->Measure) / fabsf(pid->Ref)) > 0.95f)
    {
        // Motor blocked counting
        pid->ERRORHandler.error_count++;
    }
    else
    {
        pid->ERRORHandler.error_count = 0;
    }

    if (pid->ERRORHandler.error_count > 500)
    {
        // Motor blocked over 1000times
        pid->ERRORHandler.error_type = PID_MOTOR_BLOCKED_ERROR;
    }
}

/* ---------------------------涓嬮潰鏄疨ID鐨勫閮ㄧ畻娉曟帴鍙?-------------------------- */

/**
 * @brief 鍒濆鍖朠ID瀹炰緥,骞惰繑鍥濸ID瀹炰緥鎸囬拡
 * @param config PID鍒濆鍖栬缃?
 */
pid_obj_t *pid_register(pid_config_t *config)
{
    pid_obj_t *object = (pid_obj_t *)malloc(sizeof(pid_obj_t));
    memset(object, 0, sizeof(pid_obj_t));

    // basic parameter
    object->Kp = config->Kp;
    object->Ki = config->Ki;
    object->Kd = config->Kd;
    object->MaxOut = config->MaxOut;
    object->DeadBand = config->DeadBand;

    // improve parameter
    object->Improve = config->Improve;
    object->IntegralLimit = config->IntegralLimit;
    object->CoefA = config->CoefA;
    object->CoefB = config->CoefB;
    object->Output_LPF_RC = config->Output_LPF_RC;
    object->Derivative_LPF_RC = config->Derivative_LPF_RC;

    object->time_stamp_us = pid_time_us();

    pid_obj[idx++] = object;
    return object;
}

/**
 * @brief          PID璁＄畻
 * @param[in]      PID缁撴瀯浣?
 * @param[in]      娴嬮噺鍊?
 * @param[in]      鏈熸湜鍊?
 * @retval         杩斿洖绌?
 */
float pid_calculate(pid_obj_t *pid, float measure, float ref)
{
    // 鍫佃浆妫€娴?
    if (pid->Improve & PID_ErrorHandle)
        f_PID_ErrorHandle(pid);

    pid->dt = pid_get_delta(&pid->time_stamp_us);

    // 淇濆瓨涓婃鐨勬祴閲忓€煎拰璇樊,璁＄畻褰撳墠error
    pid->Measure = measure;
    pid->Ref = ref;
    pid->Err = pid->Ref - pid->Measure;

    // 濡傛灉鍦ㄦ鍖哄,鍒欒绠桺ID
    if (usr_abs(pid->Err) > pid->DeadBand)
    {
        // 鍩烘湰鐨刾id璁＄畻,浣跨敤浣嶇疆寮?
        pid->Pout = pid->Kp * pid->Err;
        pid->ITerm = pid->Ki * pid->Err * pid->dt;
        pid->Dout = pid->Kd * (pid->Err - pid->Last_Err) / pid->dt;

        // 姊舰绉垎
        if (pid->Improve & PID_Trapezoid_Intergral)
            f_Trapezoid_Intergral(pid);
        // 鍙橀€熺Н鍒?
        if (pid->Improve & PID_ChangingIntegrationRate)
            f_Changing_Integration_Rate(pid);
        // 寰垎鍏堣
        if (pid->Improve & PID_Derivative_On_Measurement)
            f_Derivative_On_Measurement(pid);
        // 寰垎婊ゆ尝鍣?
        if (pid->Improve & PID_DerivativeFilter)
            f_Derivative_Filter(pid);
        // 绉垎闄愬箙
        if (pid->Improve & PID_Integral_Limit)
            f_Integral_Limit(pid);

        pid->Iout += pid->ITerm;                         // 绱姞绉垎
        pid->Output = pid->Pout + pid->Iout + pid->Dout; // 璁＄畻杈撳嚭

        // 杈撳嚭婊ゆ尝
        if (pid->Improve & PID_OutputFilter)
            f_Output_Filter(pid);

        // 杈撳嚭闄愬箙
        f_Output_Limit(pid);
    }
    else // 杩涘叆姝诲尯, 鍒欐竻绌虹Н鍒嗗拰杈撳嚭
    {
        pid->Output = 0;
        pid->ITerm = 0;
    }

    // 淇濆瓨褰撳墠鏁版嵁,鐢ㄤ簬涓嬫璁＄畻
    pid->Last_Measure = pid->Measure;
    pid->Last_Output = pid->Output;
    pid->Last_Dout = pid->Dout;
    pid->Last_Err = pid->Err;
    pid->Last_ITerm = pid->ITerm;

    return pid->Output;
}

/**
 * @brief 娓呯┖涓€涓猵id鐨勫巻鍙叉暟鎹?
 *
 * @param pid    PID瀹炰緥
 */
void pid_clear(pid_obj_t *pid)
{
    pid->Measure = 0;
    pid->Last_Measure = 0;
    pid->Err=0;
    pid->Last_Err=0;
    pid->Last_ITerm=0;
    pid->Pout=0;
    pid->Iout=0;
    pid->Dout=0;
    pid->ITerm=0;
    pid->Output=0;
    pid->Last_Output=0;
    pid->Last_Dout=0;
    pid->ERRORHandler.error_count=0;
    pid->ERRORHandler.error_type=0;
}
