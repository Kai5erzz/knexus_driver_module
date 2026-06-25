/**
 ******************************************************************************
 * @file     user_lib.h
 * @author  Wang Hongxi
 * @version V1.0.0
 * @date    2021/2/18
 * @brief
 ******************************************************************************
 * @attention
 *
 ******************************************************************************
 */
#ifndef _USER_LIB_H
#define _USER_LIB_H

#include "stdint.h"
#include "stdlib.h"
#include "arm_math.h"


#ifndef user_malloc
#ifdef _CMSIS_OS_H
#define user_malloc pvPortMalloc
#else
#define user_malloc malloc
#endif
#endif

#define msin(x) (arm_sin_f32(x))
#define mcos(x) (arm_cos_f32(x))

typedef arm_matrix_instance_f32 mat;
// 锟斤拷锟斤拷锟斤拷锟劫度诧拷锟斤拷,锟斤拷锟斤拷使锟斤拷q31锟斤拷锟斤拷f32,锟斤拷锟角撅拷锟饺会降锟斤拷
#define MatAdd arm_mat_add_f32
#define MatSubtract arm_mat_sub_f32
#define MatMultiply arm_mat_mult_f32
#define MatTranspose arm_mat_trans_f32
#define MatInverse arm_mat_inverse_f32
void MatInit(mat *m, uint8_t row, uint8_t col);

/* boolean type definitions */
#ifndef TRUE
#define TRUE 1 /**< boolean true  */
#endif

#ifndef FALSE
#define FALSE 0 /**< boolean fails */
#endif

/* circumference ratio */
#ifndef PI
#define PI 3.14159265354f
#endif
/* 锟角讹拷转锟斤拷锟斤拷系锟斤拷 */
#define RADIAN_COEF          57.296f

#define VAL_LIMIT(val, min, max) \
    do                           \
    {                            \
        if ((val) <= (min))      \
        {                        \
            (val) = (min);       \
        }                        \
        else if ((val) >= (max)) \
        {                        \
            (val) = (max);       \
        }                        \
    } while (0)

#define ANGLE_LIMIT_360(val, angle)     \
    do                                  \
    {                                   \
        (val) = (angle) - (int)(angle); \
        (val) += (int)(angle) % 360;    \
    } while (0)

#define ANGLE_LIMIT_360_TO_180(val) \
    do                              \
    {                               \
        if ((val) > 180)            \
            (val) -= 360;           \
    } while (0)

#define VAL_MIN(a, b) ((a) < (b) ? (a) : (b))
#define VAL_MAX(a, b) ((a) > (b) ? (a) : (b))

/**
 * @brief 锟斤拷锟斤拷一锟斤拷删锟斤拷锟斤拷诖锟?锟斤拷锟斤拷锟斤拷然锟斤拷要强锟斤拷转锟斤拷为锟斤拷锟斤拷要锟斤拷锟斤拷锟斤拷
 *
 * @param size 锟斤拷锟斤拷锟叫?
 * @return void*
 */
void *zmalloc(size_t size);

// 锟斤拷锟劫匡拷锟斤拷
float Sqrt(float x);
// 锟斤拷锟斤拷值锟斤拷锟斤拷
float abs_limit(float num, float Limit);
// 锟叫断凤拷锟斤拷位
float sign(float value);
// 锟斤拷锟斤拷锟斤拷锟斤拷
float float_deadband(float Value, float minValue, float maxValue);
// 锟睫凤拷锟斤拷锟斤拷
float float_constrain(float Value, float minValue, float maxValue);
// 锟睫凤拷锟斤拷锟斤拷
int16_t int16_constrain(int16_t Value, int16_t minValue, int16_t maxValue);
// 循锟斤拷锟睫凤拷锟斤拷锟斤拷
float loop_float_constrain(float Input, float minValue, float maxValue);
// 锟角度革拷式锟斤拷为-180~180
float theta_format(float Ang);

int float_rounding(float raw);

float *Norm3d(float *v);

float NormOf3d(float *v);

void Cross3d(float *v1, float *v2, float *res);

float Dot3d(float *v1, float *v2);

float AverageFilter(float new_data, float *buf, uint8_t len);

#define rad_format(Ang) loop_float_constrain((Ang), -PI, PI)

#endif
