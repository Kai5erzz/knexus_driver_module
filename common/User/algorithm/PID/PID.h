#ifndef _PID_H
#define _PID_H

#include <math.h>
#include <stdint.h>

#define PID_NUM_MAX 30

#ifndef usr_abs
#define usr_abs(x) ((x > 0) ? x : -(x))
#endif

#define INIT_PID_CONFIG(Kp_val, Ki_val, Kd_val, IntegralLimit_val, MaxOut_val, Improve_val) \
    {                                                                                      \
        .Kp = Kp_val,                                                                      \
        .Ki = Ki_val,                                                                      \
        .Kd = Kd_val,                                                                      \
        .IntegralLimit = IntegralLimit_val,                                                \
        .MaxOut = MaxOut_val,                                                              \
        .Improve = Improve_val,                                                            \
    }

typedef enum
{
    PID_IMPROVE_NONE = 0x00,
    PID_Integral_Limit = 0x01,
    PID_Derivative_On_Measurement = 0x02,
    PID_Trapezoid_Intergral = 0x04,
    PID_Proportional_On_Measurement = 0x08,
    PID_OutputFilter = 0x10,
    PID_ChangingIntegrationRate = 0x20,
    PID_DerivativeFilter = 0x40,
    PID_ErrorHandle = 0x80,
} pid_improvement_e;

typedef enum error_type_e
{
    PID_ERROR_NONE = 0x00U,
    PID_MOTOR_BLOCKED_ERROR = 0x01U
} error_type_e;

typedef struct
{
    uint64_t error_count;
    error_type_e error_type;
} pid_ErrorHandler_t;

typedef struct
{
    float Kp;
    float Ki;
    float Kd;
    float MaxOut;
    float DeadBand;

    pid_improvement_e Improve;
    float IntegralLimit;
    float CoefA;
    float CoefB;
    float Output_LPF_RC;
    float Derivative_LPF_RC;

    float Measure;
    float Last_Measure;
    float Err;
    float Last_Err;
    float Last_ITerm;

    float Pout;
    float Iout;
    float Dout;
    float ITerm;

    float Output;
    float Last_Output;
    float Last_Dout;

    float Ref;

    uint32_t time_stamp_us;
    float dt;

    pid_ErrorHandler_t ERRORHandler;
} pid_obj_t;

typedef struct
{
    float Kp;
    float Ki;
    float Kd;
    float MaxOut;
    float DeadBand;

    pid_improvement_e Improve;
    float IntegralLimit;
    float CoefA;
    float CoefB;
    float Output_LPF_RC;
    float Derivative_LPF_RC;
} pid_config_t;

pid_obj_t *pid_register(pid_config_t *config);
float pid_calculate(pid_obj_t *pid, float measure, float ref);
void pid_clear(pid_obj_t *pid);

#endif /* _PID_H */
