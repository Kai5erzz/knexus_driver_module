#ifndef __ENCODER_H
#define __ENCODER_H

#include "knx_encoder.h"
#include "knx_types.h"

/* Direction signs: positive speed means the wheel moves the chassis forward. */
#define ENC1_DIR_SIGN       -1
#define ENC2_DIR_SIGN       -1

/* 65 mm wheel, 13 PPR motor encoder, 28:1 gearbox. */
#define ENC_PPR             13
#define MOTOR_GEAR_RATIO    28
#define WHEEL_DIAMETER_MM   65.0f
#define ENC_SPEED_LPF_HZ    10.0f
#define ENC_SPEED_UPDATE_MS 10U

#define ENC_COUNTS_PER_WHEEL_REV_TIM   (ENC_PPR * MOTOR_GEAR_RATIO * 4)
#define ENC_COUNTS_PER_WHEEL_REV_LPTIM (ENC_PPR * MOTOR_GEAR_RATIO * 2)

typedef struct {
    const knx_encoder_port_t *port;
    uint8_t is_lptim;
    int8_t dir_sign;
    int32_t total_count;
    uint16_t last_raw;
    float speed_rpm;
    float speed_mps;
    float speed_mps_raw;
    float delta_m;
    float position_m;
    uint32_t last_calc_tick;
} Encoder_t;

extern Encoder_t encoder_left;
extern Encoder_t encoder_right;

knx_status_t Encoder_AttachPorts(const knx_encoder_port_t *left,
                         const knx_encoder_port_t *right);
knx_status_t Encoder_Init(void);
void Encoder_Update(void);
int32_t Encoder_GetCount(Encoder_t *enc);
void Encoder_Reset(Encoder_t *enc);
void Encoder_CalcSpeed(Encoder_t *enc);

#endif /* __ENCODER_H */
