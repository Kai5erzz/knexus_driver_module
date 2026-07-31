#ifndef KNX_GRAYSCALE_H
#define KNX_GRAYSCALE_H

#include "ir_line_sensor.h"
#include "knx_types.h"
#include "octolinker.h"

#define KNX_GRAYSCALE_CH_NUM        8U
#define KNX_GRAYSCALE_CAL_SAMPLES   100U  /* Legacy compatibility only. */
#define KNX_GRAYSCALE_DIGITAL_TH    0.5f  /* Legacy compatibility only. */

typedef struct {
    uint16_t raw[KNX_GRAYSCALE_CH_NUM];
    float normalized[KNX_GRAYSCALE_CH_NUM]; /**< 0 or 1; 1 means black line. */
    uint8_t digital[KNX_GRAYSCALE_CH_NUM];
    uint8_t digital_byte;
    float line_error;                        /**< -3.5..+3.5, zero=center. */
    uint16_t cal_min[KNX_GRAYSCALE_CH_NUM];
    uint16_t cal_max[KNX_GRAYSCALE_CH_NUM];
    uint8_t is_calibrated;                   /**< Always 1 for new hardware. */
} knx_grayscale_data_t;

void knx_grayscale_init(void);
void knx_grayscale_attach_sensor(ir_line_sensor_t *sensor);
knx_status_t knx_grayscale_update(void);

/* These calibration APIs remain so old upper-layer code still compiles.  The
 * new line-sensor module applies its threshold internally, so both are no-op. */
void knx_grayscale_set_calibration(const uint16_t *cal_min,
                                   const uint16_t *cal_max);
void knx_grayscale_calibrate(void);

void knx_grayscale_snapshot(knx_grayscale_data_t *data);
float knx_grayscale_get_line_error(void);
uint8_t knx_grayscale_get_digital_byte(void);
uint8_t knx_grayscale_is_calibrated(void);
void knx_grayscale_debug_octo(Octolinker_Instance_t *octo);

#endif
