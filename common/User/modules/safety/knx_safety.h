#ifndef KNX_SAFETY_H
#define KNX_SAFETY_H

#include "knx_types.h"
#include "octolinker.h"

typedef enum {
    KNX_FAULT_NONE          = 0,
    KNX_FAULT_OVERCURRENT   = (1 << 0),
    KNX_FAULT_IMU_FAIL      = (1 << 1),
    KNX_FAULT_ENCODER_FAIL  = (1 << 2),
    KNX_FAULT_WATCHDOG      = (1 << 3),
    KNX_FAULT_TILT          = (1 << 4),
    KNX_FAULT_OVERSPEED     = (1 << 5),
    KNX_FAULT_GIMBAL_LIMIT  = (1 << 6),
    KNX_FAULT_GIMBAL_COMMAND = (1 << 7),
    KNX_FAULT_HOST_COMM_LOST = (1 << 8),
} knx_fault_t;

typedef enum {
    KNX_WARN_NONE           = 0,
    KNX_WARN_STARTUP        = (1 << 0),
    KNX_WARN_VISION_STALE   = (1 << 1),
    KNX_WARN_DISABLED       = (1 << 2),
} knx_warning_t;

typedef enum {
    KNX_SAFETY_LEVEL_OK     = 0,
    KNX_SAFETY_LEVEL_WARN   = 1,
    KNX_SAFETY_LEVEL_FAULT  = 2,
} knx_safety_level_t;

knx_status_t          knx_safety_init(void);
knx_status_t          knx_safety_update(void);   /* called periodically (1ms) */
knx_safety_level_t    knx_safety_get_level(void);
uint32_t              knx_safety_get_faults(void);   /* bitmask of knx_fault_t */
uint32_t              knx_safety_get_warnings(void);
void                  knx_safety_clear_faults(void);
void                  knx_safety_debug_octo(Octolinker_Instance_t *octo, uint16_t base_id);

extern float knx_safety_gimbal_enable;
extern float knx_safety_gimbal_startup_grace_ms;
extern float knx_safety_gimbal_imu_timeout_ms;
extern float knx_safety_gimbal_vision_warn_ms;
extern float knx_safety_gimbal_yaw_fault_deg;
extern float knx_safety_gimbal_pitch_fault_deg;
extern float knx_safety_gimbal_cmd_fault_rpm;

extern float knx_safety_level_debug;
extern float knx_safety_faults_debug;
extern float knx_safety_warnings_debug;
extern float knx_safety_imu_age_ms_debug;
extern float knx_safety_vision_age_ms_debug;
extern uint32_t knx_safety_update_count;

#endif /* KNX_SAFETY_H */
