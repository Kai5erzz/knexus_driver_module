#include "test_dm_imu_l1.h"
#include "dm_imu_l1.h"
#include "knx_time.h"
#include "octolinker.h"

#define TEST_DM_IMU_L1_TELEM_BASE 180U

Octolinker_Instance_t *test_dm_imu_l1_octo;

float test_dm_imu_l1_debug_enable = 1.0f;
float test_dm_imu_l1_debug_period_ms = 100.0f;
float test_dm_imu_l1_active_report = 1.0f;
float test_dm_imu_l1_interval_ms = 1.0f;

float test_dm_imu_l1_roll = 0.0f;
float test_dm_imu_l1_pitch = 0.0f;
float test_dm_imu_l1_yaw = 0.0f;
float test_dm_imu_l1_yaw_total = 0.0f;
float test_dm_imu_l1_temperature = 0.0f;
uint32_t test_dm_imu_l1_rx_count = 0U;
uint32_t test_dm_imu_l1_tx_count = 0U;
uint32_t test_dm_imu_l1_error_count = 0U;
uint32_t test_dm_imu_l1_last_rx_id = 0U;
uint8_t test_dm_imu_l1_last_reg = 0U;
uint8_t test_dm_imu_l1_data_flags = 0U;
int32_t test_dm_imu_l1_last_status = 0;

static uint32_t s_debug_tick;
static float s_last_active_report;
static float s_last_interval_ms;

static uint32_t test_dm_imu_l1_period_ms(void)
{
    if (test_dm_imu_l1_debug_period_ms < 10.0f) {
        return 10U;
    }
    return (uint32_t)test_dm_imu_l1_debug_period_ms;
}

static void test_dm_imu_l1_update_mirrors(void)
{
    test_dm_imu_l1_roll = dm_imu_l1_data.roll;
    test_dm_imu_l1_pitch = dm_imu_l1_data.pitch;
    test_dm_imu_l1_yaw = dm_imu_l1_data.yaw;
    test_dm_imu_l1_yaw_total = dm_imu_l1_data.yaw_total;
    test_dm_imu_l1_temperature = dm_imu_l1_data.temperature;
    test_dm_imu_l1_rx_count = dm_imu_l1_data.rx_count;
    test_dm_imu_l1_tx_count = dm_imu_l1_data.tx_count;
    test_dm_imu_l1_error_count = dm_imu_l1_data.error_count;
    test_dm_imu_l1_last_rx_id = dm_imu_l1_data.last_rx_id;
    test_dm_imu_l1_last_reg = dm_imu_l1_data.last_reg;
    test_dm_imu_l1_data_flags = dm_imu_l1_data.data_flags;
    test_dm_imu_l1_last_status = (int32_t)dm_imu_l1_data.last_status;
}

void test_dm_imu_l1_init(void *octo)
{
    test_dm_imu_l1_octo = (Octolinker_Instance_t *)octo;
    s_debug_tick = 0U;
    s_last_active_report = test_dm_imu_l1_active_report;
    s_last_interval_ms = test_dm_imu_l1_interval_ms;
}

void test_dm_imu_l1_loop(void)
{
    uint32_t now = knx_millis();

    if (test_dm_imu_l1_active_report != s_last_active_report) {
        s_last_active_report = test_dm_imu_l1_active_report;
        (void)DM_IMU_L1_SetActiveReport(test_dm_imu_l1_active_report > 0.0f);
    }

    if (test_dm_imu_l1_interval_ms != s_last_interval_ms) {
        s_last_interval_ms = test_dm_imu_l1_interval_ms;
        if (test_dm_imu_l1_interval_ms >= 1.0f && test_dm_imu_l1_interval_ms <= 1000.0f) {
            (void)DM_IMU_L1_SetActiveIntervalMs((uint16_t)test_dm_imu_l1_interval_ms);
        }
    }

    test_dm_imu_l1_update_mirrors();

    if (test_dm_imu_l1_debug_enable > 0.0f &&
        now - s_debug_tick >= test_dm_imu_l1_period_ms()) {
        s_debug_tick = now;
        DM_IMU_L1_DebugOcto(test_dm_imu_l1_octo, TEST_DM_IMU_L1_TELEM_BASE);
    }
}
