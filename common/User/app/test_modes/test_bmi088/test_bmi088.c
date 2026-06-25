#include "test_bmi088.h"
#include "knx_imu.h"
#include "bmi088.h"
#include "octolinker.h"

static Octolinker_Instance_t *s_octo;

void test_bmi088_init(void *octo)
{
    s_octo = (Octolinker_Instance_t *)octo;
}

void test_bmi088_loop(void)
{
    knx_imu_update();
    if (s_octo != NULL) {
        BMI088_DebugOcto(s_octo);
    }
}
