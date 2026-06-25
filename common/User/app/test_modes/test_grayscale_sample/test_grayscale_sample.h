/**
 * @file    test_grayscale_sample.h
 * @author  kaiser
 * @version V2.0.0
 * @date    2026-06-07
 * @brief   路口样本采集程序
 *
 * 流程:
 *   1. IDLE         - 等待 KEY0 开始校准
 *   2. CALIBRATING  - 灰度黑白校准 (LED1 快闪, 完成后常亮)
 *   3. WAIT_SAMPLE  - 等待 KEY1 开始采样运行
 *   4. SAMPLING     - 前进 20cm, 等距离采样灰度到 64×8 矩阵 (LED2 亮)
 *   5. SAMPLE_DONE  - 停车 (LED2 灭)
 *      - KEY0: 发送采样矩阵 (Octolinker_SendU8Matrix)
 *      - KEY1: 清空矩阵, 重新采样
 */

#ifndef TEST_GRAYSCALE_SAMPLE_H
#define TEST_GRAYSCALE_SAMPLE_H

#include "knx_types.h"

void test_grayscale_sample_init(void *octo);
void test_grayscale_sample_loop(void);

#endif /* TEST_GRAYSCALE_SAMPLE_H */
