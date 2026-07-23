#ifndef TEST_STM32_BOARD_BRINGUP_H
#define TEST_STM32_BOARD_BRINGUP_H

/* STM32H743 new-baseboard acceptance test.
 *
 * OctoLink:
 *   BMI088 driver telemetry: IDs 20..25
 *   Board-test telemetry:   IDs 600..619
 *
 * Motor safety:
 *   - outputs are stopped at boot;
 *   - hold KEY0 for one second to arm/disarm the low-duty sweep;
 *   - press KEY1 to stop immediately.
 */

void test_stm32_board_bringup_init(void *octo);
void test_stm32_board_bringup_loop(void);

#endif /* TEST_STM32_BOARD_BRINGUP_H */
