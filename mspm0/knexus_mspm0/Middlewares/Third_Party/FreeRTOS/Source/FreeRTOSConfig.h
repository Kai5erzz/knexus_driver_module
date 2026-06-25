/**
 * @file FreeRTOSConfig.h
 * @brief FreeRTOS configuration for MSPM0G3507 (80MHz, Cortex-M0+)
 */

#ifndef FREERTOS_CONFIG_H
#define FREERTOS_CONFIG_H

/* Scheduler */
#define configUSE_PREEMPTION                    1
#define configUSE_PORT_OPTIMISED_TASK_SELECTION  0
#define configTICK_RATE_HZ                      ((TickType_t) 1000)
#define configMAX_PRIORITIES                    (10UL)
#define configIDLE_SHOULD_YIELD                 0
#define configUSE_16_BIT_TICKS                  0
#define configUSE_TIME_SLICING                  1

/* Hardware */
#define configCPU_CLOCK_HZ                      ((unsigned long) 80000000)
#define configMINIMAL_STACK_SIZE                ((unsigned short) 128)
#define configMAX_TASK_NAME_LEN                 (12)
#define configTOTAL_HEAP_SIZE                   ((size_t)(16 * 1024))

/* Features */
#define configSUPPORT_STATIC_ALLOCATION         1
#define configUSE_MUTEXES                       1
#define configUSE_COUNTING_SEMAPHORES           1
#define configUSE_RECURSIVE_MUTEXES             1
#define configUSE_TASK_NOTIFICATIONS            1
#define configUSE_QUEUE_SETS                    0
#define configUSE_CO_ROUTINES                   0

/* Hooks */
#define configUSE_IDLE_HOOK                     0
#define configUSE_TICK_HOOK                     0
#define configUSE_MALLOC_FAILED_HOOK            0
#define configCHECK_FOR_STACK_OVERFLOW          2

/* Debug */
#define configASSERT(x)           \
    if ((x) == 0) {               \
        taskDISABLE_INTERRUPTS(); \
        for (;;)                  \
            ;                     \
    }

/* Software timers */
#define configUSE_TIMERS                        1
#define configTIMER_TASK_PRIORITY               (5)
#define configTIMER_QUEUE_LENGTH                (20)
#define configTIMER_TASK_STACK_DEPTH            (configMINIMAL_STACK_SIZE)

/* API inclusion */
#define INCLUDE_vTaskPrioritySet                1
#define INCLUDE_uxTaskPriorityGet               1
#define INCLUDE_vTaskDelete                     1
#define INCLUDE_vTaskSuspend                    1
#define INCLUDE_vTaskDelayUntil                 1
#define INCLUDE_vTaskDelay                      1
#define INCLUDE_eTaskGetState                   1
#define INCLUDE_xTaskGetCurrentTaskHandle       1
#define INCLUDE_xTaskGetSchedulerState          1

/* Cortex-M0 interrupt priority (2 bits = 4 levels) */
#ifdef __NVIC_PRIO_BITS
#define configPRIO_BITS __NVIC_PRIO_BITS
#else
#define configPRIO_BITS 2
#endif

#define configLIBRARY_LOWEST_INTERRUPT_PRIORITY         0x03
#define configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY    1
#define configKERNEL_INTERRUPT_PRIORITY \
    (configLIBRARY_LOWEST_INTERRUPT_PRIORITY << (8 - configPRIO_BITS))
#define configMAX_SYSCALL_INTERRUPT_PRIORITY \
    (configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY << (8 - configPRIO_BITS))

/* Map FreeRTOS handlers to CMSIS names */
#define xPortPendSVHandler      PendSV_Handler
#define vPortSVCHandler         SVC_Handler
#define xPortSysTickHandler     SysTick_Handler

#endif /* FREERTOS_CONFIG_H */
