cmake_minimum_required(VERSION 4.2)

# MSPM0 SDK.
set(MSPM0_SDK_DIR "D:/application_restore/ti/mspm0_sdk/mspm0_sdk_2_05_00_05"
    CACHE PATH "TI MSPM0 SDK root")

set(DEVICE_FAMILY "mspm0g350x")

# FreeRTOS local copy.
set(FREERTOS_DIR ${CMAKE_CURRENT_LIST_DIR}/Middlewares/Third_Party/FreeRTOS/Source)

# Common driver module root.
if(NOT DEFINED KNEXUS_MODULE_ROOT)
    set(KNEXUS_MODULE_ROOT ${CMAKE_CURRENT_LIST_DIR}/../..)
endif()
if(NOT DEFINED KNEXUS_COMMON_USER_DIR)
    set(KNEXUS_COMMON_USER_DIR ${KNEXUS_MODULE_ROOT}/common/User)
endif()

# Sources.
set(STARTUP_FILE
    "${MSPM0_SDK_DIR}/source/ti/devices/msp/m0p/startup_system_files/gcc/startup_mspm0g350x_gcc.c")

set(FREERTOS_SOURCES
    ${FREERTOS_DIR}/tasks.c
    ${FREERTOS_DIR}/queue.c
    ${FREERTOS_DIR}/list.c
    ${FREERTOS_DIR}/timers.c
    ${FREERTOS_DIR}/event_groups.c
    ${FREERTOS_DIR}/stream_buffer.c
    ${FREERTOS_DIR}/croutine.c
    ${FREERTOS_DIR}/portable/GCC/ARM_CM0/port.c
    ${FREERTOS_DIR}/portable/MemMang/heap_4.c
)

# Common driver sources.
set(COMMON_DRIVER_SOURCES
    ${KNEXUS_COMMON_USER_DIR}/drv/led/knx_led.c
    ${KNEXUS_COMMON_USER_DIR}/drv/key/knx_key.c
    ${KNEXUS_COMMON_USER_DIR}/drv/beep/knx_beep.c
    ${KNEXUS_COMMON_USER_DIR}/drv/encoder/encoder.c
    ${KNEXUS_COMMON_USER_DIR}/drv/octolinker/octolinker.c
    ${KNEXUS_COMMON_USER_DIR}/drv/drv8701e/drv8701e.c
    ${KNEXUS_COMMON_USER_DIR}/drv/bmi088/bmi088.c
    ${KNEXUS_COMMON_USER_DIR}/drv/track_sensor/track_sensor.c
)

# Common module/config sources.
set(COMMON_MODULE_SOURCES
    ${KNEXUS_COMMON_USER_DIR}/config/knx_params.c
    ${KNEXUS_COMMON_USER_DIR}/util/knx_control.c
    ${KNEXUS_COMMON_USER_DIR}/modules/param/knx_param.c
    ${KNEXUS_COMMON_USER_DIR}/modules/sys/knx_sys.c
    ${KNEXUS_COMMON_USER_DIR}/modules/blackbox/knx_blackbox.c
    ${KNEXUS_COMMON_USER_DIR}/modules/health/knx_health.c
    ${KNEXUS_COMMON_USER_DIR}/modules/can_router/knx_can_router.c
    ${KNEXUS_COMMON_USER_DIR}/modules/motor/knx_motor.c
    ${KNEXUS_COMMON_USER_DIR}/modules/drive/knx_drive.c
    ${KNEXUS_COMMON_USER_DIR}/modules/imu/knx_imu.c
    ${KNEXUS_COMMON_USER_DIR}/modules/grayscale/knx_grayscale.c
    ${KNEXUS_COMMON_USER_DIR}/modules/telemetry/knx_telemetry.c
)

# Common algorithm sources.
set(COMMON_ALGO_SOURCES
    ${KNEXUS_COMMON_USER_DIR}/algorithm/PID/PID.c
    ${KNEXUS_COMMON_USER_DIR}/algorithm/Ramp/ramp.c
    ${KNEXUS_COMMON_USER_DIR}/algorithm/kalman_filter/kalman_filter.c
    ${KNEXUS_COMMON_USER_DIR}/algorithm/QuaternionEKF/QuaternionEKF.c
)

# Common test mode sources.
set(COMMON_TEST_MODE_SOURCES
    ${KNEXUS_COMMON_USER_DIR}/app/test_modes/test_drv8701e/test_drv8701e.c
)

file(GLOB_RECURSE MSPM0_BOARD_SOURCES
    "${CMAKE_CURRENT_LIST_DIR}/User/board/*.c"
)
file(GLOB_RECURSE MSPM0_PLATFORM_SOURCES
    "${CMAKE_CURRENT_LIST_DIR}/User/platform/*.c"
)
list(FILTER MSPM0_PLATFORM_SOURCES EXCLUDE REGEX "knx_telemetry_mspm0_stub\\.c$")
file(GLOB_RECURSE MSPM0_APP_SOURCES
    "${CMAKE_CURRENT_LIST_DIR}/User/app/*.c"
)
file(GLOB_RECURSE MSPM0_TASK_SOURCES
    "${CMAKE_CURRENT_LIST_DIR}/User/tasks/*.c"
)

add_executable(knexus_mspm0.elf
    main.c
    syscalls.c
    ti_msp_dl_config.c
    ${STARTUP_FILE}
    ${FREERTOS_SOURCES}
    ${COMMON_DRIVER_SOURCES}
    ${COMMON_MODULE_SOURCES}
    ${COMMON_ALGO_SOURCES}
    ${COMMON_TEST_MODE_SOURCES}
    ${MSPM0_BOARD_SOURCES}
    ${MSPM0_PLATFORM_SOURCES}
    ${MSPM0_APP_SOURCES}
    ${MSPM0_TASK_SOURCES}
)

# Include paths.
target_include_directories(knexus_mspm0.elf PRIVATE
    # --- MSPM0 project local ---
    ${CMAKE_CURRENT_LIST_DIR}
    ${CMAKE_CURRENT_LIST_DIR}/User/board/knexus_mspm0
    ${CMAKE_CURRENT_LIST_DIR}/User/platform/mspm0
    ${CMAKE_CURRENT_LIST_DIR}/User/app
    ${CMAKE_CURRENT_LIST_DIR}/User/tasks
    # --- FreeRTOS ---
    ${FREERTOS_DIR}
    ${FREERTOS_DIR}/include
    ${FREERTOS_DIR}/portable/GCC/ARM_CM0
    # --- TI SDK ---
    ${MSPM0_SDK_DIR}/source
    ${MSPM0_SDK_DIR}/source/third_party/CMSIS/Core/Include
    ${MSPM0_SDK_DIR}/source/third_party/CMSIS/DSP/Include
    # --- Common driver / platform headers ---
    ${KNEXUS_COMMON_USER_DIR}/platform/include
    ${KNEXUS_COMMON_USER_DIR}/config
    ${KNEXUS_COMMON_USER_DIR}/drv/led
    ${KNEXUS_COMMON_USER_DIR}/drv/key
    ${KNEXUS_COMMON_USER_DIR}/drv/beep
    ${KNEXUS_COMMON_USER_DIR}/drv/encoder
    ${KNEXUS_COMMON_USER_DIR}/drv/octolinker
    ${KNEXUS_COMMON_USER_DIR}/drv/drv8701e
    ${KNEXUS_COMMON_USER_DIR}/drv/bmi088
    ${KNEXUS_COMMON_USER_DIR}/drv/track_sensor
    ${KNEXUS_COMMON_USER_DIR}/util
    ${KNEXUS_COMMON_USER_DIR}/algorithm/PID
    ${KNEXUS_COMMON_USER_DIR}/algorithm/Ramp
    ${KNEXUS_COMMON_USER_DIR}/algorithm/kalman_filter
    ${KNEXUS_COMMON_USER_DIR}/algorithm/QuaternionEKF
    ${KNEXUS_COMMON_USER_DIR}/modules/param
    ${KNEXUS_COMMON_USER_DIR}/modules/sys
    ${KNEXUS_COMMON_USER_DIR}/modules/blackbox
    ${KNEXUS_COMMON_USER_DIR}/modules/health
    ${KNEXUS_COMMON_USER_DIR}/modules/can_router
    ${KNEXUS_COMMON_USER_DIR}/modules/motor
    ${KNEXUS_COMMON_USER_DIR}/modules/drive
    ${KNEXUS_COMMON_USER_DIR}/modules/imu
    ${KNEXUS_COMMON_USER_DIR}/modules/grayscale
    ${KNEXUS_COMMON_USER_DIR}/modules/telemetry
    ${KNEXUS_COMMON_USER_DIR}/app/test_modes/test_drv8701e
)

# Compile definitions and flags.
target_compile_definitions(knexus_mspm0.elf PRIVATE
    DeviceFamily_MSPM0G350X
    KNX_TARGET_MSPM0
    ARM_MATH_CM0PLUS
)

target_compile_options(knexus_mspm0.elf PRIVATE
    -mcpu=cortex-m0plus
    -mthumb
    -mfloat-abi=soft
    -ffunction-sections
    -fdata-sections
    -g
    -O2
)

# Linker.
target_link_options(knexus_mspm0.elf PRIVATE
    -T${CMAKE_CURRENT_LIST_DIR}/device_linker.lds
    -Wl,--gc-sections
    -Wl,-Map=${CMAKE_BINARY_DIR}/knexus_mspm0.map
    --specs=nano.specs
    -mcpu=cortex-m0plus
    -mthumb
    -mfloat-abi=soft
)

target_link_libraries(knexus_mspm0.elf PRIVATE
    ${MSPM0_SDK_DIR}/source/ti/driverlib/lib/gcc/m0p/mspm0g1x0x_g3x0x/driverlib.a
    ${MSPM0_SDK_DIR}/source/third_party/CMSIS/DSP/lib/gcc/m0p/arm_cortexM0l_math.a
    c
    m
    gcc
)

# Post-build: hex/bin/size.
add_custom_command(TARGET knexus_mspm0.elf POST_BUILD
    COMMAND ${CMAKE_OBJCOPY} -O ihex   $<TARGET_FILE:knexus_mspm0.elf>
            ${CMAKE_BINARY_DIR}/knexus_mspm0.hex
    COMMAND ${CMAKE_OBJCOPY} -O binary $<TARGET_FILE:knexus_mspm0.elf>
            ${CMAKE_BINARY_DIR}/knexus_mspm0.bin
    COMMAND ${SIZE} $<TARGET_FILE:knexus_mspm0.elf>
    COMMENT "Generating knexus_mspm0.hex / .bin"
)
