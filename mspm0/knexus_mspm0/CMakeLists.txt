cmake_minimum_required(VERSION 4.2)

# MSPM0 SDK.
set(MSPM0_SDK_DIR "D:/application_restore/ti/mspm0_sdk/mspm0_sdk_2_05_00_05"
    CACHE PATH "TI MSPM0 SDK root")

set(DEVICE_FAMILY "mspm0g350x")

if(KNEXUS_APP STREQUAL "contest_2026")
    add_compile_definitions(KNX_BUILD_CONTEST_2026=1)
endif()

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
    ${KNEXUS_COMMON_USER_DIR}/util/knx_math.c
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
    ${KNEXUS_COMMON_USER_DIR}/modules/host_comm/knx_host_comm.c
    ${KNEXUS_COMMON_USER_DIR}/modules/safety/knx_safety.c
    ${KNEXUS_COMMON_USER_DIR}/modules/chassis/knx_chassis.c
    ${KNEXUS_COMMON_USER_DIR}/modules/can_bus/knx_can_bus.c
    ${KNEXUS_COMMON_USER_DIR}/modules/track/knx_track.c
    ${KNEXUS_COMMON_USER_DIR}/modules/intersection/knx_intersection.c
    ${KNEXUS_COMMON_USER_DIR}/modules/comm/knx_comm.c
    ${KNEXUS_COMMON_USER_DIR}/app/contest_2026/knx26_app.c
    ${KNEXUS_COMMON_USER_DIR}/app/contest_2026/knx26_user.c
)

# Common algorithm sources.
# QuaternionEKF is included because knx_imu.c references QEKF_INS / IMU_QuaternionEKF_*.
set(COMMON_ALGO_SOURCES
    ${KNEXUS_COMMON_USER_DIR}/algorithm/PID/PID.c
    ${KNEXUS_COMMON_USER_DIR}/algorithm/Ramp/ramp.c
    ${KNEXUS_COMMON_USER_DIR}/algorithm/kalman_filter/kalman_filter.c
    ${KNEXUS_COMMON_USER_DIR}/algorithm/QuaternionEKF/QuaternionEKF.c
    ${KNEXUS_COMMON_USER_DIR}/algorithm/crc/crc8_crc16.c
)

# Common test mode sources.
set(COMMON_TEST_MODE_SOURCES
    ${KNEXUS_COMMON_USER_DIR}/app/test_modes/test_drv8701e/test_drv8701e.c
)
if(KNEXUS_APP STREQUAL "contest_2026")
    set(COMMON_TEST_MODE_SOURCES)
endif()

file(GLOB_RECURSE MSPM0_BOARD_SOURCES
    "${CMAKE_CURRENT_LIST_DIR}/User/board/*.c"
)
file(GLOB_RECURSE MSPM0_PLATFORM_SOURCES
    "${CMAKE_CURRENT_LIST_DIR}/User/platform/*.c"
)
# Include the MSPM0 lightweight telemetry stub (avoids pulling in
# knx_motor/knx_imu/knx_drive/knx_sys/bmi088 heavy dependencies).
# Exclude the full common telemetry module instead.
list(FILTER COMMON_MODULE_SOURCES EXCLUDE REGEX "knx_telemetry\\.c$")

# Heavy / vision-only modules are not used on MSPM0 (Cortex-M0+, 32 KB SRAM).
# They are deliberately absent from COMMON_MODULE_SOURCES above; these filters
# are a safety net to guarantee they cannot be pulled in by accident.
# NOTE: knx_grayscale.c is intentionally KEPT — it is lightweight (reuses the
# TrackSensor driver, ~5 KB source) and is actively called by
# knx_mspm0_app_init() / knx_mspm0_upper_medium_update(). Excluding it would
# break the link. QuaternionEKF is excluded separately from COMMON_ALGO_SOURCES.
list(FILTER COMMON_MODULE_SOURCES EXCLUDE REGEX "knx_gimbal_ctrl\\.c$")
list(FILTER COMMON_MODULE_SOURCES EXCLUDE REGEX "knx_vision\\.c$")
list(FILTER COMMON_MODULE_SOURCES EXCLUDE REGEX "track_tinycnn.*\\.c$")
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
    ${KNEXUS_COMMON_USER_DIR}/algorithm/crc
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
    ${KNEXUS_COMMON_USER_DIR}/modules/host_comm
    ${KNEXUS_COMMON_USER_DIR}/modules/safety
    ${KNEXUS_COMMON_USER_DIR}/modules/chassis
    ${KNEXUS_COMMON_USER_DIR}/modules/can_bus
    ${KNEXUS_COMMON_USER_DIR}/modules/track
    ${KNEXUS_COMMON_USER_DIR}/modules/intersection
    ${KNEXUS_COMMON_USER_DIR}/modules/comm
    ${KNEXUS_COMMON_USER_DIR}/app/contest_2026
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
