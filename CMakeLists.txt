cmake_minimum_required(VERSION 4.2)

set(KNEXUS_TARGET "stm32" CACHE STRING "Target platform to build: stm32 or mspm0")
set_property(CACHE KNEXUS_TARGET PROPERTY STRINGS stm32 mspm0)
set(KNEXUS_APP "contest_2026" CACHE STRING "Application profile: contest_2026 or legacy")
set_property(CACHE KNEXUS_APP PROPERTY STRINGS contest_2026 legacy)

# ARM GCC toolchain shared by STM32 and MSPM0.
set(CMAKE_SYSTEM_NAME Generic)
set(CMAKE_SYSTEM_VERSION 1)
set(CMAKE_C_COMPILER arm-none-eabi-gcc)
set(CMAKE_CXX_COMPILER arm-none-eabi-g++)
set(CMAKE_ASM_COMPILER arm-none-eabi-gcc)
set(CMAKE_AR arm-none-eabi-ar)
set(CMAKE_OBJCOPY arm-none-eabi-objcopy)
set(CMAKE_OBJDUMP arm-none-eabi-objdump)
set(SIZE arm-none-eabi-size)
set(CMAKE_TRY_COMPILE_TARGET_TYPE STATIC_LIBRARY)

project(knexus_driver_module C CXX ASM)

set(KNEXUS_MODULE_ROOT ${CMAKE_CURRENT_LIST_DIR})
set(KNEXUS_COMMON_USER_DIR ${KNEXUS_MODULE_ROOT}/common/User)

message(STATUS "KNEXUS_TARGET=${KNEXUS_TARGET}")
message(STATUS "KNEXUS_APP=${KNEXUS_APP}")
message(STATUS "KNEXUS_MODULE_ROOT=${KNEXUS_MODULE_ROOT}")
message(STATUS "KNEXUS_COMMON_USER_DIR=${KNEXUS_COMMON_USER_DIR}")

if(KNEXUS_TARGET STREQUAL "stm32")
    add_subdirectory(stm32/knexus_st ${CMAKE_BINARY_DIR}/stm32/knexus_st)
elseif(KNEXUS_TARGET STREQUAL "mspm0")
    add_subdirectory(mspm0/knexus_mspm0 ${CMAKE_BINARY_DIR}/mspm0/knexus_mspm0)
else()
    message(FATAL_ERROR "Unsupported KNEXUS_TARGET='${KNEXUS_TARGET}'. Use stm32 or mspm0.")
endif()
