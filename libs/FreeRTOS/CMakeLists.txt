cmake_minimum_required(VERSION 3.10)

project(CMakeFreeRTOS VERSION 1.0)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED True)

file(GLOB SRCS STM32FreeRTOS/src/FreeRTOS/Source/*.c)
list(APPEND SRCS STM32FreeRTOS/portable/CMSIS_RTOS_V2/cmsis_os2.c
    STM32FreeRTOS/portable/MemMang/heap_4.c
    STM32FreeRTOS/portable/GCC/ARM_CM4F/port.c)

add_library(FreeRTOS ${SRCS})

target_include_directories(FreeRTOS PUBLIC STM32FreeRTOS/src/FreeRTOS/Source/include
    STM32FreeRTOS/portable/CMSIS_RTOS_V2
    STM32FreeRTOS/portable/GCC/ARM_CM4F
    config)

target_link_libraries(FreeRTOS CMSIS)
