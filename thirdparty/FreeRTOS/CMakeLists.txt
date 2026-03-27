include(FetchContent)

FetchContent_Declare(
    freertos
    GIT_REPOSITORY https://github.com/stm32duino/STM32FreeRTOS.git
    GIT_TAG 10.3.3
)

FetchContent_Populate(freertos)

file(GLOB SRCS ${freertos_SOURCE_DIR}/src/FreeRTOS/Source/*.c)
list(APPEND SRCS 
    ${freertos_SOURCE_DIR}/portable/CMSIS_RTOS_V2/cmsis_os2.c
    ${freertos_SOURCE_DIR}/portable/MemMang/heap_4.c
    ${freertos_SOURCE_DIR}/portable/GCC/ARM_CM4F/port.c)

add_library(FreeRTOS ${SRCS})

target_include_directories(FreeRTOS PUBLIC 
    ${freertos_SOURCE_DIR}/src/FreeRTOS/Source/include
    ${freertos_SOURCE_DIR}/portable/CMSIS_RTOS_V2
    ${freertos_SOURCE_DIR}/portable/GCC/ARM_CM4F
    config)

target_link_libraries(FreeRTOS CMSIS)
