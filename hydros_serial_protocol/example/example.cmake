cmake_minimum_required(VERSION 3.16)

set(CMAKE_TRY_COMPILE_TARGET_TYPE "STATIC_LIBRARY")

# set(BUILD_TESTING OFF CACHE BOOL )
project(CMakeTest VERSION 1.0)

enable_language(C CXX ASM)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED True)

set(CMAKE_EXECUTABLE_SUFFIX_C .elf)
set(CMAKE_EXECUTABLE_SUFFIX_CXX .elf)
set(CMAKE_EXECUTABLE_SUFFIX_ASM .elf)

# This should be safe to set for a bare-metal cross-compiler
set(CMAKE_FIND_ROOT_PATH_MODE_PROGRAM NEVER)

add_compile_options(-Wall -Wextra -Wpedantic -Wc++20-compat -Wno-format-security
    -Woverloaded-virtual -Wsuggest-override -fdata-sections -ffunction-sections -g -gdwarf-2)

add_compile_options(-mcpu=cortex-m4 -mthumb -mfpu=fpv4-sp-d16 -mfloat-abi=hard)

add_link_options(-mcpu=cortex-m4 -mthumb -mfpu=fpv4-sp-d16 -mfloat-abi=hard)
add_link_options(-specs=nosys.specs -T${CMAKE_CURRENT_LIST_DIR}/STM32F407VGTx_FLASH.ld -lc -lm -lnosys
    -Wl,-Map=${CMAKE_CURRENT_BINARY_DIR}/STMprogramTest.map,--cref -Wl,--gc-sections)

add_compile_definitions(STM32F407xx)

add_executable(STMprogramTest
    ${CMAKE_CURRENT_LIST_DIR}/src/main.cpp
    ${CMAKE_CURRENT_LIST_DIR}/src/startup_stm32f407xx.s
    ${CMAKE_CURRENT_LIST_DIR}/src/system_stm32f4xx.c
    ${CMAKE_CURRENT_LIST_DIR}/src/stm32f4xx_it.c)

target_link_libraries(STMprogramTest
    HydrodriversCommon HydrodriversClock HydroOSserialProtocol)

target_link_libraries(STMprogramTest HydrolibCommon)

target_include_directories(STMprogramTest PUBLIC ${CMAKE_CURRENT_LIST_DIR}/include)

target_compile_features(STMprogramTest PUBLIC cxx_std_20)
