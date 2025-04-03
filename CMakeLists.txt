cmake_minimum_required(VERSION 3.16)

if(DRY_RUN)
    set(CMAKE_TRY_COMPILE_TARGET_TYPE "STATIC_LIBRARY")

    # set(BUILD_TESTING OFF CACHE BOOL )
    project(HydroOS VERSION 1.0)

    enable_language(C CXX ASM)

    set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

    set(CMAKE_CXX_STANDARD 20)
    set(CMAKE_CXX_STANDARD_REQUIRED True)

    # find_program(CMAKE_C_COMPILER NAMES arm-none-eabi-gcc HINTS /bin)
    # find_program(CMAKE_CXX_COMPILER NAMES arm-none-eabi-g++ HINTS /bin)
    # find_program(CMAKE_ASM_COMPILER NAMES arm-none-eabi-gcc HINTS /bin)
    set(CMAKE_EXECUTABLE_SUFFIX_C .elf)
    set(CMAKE_EXECUTABLE_SUFFIX_CXX .elf)
    set(CMAKE_EXECUTABLE_SUFFIX_ASM .elf)

    # This should be safe to set for a bare-metal cross-compiler
    set(CMAKE_FIND_ROOT_PATH_MODE_PROGRAM NEVER)

    add_compile_options(-Wall -Wextra -Wpedantic -Wc++20-compat -Wno-format-security
        -Woverloaded-virtual -Wsuggest-override -fdata-sections -ffunction-sections -g -gdwarf-2)

    add_compile_options(-mcpu=cortex-m4 -mthumb -mfpu=fpv4-sp-d16 -mfloat-abi=hard)

    add_link_options(-mcpu=cortex-m4 -mthumb -mfpu=fpv4-sp-d16 -mfloat-abi=hard)
    add_link_options(-specs=nosys.specs -T${CMAKE_CURRENT_SOURCE_DIR}/STM32F407VGTx_FLASH.ld -lc -lm -lnosys
        -Wl,-Map=${CMAKE_CURRENT_BINARY_DIR}/STMprogramTest.map,--cref -Wl,--gc-sections)

    add_compile_definitions(STM32F407xx)
else()
    project(HydroOS VERSION 1.0)

    enable_language(C CXX ASM)
endif()

add_subdirectory(Hydrodrivers)
add_subdirectory(FreeRTOS)
add_subdirectory(Hydrolib-soft)
add_subdirectory(hydros_serial_protocol)
add_subdirectory(hydros_logger)
