cmake_minimum_required(VERSION 3.10)

project(HydroOSlogger VERSION 1.0 LANGUAGES)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED True)

file(GLOB SRCS src/*.cpp)

add_library(HydroOSlogger ${SRCS})

target_include_directories(HydroOSlogger PUBLIC include)

target_link_libraries(HydroOSlogger FreeRTOS HydrodriversUART HydrolibLogger)
