cmake_minimum_required(VERSION 3.10)

project(HydroOSVectorNAV VERSION 1.0 LANGUAGES)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED True)

file(GLOB SRCS src/*.cpp)

add_library(HydroOSVectorNAV INTERFACE)

target_include_directories(HydroOSVectorNAV INTERFACE include)

target_link_libraries(HydroOSVectorNAV INTERFACE FreeRTOS INTERFACE HydrodriversUART INTERFACE HydrolibVectorNAV)
