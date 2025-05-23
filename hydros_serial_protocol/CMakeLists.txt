cmake_minimum_required(VERSION 3.10)

project(HydroOSserialProtocol VERSION 1.0 LANGUAGES)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED True)

# file(GLOB SRCS src/*.cpp)

add_library(HydroOSserialProtocol INTERFACE)

target_include_directories(HydroOSserialProtocol INTERFACE include)

target_link_libraries(HydroOSserialProtocol INTERFACE FreeRTOS)

target_link_libraries(HydroOSserialProtocol INTERFACE HydrodriversUART)

target_link_libraries(HydroOSserialProtocol INTERFACE HydrolibSerialProtocol)
