cmake_minimum_required(VERSION 3.16)

project(HydroOS VERSION 0.1.0)
enable_language(C CXX ASM)

add_subdirectory(thirdparty)

add_subdirectory(hydros_uart)
# add_subdirectory(hydros_serial_protocol)
# add_subdirectory(hydros_vectornav)
