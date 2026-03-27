set(TARGET_NAME HydroOSUARTExample)

add_executable(${TARGET_NAME}
    ${CMAKE_CURRENT_LIST_DIR}/src/main.cpp)

target_link_libraries(${TARGET_NAME}
    HydrodriversClockCPP HydroOSUART HydrodriversGPIO)
target_compile_features(${TARGET_NAME} PUBLIC cxx_std_20)

set(TARGET_NAME HydroOSRSExample)

add_executable(${TARGET_NAME}
    ${CMAKE_CURRENT_LIST_DIR}/src/main_rs485.cpp)

target_link_libraries(${TARGET_NAME}
    HydrodriversClockCPP HydroOSUART HydrodriversGPIO)

target_compile_features(${TARGET_NAME} PUBLIC cxx_std_20)