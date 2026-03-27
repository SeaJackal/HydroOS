set(LIBRARY_NAME HydroOSUART)

add_library(${LIBRARY_NAME} INTERFACE)

target_include_directories(${LIBRARY_NAME} INTERFACE include)

target_link_libraries(${LIBRARY_NAME} INTERFACE FreeRTOS INTERFACE HydrodriversUART)

add_subdirectory(example)
