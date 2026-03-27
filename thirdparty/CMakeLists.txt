set(FETCHCONTENT_BASE_DIR "${CMAKE_SOURCE_DIR}/.deps" CACHE 
    PATH "Base directory for CMake FetchContent downloads")

add_subdirectory(FreeRTOS)

include(FetchContent)

FetchContent_Declare(
    hydrodrivers
    GIT_REPOSITORY https://github.com/SeaJackal/Hydrodrivers.git
    GIT_TAG 631813004b1742879e637e0468465f7a7261c4bd
)

FetchContent_MakeAvailable(hydrodrivers)

set(hydrodrivers_SOURCE_DIR "${hydrodrivers_SOURCE_DIR}" PARENT_SCOPE)
