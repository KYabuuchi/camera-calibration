cmake_minimum_required(VERSION 3.10)
project(camera-calibration)

# Set default build type to Release
if(NOT DEFINED ${CMAKE_BUILD_TYPE})
    set(CMAKE_BUILD_TYPE Release)
endif()

# If toplevel
if(CMAKE_SOURCE_DIR STREQUAL CMAKE_CURRENT_LIST_DIR)
    set(TOPLEVEL True)
endif()

# OpenCV
find_package(OpenCV 3.0 REQUIRED)

# header & source
include_directories(${CMAKE_CURRENT_LIST_DIR}/include)
file(GLOB CALIBRATION_SOURCES ${CMAKE_CURRENT_LIST_DIR}/src/*.cpp)

if(NOT TOPLEVEL)
    add_library(camera-calibration STATIC ${CALIBRATION_SOURCES})
else()
    # Check C++17
    include(CheckCXXCompilerFlag)
    message("Using C++17")
    enable_language(CXX)
    check_cxx_compiler_flag("-std=gnu++17" COMPILER_SUPPORTS_CXX17)
    if(NOT ${COMPILER_SUPPORTS_CXX17})
        message(
            FATAL_ERROR
                "${CMAKE_CXX_COMPILER} はC++17をサポートしてません。C++17に対応したコンパイラを指定してください。\n"
            )
    endif()
    set(CMAKE_CXX_STANDARD 17) # C++17
    message("Compiler:\n\t${CMAKE_CXX_COMPILER}")

    # Set warning flags
    set(CXX_WARNING_FLAGS
        -Wall
        -Wextra
        -Wconversion
        -Wswitch-default
        -Wdisabled-optimization
        -Wformat
        -Winit-self
        -Woverloaded-virtual
        -Wfloat-equal
        -Wno-old-style-cast
        -Wno-pragmas)
    foreach(FLAG IN LISTS CXX_WARNING_FLAGS)
        set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${FLAG}")
    endforeach()
    message("Build type:\n\t${CMAKE_BUILD_TYPE}")

    # =====SAMPLES=====
    add_executable(monocular sample/monocular.cpp ${CALIBRATION_SOURCES})
    target_link_libraries(monocular ${OpenCV_LIBS})

    add_executable(stereo sample/stereo.cpp ${CALIBRATION_SOURCES})
    target_link_libraries(stereo ${OpenCV_LIBS})

    add_executable(record-webcam sample/record_webcam.cpp)
    target_link_libraries(record-webcam ${OpenCV_LIBS} stdc++fs)
endif()
