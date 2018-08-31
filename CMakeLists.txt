# cmake needs this line
cmake_minimum_required(VERSION 3.0)

# project name
project(camera-calibration)

find_package(OpenCV REQUIRED)

message(STATUS "OpenCV library status:")
message(STATUS "    version: ${OpenCV_VERSION}")
message(STATUS "    libraries: ${OpenCV_LIBS}")
message(STATUS "    include path: ${OpenCV_INCLUDE_DIRS}")

if(CMAKE_VERSION VERSION_LESS "2.8.11")
  # Add OpenCV headers location to your include paths
  include_directories(${OpenCV_INCLUDE_DIRS})
endif()


# COMPILER
include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=gnu++14" COMPILER_SUPPORTS_CXX14)
if(COMPILER_SUPPORTS_CXX14)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=gnu++14")
else()
    message(FATAL_ERROR
        "${CMAKE_CXX_COMPILER} C++14 is not supported. \n"
        "EXAMPLE $ cmake -DCMAKE_CXX_COMPILER=/usr/bin/g++-5 ..")
endif()
message("Compiler:\n\t${CMAKE_CXX_COMPILER}")

# Set default build type to Release
if(CMAKE_BUILD_TYPE STREQUAL "")
    set(CMAKE_BUILD_TYPE Release)
endif()

# Set compile flags
set(CMAKE_CXX_FLAGS
    "${CMAKE_CXX_FLAGS} -Wall -Wextra -Wconversion -Wswitch-default -Wdisabled-optimization -Wformat -Winit-self -Woverloaded-virtual -Wfloat-equal")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wno-old-style-cast -Wno-pragmas")
set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -O0")

# If toplevel
if(CMAKE_SOURCE_DIR STREQUAL CMAKE_CURRENT_LIST_DIR)
    set(TOPLEVEL True)
endif()

# Declare the executable target built from your sources
# Link your application with OpenCV libraries
if(TOPLEVEL)
    include_directories(${CMAKE_CURRENT_LIST_DIR})
    add_executable(sample test/main.cpp calibration/calibration.cpp)
    target_link_libraries(sample ${OpenCV_LIBS})
    message(STATUS "'YOU CAN BUILD SAMPLE'")

# Clean
    add_custom_target(cmake-clean
        COMMAND rm -rf `find ${CMAKE_BINARY_DIR} -name \"*[cC][mM]ake*\" -and -not -name \"CMakeLists.txt\"`
        COMMAND rm -rf `find ${CMAKE_BINARY_DIR} -name \"Makefile\"`)

    add_custom_target(full-clean
        COMMAND make clean
        COMMAND make cmake-clean)
else()
    # can use as Library
    include_directories(include)
    file(GLOB_RECURSE CAMERA_CALIBRATION_SOURCES  src/*.cpp)
    add_library(camera-calibration STATIC ${CAMERA_CALIBRATION_SOURCES})
endif()
