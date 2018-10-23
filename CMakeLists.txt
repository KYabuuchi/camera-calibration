# cmake needs this line
cmake_minimum_required(VERSION 3.0)

# project name
project(camera-calibration)

# If toplevel
if(CMAKE_SOURCE_DIR STREQUAL CMAKE_CURRENT_LIST_DIR)
    set(TOPLEVEL True)
endif()

if(NOT TOPLEVEL)
    # can use as Library
    include_directories(include)
    file(GLOB_RECURSE CAMERA_CALIBRATION_SOURCES  src/*.cpp)
    add_library(camera-calibration STATIC ${CAMERA_CALIBRATION_SOURCES})

else()
    # OpenCV
    find_package(OpenCV REQUIRED)
    message(STATUS "OpenCV library status:")
    message(STATUS "    version: ${OpenCV_VERSION}")
    message(STATUS "    libraries: ${OpenCV_LIBS}")
    message(STATUS "    include path: ${OpenCV_INCLUDE_DIRS}")

    find_package(Boost COMPONENTS system filesystem REQUIRED)
    if(Boost_FOUND)
        include_directories(SYSTEM ${BOOST_INCLUDE_DIRS})
        message("BOOST_FOUND ${BOOST_ROOT}")
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

    include_directories(${CMAKE_CURRENT_LIST_DIR}/include)
    add_executable(sample test/main.cpp src/calibration.cpp)
    target_link_libraries(
        sample 
        ${Boost_LIBRARIES}
        boost_filesystem
        ${OpenCV_LIBS})
    message(STATUS "'YOU CAN BUILD SAMPLE'")

    # Clean
    add_custom_target(cmake-clean
        COMMAND rm -rf `find ${CMAKE_BINARY_DIR} -name \"*[cC][mM]ake*\" -and -not -name \"CMakeLists.txt\"`
        COMMAND rm -rf `find ${CMAKE_BINARY_DIR} -name \"Makefile\"`)

    add_custom_target(full-clean
        COMMAND make clean
        COMMAND make cmake-clean)

endif()
