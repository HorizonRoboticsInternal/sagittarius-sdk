cmake_minimum_required(VERSION 3.17)

project(
    sagittarius_sdk
    VERSION 2022.05.23
    LANGUAGES C CXX
    HOMEPAGE_URL "https://github.com/HorizonRoboticsInternal/sagittarius-sdk")

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pthread")
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

# By default build the release version with O3
if (NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif()
set(CMAKE_CXX_FLAGS_RELEASE "-O3")

message(STATUS "Using compiler ${CMAKE_CXX_COMPILER_ID} ...")

#############################################################
# Options                                                   #
#############################################################

option(INCLUDE_EXAMPLES
    "Should ${PROJECT_NAME} compile the examples." ON)

option(BUILD_DAEMON
    "Should ${PROJECT_NAME} compile the daemon service." OFF)

#############################################################
# Detect Architecture                                       #
#############################################################

if (CMAKE_SYSTEM_PROCESSOR MATCHES "(x86)|(X86)|(amd64)|(AMD64)")
    set(ARCHITECTURE "amd64")
elseif (CMAKE_SYSTEM_PROCESSOR MATCHES "(arm)|(ARM)|(arm64)|(ARM64)|(aarch64)|(AARCH64)")
    set(ARCHITECTURE "arm64")
else()
    message(FATAL_ERROR "Cannot detect architecture of ${CMAKE_SYSTEM_PROCESSOR}")
endif()
message(STATUS "Architecture detected as ${ARCHITECTURE}")

#############################################################
# Dependencies                                              #
#############################################################

include_directories(include)

find_package(Boost REQUIRED COMPONENTS system thread)
find_package(Eigen3 REQUIRED NO_MODULE)

#############################################################
# Targets                                                   #
#############################################################

add_library(sdk_sagittarius_arm_constants INTERFACE)
target_sources(
    sdk_sagittarius_arm_constants
    PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_LIST_DIR}/include/sdk_sagittarius_arm/sdk_sagittarius_arm_constants.h>
    $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}/sdk_sagittarius_arm/sdk_sagittarius_arm_constants.h>)

add_library(modern_robotics STATIC)
target_sources(modern_robotics
    PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_LIST_DIR}/include/sdk_sagittarius_arm/modern_robotics.h>
    $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}/sdk_sagittarius_arm/modern_robotics.h>
    PRIVATE src/modern_robotics.cpp)
target_link_libraries(modern_robotics PUBLIC Eigen3::Eigen)

add_library(sdk_sagittarius_arm_log STATIC)
target_sources(sdk_sagittarius_arm_log
    PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_LIST_DIR}/include/sdk_sagittarius_arm/sdk_sagittarius_arm_log.h>
    $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}/sdk_sagittarius_arm/sdk_sagittarius_arm_log.h>
    PRIVATE src/sdk_sagittarius_arm_log.cpp)
target_link_libraries(sdk_sagittarius_arm_log PUBLIC sdk_sagittarius_arm_constants)

add_library(sdk_sagittarius_arm_common STATIC)
target_sources(sdk_sagittarius_arm_common
    PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_LIST_DIR}/include/sdk_sagittarius_arm/sdk_sagittarius_arm_common.h>
    $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}/sdk_sagittarius_arm/sdk_sagittarius_arm_common.h>
    PRIVATE src/sdk_sagittarius_arm_common.cpp)
target_link_libraries(sdk_sagittarius_arm_common
    PUBLIC sdk_sagittarius_arm_constants sdk_sagittarius_arm_log Boost::system Boost::thread)

add_library(sdk_sagittarius_arm_common_serial STATIC)
target_sources(sdk_sagittarius_arm_common_serial
    PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_LIST_DIR}/include/sdk_sagittarius_arm/sdk_sagittarius_arm_common_serial.h>
    $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}/sdk_sagittarius_arm/sdk_sagittarius_arm_common_serial.h>
    PRIVATE src/sdk_sagittarius_arm_common_serial.cpp)
target_link_libraries(sdk_sagittarius_arm_common_serial
    PUBLIC
    Boost::system
    sdk_sagittarius_arm_constants
    sdk_sagittarius_arm_common
    sdk_sagittarius_arm_log)

add_library(sagittarius_sdk SHARED)
target_sources(sagittarius_sdk
    PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_LIST_DIR}/include/sdk_sagittarius_arm/sdk_sagittarius_arm_real.h>
    $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}/sdk_sagittarius_arm/sdk_sagittarius_arm_real.h>
    PRIVATE src/sdk_sagittarius_arm_real.cpp)
target_link_libraries(sagittarius_sdk
    PUBLIC
    modern_robotics
    sdk_sagittarius_arm_common_serial
    sdk_sagittarius_arm_log)

if (INCLUDE_EXAMPLES)
    add_subdirectory(examples)
endif()

if (BUILD_DAEMON)
    add_subdirectory(src/daemon)
endif()


#############################################################
# Python Bindings                                           #
#############################################################

find_package(Python3 COMPONENTS Interpreter Development.Module REQUIRED)
find_package(pybind11 CONFIG REQUIRED)

pybind11_add_module(pysagittarius python/pysagittarius.cc)
target_link_libraries(pysagittarius PRIVATE sagittarius_sdk)

