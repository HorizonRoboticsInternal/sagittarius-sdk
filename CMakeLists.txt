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

#############################################################
# Options                                                   #
#############################################################

option(INCLUDE_EXAMPLES
    "Should ${PROJECT_NAME} compile the examples." ON)

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

find_package(Boost REQUIRED COMPONENTS system)
find_package(Eigen3 REQUIRED NO_MODULE)

#############################################################
# Targets                                                   #
#############################################################

add_library(sagittarius_sdk SHARED)
target_include_directories(sagittarius_sdk
    PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>)
target_sources(sagittarius_sdk
    PRIVATE
    src/modern_robotics.cpp
    src/sdk_sagittarius_arm_log.cpp
    src/sdk_sagittarius_arm_common.cpp
    src/sdk_sagittarius_arm_common_serial.cpp
    src/sdk_sagittarius_arm_real.cpp)
target_link_libraries(sagittarius_sdk PUBLIC Eigen3::Eigen Boost::system)
    
