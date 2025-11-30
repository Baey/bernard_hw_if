# Copyright (c) 2025, Błażej Szargut.
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

cmake_minimum_required(VERSION 3.8)
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

project(bernard_hw_if)

set(SRC_DIR ${CMAKE_CURRENT_SOURCE_DIR}/src)
set(INCLUDE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/include)
set(TESTS_DIR ${CMAKE_CURRENT_SOURCE_DIR}/tests)

if(WIN32)
    message(FATAL_ERROR "BERNARD Actuators Node should be build on Linux.")
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic
        -Wno-unused-parameter -Wno-missing-field-initializers)
endif()

message(STATUS "############################################")
message(STATUS "#")
message(STATUS "# Building BERNARD Actuators Control Node")
message(STATUS "#")
message(STATUS "# ===== System info ========================")
message(STATUS "# System:          ${CMAKE_SYSTEM_NAME} ${CMAKE_SYSTEM_VERSION}")
message(STATUS "# CPU ARCH:        ${CMAKE_SYSTEM_PROCESSOR}")
message(STATUS "# Compiler C:      ${CMAKE_C_COMPILER}")
message(STATUS "# Compiler CXX:    ${CMAKE_CXX_COMPILER}")
message(STATUS "# Compiler ver:    ${CMAKE_CXX_COMPILER_VERSION}")
message(STATUS "# Build type:      ${CMAKE_BUILD_TYPE}")
message(STATUS "# =========================================\n")

# CMake options
option(BERNARD_NODE_BUILD_SHARED "Build actuator node as a shared library shared object" ON)
set(CANDLESDK_BUILD_PYTHON OFF CACHE BOOL "Build python library shared object")
set(CANDLESDK_BUILD_EXAMPLES OFF CACHE BOOL "Build examples (has no effect on python builds)")
set(CANDLESDK_BUILD_CANDLETOOL OFF CACHE BOOL "Build candletool executable")
set(MAKE_TESTS OFF CACHE BOOL "Enable/disable some of the unit tests")
set(CANDLE_BUILD_STATIC OFF CACHE BOOL "Build static library (has no effect on windows or python builds)")

# find dependencies
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(rosidl_default_generators REQUIRED)

set(CMAKE_INSTALL_RPATH $ORIGIN)

set(ORIGINAL_CMAKE_MESSAGE_FUNCTION "message")
message(STATUS "# Building CANdle-SDK")
function(message)

endfunction()

# Build CANdle library
add_subdirectory(CANdle-SDK)

set(message "${ORIGINAL_CMAKE_MESSAGE_FUNCTION}")

# Build BERNARD ActuatorsControlNode
set(BERNARD_ACTUATORS_CONTROL_NODE_SOURCES
    ${SRC_DIR}/actuators_node.cpp
    ${SRC_DIR}/utils.cpp
)
if(BERNARD_NODE_BUILD_SHARED)
	add_library(bernard_actuators_control_node
        SHARED ${BERNARD_ACTUATORS_CONTROL_NODE_SOURCES}
    )
    message(STATUS "\n# BERNARD Actuators Control Node will be delivered as shared library\n")
else()
    add_library(bernard_actuators_control_node
        STATIC ${BERNARD_ACTUATORS_CONTROL_NODE_SOURCES}
    )
    message("\n# BERNARD Actuators Control Node will be delivered as static library\n")
endif()
target_link_libraries(bernard_actuators_control_node candle)
target_include_directories(bernard_actuators_control_node PUBLIC 
    $<BUILD_INTERFACE:${INCLUDE_DIR}>
    $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(bernard_actuators_control_node 
    rclcpp
    std_msgs
    sensor_msgs
)

# Build node executables
set(BERNARD_EXECUTABLES
    actuators_full_if
)
message("# Building executables:")
if (CMAKE_BUILD_TYPE STREQUAL "Debug")
    # PUB ONLY NODE
    message(STATUS "#   - actuators_pub_only_if")
    add_executable(actuators_pub_only_if ${SRC_DIR}/bernard_actuators_if.cpp)
    target_compile_definitions(actuators_pub_only_if PRIVATE PUB_ONLY_NODE)
    target_link_libraries(actuators_pub_only_if
        bernard_actuators_control_node
        candle
    )
    ament_target_dependencies(actuators_pub_only_if 
        rclcpp
        std_msgs
        sensor_msgs
    )
    list(APPEND BERNARD_EXECUTABLES actuators_pub_only_if)
    # PUB WITH JOY NODE
    message(STATUS "#   - actuators_pub_with_joy_if")
    add_executable(actuators_pub_with_joy_if ${SRC_DIR}/bernard_actuators_if.cpp)
    target_compile_definitions(actuators_pub_with_joy_if PRIVATE PUB_WITH_JOY_NODE)
    target_link_libraries(actuators_pub_with_joy_if
        bernard_actuators_control_node
        candle
    )
    ament_target_dependencies(actuators_pub_with_joy_if 
        rclcpp
        std_msgs
        sensor_msgs
    )
    list(APPEND BERNARD_EXECUTABLES actuators_pub_with_joy_if)
endif()

# FULL NODE
message(STATUS "#   - actuators_full_if")
add_executable(actuators_full_if ${SRC_DIR}/bernard_actuators_if.cpp)
target_compile_definitions(actuators_full_if PRIVATE FULL_NODE)
target_link_libraries(actuators_full_if
    bernard_actuators_control_node
    candle
)
ament_target_dependencies(actuators_full_if 
    rclcpp
    std_msgs
    sensor_msgs
)

install(TARGETS
    ${BERNARD_EXECUTABLES}
    bernard_actuators_control_node
    DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
    message(STATUS "\n# Building BERNARD tests")

    find_package(ament_cmake_gtest REQUIRED)
    find_package(GTest REQUIRED)

    add_executable(test_actuators_node
        ${TESTS_DIR}/test_actuators_node.cpp
        ${TESTS_DIR}/mockups/MockResponder.cpp
    )

    target_include_directories(test_actuators_node PUBLIC
        $<BUILD_INTERFACE:${TESTS_DIR}/mockups>
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/CANdle-SDK/commons/shared_data>
    )

    target_link_libraries(test_actuators_node
        bernard_actuators_control_node
        candle
        GTest::gmock_main
    )

    find_package(ament_lint_auto REQUIRED)
    ament_lint_auto_find_test_dependencies()

    ament_target_dependencies(test_actuators_node
        rclcpp
        std_msgs
        sensor_msgs
    )

    ament_add_test(
        test_actuators_node_test
        COMMAND test_actuators_node
        OUTPUT_ON_FAILURE
    )
endif()

ament_package()

message(STATUS "############################################")
