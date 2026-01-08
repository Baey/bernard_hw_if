// Copyright (c) 2025, Błażej Szargut.
// All rights reserved.
//
// SPDX-License-Identifier: BSD-3-Clause

#pragma once

#include <string>
#include <unordered_map>

namespace Bernard {
// ========= Platform Specific Configuration ======== //
// Initial BERNARD platform specific configuration parameters.
// These parameters should be adjusted according to the specific hardware setup. 

/// @brief Number of actuators in the robot
constexpr uint8_t ACTUATORS_NUM = 6;

/// @brief Interval for blinking LEDs during zeroing routine
constexpr std::chrono::milliseconds ZEROING_BLINK_INTERVAL = std::chrono::milliseconds(1500);

/// @brief Interval for blinking LEDs during manual selection
constexpr std::chrono::milliseconds MANUAL_SELECTION_BLINK_INTERVAL = std::chrono::milliseconds(2500);

/// @brief Map MD CAN IDs to joint names
static const std::unordered_map<int, std::string> JOINT_MAP = {
    {106, "r_hip1_joint"},
    {1105, "l_hip1_joint"},
    {1108, "r_hip2_joint"},
    {107, "l_hip2_joint"},
    {1106, "r_knee_joint"},
    {108, "l_knee_joint"},
};

/// @brief Error threshold for CAN communication before terminating node
constexpr size_t CAN_ERROR_THRESHOLD = 10;

/// @brief All CAN IDs of actuators in the robot
static const std::vector<uint16_t> ALL_CAN_ACTUATOR_IDS = {
    106, 1105, 1108, 107, 1106, 108};

// ========== Joystick Configuration ========== //
// Joystick button and axis mappings for Xbox Series controller.

/// @brief Button indices for joystick messages
constexpr int A_BTN_IDX = 0;
constexpr int B_BTN_IDX = 1;
constexpr int X_BTN_IDX = 3;
constexpr int Y_BTN_IDX = 4;
constexpr int LB_BTN_IDX = 6;
constexpr int RB_BTN_IDX = 7;
constexpr int BACK_BTN_IDX = 10;
constexpr int START_BTN_IDX = 11;

/// @brief D-pad axis indices for joystick messages
constexpr int DPAD_X_AXIS_IDX = 6;
constexpr int DPAD_Y_AXIS_IDX = 7;

/// @brief Axis indices for joystick messages
constexpr int LEFT_STICK_X_AXIS_IDX = 0;
constexpr int LEFT_STICK_Y_AXIS_IDX = 1;
constexpr int RIGHT_STICK_X_AXIS_IDX = 3;
constexpr int RIGHT_STICK_Y_AXIS_IDX = 4;

/// @brief Trigger axis indices for joystick messages
constexpr int RIGHT_TRIGGER_AXIS_IDX = 5;
constexpr int LEFT_TRIGGER_AXIS_IDX = 6;

// ========== Node Configuration ========== //
/// @brief Node name
const std::string NODE_NAME = "ActuatorsControlNode";
const std::string LOGGER_NAME = "ActuatorsControlNode";
const std::string POLICY_ACTIONS_TOPIC_NAME = "actions";
const std::string JOINT_STATES_TOPIC_NAME = "joint_states";
const std::string DRIVER_TEMPERATURES_TOPIC_NAME = "driver_temperatures";
const std::string JOYSTICK_TOPIC_NAME = "joy";
constexpr float JOINT_STATE_PUBLISH_RATE_HZ = 100.0f;
constexpr float JOINT_MOSFET_TEMP_PUBLISH_RATE_HZ = 1.0f;


}  // namespace Bernard
