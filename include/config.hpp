#pragma once
#include <string>
#include <unordered_map>

namespace bernard {

/// @brief Map MD CAN IDs to joint names
static const std::unordered_map<int, std::string> JOINT_MAP = {
    {106, "r_hip1_joint"},
    {1105, "l_hip1_joint"},
    {1108, "r_hip2_joint"},
    {107, "l_hip2_joint"},
    {1106, "r_knee_joint"},
    {108, "l_knee_joint"},
};

constexpr std::vector<uint16_t> ALL_CAN_ACTUATOR_IDS = {
    106, 1105, 1108, 107, 1106, 108
};

/// @brief Button indices for joystick messages
constexpr int A_BTN_IDX = 0;
constexpr int B_BTN_IDX = 1;
constexpr int X_BTN_IDX = 2;
constexpr int Y_BTN_IDX = 3;
constexpr int LB_BTN_IDX = 4;
constexpr int RB_BTN_IDX = 5;
constexpr int BACK_BTN_IDX = 6;
constexpr int START_BTN_IDX = 7;

/// @brief D-pad axis indices for joystick messages
constexpr int DPAD_X_AXIS_IDX = 6;
constexpr int DPAD_Y_AXIS_IDX = 7;

/// @brief Axis indices for joystick messages
constexpr int LEFT_STICK_X_AXIS_IDX = 0;
constexpr int LEFT_STICK_Y_AXIS_IDX = 1;
constexpr int RIGHT_STICK_X_AXIS_IDX = 3;
constexpr int RIGHT_STICK_Y_AXIS_IDX = 4;

/// @brief Trigger axis indices for joystick messages
constexpr int LEFT_TRIGGER_AXIS_IDX = 2;
constexpr int RIGHT_TRIGGER_AXIS_IDX = 5;

}  // namespace bernard
