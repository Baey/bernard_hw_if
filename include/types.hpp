// Copyright (c) 2025, Błażej Szargut.
// All rights reserved.
//
// SPDX-License-Identifier: BSD-3-Clause

#pragma once

namespace Bernard {

/// @brief Control modes for the robot
enum class RobotControlMode_t {
    OFF,
    MANUAL,
    RL_POLICY,
    HOLD_POSITION,
    ZERO_ENCODERS
};

/// @brief Structure to hold MD state information
struct ActuatorState {
    /// @brief Position in **rad**
    float position = 0.0f;
    /// @brief Velocity in **rad/s**
    float velocity = 0.0f;
    /// @brief Torque in **Nm**
    float torque = 0.0f;
    /// @brief Temperature in **°C**
    float temperature = 0.0f;
};

/// @brief Modes for ActuatorsControlNode
enum class ActuatorsControlNodeMode_t {
    FULL,
    PUB_ONLY,
    PUB_WITH_JOY
};

}  // namespace Bernard