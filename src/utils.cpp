// Copyright (c) 2025, Błażej Szargut.
// All rights reserved.
//
// SPDX-License-Identifier: BSD-3-Clause

#include "utils.hpp"

#include "config.hpp"

std::string motionModeToString(const mab::MdMode_E mode) {
    switch (mode) {
        case mab::MdMode_E::IDLE:
            return "IDLE";
        case mab::MdMode_E::POSITION_PID:
            return "POSITION_PID";
        case mab::MdMode_E::VELOCITY_PID:
            return "VELOCITY_PID";
        case mab::MdMode_E::RAW_TORQUE:
            return "RAW_TORQUE";
        case mab::MdMode_E::IMPEDANCE:
            return "IMPEDANCE";
        case mab::MdMode_E::POSITION_PROFILE:
            return "POSITION_PROFILE";
        case mab::MdMode_E::VELOCITY_PROFILE:
            return "VELOCITY_PROFILE";
        default:
            return "UNKNOWN_MODE";
    }
}

namespace Bernard {
std::string mdIdToJointName(const mab::canId_t id) {
    auto it = JOINT_MAP.find(static_cast<int>(id));
    if (it != JOINT_MAP.end()) {
        return it->second;
    } else {
        return "UNKNOWN_JOINT";
    }
}

std::string robotControlModeToString(const RobotControlMode_t mode) {
    switch (mode) {
        case RobotControlMode_t::OFF:
            return "OFF";
        case RobotControlMode_t::MANUAL:
            return "MANUAL";
        case RobotControlMode_t::RL_POLICY:
            return "RL_POLICY";
        case RobotControlMode_t::HOLD_POSITION:
            return "HOLD_POSITION";
        default:
            return "UNKNOWN_CONTROL_MODE";
    }
}
}  // namespace Bernard
