// Copyright (c) 2025, Błażej Szargut.
// All rights reserved.
//
// SPDX-License-Identifier: BSD-3-Clause

#pragma once

#include <gmock/gmock.h>

#include "drivers.hpp"

class MockMDActuatorDriver : public Bernard::IActuatorDriver {
   public:
    /// @brief Wrapper around mab::MD to implement IActuatorDriver interface
    /// @param canId
    /// @param candle
    MockMDActuatorDriver(mab::canId_t canId, mab::Candle* candle, float p = 0.0f, float v = 0.0f, float t = 0.0f, float temp = 0.0f);

    /// @brief Initialize the actuator
    MOCK_METHOD(mab::MD::Error_t, init, (), (override));

    /// @brief Blink the actuator's LED
    MOCK_METHOD(mab::MD::Error_t, blink, (), (override));

    /// @brief Zero the actuator encoder
    MOCK_METHOD(mab::MD::Error_t, zero, (), (override));

    /// @brief Set the motion mode of the actuator
    /// @param mode Motion mode
    MOCK_METHOD(mab::MD::Error_t, setMotionMode, (mab::MdMode_E mode), (override));

    /// @brief Set the target position of the actuator
    /// @param position Target position [rad]
    MOCK_METHOD(mab::MD::Error_t, setTargetPosition, (float position), (override));

    /// @brief Set the target torque of the actuator
    /// @param torque Target torque [Nm]
    MOCK_METHOD(mab::MD::Error_t, setTargetTorque, (float torque), (override));

    /// @brief Get the current position of the actuator
    MOCK_METHOD((std::pair<float, mab::MD::Error_t>), getPosition, (), (override));

    /// @brief Get the current velocity of the actuator
    MOCK_METHOD((std::pair<float, mab::MD::Error_t>), getVelocity, (), (override));

    /// @brief Get the current torque of the actuator
    MOCK_METHOD((std::pair<float, mab::MD::Error_t>), getTorque, (), (override));

    /// @brief Get the MOSFET temperature of the actuator
    MOCK_METHOD((std::pair<float, mab::MD::Error_t>), getMosfetTemperature, (), (override));

    /// @brief Get the CAN ID of the actuator
    mab::canId_t getCanId() const override { return _mock_can_id; }

   private:
    const mab::canId_t _mock_can_id;
    const float _mock_position;
    const float _mock_velocity;
    const float _mock_torque;
    const float _mock_temperature;
};