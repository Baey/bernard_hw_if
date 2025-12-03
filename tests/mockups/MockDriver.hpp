// Copyright (c) 2025, Błażej Szargut.
// All rights reserved.
//
// SPDX-License-Identifier: BSD-3-Clause

#pragma once

#include "drivers.hpp"

class MockMDActuatorDriver : public Bernard::IActuatorDriver {
   public:
    /// @brief Wrapper around mab::MD to implement IActuatorDriver interface
    /// @param canId
    /// @param candle
    MockMDActuatorDriver(
        mab::canId_t canId, mab::Candle* candle, float p = 0.0f, float v = 0.0f, float t = 0.0f, float temp = 0.0f)
        : _mock_can_id(canId), _mock_position(p), _mock_velocity(v), _mock_torque(t), _mock_temperature(temp) {};

    /// @brief Initialize the actuator
    mab::MD::Error_t init() override { return mab::MD::Error_t::OK; }

    /// @brief Blink the actuator's LED
    mab::MD::Error_t blink() override { return mab::MD::Error_t::OK; }

    /// @brief Zero the actuator encoder
    mab::MD::Error_t zero() override { return mab::MD::Error_t::OK; }

    /// @brief Set the motion mode of the actuator
    /// @param mode Motion mode
    mab::MD::Error_t setMotionMode(mab::MdMode_E mode) override { return mab::MD::Error_t::OK; }
    /// @brief Set the target position of the actuator
    /// @param position Target position [rad]
    mab::MD::Error_t setTargetPosition(float position) override { return mab::MD::Error_t::OK; }

    /// @brief Set the target torque of the actuator
    /// @param torque Target torque [Nm]
    mab::MD::Error_t setTargetTorque(float torque) override { return mab::MD::Error_t::OK; }
    /// @brief Get the current position of the actuator
    std::pair<float, mab::MD::Error_t> getPosition() override { return {_mock_position, mab::MD::Error_t::OK}; }

    /// @brief Get the current velocity of the actuator
    std::pair<float, mab::MD::Error_t> getVelocity() override { return {_mock_velocity, mab::MD::Error_t::OK}; }
    /// @brief Get the current torque of the actuator
    std::pair<float, mab::MD::Error_t> getTorque() override { return {_mock_torque, mab::MD::Error_t::OK}; }

    /// @brief Get the MOSFET temperature of the actuator
    std::pair<float, mab::MD::Error_t> getMosfetTemperature() override {
        return {_mock_temperature, mab::MD::Error_t::OK};
    }

    /// @brief Get the CAN ID of the actuator
    mab::canId_t getCanId() const override { return _mock_can_id; }

   private:
    const mab::canId_t _mock_can_id;
    const float _mock_position;
    const float _mock_velocity;
    const float _mock_torque;
    const float _mock_temperature;
};