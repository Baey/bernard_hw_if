// Copyright (c) 2025, Błażej Szargut.
// All rights reserved.
//
// SPDX-License-Identifier: BSD-3-Clause

#pragma once

#include <utility>
#include <vector>

#include "MD.hpp"

namespace Bernard {

class IActuatorDriver {
   public:
    virtual ~IActuatorDriver() = default;

    virtual mab::MD::Error_t init() = 0;
    virtual mab::MD::Error_t blink() = 0;
    virtual mab::MD::Error_t zero() = 0;
    virtual mab::MD::Error_t setMotionMode(mab::MdMode_E mode) = 0;
    virtual mab::MD::Error_t setTargetPosition(float position) = 0;
    virtual mab::MD::Error_t setTargetTorque(float torque) = 0;
    virtual std::pair<float, mab::MD::Error_t> getPosition() = 0;
    virtual std::pair<float, mab::MD::Error_t> getVelocity() = 0;
    virtual std::pair<float, mab::MD::Error_t> getTorque() = 0;
    virtual std::pair<float, mab::MD::Error_t> getMosfetTemperature() = 0;
    virtual mab::canId_t getCanId() const = 0;
    virtual mab::MD::Error_t enable() = 0;
    virtual mab::MD::Error_t disable() = 0;
};

class MDActuatorDriver : public IActuatorDriver {
   public:
    /// @brief Wrapper around mab::MD to implement IActuatorDriver interface
    /// @param canId
    /// @param candle
    MDActuatorDriver(mab::canId_t canId, mab::Candle* candle) : _md(canId, candle) {};

    /// @brief Initialize the actuator
    mab::MD::Error_t init() override { return _md.init(); }

    /// @brief Blink the actuator's LED
    mab::MD::Error_t blink() override { return _md.blink(); }

    /// @brief Zero the actuator encoder
    mab::MD::Error_t zero() override { return _md.zero(); }

    /// @brief Set the motion mode of the actuator
    /// @param mode Motion mode
    mab::MD::Error_t setMotionMode(mab::MdMode_E mode) override { return _md.setMotionMode(mode); }

    /// @brief Set the target position of the actuator
    /// @param position Target position [rad]
    mab::MD::Error_t setTargetPosition(float position) override { return _md.setTargetPosition(position); }

    /// @brief Set the target torque of the actuator
    /// @param torque Target torque [Nm]
    mab::MD::Error_t setTargetTorque(float torque) override { return _md.setTargetTorque(torque); }

    /// @brief Get the current position of the actuator
    std::pair<float, mab::MD::Error_t> getPosition() override { return _md.getPosition(); }

    /// @brief Get the current velocity of the actuator
    std::pair<float, mab::MD::Error_t> getVelocity() override { return _md.getVelocity(); }

    /// @brief Get the current torque of the actuator
    std::pair<float, mab::MD::Error_t> getTorque() override { return _md.getTorque(); }

    /// @brief Get the MOSFET temperature of the actuator
    std::pair<float, mab::MD::Error_t> getMosfetTemperature() override {
        mab::MD::Error_t r = _md.readRegisters(_registers.mosfetTemperature);
        return {_registers.mosfetTemperature.value, r};
    }

    /// @brief Get the CAN ID of the actuator
    mab::canId_t getCanId() const override { return _md.m_canId; }

    /// @brief Enable the actuator
    mab::MD::Error_t enable() override { return _md.enable(); }

    /// @brief Disable the actuator
    mab::MD::Error_t disable() override { return _md.disable(); }

   private:
    /// @brief Underlying mab::MD instance
    mab::MD _md;

    /// @brief Register buffer
    mab::MDRegisters_S _registers;
};
}  // namespace Bernard