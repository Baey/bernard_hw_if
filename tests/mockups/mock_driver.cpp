// Copyright (c) 2025, Błażej Szargut.
// All rights reserved.
//
// SPDX-License-Identifier: BSD-3-Clause

#include "mock_driver.hpp"

#include <gmock/gmock.h>

using ::testing::_;
using ::testing::AtLeast;
using ::testing::Invoke;
using ::testing::Return;

MockMDActuatorDriver::MockMDActuatorDriver(
    mab::canId_t canId, mab::Candle* candle, float p, float v, float t, float temp)
    : _mock_can_id(canId), _mock_position(p), _mock_velocity(v), _mock_torque(t), _mock_temperature(temp) {
    // Default implementations for all methods
    ON_CALL(*this, init()).WillByDefault(Return(mab::MD::Error_t::OK));
    ON_CALL(*this, blink()).WillByDefault(Return(mab::MD::Error_t::OK));
    ON_CALL(*this, zero()).WillByDefault(Return(mab::MD::Error_t::OK));
    ON_CALL(*this, setMotionMode(::testing::_)).WillByDefault(Return(mab::MD::Error_t::OK));
    ON_CALL(*this, setTargetPosition(::testing::_)).WillByDefault(Return(mab::MD::Error_t::OK));
    ON_CALL(*this, setTargetTorque(::testing::_)).WillByDefault(Return(mab::MD::Error_t::OK));
    ON_CALL(*this, getPosition()).WillByDefault(Return(std::make_pair(_mock_position, mab::MD::Error_t::OK)));
    ON_CALL(*this, getVelocity()).WillByDefault(Return(std::make_pair(_mock_velocity, mab::MD::Error_t::OK)));
    ON_CALL(*this, getTorque()).WillByDefault(Return(std::make_pair(_mock_torque, mab::MD::Error_t::OK)));
    ON_CALL(*this, getMosfetTemperature()).WillByDefault(Return(std::make_pair(_mock_temperature, mab::MD::Error_t::OK)));
}