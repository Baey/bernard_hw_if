// Copyright (c) 2025, Błażej Szargut.
// All rights reserved.
//
// SPDX-License-Identifier: BSD-3-Clause

#include "MD.hpp"
#include "actuators_node.hpp"
#include "drivers.hpp"
#include "candle.hpp"
#include "config.hpp"
#include "utils.hpp"

using Bernard::ActuatorsControlNodeMode_t;

#ifdef FULL_NODE
constexpr ActuatorsControlNodeMode_t NODE_MODE = ActuatorsControlNodeMode_t::FULL;
#elif defined(PUB_ONLY_NODE)
constexpr ActuatorsControlNodeMode_t NODE_MODE = ActuatorsControlNodeMode_t::PUB_ONLY;
#elif defined(PUB_WITH_JOY_NODE)
constexpr ActuatorsControlNodeMode_t NODE_MODE = ActuatorsControlNodeMode_t::PUB_WITH_JOY;
#else
constexpr ActuatorsControlNodeMode_t NODE_MODE = ActuatorsControlNodeMode_t::PUB_ONLY;
#endif

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);

    // Initialize CANdle instance
    auto candle = std::unique_ptr<mab::Candle>(
        mab::attachCandle(mab::CANdleDatarate_E::CAN_DATARATE_1M, mab::candleTypes::busTypes_t::USB));

    // Discover and initialize MD actuators
    std::vector<std::unique_ptr<Bernard::IActuatorDriver>> mds;
    for (const auto& id : mab::MD::discoverMDs(candle.get())) {
        RCLCPP_INFO(rclcpp::get_logger("ActuatorsNode"), "Found MD with CAN ID: %s (%u)", Bernard::mdIdToJointName(id).c_str(), id);
        mds.emplace_back(std::make_unique<Bernard::MDActuatorDriver>(id, candle.get()));
        mds.back()->init();
    }

    if (mds.size() != Bernard::ACTUATORS_NUM) {
        RCLCPP_ERROR(rclcpp::get_logger("ActuatorsNode"), "Expected %u actuators, but found %zu. ActuatorsNode will not start.", Bernard::ACTUATORS_NUM, mds.size());
        return 1;
    }

    // Create ActuatorsControlNode
    auto node = std::make_shared<Bernard::ActuatorsControlNode>(std::move(candle), std::move(mds), NODE_MODE);

    // Spin the node
    rclcpp::spin(node);

    rclcpp::shutdown();
    return 0;
}