// Copyright (c) 2025, Błażej Szargut.
// All rights reserved.
//
// SPDX-License-Identifier: BSD-3-Clause

#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include <atomic>
#include <thread>

#include "I_communication_interface_mock.hpp"
#include "MD.hpp"
#include "MockResponder.hpp"
#include "actuators_node.hpp"
#include "config.hpp"
#include "rclcpp/rclcpp.hpp"

using ::testing::_;
using ::testing::Invoke;
using ::testing::Return;

class ActuatorsNodeTest : public ::testing::Test {
   public:
    static void SetUpTestSuite() {
        int argc = 0;
        rclcpp::init(argc, nullptr);
    }

    static void TearDownTestSuite() {
        rclcpp::shutdown();
    }
};

TEST_F(ActuatorsNodeTest, ConstructorStartStopWorker) {
    std::unique_ptr<mab::Candle> candle = nullptr;
    std::vector<mab::MD> mds{};

    {
        auto node = std::make_shared<Bernard::ActuatorsControlNode>(std::move(candle), std::move(mds));
        rclcpp::spin_some(node);
    }
    SUCCEED();
}

TEST_F(ActuatorsNodeTest, CommandQueueingWithMockBus) {
    auto mock = new MockBus();
    std::unique_ptr<mab::I_CommunicationInterface> bus_ptr(mock);

    EXPECT_CALL(*mock, connect()).WillOnce(Return(mab::I_CommunicationInterface::OK));

    auto responder = std::make_shared<MockResponder>();
    responder->setAutoPadResponses(true);

    // WRITE register legacy -> small success frame used by upstream tests
    responder->addPredicateResponse(
        [](const std::vector<uint8_t>& d) { return !d.empty() && d[0] == static_cast<uint8_t>(mab::MdFrameId_E::WRITE_REGISTER_LEGACY); },
        std::vector<uint8_t>{0x04, 0x01, static_cast<uint8_t>(mab::MdFrameId_E::RESPONSE_LEGACY), 0x00},
        mab::I_CommunicationInterface::OK);

    // READ register -> minimal read header; MockResponder will pad to expected size
    responder->addPredicateResponse(
        [](const std::vector<uint8_t>& d) { return !d.empty() && d[0] == static_cast<uint8_t>(mab::MdFrameId_E::READ_REGISTER); },
        std::vector<uint8_t>{0x04, 0x01, static_cast<uint8_t>(mab::MdFrameId_E::READ_REGISTER), 0x00},
        mab::I_CommunicationInterface::OK);

    // Forward gmock calls to the responder (3-arg transfer). Use EXPECT_CALL(...).WillRepeatedly
    // so calls are not treated as "uninteresting" by gmock.
    EXPECT_CALL(*mock, transfer(_, _, _))
        .WillRepeatedly(Invoke([responder](std::vector<u8> data, const u32 timeoutMs, const size_t expectedReceivedDataSize) {
            return responder->handleTransfer(data, timeoutMs, expectedReceivedDataSize);
        }));

    // 2-arg transfer -> no response expected
    EXPECT_CALL(*mock, transfer(_, _))
        .WillRepeatedly(Invoke([responder](std::vector<u8> data, const u32 timeoutMs) {
            return responder->handleTransferNoResponse(data, timeoutMs);
        }));

    EXPECT_CALL(*mock, disconnect()).Times(::testing::AtLeast(0)).WillRepeatedly(Return(mab::I_CommunicationInterface::OK));

    auto candle = std::make_unique<mab::Candle>(mab::CAN_DATARATE_1M, std::move(bus_ptr));
    ASSERT_TRUE(candle != nullptr);
    EXPECT_EQ(candle->init(), mab::candleTypes::Error_t::OK);

    std::vector<mab::MD> mds;
    const size_t n_mds = Bernard::ACTUATORS_NUM;
    for (size_t i = 0; i < n_mds; ++i) {
        mab::canId_t id = Bernard::ALL_CAN_ACTUATOR_IDS[i];
        mds.emplace_back(id, candle.get());
    }

    auto node = std::make_shared<Bernard::ActuatorsControlNode>(std::move(candle), std::move(mds), Bernard::ActuatorsControlNodeMode_t::FULL);

    auto pub_node = std::make_shared<rclcpp::Node>("test_actions_pub");
    auto pub = pub_node->create_publisher<std_msgs::msg::Float32MultiArray>("actions", 10);

    node->setRobotControlMode(Bernard::RobotControlMode_t::RL_POLICY);

    const size_t msg_count = 3;
    for (size_t m = 0; m < msg_count; ++m) {
        std_msgs::msg::Float32MultiArray msg;
        msg.data.resize(n_mds);
        for (size_t i = 0; i < n_mds; ++i) msg.data[i] = static_cast<float>(m + i + 1);
        pub->publish(msg);
    }

    const int expected = static_cast<int>(msg_count * n_mds);
    // Wait deterministically for observed transfers
    bool success = responder->waitForTransfers(static_cast<size_t>(expected), std::chrono::seconds(4));
    EXPECT_TRUE(success) << "Expected at least " << expected << " transfers, got " << responder->observedTransfers();
}

int main(int argc, char** argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
