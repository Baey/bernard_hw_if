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
#include "actuators_node.hpp"
#include "config.hpp"
#include "gtest_vector_utils.hpp"
#include "mock_driver.hpp"
#include "mock_responder.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "test_nodes.hpp"
#include "utils.hpp"

using ::testing::_;
using ::testing::AtLeast;
using ::testing::Invoke;
using ::testing::Return;

const std::vector<u8> mockResponse = {0x0,  // header
                                      0x01,
                                      0xA0,  // payload
                                      0x00};

class ActuatorsNodeTest : public ::testing::Test {
   public:
    static void SetUpTestSuite() {
        int argc = 0;
        rclcpp::init(argc, nullptr);
        rclcpp::Logger logger = rclcpp::get_logger(Bernard::LOGGER_NAME);
        logger.set_level(rclcpp::Logger::Level::Info);
    }

    static void TearDownTestSuite() {
        rclcpp::shutdown();
    }
};

TEST_F(ActuatorsNodeTest, ConstructorStartStopWorker) {
    rclcpp::Logger logger = rclcpp::get_logger(Bernard::LOGGER_NAME);
    logger.set_level(rclcpp::Logger::Level::Error);

    auto m_bus = std::make_unique<MockBus>();
    MockBus* m_debugBus = m_bus.get();

    CANDLE_MOCK_RESPONDER_INIT(m_debugBus, mockResponse);

    auto m_candle = std::unique_ptr<mab::Candle>(
        mab::attachCandle(mab::CANdleDatarate_E::CAN_DATARATE_1M, std::move(m_bus)));
    std::vector<std::unique_ptr<Bernard::IActuatorDriver>> mds{};

    for (size_t i = 0; i < Bernard::ACTUATORS_NUM; ++i) {
        mds.emplace_back(std::make_unique<MockMDActuatorDriver>(Bernard::ALL_CAN_ACTUATOR_IDS[i], m_candle.get()));
    }

    EXPECT_CALL_ALL_TIMES(mds, MockMDActuatorDriver, getMosfetTemperature(), ::testing::AtMost(1));
    EXPECT_CALL_ALL_TIMES(mds, MockMDActuatorDriver, getTorque(), ::testing::AtMost(1));
    EXPECT_CALL_ALL_TIMES(mds, MockMDActuatorDriver, getPosition(), ::testing::AtMost(1));
    EXPECT_CALL_ALL_TIMES(mds, MockMDActuatorDriver, getVelocity(), ::testing::AtMost(1));

    {
        auto node = std::make_shared<Bernard::ActuatorsControlNode>(std::move(m_candle), std::move(mds));
        rclcpp::spin_some(node);
    }

    SUCCEED();
}

TEST_F(ActuatorsNodeTest, InsufficientMDsThrows) {
    rclcpp::Logger logger = rclcpp::get_logger(Bernard::LOGGER_NAME);
    logger.set_level(rclcpp::Logger::Level::Fatal);

    std::unique_ptr<mab::Candle> candle = nullptr;
    std::vector<std::unique_ptr<Bernard::IActuatorDriver>> mds{};

    for (size_t i = 0; i < Bernard::ACTUATORS_NUM - 1; ++i) {
        mds.emplace_back(std::make_unique<Bernard::MDActuatorDriver>(106 + static_cast<mab::canId_t>(i), candle.get()));
    }

    EXPECT_THROW({ auto node = std::make_shared<Bernard::ActuatorsControlNode>(std::move(candle), std::move(mds)); }, std::runtime_error);
}

TEST_F(ActuatorsNodeTest, WrongCANIdThrows) {
    rclcpp::Logger logger = rclcpp::get_logger(Bernard::LOGGER_NAME);
    logger.set_level(rclcpp::Logger::Level::Fatal);

    std::unique_ptr<mab::Candle> candle = nullptr;
    std::vector<std::unique_ptr<Bernard::IActuatorDriver>> mds{};

    for (size_t i = 0; i < Bernard::ACTUATORS_NUM; ++i) {
        mds.emplace_back(std::make_unique<Bernard::MDActuatorDriver>(Bernard::ALL_CAN_ACTUATOR_IDS[i] + 1, candle.get()));
    }

    EXPECT_THROW({ auto node = std::make_shared<Bernard::ActuatorsControlNode>(std::move(candle), std::move(mds)); }, std::runtime_error);
}

TEST_F(ActuatorsNodeTest, CheckMDStatePublishingBasic) {
    auto m_bus = std::make_unique<MockBus>();
    MockBus* m_debugBus = m_bus.get();

    CANDLE_MOCK_RESPONDER_INIT(m_debugBus, mockResponse);

    auto m_candle = std::unique_ptr<mab::Candle>(
        mab::attachCandle(mab::CANdleDatarate_E::CAN_DATARATE_1M, std::move(m_bus)));
    std::vector<std::unique_ptr<Bernard::IActuatorDriver>> mds{};

    for (size_t i = 0; i < Bernard::ACTUATORS_NUM; ++i) {
        mds.emplace_back(std::make_unique<MockMDActuatorDriver>(Bernard::ALL_CAN_ACTUATOR_IDS[i], m_candle.get(), 1.0f * i, 0.1f * i, 0.01f * i));
    }

    EXPECT_CALL_ALL_TIMES(mds, MockMDActuatorDriver, getMosfetTemperature(), ::testing::AtMost(1));
    EXPECT_CALL_ALL_TIMES(mds, MockMDActuatorDriver, getTorque(), ::testing::AtMost(2));
    EXPECT_CALL_ALL_TIMES(mds, MockMDActuatorDriver, getPosition(), ::testing::AtMost(2));
    EXPECT_CALL_ALL_TIMES(mds, MockMDActuatorDriver, getVelocity(), ::testing::AtMost(2));

    {
        auto subscriber = std::make_shared<TestSubscriber<sensor_msgs::msg::JointState>>("test_sub", "joint_states");
        auto node = std::make_shared<Bernard::ActuatorsControlNode>(std::move(m_candle), std::move(mds));

        rclcpp::executors::SingleThreadedExecutor executor;
        executor.add_node(node);
        executor.add_node(subscriber);
        std::thread spinner_thread([&]() {
            executor.spin();
        });

        auto msg = subscriber->wait_for_message(std::chrono::milliseconds(100));

        executor.cancel();
        if (spinner_thread.joinable()) {
            spinner_thread.join();
        }

        ASSERT_NE(msg, nullptr);
        EXPECT_EQ(msg->name.size(), Bernard::ACTUATORS_NUM);
        EXPECT_EQ(msg->position.size(), Bernard::ACTUATORS_NUM);
        EXPECT_EQ(msg->velocity.size(), Bernard::ACTUATORS_NUM);
        EXPECT_EQ(msg->effort.size(), Bernard::ACTUATORS_NUM);
        for (size_t i = 0; i < Bernard::ACTUATORS_NUM; ++i) {
            EXPECT_EQ(msg->name[i], Bernard::mdIdToJointName(Bernard::ALL_CAN_ACTUATOR_IDS[i]));
            EXPECT_FLOAT_EQ(msg->position[i], 1.0f * i);
            EXPECT_FLOAT_EQ(msg->velocity[i], 0.1f * i);
            EXPECT_FLOAT_EQ(msg->effort[i], 0.01f * i);
        }
    }

    SUCCEED();
}

TEST_F(ActuatorsNodeTest, CheckMDStatePublishingRate) {
    auto m_bus = std::make_unique<MockBus>();
    MockBus* m_debugBus = m_bus.get();

    CANDLE_MOCK_RESPONDER_INIT(m_debugBus, mockResponse);

    auto m_candle = std::unique_ptr<mab::Candle>(
        mab::attachCandle(mab::CANdleDatarate_E::CAN_DATARATE_1M, std::move(m_bus)));
    std::vector<std::unique_ptr<Bernard::IActuatorDriver>> mds{};

    for (size_t i = 0; i < Bernard::ACTUATORS_NUM; ++i) {
        mds.emplace_back(std::make_unique<MockMDActuatorDriver>(Bernard::ALL_CAN_ACTUATOR_IDS[i], m_candle.get()));
    }

    EXPECT_CALL_ALL_TIMES(mds, MockMDActuatorDriver, getMosfetTemperature(), ::testing::AnyNumber());
    EXPECT_CALL_ALL_TIMES(mds, MockMDActuatorDriver, getTorque(), ::testing::AnyNumber());
    EXPECT_CALL_ALL_TIMES(mds, MockMDActuatorDriver, getPosition(), ::testing::AnyNumber());
    EXPECT_CALL_ALL_TIMES(mds, MockMDActuatorDriver, getVelocity(), ::testing::AnyNumber());

    {
        auto subscriber = std::make_shared<TestSubscriber<sensor_msgs::msg::JointState>>("test_sub", "joint_states");
        auto node = std::make_shared<Bernard::ActuatorsControlNode>(std::move(m_candle), std::move(mds));

        rclcpp::executors::SingleThreadedExecutor executor;
        executor.add_node(node);
        executor.add_node(subscriber);
        std::thread spinner_thread([&]() {
            executor.spin();
        });

        float rate = subscriber->measurePubRate(std::chrono::seconds(2));

        executor.cancel();
        if (spinner_thread.joinable()) {
            spinner_thread.join();
        }

        EXPECT_GE(rate, Bernard::JOINT_STATE_PUBLISH_RATE_HZ - 3.0f);
        EXPECT_LE(rate, Bernard::JOINT_STATE_PUBLISH_RATE_HZ + 3.0f);
    }

    SUCCEED();
}

TEST_F(ActuatorsNodeTest, CheckMDStateTempBasic) {
    auto m_bus = std::make_unique<MockBus>();
    MockBus* m_debugBus = m_bus.get();

    CANDLE_MOCK_RESPONDER_INIT(m_debugBus, mockResponse);

    auto m_candle = std::unique_ptr<mab::Candle>(
        mab::attachCandle(mab::CANdleDatarate_E::CAN_DATARATE_1M, std::move(m_bus)));
    std::vector<std::unique_ptr<Bernard::IActuatorDriver>> mds{};

    for (size_t i = 0; i < Bernard::ACTUATORS_NUM; ++i) {
        mds.emplace_back(std::make_unique<MockMDActuatorDriver>(Bernard::ALL_CAN_ACTUATOR_IDS[i], m_candle.get(), 0.0f, 0.0f, 0.0f, 30.0f + i));
    }

    EXPECT_CALL_ALL_TIMES(mds, MockMDActuatorDriver, getMosfetTemperature(), ::testing::AtMost(2));
    EXPECT_CALL_ALL_TIMES(mds, MockMDActuatorDriver, getTorque(), ::testing::AnyNumber());
    EXPECT_CALL_ALL_TIMES(mds, MockMDActuatorDriver, getPosition(), ::testing::AnyNumber());
    EXPECT_CALL_ALL_TIMES(mds, MockMDActuatorDriver, getVelocity(), ::testing::AnyNumber());

    {
        auto subscriber = std::make_shared<TestSubscriber<std_msgs::msg::Float32MultiArray>>("test_sub", "driver_temperatures");
        auto node = std::make_shared<Bernard::ActuatorsControlNode>(std::move(m_candle), std::move(mds));

        rclcpp::executors::SingleThreadedExecutor executor;
        executor.add_node(node);
        executor.add_node(subscriber);
        std::thread spinner_thread([&]() {
            executor.spin();
        });

        auto msg = subscriber->wait_for_message(std::chrono::milliseconds(1100));

        executor.cancel();
        if (spinner_thread.joinable()) {
            spinner_thread.join();
        }

        ASSERT_NE(msg, nullptr);
        EXPECT_EQ(msg->data.size(), Bernard::ACTUATORS_NUM);
        for (size_t i = 0; i < Bernard::ACTUATORS_NUM; ++i) {
            EXPECT_FLOAT_EQ(msg->data[i], 30.0f + i);
        }
    }

    SUCCEED();
}

TEST_F(ActuatorsNodeTest, CheckMDStateTempBasicRate) {
    auto m_bus = std::make_unique<MockBus>();
    MockBus* m_debugBus = m_bus.get();

    CANDLE_MOCK_RESPONDER_INIT(m_debugBus, mockResponse);

    auto m_candle = std::unique_ptr<mab::Candle>(
        mab::attachCandle(mab::CANdleDatarate_E::CAN_DATARATE_1M, std::move(m_bus)));
    std::vector<std::unique_ptr<Bernard::IActuatorDriver>> mds{};

    for (size_t i = 0; i < Bernard::ACTUATORS_NUM; ++i) {
        mds.emplace_back(std::make_unique<MockMDActuatorDriver>(Bernard::ALL_CAN_ACTUATOR_IDS[i], m_candle.get()));
    }

    EXPECT_CALL_ALL_TIMES(mds, MockMDActuatorDriver, getMosfetTemperature(), ::testing::AnyNumber());
    EXPECT_CALL_ALL_TIMES(mds, MockMDActuatorDriver, getTorque(), ::testing::AnyNumber());
    EXPECT_CALL_ALL_TIMES(mds, MockMDActuatorDriver, getPosition(), ::testing::AnyNumber());
    EXPECT_CALL_ALL_TIMES(mds, MockMDActuatorDriver, getVelocity(), ::testing::AnyNumber());

    {
        auto subscriber = std::make_shared<TestSubscriber<std_msgs::msg::Float32MultiArray>>("test_sub", "driver_temperatures");
        auto node = std::make_shared<Bernard::ActuatorsControlNode>(std::move(m_candle), std::move(mds));

        rclcpp::executors::SingleThreadedExecutor executor;
        executor.add_node(node);
        executor.add_node(subscriber);
        std::thread spinner_thread([&]() {
            executor.spin();
        });

        float rate = subscriber->measurePubRate(std::chrono::seconds(5));

        executor.cancel();
        if (spinner_thread.joinable()) {
            spinner_thread.join();
        }

        EXPECT_GE(rate, Bernard::JOINT_MOSFET_TEMP_PUBLISH_RATE_HZ - 0.3f);
        EXPECT_LE(rate, Bernard::JOINT_MOSFET_TEMP_PUBLISH_RATE_HZ + 0.3f);
    }

    SUCCEED();
}

TEST_F(ActuatorsNodeTest, JoyInputEnableDisableDrivers) {
    auto m_bus = std::make_unique<MockBus>();
    MockBus* m_debugBus = m_bus.get();

    CANDLE_MOCK_RESPONDER_INIT(m_debugBus, mockResponse);

    auto m_candle = std::unique_ptr<mab::Candle>(
        mab::attachCandle(mab::CANdleDatarate_E::CAN_DATARATE_1M, std::move(m_bus)));
    std::vector<std::unique_ptr<Bernard::IActuatorDriver>> mds{};

    for (size_t i = 0; i < Bernard::ACTUATORS_NUM; ++i) {
        mds.emplace_back(std::make_unique<MockMDActuatorDriver>(Bernard::ALL_CAN_ACTUATOR_IDS[i], m_candle.get()));
    }

    EXPECT_CALL_ALL_TIMES(mds, MockMDActuatorDriver, getMosfetTemperature(), ::testing::AnyNumber());
    EXPECT_CALL_ALL_TIMES(mds, MockMDActuatorDriver, getTorque(), ::testing::AnyNumber());
    EXPECT_CALL_ALL_TIMES(mds, MockMDActuatorDriver, getPosition(), ::testing::AnyNumber());
    EXPECT_CALL_ALL_TIMES(mds, MockMDActuatorDriver, getVelocity(), ::testing::AnyNumber());

    EXPECT_CALL_ALL_TIMES(mds, MockMDActuatorDriver, setMotionMode(::testing::_), ::testing::Exactly(2));
    EXPECT_CALL_ALL_TIMES(mds, MockMDActuatorDriver, setTargetPosition(::testing::_), ::testing::Exactly(1));

    {
        auto node = std::make_shared<Bernard::ActuatorsControlNode>(std::move(m_candle), std::move(mds), Bernard::ActuatorsControlNodeMode_t::PUB_WITH_JOY);
        auto publisher = std::make_shared<TestPublisher<sensor_msgs::msg::Joy>>("test_joy_pub", "joy");

        rclcpp::executors::SingleThreadedExecutor executor;
        executor.add_node(node);
        executor.add_node(publisher);
        std::thread spinner_thread([&]() {
            executor.spin();
        });

        // Publish neutral joystick message
        sensor_msgs::msg::Joy joy_msg;
        joy_msg.buttons.resize(15, 0);
        joy_msg.axes.resize(8, 0.0f);
        publisher->publish(std::make_shared<sensor_msgs::msg::Joy>(joy_msg));

        // Publish enable actuators joystick message
        joy_msg.buttons[Bernard::LB_BTN_IDX] = 1;
        joy_msg.buttons[Bernard::RB_BTN_IDX] = 1;
        publisher->publish(std::make_shared<sensor_msgs::msg::Joy>(joy_msg));

        std::this_thread::sleep_for(std::chrono::milliseconds(20));

        EXPECT_EQ(node->getRobotControlMode(), Bernard::RobotControlMode_t::HOLD_POSITION);

        // Publish disable actuators joystick message
        joy_msg.buttons[Bernard::LB_BTN_IDX] = 0;
        joy_msg.buttons[Bernard::RB_BTN_IDX] = 0;
        publisher->publish(std::make_shared<sensor_msgs::msg::Joy>(joy_msg));
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
        joy_msg.buttons[Bernard::LB_BTN_IDX] = 1;
        joy_msg.buttons[Bernard::RB_BTN_IDX] = 1;
        publisher->publish(std::make_shared<sensor_msgs::msg::Joy>(joy_msg));

        std::this_thread::sleep_for(std::chrono::milliseconds(20));

        EXPECT_EQ(node->getRobotControlMode(), Bernard::RobotControlMode_t::OFF);

        executor.cancel();
        if (spinner_thread.joinable()) {
            spinner_thread.join();
        }
    }

    SUCCEED();
}

TEST_F(ActuatorsNodeTest, JoyInputToggleMotionMode) {
    auto m_bus = std::make_unique<MockBus>();
    MockBus* m_debugBus = m_bus.get();

    CANDLE_MOCK_RESPONDER_INIT(m_debugBus, mockResponse);

    auto m_candle = std::unique_ptr<mab::Candle>(
        mab::attachCandle(mab::CANdleDatarate_E::CAN_DATARATE_1M, std::move(m_bus)));
    std::vector<std::unique_ptr<Bernard::IActuatorDriver>> mds{};

    for (size_t i = 0; i < Bernard::ACTUATORS_NUM; ++i) {
        mds.emplace_back(std::make_unique<MockMDActuatorDriver>(Bernard::ALL_CAN_ACTUATOR_IDS[i], m_candle.get()));
    }

    EXPECT_CALL_ALL_TIMES(mds, MockMDActuatorDriver, getMosfetTemperature(), ::testing::AnyNumber());
    EXPECT_CALL_ALL_TIMES(mds, MockMDActuatorDriver, getTorque(), ::testing::AnyNumber());
    EXPECT_CALL_ALL_TIMES(mds, MockMDActuatorDriver, getPosition(), ::testing::AnyNumber());
    EXPECT_CALL_ALL_TIMES(mds, MockMDActuatorDriver, getVelocity(), ::testing::AnyNumber());

    EXPECT_CALL_ALL_TIMES(mds, MockMDActuatorDriver, setMotionMode(::testing::_), ::testing::Exactly(3));
    EXPECT_CALL_ALL_TIMES(mds, MockMDActuatorDriver, setTargetPosition(::testing::_), ::testing::Exactly(2));
    EXPECT_CALL_ALL_TIMES(mds, MockMDActuatorDriver, blink(), ::testing::Exactly(2));

    {
        auto node = std::make_shared<Bernard::ActuatorsControlNode>(std::move(m_candle), std::move(mds), Bernard::ActuatorsControlNodeMode_t::PUB_WITH_JOY);
        auto publisher = std::make_shared<TestPublisher<sensor_msgs::msg::Joy>>("test_joy_pub", "joy");

        rclcpp::executors::SingleThreadedExecutor executor;
        executor.add_node(node);
        executor.add_node(publisher);
        std::thread spinner_thread([&]() {
            executor.spin();
        });

        // Publish neutral joystick message
        sensor_msgs::msg::Joy joy_msg;
        joy_msg.buttons.resize(15, 0);
        joy_msg.axes.resize(8, 0.0f);
        publisher->publish(std::make_shared<sensor_msgs::msg::Joy>(joy_msg));

        // Publish enable actuators joystick message
        joy_msg.buttons[Bernard::LB_BTN_IDX] = 1;
        joy_msg.buttons[Bernard::RB_BTN_IDX] = 1;
        publisher->publish(std::make_shared<sensor_msgs::msg::Joy>(joy_msg));

        std::this_thread::sleep_for(std::chrono::milliseconds(20));

        EXPECT_EQ(node->getRobotControlMode(), Bernard::RobotControlMode_t::HOLD_POSITION);

        // Publish change motion mode joystick message (HOLD_POSITION -> MANUAL)
        joy_msg.buttons[Bernard::LB_BTN_IDX] = 0;
        joy_msg.buttons[Bernard::RB_BTN_IDX] = 0;
        joy_msg.buttons[Bernard::X_BTN_IDX] = 1;
        publisher->publish(std::make_shared<sensor_msgs::msg::Joy>(joy_msg));
        std::this_thread::sleep_for(std::chrono::milliseconds(20));

        joy_msg.buttons[Bernard::X_BTN_IDX] = 0;
        publisher->publish(std::make_shared<sensor_msgs::msg::Joy>(joy_msg));

        EXPECT_EQ(node->getRobotControlMode(), Bernard::RobotControlMode_t::MANUAL);

        joy_msg.buttons[Bernard::X_BTN_IDX] = 1;
        publisher->publish(std::make_shared<sensor_msgs::msg::Joy>(joy_msg));
        std::this_thread::sleep_for(std::chrono::milliseconds(20));

        joy_msg.buttons[Bernard::X_BTN_IDX] = 0;
        publisher->publish(std::make_shared<sensor_msgs::msg::Joy>(joy_msg));

        EXPECT_EQ(node->getRobotControlMode(), Bernard::RobotControlMode_t::HOLD_POSITION);

        executor.cancel();
        if (spinner_thread.joinable()) {
            spinner_thread.join();
        }
    }

    SUCCEED();
}

TEST_F(ActuatorsNodeTest, ReadPolicyActions) {
    auto m_bus = std::make_unique<MockBus>();
    MockBus* m_debugBus = m_bus.get();

    CANDLE_MOCK_RESPONDER_INIT(m_debugBus, mockResponse);

    auto m_candle = std::unique_ptr<mab::Candle>(
        mab::attachCandle(mab::CANdleDatarate_E::CAN_DATARATE_1M, std::move(m_bus)));
    std::vector<std::unique_ptr<Bernard::IActuatorDriver>> mds{};

    for (size_t i = 0; i < Bernard::ACTUATORS_NUM; ++i) {
        mds.emplace_back(std::make_unique<MockMDActuatorDriver>(Bernard::ALL_CAN_ACTUATOR_IDS[i], m_candle.get()));
    }

    EXPECT_CALL_ALL_TIMES(mds, MockMDActuatorDriver, getMosfetTemperature(), ::testing::AnyNumber());
    EXPECT_CALL_ALL_TIMES(mds, MockMDActuatorDriver, getTorque(), ::testing::AnyNumber());
    EXPECT_CALL_ALL_TIMES(mds, MockMDActuatorDriver, getPosition(), ::testing::AnyNumber());
    EXPECT_CALL_ALL_TIMES(mds, MockMDActuatorDriver, getVelocity(), ::testing::AnyNumber());

    EXPECT_CALL_ALL_TIMES(mds, MockMDActuatorDriver, setMotionMode(::testing::_), ::testing::Exactly(2));
    EXPECT_CALL_ALL_TIMES(mds, MockMDActuatorDriver, setTargetPosition(::testing::_), ::testing::Exactly(1));
    EXPECT_CALL_ALL_TIMES(mds, MockMDActuatorDriver, setTargetTorque(::testing::_), ::testing::Exactly(1));
    EXPECT_CALL_ALL_TIMES(mds, MockMDActuatorDriver, blink(), ::testing::Exactly(1));

    {
        auto node = std::make_shared<Bernard::ActuatorsControlNode>(std::move(m_candle), std::move(mds), Bernard::ActuatorsControlNodeMode_t::FULL);
        auto publisher = std::make_shared<TestPublisher<sensor_msgs::msg::Joy>>("test_joy_pub", "joy");
        auto publisher_policy = std::make_shared<TestPublisher<std_msgs::msg::Float32MultiArray>>("test_policy_pub", "actions");

        rclcpp::executors::SingleThreadedExecutor executor;
        executor.add_node(node);
        executor.add_node(publisher);
        executor.add_node(publisher_policy);
        std::thread spinner_thread([&]() {
            executor.spin();
        });

        // Publish neutral joystick message
        sensor_msgs::msg::Joy joy_msg;
        joy_msg.buttons.resize(15, 0);
        joy_msg.axes.resize(8, 0.0f);
        publisher->publish(std::make_shared<sensor_msgs::msg::Joy>(joy_msg));

        // Publish enable actuators joystick message
        joy_msg.buttons[Bernard::LB_BTN_IDX] = 1;
        joy_msg.buttons[Bernard::RB_BTN_IDX] = 1;
        publisher->publish(std::make_shared<sensor_msgs::msg::Joy>(joy_msg));

        std::this_thread::sleep_for(std::chrono::milliseconds(20));

        EXPECT_EQ(node->getRobotControlMode(), Bernard::RobotControlMode_t::HOLD_POSITION);

        // Publish change motion mode joystick message (HOLD_POSITION -> RL_POLICY)
        joy_msg.buttons[Bernard::LB_BTN_IDX] = 0;
        joy_msg.buttons[Bernard::RB_BTN_IDX] = 0;
        joy_msg.buttons[Bernard::START_BTN_IDX] = 1;
        publisher->publish(std::make_shared<sensor_msgs::msg::Joy>(joy_msg));
        std::this_thread::sleep_for(std::chrono::milliseconds(20));

        joy_msg.buttons[Bernard::START_BTN_IDX] = 0;
        publisher->publish(std::make_shared<sensor_msgs::msg::Joy>(joy_msg));

        EXPECT_EQ(node->getRobotControlMode(), Bernard::RobotControlMode_t::RL_POLICY);

        // Publish some actions
        std_msgs::msg::Float32MultiArray actions_msg;
        actions_msg.data = {1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f};
        publisher_policy->publish(std::make_shared<std_msgs::msg::Float32MultiArray>(actions_msg));
        std::this_thread::sleep_for(std::chrono::milliseconds(20));

        // joy_msg.buttons[Bernard::X_BTN_IDX] = 0;
        // publisher->publish(std::make_shared<sensor_msgs::msg::Joy>(joy_msg));

        // EXPECT_EQ(node->getRobotControlMode(), Bernard::RobotControlMode_t::HOLD_POSITION);

        executor.cancel();
        if (spinner_thread.joinable()) {
            spinner_thread.join();
        }
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

    std::vector<std::unique_ptr<Bernard::IActuatorDriver>> mds;
    const size_t n_mds = Bernard::ACTUATORS_NUM;
    for (size_t i = 0; i < n_mds; ++i) {
        mab::canId_t id = Bernard::ALL_CAN_ACTUATOR_IDS[i];
        mds.emplace_back(std::make_unique<Bernard::MDActuatorDriver>(id, candle.get()));
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
