#include "actuators_node.hpp"

#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "candle.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include "utils.hpp"

using namespace std::chrono_literals;

namespace bernard {

ActuatorsControlNode::ActuatorsControlNode(const std::unique_ptr<mab::Candle> candle, const std::vector<mab::MD>& mds)
    : Node("actuators_interface"), count_(0), candle_(std::move(candle)), mds_(mds) {
    _state_publisher = this->create_publisher<sensor_msgs::msg::JointState>("joint_states", 10);
    _temp_publisher = this->create_publisher<std_msgs::msg::Float32MultiArray>("driver_temperatures", 10);
    _action_subscriber = this->create_subscription<std_msgs::msg::Float32MultiArray>(
        "actions", 10,
        std::bind(&ActuatorsControlNode::command_callback, this, std::placeholders::_1));
    _joy_subscriber = this->create_subscription<sensor_msgs::msg::Joy>(
        "joy", 10,
        std::bind(&ActuatorsControlNode::joy_callback, this, std::placeholders::_1));

    _state_timer = this->create_wall_timer(10ms, std::bind(&ActuatorsControlNode::publish_joint_states, this));
    _temp_timer = this->create_wall_timer(5000ms, std::bind(&ActuatorsControlNode::publish_joint_temperatures, this));

    RCLCPP_INFO(this->get_logger(), "ActuatorsControlNode initialized");
}

ActuatorsControlNode::~ActuatorsControlNode() {
    RCLCPP_INFO(this->get_logger(), "ActuatorsControlNode shutting down");
}

void ActuatorsControlNode::publish_joint_states() {
    sensor_msgs::msg::JointState msg;
    msg.header.stamp = this->now();

    _mutex.lock();
    for (auto& md : mds_) {
        msg.name.push_back(JOINT_MAP.at(md.m_canId));
        msg.position.push_back(md.getPosition().first);
        msg.velocity.push_back(md.getVelocity().first);
        msg.effort.push_back(md.getTorque().first);
    }
    _mutex.unlock();

    _state_publisher->publish(msg);
}

void ActuatorsControlNode::publish_joint_temperatures() {
    mab::MDRegisters_S registerBuffer;
    std_msgs::msg::Float32MultiArray msg;

    _mutex.lock();
    for (auto& md : mds_) {
        md.readRegisters(registerBuffer.mosfetTemperature);
        msg.data.push_back(registerBuffer.mosfetTemperature.value);
    }
    _mutex.unlock();
    _temp_publisher->publish(msg);
}

void ActuatorsControlNode::zeroEncoders() {
    RCLCPP_INFO(this->get_logger(), "Initiating zeroing of all encoders");
    mab::MD::Error_t md_result, result = mab::MD::Error_t::OK;

    for (auto& md : mds_) {
        RCLCPP_INFO(this->get_logger(), "Do you want to zero encoder for joint: %d (%s)? (y/n)", md.m_canId, mdIdToJointName(md.m_canId));
        char response;
        std::cin >> response;
        response = std::tolower(response);
        if (response != 'y') {
            RCLCPP_INFO(this->get_logger(), "Skipping zeroing for joint: %d (%s)", md.m_canId, mdIdToJointName(md.m_canId));
            continue;
        }
        _mutex.lock();
        md_result = md.zero();
        if (md_result != mab::MD::Error_t::OK) {
            RCLCPP_ERROR(this->get_logger(), "Failed to zero encoder for joint: %d (%s)", md.m_canId, mdIdToJointName(md.m_canId));
            result = md_result;
        }
        _mutex.unlock();
    }

    return result;
}

void ActuatorsControlNode::setActuatorsMotionMode(const mab::MdMode_E mode) {
    RCLCPP_INFO(this->get_logger(), "Changing motion mode of all actuators to: %s", motionModeToString(mode));
    _actuator_motion_mode = mode;
    mab::MD::Error_t md_result, result = mab::MD::Error_t::OK;

    _mutex.lock();
    for (auto& md : mds_) {
        md_result = md.setMotionMode(mode);
        if (md_result != mab::MD::Error_t::OK) {
            RCLCPP_ERROR(this->get_logger(), "Failed to change motion mode for joint: %d (%s)", md.m_canId, mdIdToJointName(md.m_canId));
            result = md_result;
        }
    }
    _mutex.unlock();
    return result;
}

void ActuatorsControlNode::setRobotControlMode(const RobotControlMode_t mode) {
    _control_mode = mode;
}

mab::MD::Error_t ActuatorsControlNode::blinkActuators(const std::vector<mab::MD>& mds) {
    RCLCPP_INFO(this->get_logger(), "Blinking actuators' LEDs");
    mab::MD::Error_t md_result, result = mab::MD::Error_t::OK;

    _mutex.lock();
    for (const auto& md : mds) {
        md_result = md.blink();
        if (md_result != mab::MD::Error_t::OK) {
            RCLCPP_ERROR(this->get_logger(), "Failed to blink LED for joint: %d (%s)", md.m_canId, mdIdToJointName(md.m_canId));
            result = md_result;
        }
    }
    _mutex.unlock();
    return result;
}

void ActuatorsControlNode::joy_callback(const sensor_msgs::msg::Joy::SharedPtr msg) {
    _last_joy_msg = msg;

    // Buttons
    const bool btn_a = msg->buttons[0];
    const bool btn_x = msg->buttons[2];
    const bool btn_start = msg->buttons[7];
    const bool btn_lb = msg->buttons[4];
    const bool btn_rb = msg->buttons[5];

    // Axes
    const float dpad_x = msg->axes[6];

    static int selected_motor = 0;
    const int motor_count = static_cast<int>(_mds.size());

    // --- Toggle actuators ON/OFF -------------------------
    if (btn_lb && btn_rb) {
        if (_control_mode == RobotControlMode_t::OFF) {
            _control_mode = RobotControlMode_t::HOLD_POSITION;
            setActuatorsMotionMode(mab::MdMode_E::POSITION_PID);
            RCLCPP_INFO(this->get_logger(), "Motors ENABLED (POSITION_PID)");
        } else {
            _control_mode = RobotControlMode_t::OFF;
            setActuatorsMotionMode(mab::MdMode_E::IDLE);
            RCLCPP_INFO(this->get_logger(), "Motors DISABLED");
        }
        return;
    }

    // --- Select control mode -----------------------------
    if (_control_mode != RobotControlMode_t::OFF) {
        if (btn_x) {
            _control_mode = _control_mode == RobotControlMode_t::HOLD_POSITION ? RobotControlMode_t::MANUAL
                                                                               : RobotControlMode_t::HOLD_POSITION;
            RCLCPP_INFO(this->get_logger(), "X Button pressed. Control mode: %s", robotControlModeToString(_control_mode));
            blinkActuators(_mds);
            return;
        } else if (btn_start) {
            _control_mode = _control_mode == RobotControlMode_t::RL_POLICY ? RobotControlMode_t::HOLD_POSITION
                                                                           : RobotControlMode_t::RL_POLICY;
            RCLCPP_INFO(this->get_logger(), "START Button pressed. Control mode: %s", robotControlModeToString(_control_mode));
            blinkActuators(_mds);
            return;
        }
    }

    // --- Manual mode actuator selection ------------------
    if (_control_mode == RobotControlMode_t::MANUAL) {
        if (dpad_x == 1.0f) {  // Right
            _manual_control_actuator_idx = (_manual_control_actuator_idx + 1) % ACTUATORS_NUM;
            RCLCPP_INFO(this->get_logger(), "Selected motor: %d (%s)", _manual_control_actuator_idx, mdIdToJointName(_mds[_manual_control_actuator_idx].m_canId));
            blinkActuators({_mds[_manual_control_actuator_idx]});
            return;
        } else if (dpad_x == -1.0f) {  // Left
            _manual_control_actuator_idx = (_manual_control_actuator_idx - 1 + ACTUATORS_NUM) % ACTUATORS_NUM;
            RCLCPP_INFO(this->get_logger(), "Selected motor: %d (%s)", _manual_control_actuator_idx, mdIdToJointName(_mds[_manual_control_actuator_idx].m_canId));
            blinkActuators({_mds[_manual_control_actuator_idx]});
            return;
        }

    if (_control_mode == RobotControlMode_t::MANUAL ||
        _control_mode == RobotControlMode_t::HOLD_POSITION) {
        float manual_cmd = 0.0f;

        // Triggers as ±1 command
        manual_cmd = (rt - 0.0f) - (lt - 0.0f);  // prosty sygnał

        // Optional: add stick Y as fine control
        manual_cmd += 0.3f * ly;

        if (std::abs(manual_cmd) > 0.05f) {
            auto& md = _mds[selected_motor];
            md.setTargetVelocity(manual_cmd * 5.0f);  // przykładowy scale
            RCLCPP_DEBUG(this->get_logger(),
                         "Manual motor %d velocity = %.2f",
                         selected_motor, manual_cmd);
        }
    }

    // Debug minimalny
    RCLCPP_INFO(this->get_logger(),
                "Lx: %.2f Ly: %.2f LT: %.2f RT: %.2f | A:%d sel:%d",
                lx, ly, lt, rt, btn_a, selected_motor);
}

}  // namespace bernard
