// Copyright (c) 2025, Błażej Szargut.
// All rights reserved.
//
// SPDX-License-Identifier: BSD-3-Clause

#include "actuators_node.hpp"

#include <chrono>
#include <functional>
#include <future>
#include <memory>
#include <string>

#include "candle.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include "utils.hpp"

using namespace std::chrono_literals;

namespace Bernard {

ActuatorsControlNode::ActuatorsControlNode(
    std::unique_ptr<mab::Candle> candle, std::vector<std::unique_ptr<IActuatorDriver>>&& mds, ActuatorsControlNodeMode_t mode)
    : Node(NODE_NAME), _candle(std::move(candle)), _mds(std::move(mds)), _count(0) {
    if (_mds.size() != ACTUATORS_NUM) {
        RCLCPP_ERROR(this->get_logger(), "BERNARD expects %d actuators, but got %zu", ACTUATORS_NUM, _mds.size());
        throw std::runtime_error("Incorrect number of MD instances provided");
    }
    for (auto& md : _mds) {
        if (std::find(ALL_CAN_ACTUATOR_IDS.begin(), ALL_CAN_ACTUATOR_IDS.end(), md->getCanId()) == ALL_CAN_ACTUATOR_IDS.end()) {
            RCLCPP_ERROR(this->get_logger(), "MD with CAN ID %d is not recognized for BERNARD", md->getCanId());
            throw std::runtime_error("Unrecognized MD CAN ID");
        }
    }
    // Initialize publishers and subscribers
    _state_publisher = this->create_publisher<sensor_msgs::msg::JointState>("joint_states", 10);
    _temp_publisher = this->create_publisher<std_msgs::msg::Float32MultiArray>("driver_temperatures", 10);

    if (mode == ActuatorsControlNodeMode_t::FULL) {
        RCLCPP_INFO(this->get_logger(), "ActuatorsControlNode running in FULL mode");
    } else if (mode == ActuatorsControlNodeMode_t::PUB_ONLY) {
        RCLCPP_INFO(this->get_logger(), "ActuatorsControlNode running in PUB_ONLY mode");
    } else if (mode == ActuatorsControlNodeMode_t::PUB_WITH_JOY) {
        RCLCPP_INFO(this->get_logger(), "ActuatorsControlNode running in PUB_WITH_JOY mode");
    }

    if (mode == ActuatorsControlNodeMode_t::FULL) {
        _action_subscriber = this->create_subscription<std_msgs::msg::Float32MultiArray>(
            POLICY_ACTIONS_TOPIC_NAME, 10,
            std::bind(&ActuatorsControlNode::command_callback, this, std::placeholders::_1));
    }
    if (mode == ActuatorsControlNodeMode_t::FULL || mode == ActuatorsControlNodeMode_t::PUB_WITH_JOY) {
        _joy_subscriber = this->create_subscription<sensor_msgs::msg::Joy>(
            JOYSTICK_TOPIC_NAME, 10,
            std::bind(&ActuatorsControlNode::joy_callback, this, std::placeholders::_1));
    }

    _state_timer = this->create_wall_timer(std::chrono::duration<float>(1.0f / JOINT_STATE_PUBLISH_RATE_HZ), std::bind(&ActuatorsControlNode::publish_joint_states, this));
    _temp_timer = this->create_wall_timer(std::chrono::duration<float>(1.0f / JOINT_MOSFET_TEMP_PUBLISH_RATE_HZ), std::bind(&ActuatorsControlNode::publish_joint_temperatures, this));
    _md_states.resize(_mds.size());
    // Ensure last_joy_msg is a valid shared_ptr to avoid deref before first message
    _last_joy_msg = std::make_shared<sensor_msgs::msg::Joy>();

    // start CAN worker thread
    _worker_thread = std::thread(&ActuatorsControlNode::canWorkerLoop, this);

    RCLCPP_INFO(this->get_logger(), "ActuatorsControlNode initialized");
}

ActuatorsControlNode::~ActuatorsControlNode() {
    RCLCPP_INFO(this->get_logger(), "ActuatorsControlNode shutting down");
    _worker_stop.store(true);
    _cmd_cv.notify_all();
    if (_worker_thread.joinable()) _worker_thread.join();
}

void ActuatorsControlNode::enqueueTask(std::function<void()> task) {
    {
        std::lock_guard<std::mutex> lk(_cmd_mutex);
        _cmd_queue.push(std::move(task));
    }
    _cmd_cv.notify_one();
}

void ActuatorsControlNode::canWorkerLoop() {
    using namespace std::chrono;
    auto next_poll = steady_clock::now();
    auto next_temp_poll = next_poll;

    while (!_worker_stop.load()) {
        // wait for either a command or poll interval
        std::unique_lock<std::mutex> lk(_cmd_mutex);
        if (_cmd_queue.empty()) {
            _cmd_cv.wait_until(lk, next_poll, [this] {
                return !_cmd_queue.empty() || _worker_stop.load();
            });
        }

        // process all queued tasks
        while (!_cmd_queue.empty()) {
            auto task = std::move(_cmd_queue.front());
            _cmd_queue.pop();
            lk.unlock();

            try {
                task();
            } catch (const std::exception& e) {
                RCLCPP_ERROR(this->get_logger(), "Exception in CAN task: %s", e.what());
            } catch (...) {
                RCLCPP_ERROR(this->get_logger(), "Unknown exception in CAN task");
            }
            lk.lock();
        }
        lk.unlock();

        if (steady_clock::now() >= next_poll) {
            for (size_t i = 0; i < _mds.size(); ++i) {
                auto pos_pair = _mds[i]->getPosition();
                if (pos_pair.second != mab::MD::Error_t::OK) {
                    RCLCPP_WARN(this->get_logger(), "getPosition failed for MD %zu (CAN %d): %d", i, _mds[i]->getCanId(), static_cast<int>(pos_pair.second));
                    continue;
                }
                auto vel_pair = _mds[i]->getVelocity();
                if (vel_pair.second != mab::MD::Error_t::OK) {
                    RCLCPP_WARN(this->get_logger(), "getVelocity failed for MD %zu (CAN %d): %d", i, _mds[i]->getCanId(), static_cast<int>(vel_pair.second));
                    continue;
                }
                auto torque_pair = _mds[i]->getTorque();
                if (torque_pair.second != mab::MD::Error_t::OK) {
                    RCLCPP_WARN(this->get_logger(), "getTorque failed for MD %zu (CAN %d): %d", i, _mds[i]->getCanId(), static_cast<int>(torque_pair.second));
                    continue;
                }

                {
                    std::lock_guard<std::mutex> lk(_state_mutex);
                    _md_states[i].position = pos_pair.first;
                    _md_states[i].velocity = vel_pair.first;
                    _md_states[i].torque = torque_pair.first;
                }
            }

            next_poll = steady_clock::now() + _poll_interval_ms;
        }

        if (steady_clock::now() >= next_temp_poll) {
            mab::MDRegisters_S registerBuffer;
            for (size_t i = 0; i < _mds.size(); ++i) {
                auto temp_pair = _mds[i]->getMosfetTemperature();
                if (temp_pair.second != mab::MD::Error_t::OK) {
                    RCLCPP_WARN(this->get_logger(), "readRegisters failed for MD %zu (CAN %d): %d", i, _mds[i]->getCanId(), static_cast<int>(temp_pair.second));
                    continue;
                }

                {
                    std::lock_guard<std::mutex> st_lk(_state_mutex);
                    _md_states[i].temperature = temp_pair.first;
                }
            }

            next_temp_poll = steady_clock::now() + _temp_poll_interval_ms;
        }

        // Manual-mode: periodic blink of selected actuator
        RobotControlMode_t mode;
        {
            std::lock_guard<std::mutex> lk(_ctrl_mode_mutex);
            mode = _control_mode;
        }
        if (mode == RobotControlMode_t::MANUAL) {
            auto now = steady_clock::now();
            if (now - _last_manual_blink >= MANUAL_SELECTION_BLINK_INTERVAL) {
                try {
                    // worker is allowed to call _mds directly
                    _mds[_manual_control_actuator_idx]->blink();
                } catch (const std::exception& e) {
                    RCLCPP_ERROR(this->get_logger(), "Error blinking manual actuator: %s", e.what());
                } catch (...) {
                    RCLCPP_ERROR(this->get_logger(), "Unknown error blinking manual actuator");
                }
                _last_manual_blink = now;
            }
        }

        std::this_thread::sleep_for(1ms);
    }  // while
}

void ActuatorsControlNode::publish_joint_states() {
    sensor_msgs::msg::JointState msg;
    msg.header.stamp = this->now();

    std::lock_guard<std::mutex> lk(_state_mutex);
    for (size_t i = 0; i < _mds.size(); ++i) {
        msg.name.push_back(mdIdToJointName(_mds[i]->getCanId()));
        msg.position.push_back(_md_states[i].position);
        msg.velocity.push_back(_md_states[i].velocity);
        msg.effort.push_back(_md_states[i].torque);
    }

    _state_publisher->publish(msg);
}

void ActuatorsControlNode::publish_joint_temperatures() {
    mab::MDRegisters_S registerBuffer;
    std_msgs::msg::Float32MultiArray msg;

    std::lock_guard<std::mutex> lk(_state_mutex);
    for (size_t i = 0; i < _md_states.size(); ++i) {
        msg.data.push_back(_md_states[i].temperature);
    }

    _temp_publisher->publish(msg);
}

mab::MD::Error_t ActuatorsControlNode::zeroEncoders() {
    RCLCPP_INFO(this->get_logger(), "Initiating zeroing of all encoders");
    // Run the worker-only zeroing routine on the worker thread and wait for result
    auto work = [this]() -> mab::MD::Error_t { return this->zeroEncodersWork(); };
    auto result = enqueueTaskWithResult<mab::MD::Error_t>(work, std::chrono::milliseconds(60000));
    if (result.has_value()) return result.value();
    RCLCPP_WARN(this->get_logger(), "zeroEncoders timed out waiting for worker result");
    return mab::MD::Error_t::TRANSFER_FAILED;
}

// worker-only implementation -> should run on the worker thread
mab::MD::Error_t ActuatorsControlNode::zeroEncodersWork() {
    mab::MD::Error_t overall = mab::MD::Error_t::OK;
    // reset abort flag
    _zero_abort.store(false);

    for (size_t i = 0; i < ACTUATORS_NUM; ++i) {
        {
            // inform other parts that worker is waiting for confirmation
            std::lock_guard<std::mutex> lk(_zero_mutex);
            _zero_waiting.store(true);
            _zero_response = false;
        }
        RCLCPP_INFO(this->get_logger(), "Ready to zero encoder for joint %zu (%s). Press A to zero, B to skip, START to cancel remaining.",
                    i, mdIdToJointName(_mds[i]->getCanId()).c_str());

        // Blink current MD periodically while waiting for user input
        auto last_blink = std::chrono::steady_clock::now();

        // Wait until joy_callback notifies or abort
        std::unique_lock<std::mutex> zlk(_zero_mutex);
        while (_zero_waiting.load() && !_zero_abort.load()) {
            auto now = std::chrono::steady_clock::now();
            if (now - last_blink >= ZEROING_BLINK_INTERVAL) {
                try {
                    _mds[i]->blink();
                } catch (...) {
                    RCLCPP_WARN(this->get_logger(), "blink failed while waiting for zero confirmation for joint %zu", i);
                }
                last_blink = now;
            }
            // wait with timeout to allow blinking loop to run
            _zero_cv.wait_for(zlk, std::chrono::milliseconds(100));
        }

        // If aborted entire procedure
        if (_zero_abort.load()) {
            RCLCPP_WARN(this->get_logger(), "Zeroing aborted by user.");
            overall = mab::MD::Error_t::OK;
            break;
        }

        // thread woke up with _zero_waiting == false and _zero_response set
        bool do_zero = _zero_response;
        // reset waiting flag
        _zero_waiting.store(false);
        zlk.unlock();

        if (do_zero) {
            mab::MD::Error_t r = _mds[i]->zero();
            if (r != mab::MD::Error_t::OK) {
                RCLCPP_ERROR(this->get_logger(), "Failed to zero encoder for joint: %d (%s)", _mds[i]->getCanId(), mdIdToJointName(_mds[i]->getCanId()).c_str());
                overall = r;
            } else {
                RCLCPP_INFO(this->get_logger(), "Zeroed encoder for joint %d (%s)", _mds[i]->getCanId(), mdIdToJointName(_mds[i]->getCanId()).c_str());
            }
        } else {
            RCLCPP_INFO(this->get_logger(), "Skipped zeroing for joint %d (%s)", _mds[i]->getCanId(), mdIdToJointName(_mds[i]->getCanId()).c_str());
        }
    }  // for

    {
        std::lock_guard<std::mutex> lk(_zero_mutex);
        _zero_waiting.store(false);
        _zero_abort.store(false);
    }

    setRobotControlMode(RobotControlMode_t::OFF);
    
    return overall;
}

mab::MD::Error_t ActuatorsControlNode::setActuatorsMotionMode(const mab::MdMode_E mode) {
    RCLCPP_INFO(this->get_logger(), "Changing motion mode of all actuators to: %s", motionModeToString(mode).c_str());
    {
        std::lock_guard<std::mutex> lk(_act_mode_mutex);
        _actuator_motion_mode = mode;
    }

    auto work = [this, mode]() -> mab::MD::Error_t {
        mab::MD::Error_t result = mab::MD::Error_t::OK;
        for (auto& md : _mds) {
            mab::MD::Error_t r = md->setMotionMode(mode);
            if (r != mab::MD::Error_t::OK) {
                RCLCPP_ERROR(this->get_logger(), "Failed to change motion mode for joint: %d (%s)", md->getCanId(), mdIdToJointName(md->getCanId()).c_str());
                result = r;
            }
        }
        return result;
    };

    {
        auto res = enqueueTaskWithResult<mab::MD::Error_t>(work, 2000ms);
        if (res.has_value()) return res.value();
        RCLCPP_WARN(this->get_logger(), "setActuatorsMotionMode timed out waiting for worker");
        return mab::MD::Error_t::TRANSFER_FAILED;
    }
}

void ActuatorsControlNode::setRobotControlMode(const RobotControlMode_t mode) {
    RobotControlMode_t prev_mode;
    {
        std::lock_guard<std::mutex> lk(_ctrl_mode_mutex);
        prev_mode = _control_mode;
    }

    mab::MdMode_E act_mode;
    {
        std::lock_guard<std::mutex> lk(_act_mode_mutex);
        act_mode = _actuator_motion_mode;
    }

    // Schedule worker actions for mode transitions. All direct communication with MDs
    // must happen from the worker thread, so enqueueTask is used.
    switch (mode) {
        case RobotControlMode_t::OFF:
            if (act_mode == mab::MdMode_E::IDLE) break;
            {
                std::lock_guard<std::mutex> lk(_act_mode_mutex);
                _actuator_motion_mode = mab::MdMode_E::IDLE;
            }
            enqueueTask([this, &prev_mode]() {
                for (auto& md : _mds) {
                    mab::MD::Error_t r = md->setMotionMode(mab::MdMode_E::IDLE);
                    if (r != mab::MD::Error_t::OK) {
                        RCLCPP_WARN(this->get_logger(), "Failed to set MD %d to IDLE: %d", md->getCanId(), static_cast<int>(r));
                    }
                }

            if (prev_mode != RobotControlMode_t::OFF) {
                    RCLCPP_INFO(this->get_logger(), "Actuators are on - disabling all actuators");
                    for (size_t i = 0; i < ACTUATORS_NUM; ++i) {
                        mab::MD::Error_t r = _mds[i]->disable();
                        if (r != mab::MD::Error_t::OK) {
                            RCLCPP_WARN(this->get_logger(), "Failed to disable MD %d: %d", _mds[i]->getCanId(), static_cast<int>(r));
                        }
                    }
                }
            });
            break;
        case RobotControlMode_t::MANUAL:
        case RobotControlMode_t::HOLD_POSITION:
            if (act_mode == mab::MdMode_E::POSITION_PID) break;
            {
                std::lock_guard<std::mutex> lk(_act_mode_mutex);
                _actuator_motion_mode = mab::MdMode_E::POSITION_PID;
            }
            // ensure controllers are in position mode and set current position as target
            enqueueTask([this, &prev_mode]() {
                for (size_t i = 0; i < ACTUATORS_NUM; ++i) {
                    // switch to position control
                    mab::MD::Error_t r = _mds[i]->setMotionMode(mab::MdMode_E::POSITION_PID);
                    if (r != mab::MD::Error_t::OK) {
                        RCLCPP_WARN(this->get_logger(), "Failed to set MD %d to POSITION_PID: %d", _mds[i]->getCanId(), static_cast<int>(r));
                    }
                }

                // set current position as target for all motors
                for (size_t i = 0; i < ACTUATORS_NUM; ++i) {
                    float pos = 0.0f;
                    {
                        // fallback to last known state
                        std::lock_guard<std::mutex> lk(_state_mutex);
                        if (i < _md_states.size()) pos = _md_states[i].position;
                    }

                    mab::MD::Error_t r = _mds[i]->setTargetPosition(pos);
                    if (r != mab::MD::Error_t::OK) {
                        RCLCPP_WARN(this->get_logger(), "Failed to set hold position for MD %d: %d", _mds[i]->getCanId(), static_cast<int>(r));
                    }
                }

                if (prev_mode == RobotControlMode_t::OFF) {
                    RCLCPP_INFO(this->get_logger(), "Actuators are off - enabling all actuators");
                    for (size_t i = 0; i < ACTUATORS_NUM; ++i) {
                        mab::MD::Error_t r = _mds[i]->enable();
                        if (r != mab::MD::Error_t::OK) {
                            RCLCPP_WARN(this->get_logger(), "Failed to enable MD %d: %d", _mds[i]->getCanId(), static_cast<int>(r));
                        }
                    }
                }
            });
            break;
        case RobotControlMode_t::RL_POLICY:
            // policy uses torque control
            if (act_mode == mab::MdMode_E::RAW_TORQUE) break;
            {
                std::lock_guard<std::mutex> lk(_act_mode_mutex);
                _actuator_motion_mode = mab::MdMode_E::RAW_TORQUE;
            }
            enqueueTask([this]() {
                for (auto& md : _mds) {
                    mab::MD::Error_t r = md->setMotionMode(mab::MdMode_E::RAW_TORQUE);
                    if (r != mab::MD::Error_t::OK) {
                        RCLCPP_WARN(this->get_logger(), "Failed to set MD %d to TORQUE: %d", md->getCanId(), static_cast<int>(r));
                    }
                }
            });
        case RobotControlMode_t::ZERO_ENCODERS:
            break;
        default:
            RCLCPP_ERROR(this->get_logger(), "Unknown target robot control mode: %s", robotControlModeToString(mode).c_str());
            break;
    }

    {
        std::lock_guard<std::mutex> lk(_ctrl_mode_mutex);
        _control_mode = mode;
    }

    return;
}

mab::MD::Error_t ActuatorsControlNode::blinkActuators(const std::vector<uint16_t>& joint_ids) {
    RCLCPP_INFO(this->get_logger(), "Blinking actuators' LEDs");
    auto work = [this, joint_ids]() -> mab::MD::Error_t {
        mab::MD::Error_t result = mab::MD::Error_t::OK;
        for (auto id : joint_ids) {
            for (auto& md : _mds) {
                if (md->getCanId() == id) {
                    mab::MD::Error_t r = md->blink();
                    if (r != mab::MD::Error_t::OK) {
                        RCLCPP_ERROR(this->get_logger(), "Failed to blink LED for joint: %d (%s)", md->getCanId(), mdIdToJointName(md->getCanId()).c_str());
                        result = r;
                    }
                    break;
                }
            }
        }
        return result;
    };
    {
        auto res = enqueueTaskWithResult<mab::MD::Error_t>(work, 2000ms);
        if (res.has_value()) return res.value();
        RCLCPP_WARN(this->get_logger(), "blinkActuators timed out waiting for worker");
        return mab::MD::Error_t::TRANSFER_FAILED;
    }
}

void ActuatorsControlNode::joy_callback(const sensor_msgs::msg::Joy::SharedPtr msg) {
    if (!msg) return;

    // safe access helpers - joystick messages may have variable lengths
    auto safeButton = [&](size_t idx) -> int {
        if (!msg) return 0;
        return (idx < msg->buttons.size()) ? msg->buttons[idx] : 0;
    };
    auto safePrevButton = [&](size_t idx) -> int {
        if (!_last_joy_msg) return 0;
        return (idx < _last_joy_msg->buttons.size()) ? _last_joy_msg->buttons[idx] : 0;
    };
    auto safeAxis = [&](size_t idx) -> float {
        if (!msg) return 0.0f;
        return (idx < msg->axes.size()) ? msg->axes[idx] : 0.0f;
    };
    auto safePrevAxis = [&](size_t idx) -> float {
        if (!_last_joy_msg) return 0.0f;
        return (idx < _last_joy_msg->axes.size()) ? _last_joy_msg->axes[idx] : 0.0f;
    };

    // current button/axis values (safely extracted)
    const bool btn_lb = safeButton(LB_BTN_IDX);
    const bool btn_rb = safeButton(RB_BTN_IDX);
    const float left_trigger = safeAxis(LEFT_TRIGGER_AXIS_IDX);
    const float right_trigger = safeAxis(RIGHT_TRIGGER_AXIS_IDX);
    const float dpad_x = safeAxis(DPAD_X_AXIS_IDX);

    RobotControlMode_t mode;
    {
        std::lock_guard<std::mutex> lk(_ctrl_mode_mutex);
        mode = _control_mode;
    }

    // rising edge detection helpers
    auto risingBtn = [&](size_t idx) -> bool {
        return (safeButton(idx) == 1 && safePrevButton(idx) == 0);
    };
    // Axis rising for positive/negative edges with threshold
    auto risingAxis = [&](size_t idx, bool positive = true, float threshold = 0.9f) -> bool {
        float cur = safeAxis(idx);
        float prev = safePrevAxis(idx);
        if (positive) {
            return (cur >= threshold && prev < threshold);
        } else {
            return (cur <= -threshold && prev > -threshold);
        }
    };

    // If worker is waiting for zero confirmation, handle A/B/START for confirmation/skip/abort
    if (_zero_waiting.load()) {
        if (risingBtn(A_BTN_IDX)) {
            {
                std::lock_guard<std::mutex> lk(_zero_mutex);
                _zero_response = true;
                _zero_waiting.store(false);
            }
            _zero_cv.notify_all();
            // still update last_buttons below
        } else if (risingBtn(B_BTN_IDX)) {
            {
                std::lock_guard<std::mutex> lk(_zero_mutex);
                _zero_response = false;
                _zero_waiting.store(false);
            }
            _zero_cv.notify_all();
        } else if (risingBtn(START_BTN_IDX)) {
            _zero_abort.store(true);
            _zero_waiting.store(false);
            _zero_cv.notify_all();
        }
        return;
    }

    // --- Toggle actuators ON/OFF -------------------------
    if (btn_lb && btn_rb && (risingBtn(LB_BTN_IDX) || risingBtn(RB_BTN_IDX))) {
        if (mode == RobotControlMode_t::OFF) {
            setRobotControlMode(RobotControlMode_t::HOLD_POSITION);
            RCLCPP_INFO(this->get_logger(), "Motors ENABLED (HOLD_POSITION)");
        } else {
            setRobotControlMode(RobotControlMode_t::OFF);
            RCLCPP_INFO(this->get_logger(), "Motors DISABLED");
        }
        _last_joy_msg = msg;
        return;
    }

    // --- Start interactive zeroing with Y (rising edge) ---
    if (risingBtn(Y_BTN_IDX) && mode == RobotControlMode_t::OFF) {
        setRobotControlMode(RobotControlMode_t::ZERO_ENCODERS);
        // only allow starting if not already waiting
        if (!_zero_waiting.load()) {
            RCLCPP_INFO(this->get_logger(), "User requested interactive zeroing via gamepad (Y pressed). Starting interactive zeroing on worker.");
            enqueueTask([this]() {
                this->zeroEncodersWork();
            });
        }
        _last_joy_msg = msg;
        return;
    }

    // --- Select control mode -----------------------------
    if (mode != RobotControlMode_t::OFF) {
        if (risingBtn(X_BTN_IDX)) {
            auto new_mode = mode == RobotControlMode_t::HOLD_POSITION ? RobotControlMode_t::MANUAL
                                                                      : RobotControlMode_t::HOLD_POSITION;
            setRobotControlMode(new_mode);
            RCLCPP_INFO(this->get_logger(), "X Button pressed. Control mode: %s", robotControlModeToString(new_mode).c_str());
            blinkActuators(ALL_CAN_ACTUATOR_IDS);
            _last_joy_msg = msg;
            return;
        } else if (risingBtn(START_BTN_IDX)) {
            auto new_mode = mode == RobotControlMode_t::RL_POLICY ? RobotControlMode_t::HOLD_POSITION
                                                                  : RobotControlMode_t::RL_POLICY;
            setRobotControlMode(new_mode);
            RCLCPP_INFO(this->get_logger(), "START Button pressed. Control mode: %s", robotControlModeToString(mode).c_str());
            blinkActuators(ALL_CAN_ACTUATOR_IDS);
            _last_joy_msg = msg;
            return;
        }
    }

    // --- Manual mode actuator selection ------------------
    if (mode == RobotControlMode_t::MANUAL) {
        if (dpad_x == 1.0f && risingAxis(DPAD_X_AXIS_IDX, true)) {
            if (!_mds.empty()) {
                _manual_control_actuator_idx = (_manual_control_actuator_idx + 1) % _mds.size();
                auto can_id = _mds[_manual_control_actuator_idx]->getCanId();
                RCLCPP_INFO(this->get_logger(), "Selected motor: %zu (%s)", _manual_control_actuator_idx, mdIdToJointName(can_id).c_str());
                blinkActuators({can_id});
            }
            _last_joy_msg = msg;
            return;
        } else if (dpad_x == -1.0f && risingAxis(DPAD_X_AXIS_IDX, false)) {
            if (!_mds.empty()) {
                _manual_control_actuator_idx = (_manual_control_actuator_idx + _mds.size() - 1) % _mds.size();
                auto can_id = _mds[_manual_control_actuator_idx]->getCanId();
                RCLCPP_INFO(this->get_logger(), "Selected motor: %zu (%s)", _manual_control_actuator_idx, mdIdToJointName(can_id).c_str());
                blinkActuators({can_id});
            }
            _last_joy_msg = msg;
            return;
        }

        // Update command only trigger value has changed
        if (right_trigger != safePrevAxis(RIGHT_TRIGGER_AXIS_IDX) || left_trigger != safePrevAxis(LEFT_TRIGGER_AXIS_IDX)) {
            float delta_pos = 0.0f;
            delta_pos += 1 - right_trigger;              // move forward
            delta_pos -= 1 - left_trigger;               // move backward
            float command_value = delta_pos * 0.05f;  // scale down for fine control

            RCLCPP_INFO(this->get_logger(), "Manual control command for MD %d: %.4f (RT: %.2f, LT: %.2f)", _mds[_manual_control_actuator_idx]->getCanId(), command_value, right_trigger, left_trigger);

            // enqueue task that applies command to selected MD
            enqueueTask([this, idx = _manual_control_actuator_idx, val = command_value]() {
                try {
                    float current_pos = _md_states[idx].position;
                    float target_pos = current_pos + val;
                    mab::MD::Error_t r = _mds[idx]->setTargetPosition(target_pos);
                    if (r != mab::MD::Error_t::OK) {
                        RCLCPP_ERROR(this->get_logger(), "Failed to set manual position command for MD %d (CAN %d): %d", (int)idx, _mds[idx]->getCanId(), static_cast<int>(r));
                    }
                } catch (...) {
                    RCLCPP_ERROR(this->get_logger(), "Failed to apply manual command to MD %d", (int)idx);
                }
            });
        }

        _last_joy_msg = msg;
        return;
    }

    _last_joy_msg = msg;
    return;
}

void ActuatorsControlNode::command_callback(const std_msgs::msg::Float32MultiArray::SharedPtr msg) {
    RobotControlMode_t mode;
    {
        std::lock_guard<std::mutex> lk(_ctrl_mode_mutex);
        mode = _control_mode;
    }

    if (mode != RobotControlMode_t::RL_POLICY) return;

    std::vector<float> actions = msg->data;

    enqueueTask([this, actions = std::move(actions)]() {
        for (size_t i = 0; i < _mds.size() && i < actions.size(); ++i) {
            float a = actions[i];
            try {
                mab::MD::Error_t r = _mds[i]->setTargetTorque(a);
                if (r != mab::MD::Error_t::OK) {
                    RCLCPP_ERROR(this->get_logger(), "Failed to set torque command for MD %zu (CAN %d): %d", i, _mds[i]->getCanId(), static_cast<int>(r));
                }
            } catch (...) {
                RCLCPP_ERROR(this->get_logger(), "Failed to apply action to MD %zu", i);
            }
        }
    });
}

}  // namespace Bernard
