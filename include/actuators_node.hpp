// Copyright (c) 2025, Błażej Szargut.
// All rights reserved.
//
// SPDX-License-Identifier: BSD-3-Clause

#pragma once

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <functional>
#include <future>
#include <memory>
#include <mutex>
#include <optional>
#include <queue>
#include <thread>
#include <vector>

#include "MD.hpp"
#include "candle.hpp"
#include "config.hpp"
#include "drivers.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"

namespace Bernard {

/// @brief Control modes for the robot
enum class RobotControlMode_t {
    OFF,
    MANUAL,
    RL_POLICY,
    HOLD_POSITION
};

/// @brief Structure to hold MD state information
struct ActuatorState {
    /// @brief Position in **rad**
    float position = 0.0f;
    /// @brief Velocity in **rad/s**
    float velocity = 0.0f;
    /// @brief Torque in **Nm**
    float torque = 0.0f;
    /// @brief Temperature in **°C**
    float temperature = 0.0f;
};

/// @brief Modes for ActuatorsControlNode
enum class ActuatorsControlNodeMode_t {
    FULL,
    PUB_ONLY,
    PUB_WITH_JOY
};

class ActuatorsControlNode : public rclcpp::Node {
   public:
    /**
     * @brief Constructor for ActuatorsControlNode
     * @param candle Pointer to CANdle instance
     * @param mds Reference to vector of MD instances
     * @param Mode Node operation mode
     */
    ActuatorsControlNode(
        std::unique_ptr<mab::Candle> candle,
        std::vector<std::unique_ptr<IActuatorDriver>>&& mds,
        ActuatorsControlNodeMode_t mode = ActuatorsControlNodeMode_t::FULL);

    /**
     * @brief Destructor for ActuatorsControlNode
     */
    ~ActuatorsControlNode();

    /**
     * @brief Change control mode of all actuators
     * @param mode New control mode to set
     * @return mab::MD::Error_t indicating success or failure
     */
    mab::MD::Error_t setActuatorsMotionMode(const mab::MdMode_E mode);

    /**
     * @brief Get the current motion mode of all actuators
     * @return Current motion mode
     */
    mab::MdMode_E getActuatorsMotionMode() const { return _actuator_motion_mode; }

    /**
     * @brief Zero encoders of all actuators
     * @return mab::MD::Error_t indicating success or failure
     */
    mab::MD::Error_t zeroEncoders();

    /**
     * @brief Worker-only implementation of zeroing routine
     *
     * Contains the interactive, blocking loop that communicates with MDs and waits for
     * user confirmations. This MUST run on the worker thread (it will block while
     * waiting for joystick input) and should be enqueued via enqueueTask when run
     * asynchronously.
     */
    mab::MD::Error_t zeroEncodersWork();

    /**
     * @brief Blink selected actuator LEDs
     * @param joint_ids Vector of joint IDs to blink
     * @return mab::MD::Error_t indicating success or failure
     */
    mab::MD::Error_t blinkActuators(const std::vector<uint16_t>& can_ids);

    /**
     * @brief Get the current control mode
     * @return Current control mode
     */
    RobotControlMode_t getControlMode() const { return _control_mode; };

    /**
     * @brief Set robot control mode
     * @param mode New control mode to set
     * @return mab::MD::Error_t indicating success or failure
     */
    void setRobotControlMode(const RobotControlMode_t mode);

    /**
     * @brief Get robot control mode
     * @return Current robot control mode
     */
    RobotControlMode_t getRobotControlMode() const { return _control_mode; };

   private:
    /// @brief Publish joint states of all robot actuators
    void publish_joint_states();

    /// @brief Publish joint temperatures of all robot actuators
    void publish_joint_temperatures();

    /// @brief Callback for joystick messages
    void joy_callback(const sensor_msgs::msg::Joy::SharedPtr msg);

    /// @brief Callback for command messages
    void command_callback(const std_msgs::msg::Float32MultiArray::SharedPtr msg);

    /// @brief Worker thread loop for CAN operations
    void canWorkerLoop();

    /// @brief Enqueue a task to be executed by the worker thread
    /// @param task The task to enqueue
    void enqueueTask(std::function<void()> task);

    /// @brief Enqueue a task with a result to be executed by the worker thread
    /// @param work The task to enqueue
    /// @param timeout The maximum time to wait for the result
    /// @return The result of the task
    template <typename R>
    inline std::optional<R> enqueueTaskWithResult(
        std::function<R()> work,
        std::chrono::milliseconds timeout = std::chrono::milliseconds(1000),
        std::optional<R> defaultOnTimeout = std::nullopt) {
        auto p = std::make_shared<std::promise<R>>();
        std::future<R> f = p->get_future();
        enqueueTask([work = std::move(work), p = std::move(p)]() mutable {
            try {
                R res = work();
                p->set_value(res);
            } catch (...) {
                try {
                    p->set_exception(std::current_exception());
                } catch (...) {
                    // nothing else we can do
                }
            }
        });
        if (f.wait_for(timeout) == std::future_status::ready) {
            try {
                R value = f.get();
                return std::make_optional(std::move(value));
            } catch (...) {
                RCLCPP_ERROR(rclcpp::get_logger(LOGGER_NAME), "Exception while retrieving future result");
                return std::nullopt;
            }
        } else {
            RCLCPP_WARN(rclcpp::get_logger(LOGGER_NAME), "enqueueTaskWithResult timed out");
            if (defaultOnTimeout.has_value()) return defaultOnTimeout;
            return std::nullopt;
        }
    }

    /// @brief Pointer to CANdle instance
    std::unique_ptr<mab::Candle> _candle{nullptr};

    /// @brief Reference to vector of MD instances
    std::vector<std::unique_ptr<IActuatorDriver>> _mds;

    /// @brief Vector to hold MD state information
    std::vector<ActuatorState> _md_states;

    /// @brief Message counter
    size_t _count{0};

    /// @brief ROS2 state timer
    rclcpp::TimerBase::SharedPtr _state_timer;

    /// @brief ROS2 temperature timer
    rclcpp::TimerBase::SharedPtr _temp_timer;

    /// @brief ROS2 joint state publisher
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr _state_publisher;

    /// @brief ROS2 temperature publisher
    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr _temp_publisher;

    /// @brief ROS2 action subscriber
    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr _action_subscriber;

    /// @brief ROS2 joystick subscriber
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr _joy_subscriber;

    /// @brief Actuator control mode
    mab::MdMode_E _actuator_motion_mode = mab::MdMode_E::IDLE;

    /// @brief Robot control mode
    RobotControlMode_t _control_mode = RobotControlMode_t::OFF;

    /// @brief Manual control selected motor index
    size_t _manual_control_actuator_idx{0};

    /// @brief Stored joystick message
    sensor_msgs::msg::Joy::SharedPtr _last_joy_msg{nullptr};

    /// @brief Mutex for thread-safe operations
    std::mutex _mutex;

    /// @brief Mutex for state variables
    std::mutex _state_mutex;

    /// @brief Worker thread and task queue
    std::mutex _cmd_mutex;

    /// @brief Condition variable to signal the worker thread
    std::condition_variable _cmd_cv;

    /// @brief Queue of tasks to be executed by the worker thread
    std::queue<std::function<void()>> _cmd_queue;

    /// @brief Worker thread
    std::thread _worker_thread;

    /// @brief Flag to stop the worker thread
    std::atomic<bool> _worker_stop{false};

    /// @brief Polling interval for the worker thread (joint states)
    std::chrono::milliseconds _poll_interval_ms{static_cast<int>(1000.0f / JOINT_STATE_PUBLISH_RATE_HZ)};

    /// @brief Polling interval for the worker thread (joint temperatures)
    std::chrono::milliseconds _temp_poll_interval_ms{static_cast<int>(1000.0f / JOINT_MOSFET_TEMP_PUBLISH_RATE_HZ)};

    /// @brief Mutex for zeroing procedure synchronization
    std::mutex _zero_mutex;

    /// @brief Condition variable for zeroing procedure synchronization
    std::condition_variable _zero_cv;

    /// @brief Flag indicating if zeroing procedure is waiting for user input
    std::atomic<bool> _zero_waiting{false};

    /// @brief Flag indicating if zeroing procedure received user response
    bool _zero_response = false;

    /// @brief Flag indicating if zeroing procedure should be aborted
    std::atomic<bool> _zero_abort{false};

    /// @brief Timestamp of last manual control blink
    std::chrono::steady_clock::time_point _last_manual_blink{std::chrono::steady_clock::now()};

    /// @brief Error counter for CAN communication
    size_t _can_error_count{0};
};

}  // namespace Bernard