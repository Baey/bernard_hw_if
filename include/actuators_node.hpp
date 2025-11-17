#pragma once

#include <memory>
#include <mutex>

#include "MD.hpp"
#include "candle.hpp"
#include "config.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"

namespace bernard {

constexpr uint8_t ACTUATORS_NUM = 6;

class ActuatorsControlNode : public rclcpp::Node {
   public:
    /**
     * @brief Constructor for ActuatorsControlNode
     * @param candle Pointer to CANdle instance
     * @param mds Reference to vector of MD instances
     */
    ActuatorsControlNode(std::unique_ptr<mab::Candle> candle, const std::vector<mab::MD>& mds);

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
    const mab::MD::Error_t getActuatorsMotionMode() const { return _actuator_motion_mode; };

    /**
     * @brief Zero encoders of all actuators
     * @return mab::MD::Error_t indicating success or failure
     */
    mab::MD::Error_t zeroEncoders();

    /**
     * @brief Blink selected actuator LEDs
     * @param joint_ids Vector of joint IDs to blink
     * @return mab::MD::Error_t indicating success or failure
     */
    mab::MD::Error_t blinkActuators(const std::vector<mab::MD>& mds);

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

    /**
     * @brief Get the current busy status
     * @return True if busy, false otherwise
     */
    bool isBusy() const { return _is_busy; };

   private:
    /// @brief Publish joint states of all robot actuators
    void publish_joint_states();

    /// @brief Publish joint temperatures of all robot actuators
    void publish_joint_temperatures();

    /// @brief Callback for joystick messages
    void joy_callback(const sensor_msgs::msg::Joy::SharedPtr msg);

    /// @brief Callback for command messages
    void command_callback(const std_msgs::msg::Float32MultiArray::SharedPtr msg);

    /// @brief Pointer to CANdle instance
    const std::unique_ptr<mab::Candle> _candle;

    /// @brief Reference to vector of MD instances
    const std::vector<mab::MD> _mds;

    /// @brief Message counter
    size_t count_;

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
    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr _joy_subscriber;

    /// @brief Actuator control mode
    ControlMode_t _actuator_motion_mode = mab::MdMode_E::IDLE;

    /// @brief Robot control mode
    RobotControlMode_t _control_mode = RobotControlMode_t::OFF;

    /// @brief Manual control selected motor index
    uint8_t _manual_control_actuator_idx = 0;

    /// @brief Stored joystick message
    sensor_msgs::msg::Joy::SharedPtr _last_joy_msg = nullptr;

    /// @brief If busy processing commands
    bool _is_busy = false;

    /// @brief Mutex for thread-safe operations
    std::mutex _mutex;
};

/// @brief Control modes for the robot
enum class RobotControlMode_t {
    OFF,
    MANUAL,
    RL_POLICY,
    HOLD_POSITION
};

}  // namespace bernard