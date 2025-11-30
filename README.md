# BERNARD Hardware Interface (bernard_hw_if)

## Overview

This ROS 2 package provides the **hardware interface** for the BERNARD bipedal robot. It handles real-time communication with the MD80 servo actuators via the CANdle CAN interface and publishes sensor data (joint states, IMU, foot pressure) received from the STM32 microcontroller. This package is part of the larger [BERNARD Bipedal Robot Project](https://github.com/Baey/bernard-bipedal-robot).

**Key Features:**

- **CAN Communication**: Real-time control of 6 MD80 servo actuators via CANdle interface
- **Robot Control Modes**: OFF, MANUAL (joystick), RL_POLICY (reinforcement learning), and HOLD_POSITION
- **Sensor Integration**: Publishes IMU data, joint states, and foot pressure sensor readings from STM32 microcontroller
- **JointState Interface**: Standard ROS 2 JointState messages for compatibility with control stacks
- **Robust Error Handling**: Hardware fault detection and graceful error recovery

**Project Structure:**

This repository is part of a larger ecosystem:

- **[bernard-bipedal-robot](https://github.com/Baey/bernard-bipedal-robot)**: Main monorepo containing all submodules
- **[bernard-rl](https://github.com/Baey/bernard-rl)**: IsaacLab extension with RL environments for simulation training
- **[bernard-stm32-ros-node](https://github.com/Baey/bernard-stm32-ros-node)**: Firmware for STM32 microcontroller (IMU, foot pressure sensors, ROS bridge)

---

## System Architecture

```
┌─────────────────────┐
│   RL Policy /       ├───────────────┐
│ Joystick Input      │               │
└──────────▲──────────┘               │
           │                          │
      ┌────▼──────────────────────┐   │
      │   bernard_hw_if (ROS 2)   │   │
      │  - MD80 actuator control  │   │
      │  - Sensor data fusion     │   │
      └───▲────────────▲──────────┘   │
          │ CAN Bus    │ Serial/TCP   │
      ┌───▼────┐  ┌────┴──────────┐   │
      │ MD80   │  │   STM32 uC    │   │
      │ Drives │  │ - IMU (BNO055)├───┘
      └────────┘  │ - Foot sensors│
                  │ - micro-ROS   │
                  └───────────────┘
```

---

## Requirements

### Hardware

- **STM32 Microcontroller** (e.g., Nucleo L476RG or similar)
  - BNO055 IMU for orientation/acceleration data
  - Analog pressure sensors for foot contact detection
  - TFT display for GUI (optional)
  
- **MD80 Servo Actuators** (6x)
  - CAN interface compatible
  
- **CANdle CAN Interface** for CAN communication
  - Supports real-time actuator control at 200 Hz+

### Software

- **ROS 2** (Foxy or later)
- **C++20** or later
- **ament_cmake** build system
- **rclcpp** - ROS 2 C++ client library
- **CANdle SDK** (included as submodule)

---

## Installation

### 1. Clone the Repository

```bash
# Clone the main repository with all submodules
git clone --recurse-submodules https://github.com/Baey/bernard-bipedal-robot.git
cd bernard-bipedal-robot
```

### 2. Install Dependencies

#### On Ubuntu/Debian:

```bash
# Install ROS 2 (if not already installed)
# Follow: https://docs.ros.org/en/iron/Installation.html

# Source ROS 2 environment
source /opt/ros/$ROS_DISTRO/setup.bash

# Install build dependencies
sudo apt-get install -y \
    ros-$ROS_DISTRO-rclcpp \
    ros-$ROS_DISTRO-sensor-msgs \
    build-essential \
    cmake

# Install colcon build tool
sudo apt-get install -y python3-colcon-common-extensions
```

### 3. Build the Package

```bash
# Navigate to the workspace root
cd bernard-bipedal-robot

# Source ROS 2
source /opt/ros/$ROS_DISTRO/setup.bash

# Build the package using colcon
colcon build --packages-select bernard_hw_if

# Source the install space
source install/local_setup.bash
```

---

## Usage

### Running the Node

```bash
# Source the ROS 2 environment
source /opt/ros/$ROS_DISTRO/setup.bash
source install/local_setup.bash

# Launch the actuators node
ros2 run bernard_hw_if actuators_node

# Or with a launch file (if available)
ros2 launch bernard_hw_if actuators.launch.py
```

### Control Modes

The hardware interface supports multiple control modes:

1. **OFF**: All actuators disabled (safe state)
2. **MANUAL**: Real-time joystick/gamepad control via joy input
3. **RL_POLICY**: Receives commanded actions from RL policy running in simulation
4. **HOLD_POSITION**: Maintains current joint positions (compliant control)

### Published Topics

| Topic | Message Type | Description |
|-------|--------------|-------------|
| `/joint_states` | `sensor_msgs/JointState` | Current joint positions, velocities, and torques |
| `/imu` | `sensor_msgs/Imu` | IMU accelerations and angular velocities |
| `/feet_pressure` | `std_msgs/Float32MultiArray` | Foot contact pressure sensor readings |
| `/status` | Custom msg (optional) | Robot system status |

### Subscribed Topics

| Topic | Message Type | Description |
|-------|--------------|-------------|
| `/joy` | `sensor_msgs/Joy` | Joystick input for manual control |
| `/actions` | `std_msgs/Float32MultiArray` | Action commands from RL policy |
| `/mode_command` | Custom msg (optional) | Robot control mode selection |

---

## Configuration

Configuration parameters are defined in `include/config.hpp`:

```cpp
// Number of actuators
constexpr uint8_t ACTUATORS_NUM = 6;

// Timing parameters
constexpr std::chrono::milliseconds ZEROING_BLINK_INTERVAL = std::chrono::milliseconds(1500);
constexpr std::chrono::milliseconds MANUAL_SELECTION_BLINK_INTERVAL = std::chrono::milliseconds(2500);

// CAN/actuator specific parameters
```

Modify these values to match your hardware configuration.

---

## Hardware Interface Details

### MD80 Actuators

The interface communicates with 6 MD80 servo actuators over CAN bus. Each actuator provides:
- Joint position feedback
- Velocity feedback  
- Torque feedback
- Temperature monitoring

### STM32 Microcontroller Integration

The STM32 node (from [bernard-stm32-ros-node](https://github.com/Baey/bernard-stm32-ros-node)) communicates IMU and foot pressure data via:
- **Serial connection** for real-time sensor data
- **micro-ROS** bridge for ROS topic integration

### Joystick Control

In MANUAL mode, joystick input is processed from Joy messages. Map your gamepad axes/buttons as needed in the code.

---

## Building and Testing

### Build the Package

```bash
colcon build --packages-select bernard_hw_if
```

### Run Tests

```bash
# Run unit tests
colcon test --packages-select bernard_hw_if --executor sequential

# View test results
colcon test-result --verbose
```

### Build Options

You can customize the build with CMake flags:

```bash
colcon build --packages-select bernard_hw_if \
    -DCMAKE_BUILD_TYPE=Release \
    -DBUILD_TESTING=ON
```

---

## Troubleshooting

### CAN Interface Not Found

Ensure the CANdle interface is properly connected and recognized by the system:

```bash
# List connected USB devices
lsusb

# Check for CAN interfaces
ip link show
```

### Actuators Not Responding

1. Verify CAN bus wiring and termination
2. Check actuator power supply
3. Run diagnostic command to test communication

### Sensor Data Not Received

1. Verify STM32 is running and connected
2. Check serial port permissions: `ls -l /dev/ttyUSB*`
3. Ensure micro-ROS agent is running

---

## Related Repositories

- **[bernard-bipedal-robot](https://github.com/Baey/bernard-bipedal-robot)**: Main repository with all submodules
- **[bernard-rl](https://github.com/Baey/bernard-rl)**: Reinforcement learning extension for Isaac Lab
- **[bernard-stm32-ros-node](https://github.com/Baey/bernard-stm32-ros-node)**: STM32 firmware for sensor interface

---

## Project Information

**Author:** Błażej Szargut  
**License:** BSD-3-Clause  
**Affiliation:** AGH University of Krakow, Master's Thesis Project

This hardware interface is part of a comprehensive bipedal robot platform combining:
- Real-time hardware control via ROS 2
- Advanced reinforcement learning in simulation (Isaac Lab)
- Sensor fusion from multiple sources (IMU, joint feedback, foot pressure)

---

## Contributing

To contribute improvements or bug fixes:

1. Fork the repository
2. Create a feature branch
3. Make your changes
4. Test thoroughly
5. Submit a pull request

Please follow the existing code style and include appropriate tests for new features.

---

## Acknowledgments

This project is developed as a Master's thesis at AGH University of Krakow. Special thanks to:
- The Isaac Lab and IsaacSim teams for simulation framework
- The ROS 2 community for middleware and tools
- Contributors to the CANdle SDK

---

## License

This project is licensed under the **BSD-3-Clause License**. See `LICENSE` file for details.

---

## Quick Start Example

```bash
# Terminal 1: Start the hardware interface node
source /opt/ros/$ROS_DISTRO/setup.bash
source ~/ws/install/local_setup.bash
ros2 run bernard_hw_if actuators_node

# Terminal 2: Publish a manual control command
ros2 topic pub /actions std_msgs/msg/Float32MultiArray \
    '{data: [0.5, -0.3, 0.2, 0.1, -0.4, 0.25]}'

# Terminal 3: Monitor joint states
ros2 topic echo /joint_states
```

For more advanced usage and RL training, see the [bernard-rl](https://github.com/Baey/bernard-rl) repository.
