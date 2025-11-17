#pragma once

#include "MD.hpp"
#include "actuators_node.hpp"
#include "candle.hpp"

/** @brief Convert MD motion mode enum to string
 * @param mode Motion mode enum value
 * @return String representation of the motion mode
 */
std::string motionModeToString(const mab::MdMode_E mode);

namespace bernard {
/** @brief Convert MD ID enum to joint name string
 * @param id MD ID enum value
 * @return String representation of the joint name
 */
std::string mdIdToJointName(const mab::canId_t id);

/** @brief Convert robot control mode enum to string
 * @param mode Robot control mode enum value
 * @return String representation of the robot control mode
 */
std::string robotControlModeToString(const RobotControlMode_t mode);
}  // namespace bernard