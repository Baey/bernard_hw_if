#pragma once
#include <string>
#include <unordered_map>

namespace bernard {
static const std::unordered_map<int, std::string> JOINT_MAP = {
    {106, "r_hip1_joint"},
    {1105, "l_hip1_joint"},
    {1108, "r_hip2_joint"},
    {107, "l_hip2_joint"},
    {1106, "r_knee_joint"},
    {108, "l_knee_joint"},
};
}  // namespace bernard
