#include <array>

#include "nlohmann/json.hpp"

namespace horizon {
namespace sagittarius {

struct GripperStatus {
  float radian;
  float distance;
  NLOHMANN_DEFINE_TYPE_INTRUSIVE(GripperStatus, radian, distance);
};

struct ArmStatus {
  std::array<float, 6> joint_positions;
  GripperStatus gripper;
  NLOHMANN_DEFINE_TYPE_INTRUSIVE(ArmStatus, joint_positions, gripper);
};

}  // namespace sagittarius
}  // namespace horizon
