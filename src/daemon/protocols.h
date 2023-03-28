#include <array>

#include "nlohmann/json.hpp"

namespace horizon {
namespace sagittarius {

struct ArmStatus {
  std::array<float, 6> joint_positions;
  NLOHMANN_DEFINE_TYPE_INTRUSIVE(ArmStatus, joint_positions);
};

}  // namespace sagittarius
}  // namespace horizon
