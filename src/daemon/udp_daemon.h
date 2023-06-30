#pragma once

#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include "boost/asio.hpp"
#include "protocols.h"
#include "sdk_sagittarius_arm/sdk_sagittarius_arm_real.h"

namespace horizon {
namespace sagittarius {

class UDPDaemon {
 public:
  using udp = boost::asio::ip::udp;

  static constexpr char CMD_STATUS[] = "STATUS";
  static constexpr char CMD_BOUNDS[] = "BOUNDS";
  static constexpr char CMD_SETPOS[] = "SETPOS";
  static constexpr char CMD_LISTEN[] = "LISTEN";

  explicit UDPDaemon(int port, bool sync=false);

  // @param baudrate - usually 1_000_000
  //
  // @prama velocity - the maximum operation velocity of the servos. The value
  //                   should be between 0 ~ 4096. Recommended 1000.
  //
  // @param acceleration - The acceleration of the servo. It should take value
  //                       between 0 and 254. Recommended <= 10.
  void Start(const std::string& device, int baudrate, int velocity, int acceleration);

  // This does not actually connect to the Sagittarius robotic arm and
  // is useful for testing UDP networking.
  void StartMock();

 private:
  // ---------- Actual Command Handlers ----------

  // Command: STATUS
  ArmStatus GetStatus();

  // Command: SETPOS
  void SetPosition(const std::vector<float>& positions);

  // Command: BOUNDS
  std::vector<PositionBound> GetBounds();

  int port_;
  std::unique_ptr<boost::asio::io_service> io_service_{};
  std::unique_ptr<udp::socket> socket_{};

  // Low level arm interface.
  std::unique_ptr<sdk_sagittarius_arm::SagittariusArmReal> arm_low_;
  bool sync_mode_ = false;
};

}  // namespace sagittarius
}  // namespace horizon
