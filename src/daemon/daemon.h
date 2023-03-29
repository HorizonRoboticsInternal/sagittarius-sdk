#pragma once

#include <memory>
#include <mutex>
#include <vector>

#include "protocols.h"
#include "sdk_sagittarius_arm/sdk_sagittarius_arm_real.h"
#include "websocketpp/config/asio_no_tls.hpp"
#include "websocketpp/server.hpp"

namespace horizon {
namespace sagittarius {

class Daemon {
 public:
  using Server     = websocketpp::server<websocketpp::config::asio>;
  using Connection = websocketpp::connection_hdl;

  Daemon(int port);

  // @param baudrate - usually 1_000_000
  //
  // @prama velocity - the maximum operation velocity of the servos. The value
  //                   should be between 0 ~ 4096. Recommended 1000.
  //
  // @param acceleration - The acceleration of the servo. It should take value
  //                       between 0 and 254. Recommended <= 10.
  void Start(int baudrate, int velocity, int acceleration);

 private:
  // ---------- Handlers ----------
  void OnMessage(Connection conn, websocketpp::config::asio::message_type::ptr msg);
  void OnOpen(Connection conn);
  void OnClose(Connection conn);
  void Send(Connection conn, const std::string &message);

  // ---------- Processors ----------
  // Command: BOUNDS
  std::vector<PositionBound> GetBounds();

  // Command: SETPOS
  void SetPosition(const std::vector<float> &positions);
  // Command: SET_EE
  void SetEndEffector();
  // Command: STATUS
  ArmStatus GetStatus();

  int port_;
  Server server_;
  std::mutex mutex_{};  // protects connections_ below.
  std::vector<Connection> connections_{};

  // Low level arm interface.
  std::unique_ptr<sdk_sagittarius_arm::SagittariusArmReal> arm_low_;
};

}  // namespace sagittarius
}  // namespace horizon
