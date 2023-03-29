#include "daemon.h"

#include <functional>
#include <mutex>
#include <string_view>

#include "nlohmann/json.hpp"
#include "sdk_sagittarius_arm/sdk_sagittarius_arm_constants.h"
#include "spdlog/spdlog.h"

namespace horizon {
namespace sagittarius {

Daemon::Daemon(int port) : port_(port), server_{} {
  using namespace std::placeholders;

  // Turn off original logs
  server_.clear_access_channels(websocketpp::log::alevel::all);
  server_.clear_error_channels(websocketpp::log::elevel::all);

  server_.set_message_handler(std::bind(&Daemon::OnMessage, this, _1, _2));
  server_.set_open_handler(std::bind(&Daemon::OnOpen, this, _1));
  server_.set_close_handler(std::bind(&Daemon::OnClose, this, _1));
}

void Daemon::OnMessage(Connection conn,
                       websocketpp::config::asio::message_type::ptr msg) {
  const std::string& payload = msg->get_payload();

  if (payload.size() < 6) {
    spdlog::error("Payload size too small.");
    return;
  }

  const std::string_view command(payload.data(), 6);
  if (command == "STATUS" || command == "status") {
    nlohmann::json status = GetStatus();
    Send(conn, status.dump());
  } else if (command == "SETPOS" || command == "setpos") {
    nlohmann::json json_list =
        nlohmann::json::parse(std::string_view(payload.data() + 7, payload.data() - 7));
    std::vector<float> positions = json_list.get<std::vector<float>>();
    SetPosition(positions);
  } else if (command == "BOUNDS" || command == "bounds") {
    nlohmann::json bounds = GetBounds();
    Send(conn, bounds.dump());
  } else {
    spdlog::error("Invalid command '{}'", command);
  }
}

void Daemon::OnOpen(Connection conn) {
  {
    std::lock_guard<std::mutex> lock(mutex_);
    connections_.emplace_back(conn);
  }
  spdlog::info(
      "New connection established. "
      "Active connections: {}",
      connections_.size());
}

void Daemon::OnClose(Connection conn) {
  {
    std::lock_guard<std::mutex> lock(mutex_);
    connections_.erase(
        std::remove_if(connections_.begin(),
                       connections_.end(),
                       [&conn](Connection x) { return x.lock() == conn.lock(); }),
        connections_.end());
  }

  spdlog::info(
      "Connection closed."
      "Active connections: {}",
      connections_.size());
}

void Daemon::Send(Connection conn, const std::string& message) {
  try {
    server_.send(conn, message, websocketpp::frame::opcode::text);
  } catch (websocketpp::exception const& e) {
    spdlog::error("Send() failed because: {}", e.what());
  }
}

void Daemon::Start(int baudrate, int velocity, int acceleration) {
  // Connect to the robotic arm
  arm_low_ = std::make_unique<sdk_sagittarius_arm::SagittariusArmReal>(
      "/dev/ttyACM0", baudrate, velocity, acceleration);
  arm_low_->ControlTorque("lock");

  server_.init_asio();
  server_.listen(port_);
  server_.start_accept();
  spdlog::info("Websocket server listening on port {}", port_);
  server_.run();
}

ArmStatus Daemon::GetStatus() {
  float joints[7];
  arm_low_->GetCurrentJointStatus(joints);

  // The conversion here is based on
  // SagittariusArmReal::arm_calculate_gripper_degree_position()
  double gripper_radian = static_cast<double>(joints[6]);
  double gripper_degree = -gripper_radian / PI * 180.0;
  float gripper_dist    = static_cast<float>(gripper_degree / 3462.0 * 2.0);

  return ArmStatus{
      .joint_positions =
          {joints[0], joints[1], joints[2], joints[3], joints[4], joints[5]},
      .gripper =
          {
              .radian = joints[6],
              // TODO(breakds): Compute the distance from radian
              .distance = gripper_dist,
          },
  };
}

void Daemon::SetPosition(const std::vector<float>& positions) {
  if (positions.size() != 7) {
    spdlog::error("SetPosition() cannot handle inputs with a size of {}",
                  positions.size());
    return;
  }
  arm_low_->SetAllServoRadian(const_cast<float*>(positions.data()));
  arm_low_->arm_set_gripper_linear_position(positions[6]);
  spdlog::info("Distance: {}, Degree: {}",
               positions[6],
               arm_low_->arm_calculate_gripper_degree_position(positions[6]));
}

std::vector<PositionBound> Daemon::GetBounds() {
  return {
      PositionBound(arm_low_->lower_joint_limits[0], arm_low_->upper_joint_limits[0]),
      PositionBound(arm_low_->lower_joint_limits[1], arm_low_->upper_joint_limits[1]),
      PositionBound(arm_low_->lower_joint_limits[2], arm_low_->upper_joint_limits[2]),
      PositionBound(arm_low_->lower_joint_limits[3], arm_low_->upper_joint_limits[3]),
      PositionBound(arm_low_->lower_joint_limits[4], arm_low_->upper_joint_limits[4]),
      PositionBound(arm_low_->lower_joint_limits[5], arm_low_->upper_joint_limits[5]),
      PositionBound(-0.068, 0.0),
  };
}

}  // namespace sagittarius
}  // namespace horizon
