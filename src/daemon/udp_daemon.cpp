#include "udp_daemon.h"

#include <atomic>
#include <chrono>
#include <cstdlib>
#include <cstring>
#include <string>

#include "spdlog/spdlog.h"

using boost::asio::ip::udp;

namespace horizon {
namespace sagittarius {
namespace {

// Convert a string like "1.23 4.5 3.3" into a vector<float> like {1.23, 4.5, 3.3}.
std::vector<float> ParseArray(char* text, size_t len) {
  std::vector<float> result{};
  size_t start = 0;
  bool active  = true;
  for (size_t i = 0; i < len; ++i) {
    if (std::isdigit(text[i]) || text[i] == '.') {
      if (active) continue;
      start  = i;
      active = true;
    } else if (active) {
      char* end = text + i - 1;
      result.emplace_back(std::strtof(text + start, &end));
      active = false;
    }
  }
  if (active) {
    char* end = text + len;
    result.emplace_back(std::strtof(text + start, &end));
  }
  return result;
}

}  // namespace

class UDPPusher {
 public:
  UDPPusher(sdk_sagittarius_arm::SagittariusArmReal* arm_low,
            const std::string& address,
            int port)
      : arm_low_(arm_low), address_(address), port_(port), thread_([this]() { Run(); }) {
  }

  ~UDPPusher() {
    kill_.store(true);
    thread_.join();
  }

  void Run() {
    boost::asio::io_service io_service;

    udp::resolver resolver(io_service);
    udp::endpoint listener_endpoint =
        *resolver.resolve({udp::v4(), address_, std::to_string(port_)}).begin();
    udp::socket socket(io_service);
    socket.open(udp::v4());

    boost::system::error_code error;

    // TODO(breakds): When the listener died it should be detected
    // here and the push thread should terminate.
    if (arm_low_ == nullptr) {
      for (int i = 0; i < 100; ++i) {
        int ret = socket.send_to(
            boost::asio::buffer(std::to_string(i)), listener_endpoint, 0, error);
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        if (kill_.load()) {
          break;
        }
      }
    }
    socket.close();
  }

 private:
  sdk_sagittarius_arm::SagittariusArmReal* arm_low_ = nullptr;
  std::string address_;
  int port_;
  std::thread thread_;
  std::atomic<bool> kill_{false};
};

UDPDaemon::UDPDaemon(int port) : port_(port) {
}

void UDPDaemon::Start(const std::string& device,
                      int baudrate,
                      int velocity,
                      int acceleration) {
  try {
    io_service_ = std::make_unique<boost::asio::io_service>();
    socket_ =
        std::make_unique<udp::socket>(*io_service_, udp::endpoint(udp::v4(), port_));
  } catch (std::exception& e) {
    spdlog::critical("Fatal error while creating the socket: {}", e.what());
    std::exit(EXIT_FAILURE);
  }

  arm_low_ = std::make_unique<sdk_sagittarius_arm::SagittariusArmReal>(
      device.c_str(), baudrate, velocity, acceleration);
  arm_low_->ControlTorque("lock");

  spdlog::info("UDP Daemon for Sagittarius K1 Arm started successfully.");

  // The messages should be very small, so the buffer size is sufficient.
  char data[1024];
  size_t content_size = 0;

  while (true) {
    udp::endpoint sender_endpoint;
    // Below is a blocking call that returns as soon as there is data in. The
    // information about the sender is stored in the sneder_endpoint and the
    // message content will be in data.
    try {
      content_size = socket_->receive_from(boost::asio::buffer(data), sender_endpoint);
      // TODO(breakds): Do I need to force set null ending?
    } catch (std::exception& e) {
      spdlog::error("Error in receiving from socket: {}", e.what());
      continue;
    }

    if (std::strncmp(data, CMD_STATUS, 6) == 0) {
      nlohmann::json status = GetStatus();
      socket_->send_to(boost::asio::buffer(status.dump()), sender_endpoint);
    } else if (std::strncmp(data, CMD_BOUNDS, 6) == 0) {
      nlohmann::json bounds = GetBounds();
      socket_->send_to(boost::asio::buffer(bounds.dump()), sender_endpoint);
    } else if (std::strncmp(data, CMD_SETPOS, 6) == 0) {
      std::vector<float> positions = ParseArray(data + 8, content_size - 8);
      // TODO(breakds): Check positions has 7 numbers.
      SetPosition(positions);
    } else {
      spdlog::error("Invalid command: {}", data);
    }
  }
}

void UDPDaemon::StartMock() {
  try {
    io_service_ = std::make_unique<boost::asio::io_service>();
    socket_ =
        std::make_unique<udp::socket>(*io_service_, udp::endpoint(udp::v4(), port_));
  } catch (std::exception& e) {
    spdlog::critical("Fatal error while creating the socket: {}", e.what());
    std::exit(EXIT_FAILURE);
  }

  spdlog::info("Start mocking UDP Daemon");
  std::unique_ptr<UDPPusher> pusher;

  // The messages should be very small, so the buffer size is sufficient.
  char data[1024];
  size_t content_size = 0;

  while (true) {
    udp::endpoint sender_endpoint;
    // Below is a blocking call that returns as soon as there is data in. The
    // information about the sender is stored in the sneder_endpoint and the
    // message content will be in data.
    try {
      content_size = socket_->receive_from(boost::asio::buffer(data), sender_endpoint);
      // TODO(breakds): Do I need to force set null ending?
    } catch (std::exception& e) {
      spdlog::error("Error in receiving from socket: {}", e.what());
      continue;
    }

    if (std::strncmp(data, CMD_STATUS, 6) == 0) {
      spdlog::info("STATUS!");
      nlohmann::json status = ArmStatus{
          .joint_positions = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0},
          .gripper =
              {
                  .radian   = 0.0,
                  .distance = 0.0,
              },
      };
      socket_->send_to(boost::asio::buffer(status.dump()), sender_endpoint);
    } else if (std::strncmp(data, CMD_BOUNDS, 6) == 0) {
      nlohmann::json bounds = {
          PositionBound(-1.0, 1.0),
          PositionBound(-1.0, 1.0),
          PositionBound(-1.0, 1.0),
          PositionBound(-1.0, 1.0),
          PositionBound(-1.0, 1.0),
          PositionBound(-1.0, 1.0),
          PositionBound(-0.068, 0.0),
      };
      socket_->send_to(boost::asio::buffer(bounds.dump()), sender_endpoint);
    } else if (std::strncmp(data, CMD_SETPOS, 6) == 0) {
      std::vector<float> positions = ParseArray(data + 8, content_size - 8);
      spdlog::info("Set joint positions to [{}, {}, {}, {}, {}, {}] + {}",
                   positions[0],
                   positions[1],
                   positions[2],
                   positions[3],
                   positions[4],
                   positions[5],
                   positions[6]);
    } else if (std::strncmp(data, CMD_LISTEN, 6) == 0) {
      std::string listener_address = sender_endpoint.address().to_string();
      int listener_port            = std::stoi(std::string(data + 7, content_size - 7));
      spdlog::info("Request LISTEN from {}:{}", listener_address, listener_port);
      pusher = std::make_unique<UDPPusher>(nullptr, listener_address, listener_port);
    } else {
      spdlog::error("Invalid command: {}", data);
    }
  }
}

void UDPDaemon::SetPosition(const std::vector<float>& positions) {
  if (positions.size() != 7) {
    spdlog::error("SetPosition() cannot handle inputs with a size of {}",
                  positions.size());
    return;
  }
  arm_low_->SetAllServoRadian(const_cast<float*>(positions.data()));
  arm_low_->arm_set_gripper_linear_position(positions[6]);
}

ArmStatus UDPDaemon::GetStatus() {
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
              .radian   = joints[6],
              .distance = gripper_dist,
          },
  };
}

std::vector<PositionBound> UDPDaemon::GetBounds() {
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
