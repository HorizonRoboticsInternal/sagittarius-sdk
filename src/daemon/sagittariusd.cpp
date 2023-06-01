#include <cstdlib>
#include <cstring>
#include <string>
#include <type_traits>

#include "spdlog/spdlog.h"
#include "udp_daemon.h"

using horizon::sagittarius::UDPDaemon;

template <typename T>
T GetEnv(const char *name, T default_value) {
  const char *value = std::getenv(name);
  if (value == nullptr) {
    return default_value;
  }

  if constexpr (std::is_same_v<T, int>) {
    return std::stoi(value);
  } else if constexpr (std::is_same_v<T, bool>) {
    return (std::strcmp(value, "yes") == 0 || std::strcmp(value, "1") == 0 ||
            std::strcmp(value, "true") == 0 || std::strcmp(value, "T") == 0);
  } else if constexpr (std::is_same_v<T, std::string>) {
    return std::string(value);
  } else {
    spdlog::critical("Unspported type for GetEnv, name = '{}'", name);
    std::exit(EXIT_FAILURE);
  }
}

int main() {
  UDPDaemon daemon(GetEnv<int>("SAGITTARIUSD_PORT", 30001));

  if (GetEnv<bool>("SAGITTARIUSD_MOCK", false)) {
    // Run in mock mode
    daemon.StartMock();
  } else {
    std::string device = GetEnv<std::string>("SAGITTARIUSD_DEVICE", "/dev/ttyACM0");
    int baudrate       = GetEnv<int>("SAGITTARIUSD_BAUDRATE", 1'000'000);
    int velocity       = GetEnv<int>("SAGITTARIUSD_VELOCITY", 1'600);
    int acceleration   = GetEnv<int>("SAGITTARIUSD_ACCELERATION", 20);
    daemon.Start(device, baudrate, velocity, acceleration);
  }
  return 0;
}
