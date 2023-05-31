#include "udp_daemon.h"

using horizon::sagittarius::UDPDaemon;

int main() {
  UDPDaemon daemon(30001);
  daemon.Start(1000000, 1600, 20);
  return 0;
}
