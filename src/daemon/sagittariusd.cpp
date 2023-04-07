#include "daemon.h"

using horizon::sagittarius::Daemon;

int main() {
  Daemon daemon(30001);
  daemon.Start(1000000, 1600, 20);
  return 0;
}
