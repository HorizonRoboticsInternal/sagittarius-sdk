#include "daemon.h"

using horizon::sagittarius::Daemon;

int main() {
  Daemon daemon(30001);
  daemon.Start(1000000, 1000, 5);
  return 0;
}
