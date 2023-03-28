#include "daemon.h"

using horizon::sagittarius::Daemon;

int main()
{
    Daemon daemon(30001);
    daemon.Run();
    return 0;
}
