#include <cstring>
#include <iostream>

#include "boost/asio.hpp"

using boost::asio::ip::udp;

int main() {
  try {
    boost::asio::io_service io_service;

    udp::resolver resolver(io_service);
    udp::endpoint receiver_endpoint =
        *resolver.resolve({udp::v4(), "localhost", "30001"}).begin();
    udp::socket socket(io_service);
    socket.open(udp::v4());

    char buffer[1024];

    std::cout << "This is C++ UDP client REPL for Sagittarius K1 Arm.\n";
    std::cout << "There are 3 types of commands that you can use.\n\n";
    std::cout << "STATUS\n";
    std::cout << "BOUNDS\n";
    std::cout << "SETPOS 0.1 0 0 0 0 0 0\n\n";
    std::cout << "Here SETPOS needs to be followed by 7 floating numbers.\n\n";

    while (true) {
      std::cout << "INPUT COMMAND> ";
      std::cout.flush();

      std::string message;
      std::getline(std::cin, message);

      socket.send_to(boost::asio::buffer(message), receiver_endpoint);

      // Wait for a response
      if (strncmp(message.c_str(), "SETPOS", 6) != 0) {
        udp::endpoint sender_endpoint;
        size_t len = socket.receive_from(boost::asio::buffer(buffer), sender_endpoint);
        std::cout.write(buffer, len);
        std::cout << std::endl;
      }
    }
  } catch (std::exception& e) {
    std::cerr << e.what() << std::endl;
  }

  return 0;
}
