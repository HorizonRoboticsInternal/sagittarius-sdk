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

    while (true) {
      std::cout << "SAGITTARIUS CLIENT> ";
      std::cout.flush();

      std::string message;
      std::getline(std::cin, message);

      socket.send_to(boost::asio::buffer(message), receiver_endpoint);

      // Wait for a response
      udp::endpoint sender_endpoint;
      size_t len = socket.receive_from(boost::asio::buffer(buffer), sender_endpoint);
      std::cout.write(buffer, len);
      std::cout << std::endl;
    }
  } catch (std::exception& e) {
    std::cerr << e.what() << std::endl;
  }

  return 0;
}
