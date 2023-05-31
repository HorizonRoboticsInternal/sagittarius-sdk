#include <iostream>
#include <boost/array.hpp>
#include <boost/asio.hpp>

using boost::asio::ip::udp;

int main() {
    try {
        boost::asio::io_service io_service;

        udp::resolver resolver(io_service);
        udp::endpoint receiver_endpoint = *resolver.resolve({udp::v4(), "localhost", "12345"}).begin();
        udp::socket socket(io_service);
        socket.open(udp::v4());

        socket.send_to(boost::asio::buffer("SETPO1"), receiver_endpoint);

        boost::array<char, 128> recv_buf;
        udp::endpoint sender_endpoint;

        size_t len = socket.receive_from(boost::asio::buffer(recv_buf), sender_endpoint);

        std::cout.write(recv_buf.data(), len);
    }
    catch (std::exception& e) {
        std::cerr << e.what() << std::endl;
    }

    return 0;
}
