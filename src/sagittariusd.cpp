#include "websocketpp/config/asio_no_tls.hpp"
#include "websocketpp/server.hpp"

using websocket_server = websocketpp::server<websocketpp::config::asio>;

void on_message(websocket_server* server, websocketpp::connection_hdl hdl,
                websocket_server::message_ptr msg)
{
    try
    {
        server->send(hdl, msg->get_payload(), msg->get_opcode());
    }
    catch (websocketpp::exception const& e)
    {
        std::cout << "Echo failed because: " << e.what() << std::endl;
    }
}

int main()
{
    websocket_server server;
    server.set_message_handler(&on_message);

    server.init_asio();
    server.listen(9002);
    server.start_accept();

    std::cout << "WebSocket server listening on port 9002" << std::endl;

    server.run();

    return 0;
}
