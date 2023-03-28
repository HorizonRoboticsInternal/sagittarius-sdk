#include <functional>

#include "spdlog/spdlog.h"
#include "websocketpp/config/asio_no_tls.hpp"
#include "websocketpp/server.hpp"

using Server = websocketpp::server<websocketpp::config::asio>;
using ConnectionHdl = websocketpp::connection_hdl;

void on_message(Server* server, std::vector<ConnectionHdl>* connections,
                ConnectionHdl hdl,
                websocketpp::config::asio::message_type::ptr msg)
{
    spdlog::info("payload: {}", msg->get_payload());

    try
    {
        for (ConnectionHdl conn : *connections)
        {
            spdlog::info("This!");
            server->send(conn, msg->get_payload(),
                         websocketpp ::frame::opcode::text);
        }
        server->send(hdl, msg->get_payload(),
                     websocketpp ::frame::opcode::text);
    }
    catch (websocketpp::exception const& e)
    {
        spdlog::error("Failed because: {}", e.what());
    }
}

int main()
{
    using namespace std::placeholders;

    Server server;
    std::vector<ConnectionHdl> connections;

    // Turn off original logs
    server.clear_access_channels(websocketpp::log::alevel::all);
    server.clear_error_channels(websocketpp::log::elevel::all);

    server.set_message_handler(
        std::bind(on_message, &server, &connections, _1, _2));

    server.init_asio();
    server.listen(30001);
    server.start_accept();

    spdlog::info("Websocket server listening on port 30001");

    server.run();

    return 0;
}
