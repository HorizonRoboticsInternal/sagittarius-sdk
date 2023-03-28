#include <functional>

#include "spdlog/spdlog.h"

#include "daemon.h"

namespace horizon
{
    namespace sagittarius
    {

        Daemon::Daemon(int port) : port_(port), server_{}
        {
            using namespace std::placeholders;

            // Turn off original logs
            server_.clear_access_channels(websocketpp::log::alevel::all);
            server_.clear_error_channels(websocketpp::log::elevel::all);

            server_.set_message_handler(
                std::bind(&Daemon::OnMessage, this, _1, _2));
        }

        void Daemon::OnMessage(Daemon::Connection hdl,
                               websocketpp::config::asio::message_type::ptr msg)
        {
            spdlog::info("payload: {}", msg->get_payload());

            try
            {
                server_.send(hdl, msg->get_payload(),
                             websocketpp ::frame::opcode::text);
            }
            catch (websocketpp::exception const& e)
            {
                spdlog::error("Failed because: {}", e.what());
            }
        }

        void Daemon::Run()
        {
            server_.init_asio();
            server_.listen(port_);
            server_.start_accept();
            spdlog::info("Websocket server listening on port {}", port_);
            server_.run();
        }

    } // namespace sagittarius
} // namespace horizon
