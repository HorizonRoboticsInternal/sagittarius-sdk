#pragma once

#include "websocketpp/config/asio_no_tls.hpp"
#include "websocketpp/server.hpp"

namespace horizon
{
    namespace sagittarius
    {
        class Daemon
        {
          public:
            using Server = websocketpp::server<websocketpp::config::asio>;
            using Connection = websocketpp::connection_hdl;

            Daemon(int port);
            void Run();

          private:
            void OnMessage(Connection hdl,
                           websocketpp::config::asio::message_type::ptr msg);
            void SetPosition();
            void SetEndEffector();
            void GetStatus();

            int port_;
            Server server_;
            std::vector<Connection> connections_{};
        };

    } // namespace sagittarius
} // namespace horizon
