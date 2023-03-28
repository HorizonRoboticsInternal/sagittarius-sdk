#pragma once

#include <mutex>

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
            // ---------- Handlers ----------
            void OnMessage(Connection conn,
                           websocketpp::config::asio::message_type::ptr msg);
            void OnOpen(Connection conn);
            void OnClose(Connection conn);

            // ---------- Processors ----------
            void SetPosition();
            void SetEndEffector();
            void GetStatus();

            int port_;
            Server server_;
            std::mutex mutex_{}; // protects connections_ below.
            std::vector<Connection> connections_{};
        };

    } // namespace sagittarius
} // namespace horizon
