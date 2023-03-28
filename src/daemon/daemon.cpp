#include "daemon.h"

#include <functional>
#include <mutex>

#include "spdlog/spdlog.h"

namespace horizon {
namespace sagittarius {

Daemon::Daemon(int port) : port_(port), server_{} {
  using namespace std::placeholders;

  // Turn off original logs
  server_.clear_access_channels(websocketpp::log::alevel::all);
  server_.clear_error_channels(websocketpp::log::elevel::all);

  server_.set_message_handler(std::bind(&Daemon::OnMessage, this, _1, _2));
  server_.set_open_handler(std::bind(&Daemon::OnOpen, this, _1));
  server_.set_close_handler(std::bind(&Daemon::OnClose, this, _1));
}

void Daemon::OnMessage(Connection conn,
                       websocketpp::config::asio::message_type::ptr msg) {
  spdlog::info("payload: {}", msg->get_payload());

  try {
    server_.send(conn, msg->get_payload(), websocketpp ::frame::opcode::text);
  } catch (websocketpp::exception const& e) {
    spdlog::error("Failed because: {}", e.what());
  }
}

void Daemon::OnOpen(Connection conn) {
  {
    std::lock_guard<std::mutex> lock(mutex_);
    connections_.emplace_back(conn);
  }
  spdlog::info(
      "New connection established. "
      "Active connections: {}",
      connections_.size());
}

void Daemon::OnClose(Connection conn) {
  {
    std::lock_guard<std::mutex> lock(mutex_);
    connections_.erase(
        std::remove_if(connections_.begin(),
                       connections_.end(),
                       [&conn](Connection x) { return x.lock() == conn.lock(); }),
        connections_.end());
  }

  spdlog::info(
      "Connection closed."
      "Active connections: {}",
      connections_.size());
}

void Daemon::Run() {
  server_.init_asio();
  server_.listen(port_);
  server_.start_accept();
  spdlog::info("Websocket server listening on port {}", port_);
  server_.run();
}

}  // namespace sagittarius
}  // namespace horizon
