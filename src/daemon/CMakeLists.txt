find_package(websocketpp REQUIRED)
find_package(spdlog REQUIRED)

add_executable(sagittariusd)
target_sources(sagittariusd PRIVATE sagittariusd.cpp)
target_link_libraries(sagittariusd PRIVATE websocketpp::websocketpp spdlog::spdlog)
