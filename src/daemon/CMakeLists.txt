find_package(spdlog REQUIRED)
find_package(nlohmann_json REQUIRED)

add_library(protocols INTERFACE)
target_sources(protocols INTERFACE protocols.h)
target_link_libraries(protocols INTERFACE nlohmann_json::nlohmann_json)

# +----------------------------------------+
# | The UDP-based Server                   |
# +----------------------------------------+

add_library(udp_daemon STATIC)
target_include_directories(udp_daemon PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
target_sources(udp_daemon
    PUBLIC udp_daemon.h
    PRIVATE udp_daemon.cpp)
target_link_libraries(udp_daemon PUBLIC
    sagittarius_sdk_static
    protocols
    spdlog::spdlog)

add_executable(udp_client)
target_sources(udp_client PRIVATE udp_client.cpp)

add_executable(sagittariusd)
target_include_directories(sagittariusd PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
target_sources(sagittariusd PRIVATE sagittariusd.cpp)
target_link_libraries(sagittariusd PRIVATE udp_daemon)
