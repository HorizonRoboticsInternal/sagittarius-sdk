find_package(websocketpp REQUIRED)
find_package(spdlog REQUIRED)
find_package(nlohmann_json REQUIRED)

add_library(protocols INTERFACE)
target_sources(protocols INTERFACE protocols.h)
target_link_libraries(protocols INTERFACE nlohmann_json::nlohmann_json)

add_library(daemon STATIC)
target_include_directories(daemon PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
target_sources(daemon 
    PUBLIC daemon.h
    PRIVATE daemon.cpp)
target_link_libraries(daemon PUBLIC
    sagittarius_sdk
    protocols
    websocketpp::websocketpp
    spdlog::spdlog)

add_executable(sagittariusd)
target_include_directories(sagittariusd PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
target_sources(sagittariusd PRIVATE sagittariusd.cpp)
target_link_libraries(sagittariusd PRIVATE daemon)
