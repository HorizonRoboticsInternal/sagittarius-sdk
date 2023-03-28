find_package(websocketpp REQUIRED)
find_package(spdlog REQUIRED)
find_package(nlohmann_json REQUIRED)

message(STATUS "${CMAKE_CURRENT_SOURCE_DIR}")
add_library(daemon STATIC)
target_include_directories(daemon PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
    
target_sources(daemon 
    PUBLIC daemon.h
    PRIVATE daemon.cpp)
target_link_libraries(daemon PUBLIC
    nlohmann_json::nlohmann_json
    websocketpp::websocketpp
    spdlog::spdlog)

add_executable(sagittariusd)
target_sources(sagittariusd PRIVATE sagittariusd.cpp)
target_link_libraries(sagittariusd PRIVATE daemon)
