add_executable(sagittarius_example)
target_sources(sagittarius_example PRIVATE sagittarius_example.cpp)
target_link_libraries(sagittarius_example
    PRIVATE sagittarius_sdk Boost::thread)
