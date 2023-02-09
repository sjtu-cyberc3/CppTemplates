find_package(Threads REQUIRED)

add_library(instrumentor INTERFACE)
add_library(common::instrumentor ALIAS instrumentor)
target_include_directories(instrumentor INTERFACE include)

add_executable(instrumentor_example example/instrumentor_example.cpp)
target_link_libraries(instrumentor_example common::instrumentor Threads::Threads)
