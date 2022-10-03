project(common)

find_package(Glog)

add_library(logger SHARED logger.cc)
target_link_libraries(logger ${GLOG_LIBRARIES}
                      ${Boost_LIBRARIES})

add_executable(logger_main logger_main.cpp)
target_link_libraries(logger_main logger)

add_executable(Instrumentor_main Instrumentor_main.cpp)
target_link_libraries(Instrumentor_main -lpthread)

find_package(yaml-cpp REQUIRED)
add_library(configs SHARED configs.cpp)
target_include_directories(configs PUBLIC ${YAML_CPP_INCLUDE_DIR})
target_link_libraries(configs PUBLIC ${YAML_CPP_LIBRARIES})

add_executable(configs_main configs_main.cpp)
target_link_libraries(configs_main configs)
