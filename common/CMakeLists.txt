project(common)

# logger
add_library(logger SHARED logger.cc)
target_link_libraries(logger ${GLOG_LIBRARIES}
                      ${Boost_LIBRARIES})

add_executable(logger_main logger_main.cpp)
target_link_libraries(logger_main logger)

# Instrumentor
add_executable(Instrumentor_main Instrumentor_main.cpp)
target_link_libraries(Instrumentor_main -lpthread)

# configs
add_library(configs SHARED configs.cpp)
target_include_directories(configs
                           PUBLIC ${YAML_CPP_INCLUDE_DIR})
target_link_libraries(configs PUBLIC ${YAML_CPP_LIBRARIES})

add_executable(configs_test configs_test.cpp)
target_link_libraries(configs_test configs
                      ${GTEST_BOTH_LIBRARIES} -pthread)
gtest_discover_tests(configs_test)
