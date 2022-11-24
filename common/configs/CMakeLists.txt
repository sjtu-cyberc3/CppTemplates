# configs
add_library(configs SHARED configs.cpp)
target_include_directories(configs PUBLIC ${YAML_CPP_INCLUDE_DIR})
target_link_libraries(configs PUBLIC ${YAML_CPP_LIBRARIES})

add_executable(configs_test configs_test.cpp)
target_link_libraries(
  configs_test
  configs
  -pthread
  ${GTEST_BOTH_LIBRARIES}
  )
gtest_discover_tests(configs_test)
