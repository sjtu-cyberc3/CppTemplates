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

target_compile_definitions(configs_test PRIVATE -DCONFIGS_EXAMPLE_PATH="${CMAKE_CURRENT_SOURCE_DIR}/configs_example.yaml")
gtest_discover_tests(configs_test)
