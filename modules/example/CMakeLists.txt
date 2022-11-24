project(module_example)

add_library(module_example SHARED example.cc)
target_link_libraries(module_example)

add_executable(module_example_test example_test.cc)
target_link_libraries(
  module_example_test
  module_example
  ${GTEST_BOTH_LIBRARIES}
  -pthread
  )
gtest_discover_tests(module_example_test)
