project(modules_example)

add_library(modules_example src/example.cc)
add_library(modules::example ALIAS modules_example)
target_include_directories(modules_example PUBLIC include)

add_gtest(module_example_test test/example_test.cc)
target_link_libraries(module_example_test modules::example)
