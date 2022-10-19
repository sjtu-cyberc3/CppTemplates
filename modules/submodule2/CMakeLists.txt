project(submodule2)

add_library(submodule2 SHARED submodule2.cc)
target_link_libraries(submodule2)

add_executable(submodule2_test submodule2_test.cc)
target_link_libraries(submodule2_test submodule2
                      ${GTEST_BOTH_LIBRARIES} -pthread)
gtest_discover_tests(submodule2_test)
