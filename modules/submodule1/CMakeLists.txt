project(submodule1)

add_library(submodule1 SHARED submodule1.cc)
target_link_libraries(submodule1)

add_executable(submodule1_test submodule1_test.cc)
target_link_libraries(submodule1_test submodule1
                      ${GTEST_BOTH_LIBRARIES} -pthread)
gtest_discover_tests(submodule1_test)
