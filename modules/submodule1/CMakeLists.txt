add_executable(submodule1_test submodule1_test.cc)

# 链接测试库
target_link_libraries(submodule1_test
    ${GTEST_BOTH_LIBRARIES}
    -pthread)
gtest_discover_tests(submodule1_test)