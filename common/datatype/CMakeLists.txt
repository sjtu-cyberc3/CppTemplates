add_executable(datatype_test datatype_test.cc)
target_link_libraries(datatype_test ${GTEST_BOTH_LIBRARIES} pthread)
gtest_discover_tests(datatype_test)
