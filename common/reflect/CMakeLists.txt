add_executable(reflect_test reflect_test.cc)
target_link_libraries(reflect_test ${GTEST_BOTH_LIBRARIES} pthread)
gtest_discover_tests(reflect_test)
target_compile_options(reflect_test PRIVATE "-Wno-unused-local-typedefs")