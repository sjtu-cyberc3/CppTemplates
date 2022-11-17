add_executable(sensordata_test test.cc)
target_link_libraries(sensordata_test ${GTEST_BOTH_LIBRARIES} pthread)
gtest_discover_tests(sensordata_test)
