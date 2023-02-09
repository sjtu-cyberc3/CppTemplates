add_library(configs src/configs.cc)
add_library(common::configs ALIAS configs)
find_package(yaml-cpp REQUIRED)
target_include_directories(configs PUBLIC include ${YAML_CPP_INCLUDE_DIR})
target_link_libraries(configs PUBLIC ${YAML_CPP_LIBRARIES})

add_gtest(configs_test test/configs_test.cc)
target_link_libraries(configs_test common::configs)
target_compile_definitions(configs_test PRIVATE -DCONFIGS_TEST_FILE="${CMAKE_CURRENT_SOURCE_DIR}/test/configs_test.yaml")
