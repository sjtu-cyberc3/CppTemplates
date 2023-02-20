find_package(yaml-cpp 0.5 REQUIRED)
find_package(fmt 6.0 REQUIRED)

add_library(reflect INTERFACE)
add_library(common::reflect ALIAS reflect)
target_include_directories(reflect INTERFACE include ${YAML_CPP_INCLUDE_DIR})
target_link_libraries(reflect INTERFACE ${YAML_CPP_LIBRARIES} fmt::fmt)
target_compile_options(reflect INTERFACE -Wno-unused-local-typedefs)

add_gtest(reflect_test test/reflect_test.cc)
target_link_libraries(reflect_test common::reflect)