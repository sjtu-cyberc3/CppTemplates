add_library(logger src/logger.cc)
add_library(common::logger ALIAS logger)
find_package(Glog REQUIRED)
target_include_directories(logger PUBLIC include ${GLOG_INCLUDE_DIR})
target_link_libraries(logger PUBLIC ${GLOG_LIBRARIES})

add_executable(logger_example example/logger_example.cpp)
target_link_libraries(logger_example common::logger)
