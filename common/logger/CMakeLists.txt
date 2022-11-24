# logger
add_library(logger SHARED logger.cc)
target_link_libraries(logger ${GLOG_LIBRARIES} ${Boost_LIBRARIES})

add_executable(logger_main logger_main.cpp)
target_link_libraries(logger_main logger)
