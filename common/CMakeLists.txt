project (common)

find_package(Glog)

add_library(logger SHARED logger.cc)
target_link_libraries(logger  ${GLOG_LIBRARIES} ${Boost_LIBRARIES})

add_executable(glog_test main.cpp )
target_link_libraries(glog_test logger
)