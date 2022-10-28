file(GLOB_RECURSE srcs *.cc)
add_library(rosother SHARED ${srcs})
target_link_libraries(rosother
                      sensordata
                     )