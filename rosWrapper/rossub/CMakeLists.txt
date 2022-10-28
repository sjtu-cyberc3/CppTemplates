file(GLOB_RECURSE srcs *.cc)
add_library(rossubscriber SHARED ${srcs})
target_link_libraries(rossubscriber
                      sensordata
                     )