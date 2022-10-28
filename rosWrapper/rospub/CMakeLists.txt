file(GLOB_RECURSE srcs *.cc)
add_library(rospublisher SHARED ${srcs})
target_link_libraries(rospublisher
                      sensordata
                     )