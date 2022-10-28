file(GLOB_RECURSE srcs *.cc)
add_library(sensordata SHARED ${srcs})
target_link_libraries(sensordata
                     )