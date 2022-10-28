add_library(lidarProcessClass SHARED lidar_process.cpp)
target_link_libraries(lidarProcessClass ${PCL_LIBRARIES})
