# ##########################################################
# ROS set
# ##########################################################
find_package(
    catkin REQUIRED
    COMPONENTS 
    roscpp
    rosbag
    std_msgs
    sensor_msgs
    )


# ##########################################################
# Build program
# ##########################################################
add_executable(ros_example ros_example.cc)
target_include_directories(ros_example PRIVATE ${catkin_INCLUDE_DIRS})
target_link_libraries(ros_example PRIVATE ${catkin_LIBRARIES} app_example)