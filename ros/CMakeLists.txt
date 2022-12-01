# ##############################################################################
# ROS set
# ##############################################################################
find_package(catkin REQUIRED
             COMPONENTS roscpp
                        rosbag
                        std_msgs
                        sensor_msgs
                        message_generation
  )

add_message_files(DIRECTORY
                  msg
                  FILES
                  demo.msg
  )

add_service_files(DIRECTORY
                  srv
                  FILES
                  func.srv
  )

generate_messages(DEPENDENCIES std_msgs)

include_directories(${catkin_INCLUDE_DIRS})

link_libraries(${catkin_LIBRARIES})

# ##############################################################################
# Libraries
# ##############################################################################
add_library(ros_tools tools/rosbag.cc)
target_include_directories(ros_tools PRIVATE ${catkin_INCLUDE_DIRS})
target_link_libraries(ros_tools PRIVATE)

# ##############################################################################
# Build program
# ##############################################################################
add_executable(ros_example ros_example.cc)
target_include_directories(ros_example PRIVATE ${catkin_INCLUDE_DIRS})
target_link_libraries(ros_example PRIVATE ros_tools app_example configs)
