project(cyberc3_lio)
# ##########################################################
# ROS set
# ##########################################################
find_package(
  catkin REQUIRED
  COMPONENTS roscpp
             rospy
             tf
             std_msgs
             geometry_msgs
             sensor_msgs
             message_generation)

include_directories(${catkin_INCLUDE_DIRS})
link_libraries(${catkin_LIBRARIES})
# catkin_package()

# message(STATUS ${catkin_INCLUDE_DIRS}) message(STATUS
# ${catkin_LIBRARIES})
add_message_files(DIRECTORY msg FILES cloud_info.msg)

generate_messages(DEPENDENCIES std_msgs geometry_msgs
                  std_msgs sensor_msgs)

# ##########################################################
# add ros submodule
# ##########################################################
# add_subdirectory(modules)

set(ROS_RELATED_LIB "")

include_directories(${catkin_INCLUDE_DIRS})

list(APPEND ROS_RELATED_LIB ${PROJECT_NAME}_rossubscriber
     ${PROJECT_NAME}_rospublisher ${catkin_LIBRARIES})

add_subdirectory(rossub)
add_subdirectory(rospub)
# add_subdirectory(rossrv)
add_subdirectory(other)

# ##########################################################
# Build program
# ##########################################################
add_executable(demo app/demo_node.cpp)
target_link_libraries(demo)

add_executable(lidarProcessNode app/lidar_process_node.cpp)
target_link_libraries(
  lidarProcessNode lidarProcessClass rossubscriber
  rospublisher ${require_third_libs})
