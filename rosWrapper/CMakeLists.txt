project(demo_project)
# ##########################################################
# ROS set
# ##########################################################
find_package(catkin REQUIRED
             COMPONENTS roscpp rospy std_msgs geometry_msgs)

include_directories(${catkin_INCLUDE_DIRS})
link_libraries(${catkin_LIBRARIES})
catkin_package()

# message(STATUS ${catkin_INCLUDE_DIRS}) message(STATUS
# ${catkin_LIBRARIES})

# ##########################################################
# add ros submodule
# ##########################################################
# add_subdirectory(modules)

set(ROS_RELATED_LIB "")

list(APPEND ROS_RELATED_LIB ${PROJECT_NAME}_rossubscriber
     ${PROJECT_NAME}_rospublisher ${catkin_LIBRARIES})

# ##########################################################
# Build program
# ##########################################################
include_directories(${Muduo_INCLUDE_DIR})
add_executable(demo app/demo_node.cpp)
target_link_libraries(demo)
