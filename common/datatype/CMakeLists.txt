find_package(Eigen3 3.3 REQUIRED)
# datatype
add_library(datatype INTERFACE)
add_library(common::datatype ALIAS datatype)
target_include_directories(datatype INTERFACE include ${EIGEN3_INCLUDE_DIRS})
target_link_libraries(datatype INTERFACE common::reflect)


### TODO: datatype reflection io

# gnss
# add_gtest(datatype_gnss_test test/datatype_gnss_test.cc)
# target_link_libraries(datatype_gnss_test common::datatype)

# imu
# add_gtest(datatype_imu_test test/datatype_imu_test.cc)
# target_link_libraries(datatype_imu_test common::datatype)

# pose
# add_gtest(datatype_pose_test test/datatype_pose_test.cc)
# target_link_libraries(datatype_pose_test common::datatype)

# keypose
# add_gtest(datatype_keypose_test test/datatype_keypose_test.cc)
# target_link_libraries(datatype_keypose_test common::datatype)