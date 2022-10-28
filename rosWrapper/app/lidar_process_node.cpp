// c++ lib
#include <chrono>
#include <deque>
#include <thread>

// third lib
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>
#include <yaml-cpp/yaml.h>

// local lib
#include "config/global_defination.h"

#include "../rospub/cloudinfo.h"
#include "../rospub/pointcloud.h"
#include "../rossub/imu.h"
#include "../rossub/odometry.h"
#include "../rossub/pointcloud.h"

#include "cyberc3_lio/cloud_info.h"
#include "modules/lidar_process/lidar_process.hpp"
#include "modules/sensordata/imu_data.h"

class LidarProcessNode {
 private:
  // rosnode
  ros::NodeHandle nh_;

  // ros subscriber
  std::shared_ptr<roswrapper::CloudSubscriber>    sub_lidar;
  std::shared_ptr<roswrapper::IMUSubscriber>      sub_imu;
  std::shared_ptr<roswrapper::OdometrySubscriber> sub_odom;

  // ros subscriber msg
  std::deque<IMUData>   queue_imu;
  std::deque<KeyFrame>  queue_odom;
  std::deque<CloudData> queue_lidar;

  // ros pubslisher
  std::shared_ptr<roswrapper::CloudPublisher>     pub_surface_cloud;
  std::shared_ptr<roswrapper::CloudPublisher>     pub_corner_cloud;
  std::shared_ptr<roswrapper::CloudPublisher>     pub_extracted_cloud;
  std::shared_ptr<roswrapper::CloudInfoPublisher> pub_cloud_info;

  // main class
  std::shared_ptr<LidarProcess> lidar_process_ptr_;
  // std::shared_ptr<LivoxMidProcess> livox_mid_process_ptr_;

  // common poincloud
  CloudData cloud_in;       // 当前帧处理的 lidar msg
  CloudData cloud_surface;  // 用于可视化的点云
  CloudData cloud_corner;   // 用于可视化的点云

  cyberc3_lio::cloud_info cloudInfo;      // 新构建消息类型
  double                  time_scan_cur;  // 当前帧的开始时间
  double                  time_scan_end;  // 当前帧的末尾时间

  int lidartype_ = 0;

 public:
  LidarProcessNode() {
    // set subscriber
    sub_lidar = std::make_shared<roswrapper::CloudSubscriber>(nh_, "pointCloudTopic", 5);
    sub_odom  = std::make_shared<roswrapper::OdometrySubscriber>(nh_, "odomTopic", 200);
    sub_imu   = std::make_shared<roswrapper::IMUSubscriber>(nh_, "imuTopic", 200);

    // set publisher`
    pub_surface_cloud   = std::make_shared<roswrapper::CloudPublisher>(nh_, "cyberc3_lio/preprocess/surface_cloud", "base_link", 1);
    pub_corner_cloud    = std::make_shared<roswrapper::CloudPublisher>(nh_, "cyberc3_lio/preprocess/corner_cloud", "base_link", 1);
    pub_extracted_cloud = std::make_shared<roswrapper::CloudPublisher>(nh_, "cyberc3_lio/preprocess/extracted_cloud", "base_link", 1);
    pub_cloud_info      = std::make_shared<roswrapper::CloudInfoPublisher>(nh_, "cyberc3_lio/preprocess/cloud_info", "base_link", 1);

    // yaml node
    std::string file_path   = WORK_SPACE_PATH + "/config/config.yaml";
    YAML::Node  config_node = YAML::LoadFile(file_path);

    lidartype_ = config_node["lidarprocess"]["mode"].as<int>();
    std::cout << "lidartype_: " << lidartype_ << std::endl;

    // main class
    lidar_process_ptr_ = std::make_shared<LidarProcess>();
  }

  ~LidarProcessNode() {}

  void reset() {
    lidar_process_ptr_->odomDeskewFlag = false;
    lidar_process_ptr_->imuDeskewFlag  = false;
  }

  void lidar_processing() {
    while (ros::ok()) {
      sub_lidar->ParseData(queue_lidar);
      sub_odom->ParseData(queue_odom);
      sub_imu->ParseData(queue_imu);
      if (queue_lidar.size() >= 5) {
        if (lidartype_ == 1) {  // velodyne
          reset();
          lidar_process_ptr_->Reset();
          // 1. cachePointCloud
          auto cloud_in = queue_lidar.front();
          queue_lidar.pop_front();
          time_scan_cur = cloud_in.time;
          // time_scan_end = time_scan_cur + cloud_in->points.back().time;
          time_scan_end = time_scan_cur + 0.1;  // 暂时保证编译通过

          // 2. 特征提取
          lidar_process_ptr_->extract(cloud_in.ptr, time_scan_cur);

          // 3. publish clouds
          publish_cloud(lidar_process_ptr_->fullCloud, lidar_process_ptr_->cloud_surface, lidar_process_ptr_->cloud_corner, time_scan_cur);
        }
      }
      // sleep 2 ms every time 避免不停循环
      std::chrono::milliseconds dura(2);
      std::this_thread::sleep_for(dura);
    }
    std::cout << "lidar_processing thread end! " << std::endl;
  }

  void
  publish_cloud(pcl::PointCloud<PointType>::Ptr full, pcl::PointCloud<PointType>::Ptr surface, pcl::PointCloud<PointType>::Ptr corner, double time) {
    CloudData full1;
    full1.ptr = full;
    CloudData surface1;
    surface1.ptr = surface;
    CloudData corner1;
    corner1.ptr = corner;

    cloudInfo.cloud_deskewed = pub_extracted_cloud->publish(full1);
    cloudInfo.cloud_surface  = pub_surface_cloud->publish(surface1);
    cloudInfo.cloud_corner   = pub_corner_cloud->publish(corner1);

    pub_cloud_info->publish(cloudInfo, time);
  }
};

int main(int argc, char** argv) {
  ros::init(argc, argv, "preprocess");

  LidarProcessNode lp;

  std::thread LidarProcess_thread(&LidarProcessNode::lidar_processing, std::ref(lp));

  ros::spin();

  LidarProcess_thread.join();

  return 0;
}
