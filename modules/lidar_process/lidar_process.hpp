#pragma once

#include <boost/multi_array.hpp>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <pcl/filters/crop_box.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>

#include <cmath>
#include <vector>

typedef pcl::PointXYZI PointType;

#define FLAT_MAX 10000

struct VelodynePointXYZIRT {
  PCL_ADD_POINT4D
  PCL_ADD_INTENSITY;
  uint16_t ring;
  float    time;
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;
POINT_CLOUD_REGISTER_POINT_STRUCT(VelodynePointXYZIRT,
                                  (float, x, x)(float, y, y)(float, z, z)(float, intensity, intensity)(uint16_t, ring, ring)(float, time, time))

// Use the Velodyne point format as a common representation
using PointXYZIRT = VelodynePointXYZIRT;

// 一个标准激光点云 读取信息 构建信息 根据IMU odom进行去畸变 以及计算 特征点
class LidarProcess {
 public:
  LidarProcess();

  void allocateMemory();
  void Reset();

  PointType deskewPoint(PointType* point, double relTime, double time_scan_cur);
  void      findRotation(double pointTime, float* rotXCur, float* rotYCur, float* rotZCur);
  void      findPosition(double relTime, float* posXCur, float* posYCur, float* posZCur);
  float     point_distance(PointType p);
  float     point_distance(PointType p1, PointType p2);

  void extract(const pcl::PointCloud<pcl::PointXYZI>::Ptr& pc_in, double time_scan_cur);

  std::vector<double> imuTime;
  std::vector<double> imuRotX;
  std::vector<double> imuRotY;
  std::vector<double> imuRotZ;

  Eigen::Affine3f transBt;  //这一帧 开始到末尾的转移矩阵

  // 畸变矫正
  int  deskewFlag = 1;         // 畸变矫正 手动开关
  bool odomDeskewFlag;         // Odom 矫正使能位
  bool imuDeskewFlag;          //  imu 矫正使能位
  bool deskew_firstPointFlag;  // 本 scan 第一个点云

  Eigen::Affine3f transStartInverse;
  bool            imuAvailable  = false;
  bool            odomAvailable = false;

  pcl::PointCloud<PointType>::Ptr fullCloud;
  pcl::PointCloud<PointType>::Ptr extractedCloud;
  pcl::PointCloud<PointType>::Ptr cloud_corner;
  pcl::PointCloud<PointType>::Ptr cloud_surface;

 private:
  const int queueLength = 2000;

  // Lidar Sensor Configuration
  int   N_SCAN;         // 线数
  int   Horizon_SCAN;   // 水平采样数
  float lidarMinRange;  // lidar最小距离
  float lidarMaxRange;

  // 算法参数 LOAM
  float edgeThreshold          = 0.1;
  float surfThreshold          = 0.1;
  int   edgeFeatureMinValidNum = 10;
  int   surfFeatureMinValidNum = 100;

  // 点筛选
  struct smoothness_t {
    float  value;
    size_t ind;
  };
  struct by_value {
    bool operator()(smoothness_t const& left, smoothness_t const& right) {
      return left.value < right.value;
    }
  };
  std::vector<smoothness_t> cloudSmoothness;
  std::vector<float>        cloudCurvature;
  std::vector<int>          cloudNeighborPicked;
  std::vector<int>          cloudLabel;

  // // 似乎没有用
  // float odomIncreX;
  // float odomIncreY;
  // float odomIncreZ;

  // 深度图
  std::shared_ptr<boost::multi_array<double, 2>> rangeMat_ptr_;

  // 点云信息
  struct cloud {
    std::vector<int>   startRingIndex;  // 某一行 开始序号 避开360和0°结合部分的点
    std::vector<int>   endRingIndex;    // 某一行 结束序号
    std::vector<int>   pointColInd;     // 点云的水平 id
    std::vector<float> pointRange;      // 点云的深度信息
  };
  cloud cloudInfo;

  void projectPointCloud(const pcl::PointCloud<pcl::PointXYZI>::Ptr& pc_in, double time_scan_cur);
  void cloudExtraction();
  void calculateSmoothness();
  void markOccludedPoints();
  void extractFeatures();
};
