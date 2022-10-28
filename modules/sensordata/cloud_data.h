#pragma once

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

class CloudData {
 public:
  using POINT     = pcl::PointXYZI;
  using CLOUD     = pcl::PointCloud<POINT>;
  using CLOUD_PTR = CLOUD::Ptr;

  double    time = 0.0;
  CLOUD_PTR ptr;

 public:
  CloudData() : ptr(new CLOUD()) {}
};
