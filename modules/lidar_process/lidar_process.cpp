#include "lidar_process.hpp"

LidarProcess::LidarProcess() {
  // Lidar Sensor Configuration
  N_SCAN        = 64;
  Horizon_SCAN  = 4000;
  lidarMinRange = 2.5;
  lidarMaxRange = 50;
  // lidar to imu
  allocateMemory();

  Reset();
}

void LidarProcess::allocateMemory() {
  fullCloud.reset(new pcl::PointCloud<PointType>());
  extractedCloud.reset(new pcl::PointCloud<PointType>());
  cloud_corner.reset(new pcl::PointCloud<PointType>());
  cloud_surface.reset(new pcl::PointCloud<PointType>());

  rangeMat_ptr_ = std::make_shared<boost::multi_array<double, 2>>(boost::extents[N_SCAN][Horizon_SCAN]);
}

void LidarProcess::Reset() {
  deskew_firstPointFlag = false;

  fullCloud->clear();
  fullCloud->points.resize(N_SCAN * Horizon_SCAN);
  extractedCloud->clear();
  cloud_surface->clear();
  cloud_corner->clear();

  cloudSmoothness.resize(N_SCAN * Horizon_SCAN);
  cloudCurvature.assign(N_SCAN * Horizon_SCAN, 0);
  cloudNeighborPicked.assign(N_SCAN * Horizon_SCAN, 0);
  cloudLabel.assign(N_SCAN * Horizon_SCAN, 0);

  cloudInfo.startRingIndex.assign(N_SCAN, 0);
  cloudInfo.endRingIndex.assign(N_SCAN, 0);
  cloudInfo.pointColInd.assign(N_SCAN * Horizon_SCAN, 0);
  cloudInfo.pointRange.assign(N_SCAN * Horizon_SCAN, 0);

  imuTime.assign(queueLength, 0);
  imuRotX.assign(queueLength, 0);
  imuRotY.assign(queueLength, 0);
  imuRotZ.assign(queueLength, 0);

  for (int i = 0; i < N_SCAN; i++) {
    for (int j = 0; j < Horizon_SCAN; j++) {
      (*rangeMat_ptr_)[i][j] = FLAT_MAX;
    }
  }
}

void LidarProcess::extract(const pcl::PointCloud<pcl::PointXYZI>::Ptr& pc_in, double time_scan_cur) {
  // 映射点
  projectPointCloud(pc_in, time_scan_cur);
  // 提取 深度图
  cloudExtraction();
  // 计算平滑度
  calculateSmoothness();
  // 标记不可靠点
  markOccludedPoints();
  // 提取 surf corner特征
  extractFeatures();
}

// 主要计算 1. rangeMat 2. fullCloud
void LidarProcess::projectPointCloud(const pcl::PointCloud<pcl::PointXYZI>::Ptr& pc_in, double time_scan_cur) {

  int cloudSize = pc_in->points.size();
  // 投影为深度图
  for (int i = 0; i < cloudSize; ++i) {
    PointType thisPoint;
    thisPoint.x         = pc_in->points[i].x;
    thisPoint.y         = pc_in->points[i].y;
    thisPoint.z         = pc_in->points[i].z;
    thisPoint.intensity = pc_in->points[i].intensity;

    // 1. 移除近端反射 以及远方不可靠点
    float range = point_distance(thisPoint);
    if (range < lidarMinRange || range > lidarMaxRange)
      continue;

    // 2. 计算 线ID rowIdn
    double angle  = atan(pc_in->points[i].z / range) * 180 / M_PI;  // 点的上下倾角
    int    rowIdn = 0;
    if (N_SCAN == 16) {
      rowIdn = int((angle + 15) / 2 + 0.5);
      if (rowIdn > (N_SCAN - 1) || rowIdn < 0) {
        continue;
      }
    } else if (N_SCAN == 32) {
      rowIdn = int((angle + 92.0 / 3.0) * 3.0 / 4.0);
      if (rowIdn > (N_SCAN - 1) || rowIdn < 0) {
        continue;
      }
    } else if (N_SCAN == 64) {
      if (angle >= -8.83)
        rowIdn = int((2 - angle) * 3.0 + 0.5);
      else
        rowIdn = N_SCAN / 2 + int((-8.83 - angle) * 2.0 + 0.5);
      if (angle > 2 || angle < -24.33 || rowIdn > 63 || rowIdn < 0) {
        continue;
      }
    } else {
      printf("wrong scan number\n");
    }

    // 3. 计算 水平面ID columnIdn
    float horizonAngle = atan2(thisPoint.x, thisPoint.y) * 180 / M_PI;

    static float ang_res_x = 360.0 / float(Horizon_SCAN);  // 水平分辨率
    int          columnIdn = -round((horizonAngle - 90.0) / ang_res_x) + Horizon_SCAN / 2;
    if (columnIdn >= Horizon_SCAN)
      columnIdn -= Horizon_SCAN;

    if (columnIdn < 0 || columnIdn >= Horizon_SCAN)
      continue;

    if ((*rangeMat_ptr_)[rowIdn][columnIdn] != FLAT_MAX)
      continue;

    // 4. 去畸变
    // thisPoint = deskewPoint(&thisPoint, pc_in->points[i].time, time_scan_cur);

    // 5. 保存
    (*rangeMat_ptr_)[rowIdn][columnIdn] = range;

    int index                = columnIdn + rowIdn * Horizon_SCAN;
    fullCloud->points[index] = thisPoint;
  }
}

PointType LidarProcess::LidarProcess::deskewPoint(PointType* point, double relTime, double time_scan_cur) {
  if (deskewFlag == -1)
    return *point;

  // relTime 为这个点云相对最开始的时间
  double pointTime = time_scan_cur + relTime;

  // 角度变化量插值 使用 IMU 数据
  float rotXCur = 0;
  float rotYCur = 0;
  float rotZCur = 0;
  if (imuDeskewFlag)
    findRotation(pointTime, &rotXCur, &rotYCur, &rotZCur);

  // 位置变化量插值 使用  odom数据
  float posXCur = 0;
  float posYCur = 0;
  float posZCur = 0;
  if (odomDeskewFlag)
    findPosition(relTime, &posXCur, &posYCur, &posZCur);

  // 保存第一个点的信息 后面都是计算与其变化量
  if (deskew_firstPointFlag == true) {
    transStartInverse     = (pcl::getTransformation(posXCur, posYCur, posZCur, rotXCur, rotYCur, rotZCur)).inverse();  // 保存第一个点信息
    deskew_firstPointFlag = false;
  }

  // transform points to start
  Eigen::Affine3f transFinal = pcl::getTransformation(posXCur, posYCur, posZCur, rotXCur, rotYCur, rotZCur);  // 当前点信息
  Eigen::Affine3f transBt    = transStartInverse * transFinal;

  PointType newPoint;
  newPoint.x         = transBt(0, 0) * point->x + transBt(0, 1) * point->y + transBt(0, 2) * point->z + transBt(0, 3);
  newPoint.y         = transBt(1, 0) * point->x + transBt(1, 1) * point->y + transBt(1, 2) * point->z + transBt(1, 3);
  newPoint.z         = transBt(2, 0) * point->x + transBt(2, 1) * point->y + transBt(2, 2) * point->z + transBt(2, 3);
  newPoint.intensity = point->intensity;

  return newPoint;
}

float LidarProcess::point_distance(PointType p) {
  return sqrt(p.x * p.x + p.y * p.y);
}

float LidarProcess::point_distance(PointType p1, PointType p2) {
  return sqrt((p1.x - p2.x) * (p1.x - p2.x) + (p1.y - p2.y) * (p1.y - p2.y) + (p1.z - p2.z) * (p1.z - p2.z));
}

// 角度插值
void LidarProcess::findRotation(double pointTime, float* rotXCur, float* rotYCur, float* rotZCur) {

  // int imuPointerCur   = imuRotX.size(); // wait
  int imuPointerCur   = 1000;
  int imuPointerFront = 0;
  while (imuPointerFront < imuPointerCur) {
    // find  imuPointerFront index let:   past  [  imuPointerBack  pointtime  imuPointerFront ] now
    if (pointTime < imuTime[imuPointerFront])
      break;
    ++imuPointerFront;
  }

  if (pointTime > imuTime[imuPointerFront] || imuPointerFront == 0) {
    *rotXCur = imuRotX[imuPointerFront];
    *rotYCur = imuRotY[imuPointerFront];
    *rotZCur = imuRotZ[imuPointerFront];
  } else {
    int    imuPointerBack = imuPointerFront - 1;
    double ratioFront     = (pointTime - imuTime[imuPointerBack]) / (imuTime[imuPointerFront] - imuTime[imuPointerBack]);
    double ratioBack = (imuTime[imuPointerFront] - pointTime) / (imuTime[imuPointerFront] - imuTime[imuPointerBack]);  // replace by 1- ratioFront
    *rotXCur         = imuRotX[imuPointerFront] * ratioFront + imuRotX[imuPointerBack] * ratioBack;
    *rotYCur         = imuRotY[imuPointerFront] * ratioFront + imuRotY[imuPointerBack] * ratioBack;
    *rotZCur         = imuRotZ[imuPointerFront] * ratioFront + imuRotZ[imuPointerBack] * ratioBack;
  }
}

void LidarProcess::findPosition(double relTime, float* posXCur, float* posYCur, float* posZCur) {
  // If the sensor moves relatively slow, like walking speed, positional deskew seems to have little benefits. Thus code below is commented.

  // if (cloudInfo.imuAvailable == false || odomDeskewFlag == false)
  //     return;

  // float ratio = relTime / (time_scan_end - time_scan_cur);

  // *posXCur = ratio * odomIncreX;
  // *posYCur = ratio * odomIncreY;
  // *posZCur = ratio * odomIncreZ;
}

void LidarProcess::cloudExtraction() {
  int count = 0;  // 点云id
  // extract segmented cloud for lidar odometry
  // 最上面 最下面 两边点不好 需要丢弃
  for (int i = 0; i < N_SCAN; ++i) {
    // startRingIndex 线ID index
    cloudInfo.startRingIndex[i] = count - 1 + 5;

    for (int j = 0; j < Horizon_SCAN; ++j) {
      if ((*rangeMat_ptr_)[i][j] != FLAT_MAX) {
        // mark the points' column index for marking occlusion later
        cloudInfo.pointColInd[count] = j;
        // save range info
        cloudInfo.pointRange[count] = (*rangeMat_ptr_)[i][j];
        // save extracted cloud
        extractedCloud->push_back(fullCloud->points[j + i * Horizon_SCAN]);  // 经过处理和滤波后点云 用于发送
        // size of extracted cloud
        ++count;
      }
    }
    cloudInfo.endRingIndex[i] = count - 1 - 5;
  }
}

// 计算 平滑度
// 每一个点都初始化 cloudLabel = cloudNeighborPicked = 0
void LidarProcess::calculateSmoothness() {
  int cloudSize = extractedCloud->points.size();
  for (int i = 5; i < cloudSize - 5; i++) {
    float diffRange = cloudInfo.pointRange[i - 5] + cloudInfo.pointRange[i - 4] + cloudInfo.pointRange[i - 3] + cloudInfo.pointRange[i - 2]
                      + cloudInfo.pointRange[i - 1] - cloudInfo.pointRange[i] * 10 + cloudInfo.pointRange[i + 1] + cloudInfo.pointRange[i + 2]
                      + cloudInfo.pointRange[i + 3] + cloudInfo.pointRange[i + 4] + cloudInfo.pointRange[i + 5];
    cloudCurvature[i] = diffRange * diffRange;

    // cloudNeighborPicked[i]   = 0;  // 所有计算平滑计算的点 标记为 未处理0 意味完成初始化
    // cloudLabel[i]            = 0;
    cloudSmoothness[i].ind   = i;
    cloudSmoothness[i].value = cloudCurvature[i];
  }
}

// 标记不可靠点 cloudNeighborPicked = 1 不参与后续运算
void LidarProcess::markOccludedPoints() {
  // cloudNeighborPicked中有了点是否选择为特征点的标记
  int cloudSize = extractedCloud->points.size();
  // mark occluded points and parallel beam points
  for (int i = 5; i < cloudSize - 6; ++i) {
    // occluded points
    float depth1 = cloudInfo.pointRange[i];
    float depth2 = cloudInfo.pointRange[i + 1];
    // Horizon_SCAN 差值  是不同时间同一线采集的点
    int columnDiff = std::abs(int(cloudInfo.pointColInd[i + 1] - cloudInfo.pointColInd[i]));

    if (columnDiff < 10) {  // 这个应该总会执行 意味差10条线
      // 10 pixel diff in range image
      // 一个线中距离误差大于0.3说明 存在跳变 这个地方点不可靠 将跳变端点 标记为1 正常应该在+-0.3间 不进行处理
      if (depth1 - depth2 > 0.3) {
        cloudNeighborPicked[i - 5] = 1;
        cloudNeighborPicked[i - 4] = 1;
        cloudNeighborPicked[i - 3] = 1;
        cloudNeighborPicked[i - 2] = 1;
        cloudNeighborPicked[i - 1] = 1;
        cloudNeighborPicked[i]     = 1;
      } else if (depth2 - depth1 > 0.3) {
        cloudNeighborPicked[i + 1] = 1;
        cloudNeighborPicked[i + 2] = 1;
        cloudNeighborPicked[i + 3] = 1;
        cloudNeighborPicked[i + 4] = 1;
        cloudNeighborPicked[i + 5] = 1;
        cloudNeighborPicked[i + 6] = 1;
      }
    }

    // parallel beam
    // 同样是挑选不可靠点 不过和上面的区别在于是使用相对指标 上面0.3是绝对指标
    float diff1 = std::abs(float(cloudInfo.pointRange[i - 1] - cloudInfo.pointRange[i]));
    float diff2 = std::abs(float(cloudInfo.pointRange[i + 1] - cloudInfo.pointRange[i]));

    if (diff1 > 0.02 * cloudInfo.pointRange[i] && diff2 > 0.02 * cloudInfo.pointRange[i])
      cloudNeighborPicked[i] = 1;
  }
}

// 提取corner和surface特征
void LidarProcess::extractFeatures() {
  pcl::PointCloud<PointType>::Ptr surfaceCloudScan(new pcl::PointCloud<PointType>());

  for (int i = 0; i < N_SCAN; i++) {
    surfaceCloudScan->clear();
    for (int j = 0; j < 6; j++) {
      // 为了保证各方向均匀提取, 将深度图分为6个子图
      // 每个子图中对点的曲率进行排序，sp和ep分别是这段点云的起始点与终止点
      int sp = (cloudInfo.startRingIndex[i] * (6 - j) + cloudInfo.endRingIndex[i] * j) / 6;
      int ep = (cloudInfo.startRingIndex[i] * (5 - j) + cloudInfo.endRingIndex[i] * (j + 1)) / 6 - 1;

      if (sp >= ep)
        continue;

      // 每个子图中按曲率排序 从小到大排列
      std::sort(cloudSmoothness.begin() + sp, cloudSmoothness.begin() + ep, by_value());

      // 首先筛选曲率大的点 为潜在角点
      // 注意 cloudSmoothness 为Horizon方向计算的曲率
      int largestPickedNum = 0;
      for (int k = ep; k >= sp; k--) {
        int ind = cloudSmoothness[k].ind;
        // 曲率比较大的是边缘点
        if (cloudNeighborPicked[ind] == 0 && cloudCurvature[ind] > edgeThreshold) {
          largestPickedNum++;
          if (largestPickedNum <= 20) {  // 选取20个边缘点
            cloudLabel[ind] = 1;
            cloud_corner->push_back(extractedCloud->points[ind]);
          } else {
            break;
          }
          // 防止特征点聚集
          // 从ind+l开始后面5个点，每个点index之间的差值，
          // 确保columnDiff<=10,然后标记为我们需要的点
          cloudNeighborPicked[ind] = 1;
          for (int l = 1; l <= 5; l++) {
            int columnDiff = std::abs(int(cloudInfo.pointColInd[ind + l] - cloudInfo.pointColInd[ind + l - 1]));
            if (columnDiff > 10)
              break;
            cloudNeighborPicked[ind + l] = 1;
          }
          for (int l = -1; l >= -5; l--) {
            int columnDiff = std::abs(int(cloudInfo.pointColInd[ind + l] - cloudInfo.pointColInd[ind + l + 1]));
            if (columnDiff > 10)
              break;
            cloudNeighborPicked[ind + l] = 1;
          }
        }
      }

      // 其次筛选曲率小的点 为潜在平面点
      int smallestPickedNum = 0;  // 未使用 可以加入  smallestPickedNum< 4 限制只加入4个平面点 具体参考LeGo-LOAM
      for (int k = sp; k <= ep; k++) {
        int ind = cloudSmoothness[k].ind;
        if (cloudNeighborPicked[ind] == 0 && cloudCurvature[ind] < surfThreshold) {
          // 在还没有标记并且曲率较小的点里面选
          cloudLabel[ind]          = -1;  // -1 为平面点
          cloudNeighborPicked[ind] = 1;

          // 防止平面点聚集
          for (int l = 1; l <= 5; l++) {
            int columnDiff = std::abs(int(cloudInfo.pointColInd[ind + l] - cloudInfo.pointColInd[ind + l - 1]));
            if (columnDiff > 10)
              break;
            cloudNeighborPicked[ind + l] = 1;
          }
          for (int l = -1; l >= -5; l--) {
            int columnDiff = std::abs(int(cloudInfo.pointColInd[ind + l] - cloudInfo.pointColInd[ind + l + 1]));
            if (columnDiff > 10)
              break;
            cloudNeighborPicked[ind + l] = 1;
          }
        }
      }

      // label小于0的点是平面点
      // 这一步多余了 应该加入上面的循环 这么写可能是源于LEGO-LOAM, 但是LeGo中对于特征点分的更加细致 在此并不需要
      for (int k = sp; k <= ep; k++) {
        if (cloudLabel[k] <= 0) {
          surfaceCloudScan->push_back(extractedCloud->points[k]);
        }
      }
    }

    *cloud_surface += *surfaceCloudScan;
  }
}
