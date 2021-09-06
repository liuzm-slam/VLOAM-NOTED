// This is an advanced implementation of the algorithm described in the following paper:
//   J. Zhang and S. Singh. LOAM: Lidar Odometry and Mapping in Real-time.
//     Robotics: Science and Systems Conference (RSS). Berkeley, CA, July 2014.

// Modifier: Tong Qin               qintonguav@gmail.com
// 	         Shaozu Cao 		    saozu.cao@connect.ust.hk

// Copyright 2013, Ji Zhang, Carnegie Mellon University
// Further contributions copyright (c) 2016, Southwest Research Institute
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
// 1. Redistributions of source code must retain the above copyright notice,
//    this list of conditions and the following disclaimer.
// 2. Redistributions in binary form must reproduce the above copyright notice,
//    this list of conditions and the following disclaimer in the documentation
//    and/or other materials provided with the distribution.
// 3. Neither the name of the copyright holder nor the names of its
//    contributors may be used to endorse or promote products derived from this
//    software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

#include <lidar_odometry_mapping/scan_registration.h>

// 划分当前帧激光点云，得到平面点和边缘点
namespace vloam
{
  // 初始化，接收参数，并且定义点云
void ScanRegistration::init()
{
  ros::param::get("loam_verbose_level", verbose_level);

  systemInitCount = 0;
  systemInited = false;
  N_SCANS = 0;

  if (!ros::param::get("scan_line", N_SCANS))
    ROS_BREAK();

  if (!ros::param::get("minimum_range", MINIMUM_RANGE))
    ROS_BREAK();

  ROS_INFO("scan line number %d \n", N_SCANS);

  if (N_SCANS != 16 && N_SCANS != 32 && N_SCANS != 64)
  {
    ROS_ERROR("only support velodyne with 16, 32 or 64 scan line!");
  }

  // // NOTE: publishers and subscribers won't necessarily used, but are kept for forward compatibility
  // subLaserCloud = nh->subscribe<sensor_msgs::PointCloud2>("/velodyne_points", 100,
  // &ScanRegistration::laserCloudHandler, this);
  pubLaserCloud = nh.advertise<sensor_msgs::PointCloud2>("/velodyne_cloud_2", 100);
  pubCornerPointsSharp = nh.advertise<sensor_msgs::PointCloud2>("/laser_cloud_sharp", 100);
  pubCornerPointsLessSharp = nh.advertise<sensor_msgs::PointCloud2>("/laser_cloud_less_sharp", 100);
  pubSurfPointsFlat = nh.advertise<sensor_msgs::PointCloud2>("/laser_cloud_flat", 100);
  pubSurfPointsLessFlat = nh.advertise<sensor_msgs::PointCloud2>("/laser_cloud_less_flat", 100);
  // pubRemovePoints = nh->advertise<sensor_msgs::PointCloud2>("/laser_remove_points", 100);

  PUB_EACH_LINE = false;
  // // std::vector<ros::Publisher> pubEachScan;
  // // if(PUB_EACH_LINE)
  // // {
  // //     for(int i = 0; i < N_SCANS; i++)
  // //     {
  // //         ros::Publisher tmp = nh.advertise<sensor_msgs::PointCloud2>("/laser_scanid_" + std::to_string(i), 100);
  // //         pubEachScan.push_back(tmp);
  // //     }
  // // }

  // 发布全部激光点云
  laserCloud = boost::make_shared<pcl::PointCloud<PointType>>();
  // 发布边缘点
  cornerPointsSharp = boost::make_shared<pcl::PointCloud<PointType>>();
  cornerPointsLessSharp = boost::make_shared<pcl::PointCloud<PointType>>();
  // 发布平面点
  surfPointsFlat = boost::make_shared<pcl::PointCloud<PointType>>();
  surfPointsLessFlat = boost::make_shared<pcl::PointCloud<PointType>>();
}

// 重置参数
void ScanRegistration::reset()
{
  laserCloud->clear();
  cornerPointsSharp->clear();
  cornerPointsLessSharp->clear();
  surfPointsFlat->clear();
  surfPointsLessFlat->clear();

  laserCloudScans = std::vector<pcl::PointCloud<PointType>>(N_SCANS);  // N_SCANS = 64 for kitti
}

// 对激光雷达过滤，去除无效激光点，有些点太近
template <typename PointT>
void ScanRegistration::removeClosedPointCloud(const pcl::PointCloud<PointT>& cloud_in,
                                              pcl::PointCloud<PointT>& cloud_out, float thres)
{
  if (&cloud_in != &cloud_out)
  {
    // 对cloud_out进行赋值，激光点数量为cloud_in，cloud_out为滤波后的点云
    cloud_out.header = cloud_in.header;
    cloud_out.points.resize(cloud_in.points.size());
  }

  // 激光点有效点数 size_t不同系统大小不一样，足以用来表示对象的大小
  size_t j = 0;

  for (size_t i = 0; i < cloud_in.points.size(); ++i)
  {
    // 激光点的距离值和阈值比较，实际上是对激光滤波，去掉无效的点
    if (cloud_in.points[i].x * cloud_in.points[i].x + cloud_in.points[i].y * cloud_in.points[i].y +
            cloud_in.points[i].z * cloud_in.points[i].z <
        thres * thres)
      continue;
    cloud_out.points[j] = cloud_in.points[i];
    // j激光有效点数
    j++;
  }
  if (j != cloud_in.points.size())
  {
    cloud_out.points.resize(j);
  }

  cloud_out.height = 1;
  // 强制类型转换static_cast
  cloud_out.width = static_cast<uint32_t>(j);
  cloud_out.is_dense = true;
}

// 计算激光点的曲率，划分点云激光点到平面点和边缘点
void ScanRegistration::input(const pcl::PointCloud<pcl::PointXYZ>& laserCloudIn_)
{
  // 第一次输入时，初始化赋值
  if (!systemInited)
  {
    systemInitCount++;
    if (systemInitCount >= systemDelay)
    {
      systemInited = true;
    }
    else
      return;
  }
  if (verbose_level > 1)
  {
    ROS_INFO("scan registration starts");
    // 计时，当前时间赋值
    t_whole.tic();
  }

  std::vector<int> scanStartInd(N_SCANS, 0);
  std::vector<int> scanEndInd(N_SCANS, 0);

  // 把输入点云赋值到laserCloudIn
  pcl::PointCloud<pcl::PointXYZ> laserCloudIn = laserCloudIn_;
  // pcl::fromROSMsg(laserCloudMsg, laserCloudIn); // TODO: see if the input of the function should have another type
  // from vloam_main
  std::vector<int> indices;

  // pcl去除无效点
  pcl::removeNaNFromPointCloud(laserCloudIn, laserCloudIn, indices);
  // 调用removeClosedPointCloud函数去掉最小距离内的激光点
  removeClosedPointCloud(laserCloudIn, laserCloudIn, MINIMUM_RANGE);

  if (verbose_level > 1)
  {
    ROS_INFO("end of closed point removal");
  }

  int cloudSize = laserCloudIn.points.size();
  // 当前帧，第一个激光点的角度值
  float startOri = -atan2(laserCloudIn.points[0].y, laserCloudIn.points[0].x);
  // 当前帧，最后一个激光点的角度值
  float endOri = -atan2(laserCloudIn.points[cloudSize - 1].y, laserCloudIn.points[cloudSize - 1].x) + 2 * M_PI;

  // 对输入激光进行旋转方向判断，按照逆时针方向为正
  if (endOri - startOri > 3 * M_PI)
  {
    endOri -= 2 * M_PI;
  }
  else if (endOri - startOri < M_PI)
  {
    endOri += 2 * M_PI;
  }

  if (verbose_level > 1)
  {
    ROS_INFO("end Ori %f\n", endOri);
  }

  // ****
  bool halfPassed = false;
  int count = cloudSize;
  PointType point;
  // 处理当前点云，判断线序号
  for (int i = 0; i < cloudSize; i++)
  {
    // 激光点笛卡尔空间坐标
    point.x = laserCloudIn.points[i].x;
    point.y = laserCloudIn.points[i].y;
    point.z = laserCloudIn.points[i].z;

    // 激光点相较于激光中心的角度，单位度
    float angle = atan(point.z / sqrt(point.x * point.x + point.y * point.y)) * 180 / M_PI;
    // 激光所在的序号16，32，64，激光点所在序号
    int scanID = 0;

    // 可以使用激光雷达驱动自带的线号来确定当前激光点所在线
    if (N_SCANS == 16)
    {
      // 当前激光点所在点云中的序号，竖直方向分辨率2度，范围-15到+15度
      scanID = int((angle + 15) / 2 + 0.5);
      if (scanID > (N_SCANS - 1) || scanID < 0)
      {
        count--;
        continue;
      }
    }
    else if (N_SCANS == 32)
    {
      scanID = int((angle + 92.0 / 3.0) * 3.0 / 4.0);
      if (scanID > (N_SCANS - 1) || scanID < 0)
      {
        count--;
        continue;
      }
    }
    else if (N_SCANS == 64)
    {
      if (angle >= -8.83)
        scanID = int((2 - angle) * 3.0 + 0.5);
      else
        scanID = N_SCANS / 2 + int((-8.83 - angle) * 2.0 + 0.5);

      // use [0 50]  > 50 remove outlies
      if (angle > 2 || angle < -24.33 || scanID > 50 || scanID < 0)
      {
        count--;
        continue;
      }
    }
    else
    {
      ROS_INFO("wrong scan number\n");
      ROS_BREAK();
    }
    // ROS_INFO("angle %f scanID %d \n", angle, scanID);

    // 激光点在水平方向的角度
    float ori = -atan2(point.y, point.x);
    // halfPassed为处理一半角度的标志，转换角度值
    if (!halfPassed)
    {
      if (ori < startOri - M_PI / 2)
      {
        ori += 2 * M_PI;
      }
      else if (ori > startOri + M_PI * 3 / 2)
      {
        ori -= 2 * M_PI;
      }

      if (ori - startOri > M_PI)
      {
        halfPassed = true;
      }
    }
    else
    {
      ori += 2 * M_PI;
      if (ori < endOri - M_PI * 3 / 2)
      {
        ori += 2 * M_PI;
      }
      else if (ori > endOri + M_PI / 2)
      {
        ori -= 2 * M_PI;
      }
    }

    float relTime = (ori - startOri) / (endOri - startOri);
    // scanPeriod扫描频率，point.intensity代表了激光点的信息，那个激光线上
    point.intensity = scanID + scanPeriod * relTime;
    laserCloudScans[scanID].push_back(point);
  }

  cloudSize = count;

  if (verbose_level > 1)
  {
    ROS_INFO("points size %d \n", cloudSize);
  }

  // 每个线数内的激光点，起始位置和终点位置的激光点序号
  for (int i = 0; i < N_SCANS; i++)
  {
    scanStartInd[i] = laserCloud->size() + 5;
    // laserCloudScans处理后的激光数据，有所在序号
    *laserCloud += laserCloudScans[i];
    // 当前序号上面的激光点的数量
    scanEndInd[i] = laserCloud->size() - 6;
  }

  if (verbose_level > 1)
  {
    ROS_INFO("prepare time %f \n", t_prepare.toc());
  }

  // 计算激光点i的曲率
  for (int i = 5; i < cloudSize - 5; i++)
  {
    float diffX = laserCloud->points[i - 5].x + laserCloud->points[i - 4].x + laserCloud->points[i - 3].x +
                  laserCloud->points[i - 2].x + laserCloud->points[i - 1].x - 10 * laserCloud->points[i].x +
                  laserCloud->points[i + 1].x + laserCloud->points[i + 2].x + laserCloud->points[i + 3].x +
                  laserCloud->points[i + 4].x + laserCloud->points[i + 5].x;
    float diffY = laserCloud->points[i - 5].y + laserCloud->points[i - 4].y + laserCloud->points[i - 3].y +
                  laserCloud->points[i - 2].y + laserCloud->points[i - 1].y - 10 * laserCloud->points[i].y +
                  laserCloud->points[i + 1].y + laserCloud->points[i + 2].y + laserCloud->points[i + 3].y +
                  laserCloud->points[i + 4].y + laserCloud->points[i + 5].y;
    float diffZ = laserCloud->points[i - 5].z + laserCloud->points[i - 4].z + laserCloud->points[i - 3].z +
                  laserCloud->points[i - 2].z + laserCloud->points[i - 1].z - 10 * laserCloud->points[i].z +
                  laserCloud->points[i + 1].z + laserCloud->points[i + 2].z + laserCloud->points[i + 3].z +
                  laserCloud->points[i + 4].z + laserCloud->points[i + 5].z;

    // 计算曲率
    cloudCurvature[i] = diffX * diffX + diffY * diffY + diffZ * diffZ;
    // 对应第几个点的曲率序号
    cloudSortInd[i] = i;
    // 激光点未处理的标志位
    cloudNeighborPicked[i] = 0;
    // 激光点的类型 2:less边缘点，1：边缘点，-1:平面点，0：没有处理的点
    cloudLabel[i] = 0;
  }

  TicToc t_pts;

  float t_q_sort = 0;
  // 提取平面点和边缘点
  for (int i = 0; i < N_SCANS; i++)
  {
    // 当前线号内的激光点大于6
    if (scanEndInd[i] - scanStartInd[i] < 6)
      continue;
    pcl::PointCloud<PointType>::Ptr surfPointsLessFlatScan(new pcl::PointCloud<PointType>);
    for (int j = 0; j < 6; j++)
    {
      int sp = scanStartInd[i] + (scanEndInd[i] - scanStartInd[i]) * j / 6;
      int ep = scanStartInd[i] + (scanEndInd[i] - scanStartInd[i]) * (j + 1) / 6 - 1;

      TicToc t_tmp;
      // ???
      std::sort(cloudSortInd + sp, cloudSortInd + ep + 1,
                [&](const int& i, const int& j) { return cloudCurvature[i] < cloudCurvature[j]; });
      t_q_sort += t_tmp.toc();

      int largestPickedNum = 0;
      // 平滑度由小到大遍历
      for (int k = ep; k >= sp; k--)
      {
        // 索引，代表激光点i的序号
        int ind = cloudSortInd[k];

        // 提取边缘点
        // 当前并非远方点变近的边缘的点、并且平滑度大于一定值、非地面数据
        // 结论：就是提取水平方向连续断开的端点，且仅提取断开近处的端点）
        // cloudNeighborPicked[ind] == 0 未处理的标志位
        // 0.1边缘点曲率阈值
        if (cloudNeighborPicked[ind] == 0 && cloudCurvature[ind] > 0.1)
        {
          // 统计满足上述条件的数量
          largestPickedNum++;
          // 记录最大的两个点，为最陡的两个点，即一圈最多6*2 = 12个点
          if (largestPickedNum <= 2)
          {
            cloudLabel[ind] = 2;
            cornerPointsSharp->push_back(laserCloud->points[ind]);
            cornerPointsLessSharp->push_back(laserCloud->points[ind]);
          }
          else if (largestPickedNum <= 20)
          {
            cloudLabel[ind] = 1;
            cornerPointsLessSharp->push_back(laserCloud->points[ind]);
          }
          else
          {
            break;
          }

          // 激光点i以及处理完成的标志
          cloudNeighborPicked[ind] = 1;

          // 如果当前点在最后5个点内不进行处理？
          for (int l = 1; l <= 5; l++)
          {
            float diffX = laserCloud->points[ind + l].x - laserCloud->points[ind + l - 1].x;
            float diffY = laserCloud->points[ind + l].y - laserCloud->points[ind + l - 1].y;
            float diffZ = laserCloud->points[ind + l].z - laserCloud->points[ind + l - 1].z;
            if (diffX * diffX + diffY * diffY + diffZ * diffZ > 0.05)
            {
              break;
            }

            cloudNeighborPicked[ind + l] = 1;
          }
          for (int l = -1; l >= -5; l--)
          {
            float diffX = laserCloud->points[ind + l].x - laserCloud->points[ind + l + 1].x;
            float diffY = laserCloud->points[ind + l].y - laserCloud->points[ind + l + 1].y;
            float diffZ = laserCloud->points[ind + l].z - laserCloud->points[ind + l + 1].z;
            if (diffX * diffX + diffY * diffY + diffZ * diffZ > 0.05)
            {
              break;
            }

            cloudNeighborPicked[ind + l] = 1;
          }
        }
      }

      // 提取平面点
      // smallestPickedNum 最平滑点提取数量
      int smallestPickedNum = 0;
      // 平滑度从小到大遍历
      // 特征点相邻的5个点也不得是特征点，
      // 与角点类似，即已经是平面特征点，则接下来相邻的5个不得再是，除非相邻的点在扫描索引相差10个以上
      for (int k = sp; k <= ep; k++)
      {
        int ind = cloudSortInd[k];

        if (cloudNeighborPicked[ind] == 0 && cloudCurvature[ind] < 0.1)
        {
          cloudLabel[ind] = -1;
          // 所有的平面点
          surfPointsFlat->push_back(laserCloud->points[ind]);

          smallestPickedNum++;
          // 最平滑的四个点
          // 即一圈最多为6*4 = 24个点
          if (smallestPickedNum >= 4)
          {
            break;
          }

          cloudNeighborPicked[ind] = 1;
          for (int l = 1; l <= 5; l++)
          {
            float diffX = laserCloud->points[ind + l].x - laserCloud->points[ind + l - 1].x;
            float diffY = laserCloud->points[ind + l].y - laserCloud->points[ind + l - 1].y;
            float diffZ = laserCloud->points[ind + l].z - laserCloud->points[ind + l - 1].z;
            if (diffX * diffX + diffY * diffY + diffZ * diffZ > 0.05)
            {
              break;
            }

            cloudNeighborPicked[ind + l] = 1;
          }
          for (int l = -1; l >= -5; l--)
          {
            float diffX = laserCloud->points[ind + l].x - laserCloud->points[ind + l + 1].x;
            float diffY = laserCloud->points[ind + l].y - laserCloud->points[ind + l + 1].y;
            float diffZ = laserCloud->points[ind + l].z - laserCloud->points[ind + l + 1].z;
            if (diffX * diffX + diffY * diffY + diffZ * diffZ > 0.05)
            {
              break;
            }

            cloudNeighborPicked[ind + l] = 1;
          }
        }
      }

      // 剩下的所有点云均为相对较平的点
      for (int k = sp; k <= ep; k++)
      {
        if (cloudLabel[k] <= 0)
        {
          // 准备降采样的点，也就是less point
          surfPointsLessFlatScan->push_back(laserCloud->points[k]);
        }
      }
    }

    // 对平面点进行降采样
    pcl::PointCloud<PointType> surfPointsLessFlatScanDS;
    pcl::VoxelGrid<PointType> downSizeFilter;
    downSizeFilter.setInputCloud(surfPointsLessFlatScan);
    downSizeFilter.setLeafSize(0.2, 0.2, 0.2);
    downSizeFilter.filter(surfPointsLessFlatScanDS);

    *surfPointsLessFlat += surfPointsLessFlatScanDS;
  }

  if (verbose_level > 1)
  {
    ROS_INFO("sort q time %f \n", t_q_sort);
    ROS_INFO("seperate points time %f \n", t_pts.toc());

    ROS_INFO("scan registration time %f ms *************\n", t_whole.toc());
  }
}

// 发布点云
void ScanRegistration::publish()
{
  sensor_msgs::PointCloud2 laserCloudOutMsg;
  pcl::toROSMsg(*laserCloud, laserCloudOutMsg);
  laserCloudOutMsg.header.stamp = ros::Time::now();
  laserCloudOutMsg.header.frame_id = "velo";
  pubLaserCloud.publish(laserCloudOutMsg);

  sensor_msgs::PointCloud2 cornerPointsSharpMsg;
  pcl::toROSMsg(*cornerPointsSharp, cornerPointsSharpMsg);
  cornerPointsSharpMsg.header.stamp = ros::Time::now();
  cornerPointsSharpMsg.header.frame_id = "velo";
  pubCornerPointsSharp.publish(cornerPointsSharpMsg);

  sensor_msgs::PointCloud2 cornerPointsLessSharpMsg;
  pcl::toROSMsg(*cornerPointsLessSharp, cornerPointsLessSharpMsg);
  cornerPointsLessSharpMsg.header.stamp = ros::Time::now();
  cornerPointsLessSharpMsg.header.frame_id = "velo";
  pubCornerPointsLessSharp.publish(cornerPointsLessSharpMsg);

  sensor_msgs::PointCloud2 surfPointsFlat2;
  pcl::toROSMsg(*surfPointsFlat, surfPointsFlat2);
  surfPointsFlat2.header.stamp = ros::Time::now();
  surfPointsFlat2.header.frame_id = "velo";
  pubSurfPointsFlat.publish(surfPointsFlat2);

  sensor_msgs::PointCloud2 surfPointsLessFlat2;
  pcl::toROSMsg(*surfPointsLessFlat, surfPointsLessFlat2);
  surfPointsLessFlat2.header.stamp = ros::Time::now();
  surfPointsLessFlat2.header.frame_id = "velo";
  pubSurfPointsLessFlat.publish(surfPointsLessFlat2);

  // pub each scan
  if (PUB_EACH_LINE)
  {
    for (int i = 0; i < N_SCANS; i++)
    {
      sensor_msgs::PointCloud2 scanMsg;
      pcl::toROSMsg(laserCloudScans[i], scanMsg);
      scanMsg.header.stamp = ros::Time::now();
      scanMsg.header.frame_id = "map";
      pubEachScan[i].publish(scanMsg);
    }
  }

  // ROS_INFO("scan registration time %f ms *************\n", t_whole.toc());
  // if(t_whole.toc() > 100)
  //     ROS_WARN("scan registration process over 100ms");
}

void ScanRegistration::output(pcl::PointCloud<PointType>::Ptr& laserCloud_,
                              pcl::PointCloud<PointType>::Ptr& cornerPointsSharp_,
                              pcl::PointCloud<PointType>::Ptr& cornerPointsLessSharp_,
                              pcl::PointCloud<PointType>::Ptr& surfPointsFlat_,
                              pcl::PointCloud<PointType>::Ptr& surfPointsLessFlat_)
{
  laserCloud_ = this->laserCloud;
  cornerPointsSharp_ = this->cornerPointsSharp;
  cornerPointsLessSharp_ = this->cornerPointsLessSharp;
  surfPointsFlat_ = this->surfPointsFlat;
  surfPointsLessFlat_ = this->surfPointsLessFlat;
}

}  // namespace vloam