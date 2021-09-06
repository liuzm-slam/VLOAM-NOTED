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

#include <lidar_odometry_mapping/laser_mapping.h>

namespace vloam
{
  void LaserMapping::init(std::shared_ptr<VloamTF> &vloam_tf_)
  {
    vloam_tf = vloam_tf_;

    if (!ros::param::get("loam_verbose_level", verbose_level))
      ROS_BREAK();

    frameCount = 0;

    // input: from odom，里程计输入上一帧的点云数据
    laserCloudCornerLast = boost::make_shared<pcl::PointCloud<PointType>>();
    laserCloudSurfLast = boost::make_shared<pcl::PointCloud<PointType>>();

    // ouput: all visualble cube points
    // laserCloudSurround所有激光点
    laserCloudSurround = boost::make_shared<pcl::PointCloud<PointType>>();

    // surround points in map to build tree
    laserCloudCornerFromMap = boost::make_shared<pcl::PointCloud<PointType>>();
    laserCloudSurfFromMap = boost::make_shared<pcl::PointCloud<PointType>>();

    // input & output: points in one frame. local --> global
    laserCloudFullRes = boost::make_shared<pcl::PointCloud<PointType>>();

    // points in every cube
    for (int j = 0; j < laserCloudNum; ++j)
    {
      laserCloudCornerArray[j] = boost::make_shared<pcl::PointCloud<PointType>>();
      laserCloudSurfArray[j] = boost::make_shared<pcl::PointCloud<PointType>>();
    }

    // kd-tree
    kdtreeCornerFromMap = boost::make_shared<pcl::KdTreeFLANN<PointType>>();
    kdtreeSurfFromMap = boost::make_shared<pcl::KdTreeFLANN<PointType>>();

    parameters[0] = 0.0;
    parameters[1] = 0.0;
    parameters[2] = 0.0;
    parameters[3] = 1.0;
    parameters[4] = 0.0;
    parameters[5] = 0.0;
    parameters[6] = 0.0;

    new (&q_w_curr) Eigen::Map<Eigen::Quaterniond>(parameters);
    new (&t_w_curr) Eigen::Map<Eigen::Vector3d>(parameters + 4);

    // wmap_T_odom * odom_T_curr = wmap_T_curr;
    // transformation between odom's world and map's world frame
    // 下面的两个变量是world坐标系下的Odometry计算的位姿和Mapping计算的位姿之间的增量
    // （也即变换，transformation）
    q_wmap_wodom = Eigen::Quaterniond(1, 0, 0, 0);
    t_wmap_wodom = Eigen::Vector3d(0, 0, 0);

    // Odometry线程计算的frame在world坐标系的位姿
    q_wodom_curr = Eigen::Quaterniond(1, 0, 0, 0);
    t_wodom_curr = Eigen::Vector3d(0, 0, 0);

    lineRes = 0;
    planeRes = 0;
    if (!ros::param::get("mapping_line_resolution", lineRes))
      ROS_BREAK();
    if (!ros::param::get("mapping_plane_resolution", planeRes))
      ROS_BREAK();
    ROS_INFO("line resolution %f plane resolution %f \n", lineRes, planeRes);
    downSizeFilterCorner.setLeafSize(lineRes, lineRes, lineRes);
    downSizeFilterSurf.setLeafSize(planeRes, planeRes, planeRes);

    pubLaserCloudSurround = nh.advertise<sensor_msgs::PointCloud2>("/laser_cloud_surround", 100);
    pubLaserCloudMap = nh.advertise<sensor_msgs::PointCloud2>("/laser_cloud_map", 100);
    pubLaserCloudFullRes = nh.advertise<sensor_msgs::PointCloud2>("/velodyne_cloud_registered", 100);
    pubOdomAftMapped = nh.advertise<nav_msgs::Odometry>("/aft_mapped_to_init", 100);
    // pubOdomAftMappedHighFrec = nh->advertise<nav_msgs::Odometry>("/aft_mapped_to_init_high_frec", 100);
    pubLaserAfterMappedPath = nh.advertise<nav_msgs::Path>("/aft_mapped_path", 100);

    laserAfterMappedPath.poses.clear();

    // for (int i = 0; i < laserCloudNum; i++)
    // {
    // 	laserCloudCornerArray[i].reset(new pcl::PointCloud<PointType>());
    // 	laserCloudSurfArray[i].reset(new pcl::PointCloud<PointType>());
    // } // TOOD: check when this is needed

    laserCloudValidNum = 0;
    laserCloudSurroundNum = 0;

    if (!ros::param::get("mapping_skip_frame", mapping_skip_frame))
      ROS_BREAK();
    if (!ros::param::get("map_pub_number", map_pub_number))
      ROS_BREAK();
  }

  // 下面的两个定义可以认为是清空submap
  void LaserMapping::reset()
  {
    laserCloudValidNum = 0;
    laserCloudSurroundNum = 0;
  }

  // // set initial guess
  // void LaserMapping::transformAssociateToMap()
  // {
  //     q_w_curr = q_wmap_wodom * q_wodom_curr;
  //     t_w_curr = q_wmap_wodom * t_wodom_curr + t_wmap_wodom;
  // }

  // 用在最后，当Mapping的位姿w_curr计算完毕后，更新增量wmap_wodom，
  // 旨在为下一次执行transformAssociateToMap函数时做准备
  void LaserMapping::transformUpdate()
  {
    q_wmap_wodom = q_w_curr * q_wodom_curr.inverse();
    t_wmap_wodom = t_w_curr - q_wmap_wodom * t_wodom_curr;
  }

  // 把激光点转换到世界坐标系中
  void LaserMapping::pointAssociateToMap(PointType const *const pi, PointType *const po)
  {
    Eigen::Vector3d point_curr(pi->x, pi->y, pi->z);
    // point_w，map坐标系下的点
    Eigen::Vector3d point_w = q_w_curr * point_curr + t_w_curr;
    po->x = point_w.x();
    po->y = point_w.y();
    po->z = point_w.z();
    po->intensity = pi->intensity;
    // po->intensity = 1.0;
  }

  // 把世界坐标系下的点投影到当前帧激光坐标系下
  void LaserMapping::pointAssociateTobeMapped(PointType const *const pi, PointType *const po)
  {
    Eigen::Vector3d point_w(pi->x, pi->y, pi->z);
    Eigen::Vector3d point_curr = q_w_curr.inverse() * (point_w - t_w_curr);
    po->x = point_curr.x();
    po->y = point_curr.y();
    po->z = point_curr.z();
    po->intensity = pi->intensity;
  }

  void LaserMapping::input(const pcl::PointCloud<PointType>::Ptr &laserCloudCornerLast_,
                           const pcl::PointCloud<PointType>::Ptr &laserCloudSurfLast_,
                           const pcl::PointCloud<PointType>::Ptr &laserCloudFullRes_,
                           const Eigen::Quaterniond &q_wodom_curr_, const Eigen::Vector3d &t_wodom_curr_,
                           const bool &skip_frame_)
  {
    skip_frame = skip_frame_;

    if (!skip_frame)
    {
      laserCloudCornerLast = boost::make_shared<pcl::PointCloud<PointType>>(*laserCloudCornerLast_);
      laserCloudSurfLast = boost::make_shared<pcl::PointCloud<PointType>>(*laserCloudSurfLast_);
      laserCloudFullRes = boost::make_shared<pcl::PointCloud<PointType>>(*laserCloudFullRes_);
    }

    q_wodom_curr = q_wodom_curr_;
    t_wodom_curr = t_wodom_curr_;

    // transformAssociateToMap
    if (skip_frame)
    {
      q_w_curr_highfreq = q_wmap_wodom * q_wodom_curr;
      t_w_curr_highfreq = q_wmap_wodom * t_wodom_curr + t_wmap_wodom;
    }
    else
    {
      // set initial guess，上一帧的增量wmap_wodom * 本帧Odometry位姿wodom_curr，
      // 旨在为本帧Mapping位姿w_curr设置一个初始值
      q_w_curr = q_wmap_wodom * q_wodom_curr;
      t_w_curr = q_wmap_wodom * t_wodom_curr + t_wmap_wodom;
    }
  }

  void LaserMapping::solveMapping()
  {
    // this->reset();

    t_whole.tic();

    // transformAssociateToMap();

    /*   
      centerCubeI，centerCubeJ，centerCubeK，当前点云对应的cube立方体中心位置索引i,j,k，在世界坐标系下的位置
      过半取一（以50米进行四舍五入的效果），由于数组下标只能为正数，而地图可能建立在原点前后，
      因此每一维偏移一个laserCloudCenWidth（该值会动态调整，以使得数组利用最大化，初始值为该维数组长度1/2）的量。 
      调整之后取值范围:3 < centerCubeI < 18， 3 < centerCubeJ < 8, 3 < centerCubeK < 18 
  */
    TicToc t_shift;
    int centerCubeI = int((t_w_curr.x() + 25.0) / 50.0) + laserCloudCenWidth;
    int centerCubeJ = int((t_w_curr.y() + 25.0) / 50.0) + laserCloudCenHeight;
    int centerCubeK = int((t_w_curr.z() + 25.0) / 50.0) + laserCloudCenDepth;

    /*   由于计算机求余是向零取整，为了不使（-50.0,50.0）求余后都向零偏移，
  当被求余数为负数时求余结果统一向左偏移一个单位，也即减一 */
    if (t_w_curr.x() + 25.0 < 0)
      centerCubeI--;
    if (t_w_curr.y() + 25.0 < 0)
      centerCubeJ--;
    if (t_w_curr.z() + 25.0 < 0)
      centerCubeK--;

  /*   
    调整边缘位置向中心移动，确保位姿在cube中的相对位置有555的邻域
    如果处于下边界，表明地图向负方向延伸的可能性比较大，则循环移位，将数组中心点向上边界调整一个单位 
  */
    // 以下注释部分参照LOAM_NOTED，结合我画的submap的示意图说明下面的6个while loop的作用：要
    // 注意世界坐标系下的点云地图是固定的，但是IJK坐标系我们是可以移动的，所以这6个while loop
    // 的作用就是调整IJK坐标系（也就是调整所有cube位置），使得五角星在IJK坐标系的坐标范围处于
    // 3 < centerCubeI < 18， 3 < centerCubeJ < 8, 3 < centerCubeK < 18，目的是为了防止后续向
    // 四周拓展cube（图中的黄色cube就是拓展的cube）时，index（即IJK坐标）成为负数。
    while (centerCubeI < 3)
    {
      for (int j = 0; j < laserCloudHeight; j++)
      {
        for (int k = 0; k < laserCloudDepth; k++)
        {
          int i = laserCloudWidth - 1;
          // i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k = i + 21 * j + 231 * k
          pcl::PointCloud<PointType>::Ptr laserCloudCubeCornerPointer =
              laserCloudCornerArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k];
          pcl::PointCloud<PointType>::Ptr laserCloudCubeSurfPointer =
              laserCloudSurfArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k];
          // 循环移位，I维度上依次后移
          // 在I方向cube[I]=cube[I-1],清空最后一个空出来的cube，实现IJK坐标系向I轴负方向移动一个cube的
          // 效果，从相对运动的角度看是图中的五角星在IJK坐标系下向I轴正方向移动了一个cube，如下面动图所示，所
          // 以centerCubeI最后++，laserCloudCenWidth也++，为下一帧Mapping时计算五角星的IJK坐标做准备
          for (; i >= 1; i--)
          {
            // 向前移动
            laserCloudCornerArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] =
                laserCloudCornerArray[i - 1 + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k];
            laserCloudSurfArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] =
                laserCloudSurfArray[i - 1 + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k];
          }
          // 将开始点赋值为最后一个点
          laserCloudCornerArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] =
              laserCloudCubeCornerPointer;
          laserCloudSurfArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] =
              laserCloudCubeSurfPointer;
          laserCloudCubeCornerPointer->clear();
          laserCloudCubeSurfPointer->clear();
        }
      }

      // 对应索引+1
      centerCubeI++;
      laserCloudCenWidth++;
    }

    while (centerCubeI >= laserCloudWidth - 3)
    {
      for (int j = 0; j < laserCloudHeight; j++)
      {
        for (int k = 0; k < laserCloudDepth; k++)
        {
          int i = 0;
          pcl::PointCloud<PointType>::Ptr laserCloudCubeCornerPointer =
              laserCloudCornerArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k];
          pcl::PointCloud<PointType>::Ptr laserCloudCubeSurfPointer =
              laserCloudSurfArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k];
          for (; i < laserCloudWidth - 1; i++)
          {
            laserCloudCornerArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] =
                laserCloudCornerArray[i + 1 + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k];
            laserCloudSurfArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] =
                laserCloudSurfArray[i + 1 + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k];
          }
          laserCloudCornerArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] =
              laserCloudCubeCornerPointer;
          laserCloudSurfArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] =
              laserCloudCubeSurfPointer;
          laserCloudCubeCornerPointer->clear();
          laserCloudCubeSurfPointer->clear();
        }
      }

      centerCubeI--;
      laserCloudCenWidth--;
    }

    while (centerCubeJ < 3)
    {
      for (int i = 0; i < laserCloudWidth; i++)
      {
        for (int k = 0; k < laserCloudDepth; k++)
        {
          int j = laserCloudHeight - 1;
          pcl::PointCloud<PointType>::Ptr laserCloudCubeCornerPointer =
              laserCloudCornerArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k];
          pcl::PointCloud<PointType>::Ptr laserCloudCubeSurfPointer =
              laserCloudSurfArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k];
          for (; j >= 1; j--)
          {
            laserCloudCornerArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] =
                laserCloudCornerArray[i + laserCloudWidth * (j - 1) + laserCloudWidth * laserCloudHeight * k];
            laserCloudSurfArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] =
                laserCloudSurfArray[i + laserCloudWidth * (j - 1) + laserCloudWidth * laserCloudHeight * k];
          }
          laserCloudCornerArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] =
              laserCloudCubeCornerPointer;
          laserCloudSurfArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] =
              laserCloudCubeSurfPointer;
          laserCloudCubeCornerPointer->clear();
          laserCloudCubeSurfPointer->clear();
        }
      }

      centerCubeJ++;
      laserCloudCenHeight++;
    }

    while (centerCubeJ >= laserCloudHeight - 3)
    {
      for (int i = 0; i < laserCloudWidth; i++)
      {
        for (int k = 0; k < laserCloudDepth; k++)
        {
          int j = 0;
          pcl::PointCloud<PointType>::Ptr laserCloudCubeCornerPointer =
              laserCloudCornerArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k];
          pcl::PointCloud<PointType>::Ptr laserCloudCubeSurfPointer =
              laserCloudSurfArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k];
          for (; j < laserCloudHeight - 1; j++)
          {
            laserCloudCornerArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] =
                laserCloudCornerArray[i + laserCloudWidth * (j + 1) + laserCloudWidth * laserCloudHeight * k];
            laserCloudSurfArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] =
                laserCloudSurfArray[i + laserCloudWidth * (j + 1) + laserCloudWidth * laserCloudHeight * k];
          }
          laserCloudCornerArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] =
              laserCloudCubeCornerPointer;
          laserCloudSurfArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] =
              laserCloudCubeSurfPointer;
          laserCloudCubeCornerPointer->clear();
          laserCloudCubeSurfPointer->clear();
        }
      }

      centerCubeJ--;
      laserCloudCenHeight--;
    }

    while (centerCubeK < 3)
    {
      for (int i = 0; i < laserCloudWidth; i++)
      {
        for (int j = 0; j < laserCloudHeight; j++)
        {
          int k = laserCloudDepth - 1;
          pcl::PointCloud<PointType>::Ptr laserCloudCubeCornerPointer =
              laserCloudCornerArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k];
          pcl::PointCloud<PointType>::Ptr laserCloudCubeSurfPointer =
              laserCloudSurfArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k];
          for (; k >= 1; k--)
          {
            laserCloudCornerArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] =
                laserCloudCornerArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * (k - 1)];
            laserCloudSurfArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] =
                laserCloudSurfArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * (k - 1)];
          }
          laserCloudCornerArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] =
              laserCloudCubeCornerPointer;
          laserCloudSurfArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] =
              laserCloudCubeSurfPointer;
          laserCloudCubeCornerPointer->clear();
          laserCloudCubeSurfPointer->clear();
        }
      }

      centerCubeK++;
      laserCloudCenDepth++;
    }

    while (centerCubeK >= laserCloudDepth - 3)
    {
      for (int i = 0; i < laserCloudWidth; i++)
      {
        for (int j = 0; j < laserCloudHeight; j++)
        {
          int k = 0;
          pcl::PointCloud<PointType>::Ptr laserCloudCubeCornerPointer =
              laserCloudCornerArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k];
          pcl::PointCloud<PointType>::Ptr laserCloudCubeSurfPointer =
              laserCloudSurfArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k];
          for (; k < laserCloudDepth - 1; k++)
          {
            laserCloudCornerArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] =
                laserCloudCornerArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * (k + 1)];
            laserCloudSurfArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] =
                laserCloudSurfArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * (k + 1)];
          }
          laserCloudCornerArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] =
              laserCloudCubeCornerPointer;
          laserCloudSurfArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] =
              laserCloudCubeSurfPointer;
          laserCloudCubeCornerPointer->clear();
          laserCloudCubeSurfPointer->clear();
        }
      }

      centerCubeK--;
      laserCloudCenDepth--;
    }

    // 生成submap的特征点云如下，需要注意的是，LOAM这种方式生成的submap并不是基于时序的滑动窗口方式，
    // 而是基于空间范围划分的方式：
    for (int i = centerCubeI - 2; i <= centerCubeI + 2; i++)
    {
      for (int j = centerCubeJ - 2; j <= centerCubeJ + 2; j++)
      {
        for (int k = centerCubeK - 1; k <= centerCubeK + 1; k++)
        {
          // 如果坐标合法
          if (i >= 0 && i < laserCloudWidth && j >= 0 && j < laserCloudHeight && k >= 0 && k < laserCloudDepth)
          {
            // 记录submap中的所有cube的index，记为有效index
            laserCloudValidInd[laserCloudValidNum] = i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k;
            laserCloudValidNum++;
            laserCloudSurroundInd[laserCloudSurroundNum] =
                i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k;
            laserCloudSurroundNum++;
          }
        }
      }
    }

    /* 
      需要两堆进行配准的点云，滤波处理
      滤波处理点云保证点云更光滑。

      构建特征点地图，查找匹配使用，选择上一时刻点作为配准的点
      先从世界坐标系转为Lidar坐标系
      滤波处理，降采样，最后得到downSizeFilterCorner和downSizeFilterSurf
    */
    laserCloudCornerFromMap->clear();
    laserCloudSurfFromMap->clear();
    // 构建特征点地图，查找匹配使用，选择上一时刻点作为配准的点
    for (int i = 0; i < laserCloudValidNum; i++)
    {
      // laserCloudCornerArray，laserCloudSurfArray 存放特征点的立方体cube
      // 将有效index的cube中的点云叠加到一起组成submap的特征点云
      *laserCloudCornerFromMap += *laserCloudCornerArray[laserCloudValidInd[i]];
      *laserCloudSurfFromMap += *laserCloudSurfArray[laserCloudValidInd[i]];
    }

    // 特征点数
    int laserCloudCornerFromMapNum = laserCloudCornerFromMap->points.size();
    int laserCloudSurfFromMapNum = laserCloudSurfFromMap->points.size();

    // 点云下采样，laserCloudCornerStack，laserCloudSurfStack下采样后的点云
    pcl::PointCloud<PointType>::Ptr laserCloudCornerStack(new pcl::PointCloud<PointType>());
    downSizeFilterCorner.setInputCloud(laserCloudCornerLast);
    downSizeFilterCorner.filter(*laserCloudCornerStack);
    int laserCloudCornerStackNum = laserCloudCornerStack->points.size();

    pcl::PointCloud<PointType>::Ptr laserCloudSurfStack(new pcl::PointCloud<PointType>());
    downSizeFilterSurf.setInputCloud(laserCloudSurfLast);
    downSizeFilterSurf.filter(*laserCloudSurfStack);
    int laserCloudSurfStackNum = laserCloudSurfStack->points.size();

    if (verbose_level > 1)
    {
      ROS_INFO("map prepare time %f ms\n", t_shift.toc());
      ROS_INFO("map corner num %d  surf num %d \n", laserCloudCornerFromMapNum, laserCloudSurfFromMapNum);
    }

    /* 
      点云配准，拿出KD树，来寻找最邻近的5个点，对点云协方差矩阵进行主成分分析
      在得到Lidar所在位置附近的地图以及当前帧点云特征点，通过配准的点云，拿出KD树寻找最临近的5个点，对点云协方差矩阵进行主成分分析。
      若这五个点分布在直线上，协方差矩阵的特征值包含一个元素显著大于其余两个，与该特征值相关的特征向量表示所处直线的方向；
      若这五个点分布在平面上，协方差矩阵的特征值存在一个显著小的元素，与该特征值相关的特征向量表示所处平面的法线方向。 
  */
    if (laserCloudCornerFromMapNum > 10 && laserCloudSurfFromMapNum > 50)
    {
      TicToc t_opt;
      TicToc t_tree;
      // 构建kd-tree
      kdtreeCornerFromMap->setInputCloud(laserCloudCornerFromMap);
      kdtreeSurfFromMap->setInputCloud(laserCloudSurfFromMap);

      if (verbose_level > 1)
        ROS_INFO("build tree time %f ms \n", t_tree.toc());

      // 迭代次数
      for (int iterCount = 0; iterCount < 2; iterCount++)
      {
        // ceres::LossFunction *loss_function = NULL;
        ceres::LossFunction *loss_function = new ceres::HuberLoss(0.1);
        ceres::LocalParameterization *q_parameterization = new ceres::EigenQuaternionParameterization();
        ceres::Problem::Options problem_options;

        ceres::Problem problem(problem_options);
        // parameters机器人的(R，t)
        problem.AddParameterBlock(parameters, 4, q_parameterization);
        problem.AddParameterBlock(parameters + 4, 3);

        TicToc t_data;
        int corner_num = 0;

        /* 
        以降采样后的submap特征点云为target，以当前帧降采样后的特征点云为source的ICP过程了，同样地也分为点到线和点到面的匹配，
        基本流程与Odometry线程相同，不同的是建立correspondence（关联）的方式不同，具体看下面的介绍，先是点到线的ICP过程： 
      */
        for (int i = 0; i < laserCloudCornerStackNum; i++)
        {
          pointOri = laserCloudCornerStack->points[i];
          // double sqrtDis = pointOri.x * pointOri.x + pointOri.y * pointOri.y + pointOri.z * pointOri.z;
          // 把当前激光点转到世界坐标系下
          // 需要注意的是submap中的点云都是world坐标系，而当前帧的点云都是Lidar坐标系，所以
          // 在搜寻最近邻点时，先用预测的Mapping位姿w_curr，将Lidar坐标系下的特征点变换到world坐标系下
          pointAssociateToMap(&pointOri, &pointSel);
          // 在submap的corner特征点（target）中，寻找距离当前帧corner特征点（source）最近的5个点
          kdtreeCornerFromMap->nearestKSearch(pointSel, 5, pointSearchInd, pointSearchSqDis);

          // pca处理
          if (pointSearchSqDis[4] < 1.0)
          {
            std::vector<Eigen::Vector3d> nearCorners;
            Eigen::Vector3d center(0, 0, 0);
            for (int j = 0; j < 5; j++)
            {
              Eigen::Vector3d tmp(laserCloudCornerFromMap->points[pointSearchInd[j]].x,
                                  laserCloudCornerFromMap->points[pointSearchInd[j]].y,
                                  laserCloudCornerFromMap->points[pointSearchInd[j]].z);
              center = center + tmp;      // 求出五个最近点的中心值的和
              nearCorners.push_back(tmp); // 当前激光点对应submap的五个最近点（nearCorners）
            }
            // pca求中心值
            center = center / 5.0;

            // 协方差矩阵covMat
            Eigen::Matrix3d covMat = Eigen::Matrix3d::Zero();
            for (int j = 0; j < 5; j++)
            {
              // tmpZeroMean去中心化
              Eigen::Matrix<double, 3, 1> tmpZeroMean = nearCorners[j] - center;
              // 求协方差矩阵
              covMat = covMat + tmpZeroMean * tmpZeroMean.transpose();
            }

            // 计算协方差矩阵的特征值和特征向量，用于判断这5个点是不是呈线状分布，此为PCA的原理
            Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> saes(covMat);

            // if is indeed line feature
            // note Eigen library sort eigenvalues in increasing order
            // 如果5个点呈线状分布，最大的特征值对应的特征向量就是该线的方向向量
            Eigen::Vector3d unit_direction = saes.eigenvectors().col(2); // 计算特征值和特征向量
            Eigen::Vector3d curr_point(pointOri.x, pointOri.y, pointOri.z);
            // 如果特征值大于其它二个特征值认为五个点成线状分布
            if (saes.eigenvalues()[2] > 3 * saes.eigenvalues()[1])
            {
              Eigen::Vector3d point_on_line = center;
              Eigen::Vector3d point_a, point_b;
              // 从中心点沿着方向向量向两端移动0.1m，构造线上的两个点
              point_a = 0.1 * unit_direction + point_on_line;
              point_b = -0.1 * unit_direction + point_on_line;

              // 然后残差函数的形式就跟Odometry一样了，残差距离即点到线的距离，到介绍lidarFactor.cpp时再说明具体计算方法
              ceres::CostFunction *cost_function = LidarEdgeFactor::Create(curr_point, point_a, point_b, 1.0);
              problem.AddResidualBlock(cost_function, loss_function, parameters, parameters + 4);
              corner_num++;
            }
          }
          /*
        else if(pointSearchSqDis[4] < 0.01 * sqrtDis)
        {
            Eigen::Vector3d center(0, 0, 0);
            for (int j = 0; j < 5; j++)
            {
                Eigen::Vector3d tmp(laserCloudCornerFromMap->points[pointSearchInd[j]].x,
                                    laserCloudCornerFromMap->points[pointSearchInd[j]].y,
                                    laserCloudCornerFromMap->points[pointSearchInd[j]].z);
                center = center + tmp;
            }
            center = center / 5.0;
            Eigen::Vector3d curr_point(pointOri.x, pointOri.y, pointOri.z);
            ceres::CostFunction *cost_function = LidarDistanceFactor::Create(curr_point, center);
            problem.AddResidualBlock(cost_function, loss_function, parameters, parameters + 4);
        }
        */
        }

        // 点到sub_map平面的距离
        int surf_num = 0;
        for (int i = 0; i < laserCloudSurfStackNum; i++)
        {
          pointOri = laserCloudSurfStack->points[i];
          // double sqrtDis = pointOri.x * pointOri.x + pointOri.y * pointOri.y + pointOri.z * pointOri.z;
          // 转换到世界坐标系下
          pointAssociateToMap(&pointOri, &pointSel);
          // 平面也在sub_map上找5个点
          kdtreeSurfFromMap->nearestKSearch(pointSel, 5, pointSearchInd, pointSearchSqDis);

          // 求平面的法向量使用最小二乘法拟合
          // 假设平面不通过原点，则平面的一般方程为Ax + By + Cz + 1 = 0，用这个假设可以少算一个参数，提效
          Eigen::Matrix<double, 5, 3> matA0;
          // Matrix3d::Ones()全为1的矩阵
          Eigen::Matrix<double, 5, 1> matB0 = -1 * Eigen::Matrix<double, 5, 1>::Ones();
          // 用上面的2个矩阵表示平面方程就是 matA0 * norm（A, B, C） = matB0，
          // 这是个超定方程组，因为数据个数超过未知数的个数
          if (pointSearchSqDis[4] < 1.0)
          {
            for (int j = 0; j < 5; j++)
            {
              // 构建五个临近点的矩阵（x,y,z）
              matA0(j, 0) = laserCloudSurfFromMap->points[pointSearchInd[j]].x;
              matA0(j, 1) = laserCloudSurfFromMap->points[pointSearchInd[j]].y;
              matA0(j, 2) = laserCloudSurfFromMap->points[pointSearchInd[j]].z;
              // ROS_INFO(" pts %f %f %f ", matA0(j, 0), matA0(j, 1), matA0(j, 2));
            }
            // find the norm of plane
            // 求解（A，B，C）超定方程，求解这个最小二乘问题，可得平面的法向量
            // （Eigen矩阵colPivHouseholderQr().solve()求解Ax=b）
            Eigen::Vector3d norm = matA0.colPivHouseholderQr().solve(matB0);
            double negative_OA_dot_norm = 1 / norm.norm();
            norm.normalize(); // 单位化

            // Here n(pa, pb, pc) is unit norm of plane
            bool planeValid = true;
            for (int j = 0; j < 5; j++)
            {
              // if OX * n > 0.2, then plane is not fit well
              // Ax+By+Cz+D=0 判断平面拟合平面的好坏
              if (fabs(norm(0) * laserCloudSurfFromMap->points[pointSearchInd[j]].x +
                       norm(1) * laserCloudSurfFromMap->points[pointSearchInd[j]].y +
                       norm(2) * laserCloudSurfFromMap->points[pointSearchInd[j]].z + negative_OA_dot_norm) > 0.2)
              {
                planeValid = false; // 平面没有拟合好，平面“不够平“
                break;
              }
            }
            Eigen::Vector3d curr_point(pointOri.x, pointOri.y, pointOri.z);
            if (planeValid)
            {
              // 构造点到面的距离残差项
              ceres::CostFunction *cost_function = LidarPlaneNormFactor::Create(curr_point, norm, negative_OA_dot_norm);
              problem.AddResidualBlock(cost_function, loss_function, parameters, parameters + 4);
              surf_num++;
            }
          }
          /*
        else if(pointSearchSqDis[4] < 0.01 * sqrtDis)
        {
            Eigen::Vector3d center(0, 0, 0);
            for (int j = 0; j < 5; j++)
            {
                Eigen::Vector3d tmp(laserCloudSurfFromMap->points[pointSearchInd[j]].x,
                                    laserCloudSurfFromMap->points[pointSearchInd[j]].y,
                                    laserCloudSurfFromMap->points[pointSearchInd[j]].z);
                center = center + tmp;
            }
            center = center / 5.0;
            Eigen::Vector3d curr_point(pointOri.x, pointOri.y, pointOri.z);
            ceres::CostFunction *cost_function = LidarDistanceFactor::Create(curr_point, center);
            problem.AddResidualBlock(cost_function, loss_function, parameters, parameters + 4);
        }
        */
        }

        if (verbose_level > 1)
        {
          // ROS_INFO("corner num %d used corner num %d \n", laserCloudCornerStackNum, corner_num);
          // ROS_INFO("surf num %d used surf num %d \n", laserCloudSurfStackNum, surf_num);

          ROS_INFO("mapping data assosiation time %f ms \n", t_data.toc());
        }

        TicToc t_solver;
        ceres::Solver::Options options;
        options.linear_solver_type = ceres::DENSE_QR;
        options.max_num_iterations = 4;
        options.minimizer_progress_to_stdout = false;
        options.check_gradients = false;
        options.gradient_check_relative_precision = 1e-4;
        ceres::Solver::Summary summary;
        ceres::Solve(options, &problem, &summary);

        if (verbose_level > 1)
          ROS_INFO("mapping solver time %f ms \n", t_solver.toc());

        // ROS_INFO("time %f \n", timeLaserOdometry);
        // ROS_INFO("corner factor num %d surf factor num %d\n", corner_num, surf_num);
        // ROS_INFO("result q %f %f %f %f result t %f %f %f\n", parameters[3], parameters[0], parameters[1],
        // parameters[2], 	   parameters[4], parameters[5], parameters[6]);
      }

      if (verbose_level > 1)
        ROS_INFO("mapping optimization time %f \n", t_opt.toc());
    }
    else
    {
      if (verbose_level > 1)
        ROS_WARN("time Map corner and surf num are not enough");
    }
    // 完成（迭代2次）的特征匹配后，用最后匹配计算出的优化变量w_curr，更新增量wmap_wodom，为下一次Mapping做准备
    transformUpdate();

    TicToc t_add;
    // 下面两个for loop的作用就是将当前帧的特征点云，逐点进行操作：转换到world坐标系并添加到对应位置的cube中
    for (int i = 0; i < laserCloudCornerStackNum; i++)
    {
      // Lidar坐标系转到world坐标系
      pointAssociateToMap(&laserCloudCornerStack->points[i], &pointSel);

      // 计算本次的特征点的IJK坐标，进而确定添加到哪个cube中
      int cubeI = int((pointSel.x + 25.0) / 50.0) + laserCloudCenWidth;
      int cubeJ = int((pointSel.y + 25.0) / 50.0) + laserCloudCenHeight;
      int cubeK = int((pointSel.z + 25.0) / 50.0) + laserCloudCenDepth;

      if (pointSel.x + 25.0 < 0)
        cubeI--;
      if (pointSel.y + 25.0 < 0)
        cubeJ--;
      if (pointSel.z + 25.0 < 0)
        cubeK--;

      if (cubeI >= 0 && cubeI < laserCloudWidth && cubeJ >= 0 && cubeJ < laserCloudHeight && cubeK >= 0 &&
          cubeK < laserCloudDepth)
      {
        int cubeInd = cubeI + laserCloudWidth * cubeJ + laserCloudWidth * laserCloudHeight * cubeK;
        laserCloudCornerArray[cubeInd]->push_back(pointSel);
      }
    }

    for (int i = 0; i < laserCloudSurfStackNum; i++)
    {
      pointAssociateToMap(&laserCloudSurfStack->points[i], &pointSel);

      // 计算本次的特征点的IJK坐标，进而确定添加到哪个cube中
      int cubeI = int((pointSel.x + 25.0) / 50.0) + laserCloudCenWidth;
      int cubeJ = int((pointSel.y + 25.0) / 50.0) + laserCloudCenHeight;
      int cubeK = int((pointSel.z + 25.0) / 50.0) + laserCloudCenDepth;

      if (pointSel.x + 25.0 < 0)
        cubeI--;
      if (pointSel.y + 25.0 < 0)
        cubeJ--;
      if (pointSel.z + 25.0 < 0)
        cubeK--;

      if (cubeI >= 0 && cubeI < laserCloudWidth && cubeJ >= 0 && cubeJ < laserCloudHeight && cubeK >= 0 &&
          cubeK < laserCloudDepth)
      {
        int cubeInd = cubeI + laserCloudWidth * cubeJ + laserCloudWidth * laserCloudHeight * cubeK;
        laserCloudSurfArray[cubeInd]->push_back(pointSel);
      }
    }

    if (verbose_level > 1)
      ROS_INFO("add points time %f ms\n", t_add.toc());

    TicToc t_filter;
    // 因为新增加了点云，对之前已经存有点云的cube全部重新进行一次降采样
    for (int i = 0; i < laserCloudValidNum; i++)
    {
      int ind = laserCloudValidInd[i];

      pcl::PointCloud<PointType>::Ptr tmpCorner(new pcl::PointCloud<PointType>());
      downSizeFilterCorner.setInputCloud(laserCloudCornerArray[ind]);
      downSizeFilterCorner.filter(*tmpCorner);
      laserCloudCornerArray[ind] = tmpCorner;

      pcl::PointCloud<PointType>::Ptr tmpSurf(new pcl::PointCloud<PointType>());
      downSizeFilterSurf.setInputCloud(laserCloudSurfArray[ind]);
      downSizeFilterSurf.filter(*tmpSurf);
      laserCloudSurfArray[ind] = tmpSurf;
    }

    if (verbose_level > 1)
      ROS_INFO("filter time %f ms \n", t_filter.toc());

    frameCount++;
  }

  void LaserMapping::publish()
  { // TODO: take in global skip frame
    TicToc t_pub;

    nav_msgs::Odometry odomAftMapped;
    odomAftMapped.header.frame_id = "map";
    odomAftMapped.child_frame_id = "aft_mapped";
    odomAftMapped.header.stamp = ros::Time::now(); // TODO: globally config time stamp
    if (!skip_frame)
    {
      odomAftMapped.pose.pose.orientation.x = q_w_curr.x();
      odomAftMapped.pose.pose.orientation.y = q_w_curr.y();
      odomAftMapped.pose.pose.orientation.z = q_w_curr.z();
      odomAftMapped.pose.pose.orientation.w = q_w_curr.w();
      odomAftMapped.pose.pose.position.x = t_w_curr.x();
      odomAftMapped.pose.pose.position.y = t_w_curr.y();
      odomAftMapped.pose.pose.position.z = t_w_curr.z();

      vloam_tf->world_MOT_base_last.setOrigin(tf2::Vector3(t_w_curr.x(), t_w_curr.y(), t_w_curr.z()));
      vloam_tf->world_MOT_base_last.setRotation(tf2::Quaternion(q_w_curr.x(), q_w_curr.y(), q_w_curr.z(), q_w_curr.w()));

      // ROS_INFO(
      //     "lidar_mapping: q = %.4f, %.4f, %.4f, %.4f; t = %.4f, %.4f, %.4f",
      //     q_w_curr.x(),
      //     q_w_curr.y(),
      //     q_w_curr.z(),
      //     q_w_curr.w(),
      //     t_w_curr.x(),
      //     t_w_curr.y(),
      //     t_w_curr.z()
      // );
    }
    else
    {
      odomAftMapped.pose.pose.orientation.x = q_w_curr_highfreq.x();
      odomAftMapped.pose.pose.orientation.y = q_w_curr_highfreq.y();
      odomAftMapped.pose.pose.orientation.z = q_w_curr_highfreq.z();
      odomAftMapped.pose.pose.orientation.w = q_w_curr_highfreq.w();
      odomAftMapped.pose.pose.position.x = t_w_curr_highfreq.x();
      odomAftMapped.pose.pose.position.y = t_w_curr_highfreq.y();
      odomAftMapped.pose.pose.position.z = t_w_curr_highfreq.z();

      vloam_tf->world_MOT_base_last.setOrigin(
          tf2::Vector3(t_w_curr_highfreq.x(), t_w_curr_highfreq.y(), t_w_curr_highfreq.z()));
      vloam_tf->world_MOT_base_last.setRotation(
          tf2::Quaternion(q_w_curr_highfreq.x(), q_w_curr_highfreq.y(), q_w_curr_highfreq.z(), q_w_curr_highfreq.w()));
    }
    pubOdomAftMapped.publish(odomAftMapped);

    geometry_msgs::PoseStamped laserAfterMappedPose;
    laserAfterMappedPose.header = odomAftMapped.header;
    laserAfterMappedPose.pose = odomAftMapped.pose.pose;
    laserAfterMappedPath.header.stamp = odomAftMapped.header.stamp;
    laserAfterMappedPath.header.frame_id = "map";
    laserAfterMappedPath.poses.push_back(laserAfterMappedPose);
    pubLaserAfterMappedPath.publish(laserAfterMappedPath);

    static tf::TransformBroadcaster br;
    tf::Transform transform;
    tf::Quaternion q;
    transform.setOrigin(tf::Vector3(t_w_curr(0), t_w_curr(1), t_w_curr(2)));
    q.setW(q_w_curr.w());
    q.setX(q_w_curr.x());
    q.setY(q_w_curr.y());
    q.setZ(q_w_curr.z());
    transform.setRotation(q);
    br.sendTransform(tf::StampedTransform(transform, odomAftMapped.header.stamp, "map", "aft_mapped"));

    if ((frameCount * mapping_skip_frame) % map_pub_number == 0) // 0.5 Hz?
    {
      pcl::PointCloud<PointType> laserCloudMap;
      for (int i = 0; i < 4851; i++)
      {
        laserCloudMap += *laserCloudCornerArray[i];
        laserCloudMap += *laserCloudSurfArray[i];
      }
      sensor_msgs::PointCloud2 laserCloudMsg;
      pcl::toROSMsg(laserCloudMap, laserCloudMsg);
      laserCloudMsg.header.stamp = ros::Time::now(); // TODO: globally config time stamp
      laserCloudMsg.header.frame_id = "velo_origin";
      pubLaserCloudMap.publish(laserCloudMsg);
      if (verbose_level > 1)
        ROS_INFO("publishing the map, with %ld points", laserCloudMap.size());
    }

    int laserCloudFullResNum = laserCloudFullRes->points.size();
    for (int i = 0; i < laserCloudFullResNum; i++)
    {
      pointAssociateToMap(&laserCloudFullRes->points[i], &laserCloudFullRes->points[i]);
    }

    sensor_msgs::PointCloud2 laserCloudFullRes3; // Q: what's the difference between laserCloudFullRes 1 2 and 3?
    pcl::toROSMsg(*laserCloudFullRes, laserCloudFullRes3);
    laserCloudFullRes3.header.stamp = ros::Time::now(); // TODO: globally config time stamp
    laserCloudFullRes3.header.frame_id = "map";
    pubLaserCloudFullRes.publish(laserCloudFullRes3);

    if (verbose_level > 1)
    {
      ROS_INFO("mapping pub time %f ms \n", t_pub.toc());

      ROS_INFO("whole mapping time %f ms +++++\n",
               t_whole.toc()); // TODO check if t_whole tictoc obj can be shared in class
    }
  }

} // namespace vloam