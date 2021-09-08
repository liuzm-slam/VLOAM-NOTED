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

#include <lidar_odometry_mapping/laser_odometry.h>

namespace vloam
{
// init before a new rosbag comes
void LaserOdometry::init(std::shared_ptr<VloamTF>& vloam_tf_)
{
  vloam_tf = vloam_tf_;

  if (!ros::param::get("loam_verbose_level", verbose_level))
    ROS_BREAK();
  if (!ros::param::get("detach_VO_LO", detach_VO_LO))
    ROS_BREAK();

  corner_correspondence = 0;
  plane_correspondence = 0;

  if (!ros::param::get("mapping_skip_frame", mapping_skip_frame))
    ROS_BREAK();

  systemInited = false;

  timeCornerPointsSharp = 0;
  timeCornerPointsLessSharp = 0;
  timeSurfPointsFlat = 0;
  timeSurfPointsLessFlat = 0;
  timeLaserCloudFullRes = 0;

  kdtreeCornerLast = boost::make_shared<pcl::KdTreeFLANN<pcl::PointXYZI>>();
  kdtreeSurfLast = boost::make_shared<pcl::KdTreeFLANN<pcl::PointXYZI>>();

  cornerPointsSharp = boost::make_shared<pcl::PointCloud<PointType>>();
  cornerPointsLessSharp = boost::make_shared<pcl::PointCloud<PointType>>();
  surfPointsFlat = boost::make_shared<pcl::PointCloud<PointType>>();
  surfPointsLessFlat = boost::make_shared<pcl::PointCloud<PointType>>();

  laserCloudCornerLast = boost::make_shared<pcl::PointCloud<PointType>>();
  laserCloudSurfLast = boost::make_shared<pcl::PointCloud<PointType>>();
  laserCloudFullRes = boost::make_shared<pcl::PointCloud<PointType>>();

  laserCloudCornerLastNum = 0;
  laserCloudSurfLastNum = 0;

  // Transformation from current frame to world frame
  // 当前帧到世界坐标系
  q_w_curr = Eigen::Quaterniond(1, 0, 0, 0);
  t_w_curr = Eigen::Vector3d(0, 0, 0);

  // q_curr_last(x, y, z, w), t_curr_last
  para_q[0] = 0.0;
  para_q[1] = 0.0;
  para_q[2] = 0.0;
  para_q[3] = 1.0;
  para_t[0] = 0.0;
  para_t[1] = 0.0;
  para_t[2] = 0.0;

  // // q_last_curr = Eigen::Map<Eigen::Quaterniond>(para_q);
  // // t_last_curr = Eigen::Map<Eigen::Vector3d>(para_t);
  new (&q_last_curr) Eigen::Map<Eigen::Quaterniond>(para_q);
  new (&t_last_curr) Eigen::Map<Eigen::Vector3d>(para_t);

  // ROS_INFO("INIT raw: q_last_curr: w = %f x = %f, y = %f, z = %f", para_q[3], para_q[0], para_q[1], para_q[2]);
  // ROS_INFO("INIT: q_last_curr: w = %f x = %f, y = %f, z = %f", q_last_curr.w(), q_last_curr.x(), q_last_curr.y(),
  // q_last_curr.z()); ROS_INFO("INIT: t_last_curr: x = %f, y = %f, z = %f", t_last_curr.x(), t_last_curr.y(),
  // t_last_curr.z());

  // Eigen::Map<Eigen::Quaterniond> q_last_curr2(para_q);
  // Eigen::Map<Eigen::Vector3d> t_last_curr2(para_t);
  // ROS_INFO("INIT2: q_last_curr: w = %f x = %f, y = %f, z = %f", q_last_curr2.w(), q_last_curr2.x(), q_last_curr2.y(),
  // q_last_curr2.z()); ROS_INFO("INIT2: t_last_curr: x = %f, y = %f, z = %f", t_last_curr2.x(), t_last_curr2.y(),
  // t_last_curr2.z());

  pubLaserCloudCornerLast = nh.advertise<sensor_msgs::PointCloud2>("/laser_cloud_corner_last", 100);
  pubLaserCloudSurfLast = nh.advertise<sensor_msgs::PointCloud2>("/laser_cloud_surf_last", 100);
  pubLaserCloudFullRes = nh.advertise<sensor_msgs::PointCloud2>("/velodyne_cloud_3", 100);
  pubLaserOdometry = nh.advertise<nav_msgs::Odometry>("/laser_odom_to_init", 100);
  pubLaserPath = nh.advertise<nav_msgs::Path>("/laser_odom_path", 100);

  laserPath.poses.clear();

  frameCount = 0;
}

// // reset after a new message comes
// void LaserOdometry::reset() {

//     // LOAM must use previous f2f odom as a prior
//     // para_q[0] = 0.0;
//     // para_q[1] = 0.0;
//     // para_q[2] = 0.0;
//     // para_q[3] = 1.0;
//     // para_t[0] = 0.0;
//     // para_t[1] = 0.0;
//     // para_t[2] = 0.0;

//     // new (&q_last_curr) Eigen::Map<Eigen::Quaterniond>(para_q);
//     // new (&t_last_curr) Eigen::Map<Eigen::Vector3d>(para_t);
// }

// 输入点云
void LaserOdometry::input(const pcl::PointCloud<PointType>::Ptr& laserCloud_,
                          const pcl::PointCloud<PointType>::Ptr& cornerPointsSharp_,
                          const pcl::PointCloud<PointType>::Ptr& cornerPointsLessSharp_,
                          const pcl::PointCloud<PointType>::Ptr& surfPointsFlat_,
                          const pcl::PointCloud<PointType>::Ptr& surfPointsLessFlat_)
{
  laserCloudFullRes = boost::make_shared<pcl::PointCloud<PointType>>(*laserCloud_);
  cornerPointsSharp = boost::make_shared<pcl::PointCloud<PointType>>(*cornerPointsSharp_);
  cornerPointsLessSharp = boost::make_shared<pcl::PointCloud<PointType>>(*cornerPointsLessSharp_);
  surfPointsFlat = boost::make_shared<pcl::PointCloud<PointType>>(*surfPointsFlat_);
  surfPointsLessFlat = boost::make_shared<pcl::PointCloud<PointType>>(*surfPointsLessFlat_);
}

// undistort lidar point
// 去畸变操作
void LaserOdometry::TransformToStart(PointType const* const pi, PointType* const po)
{
  // interpolation ratio
  double s;
  if (DISTORTION)
    // 判断当前激光点的线的索引，比例值
    s = (pi->intensity - int(pi->intensity)) / SCAN_PERIOD;
  else
    s = 1.0;
  // s = 1;
  // 进行四元数的插值计算，q_point_last旋转的四元数，t_point_last平移
  Eigen::Quaterniond q_point_last = Eigen::Quaterniond::Identity().slerp(s, q_last_curr);
  Eigen::Vector3d t_point_last = s * t_last_curr;
  Eigen::Vector3d point(pi->x, pi->y, pi->z);
  // 根据之前推测的R，t来对当前数据进行去畸变操作，un_point
  Eigen::Vector3d un_point = q_point_last * point + t_point_last;

  po->x = un_point.x();
  po->y = un_point.y();
  po->z = un_point.z();
  po->intensity = pi->intensity;
}

// transform all lidar points to the start of the next frame
void LaserOdometry::TransformToEnd(PointType const* const pi, PointType* const po)
{
  // undistort point first
  pcl::PointXYZI un_point_tmp;
  TransformToStart(pi, &un_point_tmp);

  // 去畸变后的激光点un_point_tmp
  Eigen::Vector3d un_point(un_point_tmp.x, un_point_tmp.y, un_point_tmp.z);
  // ???
  Eigen::Vector3d point_end = q_last_curr.inverse() * (un_point - t_last_curr);

  po->x = point_end.x();
  po->y = point_end.y();
  po->z = point_end.z();

  // Remove distortion time info
  po->intensity = int(pi->intensity);
}

/**
 * @description: 上一帧激光数据和当前帧激光数据匹配得到一个先验的里程计位置估计
 *               使用视觉得到的估计机器人坐标位置关系给定一个激光里程计的初始位置值
 */
void LaserOdometry::solveLO()
{
  // this->reset();

  // ROS_INFO("LO: Finished reset");

  TicToc t_whole;

  // initializing
  // 初始化
  if (!systemInited)
  {
    systemInited = true;

    if (verbose_level > 1)
    {
      ROS_INFO("Initialization finished \n");
    }
  }
  else
  {
    int cornerPointsSharpNum = cornerPointsSharp->points.size();
    int surfPointsFlatNum = surfPointsFlat->points.size();

    TicToc t_opt;
    for (size_t opti_counter = 0; opti_counter < 2; ++opti_counter)
    {
      corner_correspondence = 0;
      plane_correspondence = 0;

      // ceres::LossFunction *loss_function = NULL;
      ceres::LossFunction* loss_function = new ceres::HuberLoss(0.1);
      ceres::LocalParameterization* q_parameterization = new ceres::EigenQuaternionParameterization();
      ceres::Problem::Options problem_options;

      ceres::Problem problem(problem_options);

      // 使用视觉得到的估计机器人坐标位置关系给定一个激光里程计的初始位置值
      if (!detach_VO_LO)
      {
        para_q[0] = vloam_tf->velo_last_VOT_velo_curr.getRotation().x();
        para_q[1] = vloam_tf->velo_last_VOT_velo_curr.getRotation().y();
        para_q[2] = vloam_tf->velo_last_VOT_velo_curr.getRotation().z();
        para_q[3] = vloam_tf->velo_last_VOT_velo_curr.getRotation().w();

        para_t[0] = vloam_tf->velo_last_VOT_velo_curr.getOrigin().x();
        para_t[1] = vloam_tf->velo_last_VOT_velo_curr.getOrigin().y();
        para_t[2] = vloam_tf->velo_last_VOT_velo_curr.getOrigin().z();

        // new (&q_last_curr) Eigen::Map<Eigen::Quaterniond>(para_q);
        // new (&t_last_curr) Eigen::Map<Eigen::Vector3d>(para_t);
      }

      if (verbose_level > 1)
      {
        ROS_INFO("\nq_last_curr.x = %f, velo_last_T_velo_curr.q.x = %f", para_q[0],
                 vloam_tf->velo_last_VOT_velo_curr.getRotation().x());
        ROS_INFO("q_last_curr.y = %f, velo_last_T_velo_curr.q.y = %f", para_q[1],
                 vloam_tf->velo_last_VOT_velo_curr.getRotation().y());
        ROS_INFO("q_last_curr.z = %f, velo_last_T_velo_curr.q.z = %f", para_q[2],
                 vloam_tf->velo_last_VOT_velo_curr.getRotation().z());
        ROS_INFO("q_last_curr.w = %f, velo_last_T_velo_curr.q.w = %f", para_q[3],
                 vloam_tf->velo_last_VOT_velo_curr.getRotation().w());

        ROS_INFO("t_last_curr.x = %f, velo_last_T_velo_curr.t.x = %f", para_t[0],
                 vloam_tf->velo_last_VOT_velo_curr.getOrigin().x());
        ROS_INFO("t_last_curr.y = %f, velo_last_T_velo_curr.t.y = %f", para_t[1],
                 vloam_tf->velo_last_VOT_velo_curr.getOrigin().y());
        ROS_INFO("t_last_curr.z = %f, velo_last_T_velo_curr.t.z = %f", para_t[2],
                 vloam_tf->velo_last_VOT_velo_curr.getOrigin().z());
      }

      problem.AddParameterBlock(para_q, 4, q_parameterization);
      problem.AddParameterBlock(para_t, 3);

      // 创建两个向量，分别存放近邻的索引值、近邻的中心距
      pcl::PointXYZI pointSel;
      std::vector<int> pointSearchInd;
      std::vector<float> pointSearchSqDis;

      TicToc t_data;
      // find correspondence for corner features
      // 处理边缘点
      for (int i = 0; i < cornerPointsSharpNum; ++i)
      {
        // 边缘点去畸变
        TransformToStart(&(cornerPointsSharp->points[i]), &pointSel);
        /**         nearestKSearch (const PointT &point, int k, 
                                    std::vector<int> &k_indices, 
                                    std::vector<float> &k_sqr_distances) const;
                    point：以这个传入的点为基准，基准点。
                    k：取几个离基本点最近的点
                    k_indices：这几个K点的下脚标
                    k_sqr_distances：这几个K点对于基准点的距离
        **/
        kdtreeCornerLast->nearestKSearch(pointSel, 1, pointSearchInd, pointSearchSqDis);

        // closestPointInd上一帧最近点，minPointInd2上一帧次临近点
        int closestPointInd = -1, minPointInd2 = -1;
        // 最近领域搜索的的距离小于阈值，认为正确
        if (pointSearchSqDis[0] < DISTANCE_SQ_THRESHOLD)
        {
          // 上一帧最近点的索引值
          closestPointInd = pointSearchInd[0];
          // 上一帧最近点在16线激光的序号
          int closestPointScanID = int(laserCloudCornerLast->points[closestPointInd].intensity);

          double minPointSqDis2 = DISTANCE_SQ_THRESHOLD;
          // search in the direction of increasing scan line
          // 对应论文中的距离值在不同的16线序号上面
          for (int j = closestPointInd + 1; j < (int)laserCloudCornerLast->points.size(); ++j)
          {
            // if in the same scan line, continue
            if (int(laserCloudCornerLast->points[j].intensity) <= closestPointScanID)
              continue;

            // if not in nearby scans, end the loop
            if (int(laserCloudCornerLast->points[j].intensity) > (closestPointScanID + NEARBY_SCAN))
              break;

            double pointSqDis =
                (laserCloudCornerLast->points[j].x - pointSel.x) * (laserCloudCornerLast->points[j].x - pointSel.x) +
                (laserCloudCornerLast->points[j].y - pointSel.y) * (laserCloudCornerLast->points[j].y - pointSel.y) +
                (laserCloudCornerLast->points[j].z - pointSel.z) * (laserCloudCornerLast->points[j].z - pointSel.z);

            // 小于阈值认为找到最近对应的点
            if (pointSqDis < minPointSqDis2)
            {
              // find nearer point
              minPointSqDis2 = pointSqDis;
              minPointInd2 = j;
            }
          }

          // search in the direction of decreasing scan line
          for (int j = closestPointInd - 1; j >= 0; --j)
          {
            // if in the same scan line, continue
            if (int(laserCloudCornerLast->points[j].intensity) >= closestPointScanID)
              continue;

            // if not in nearby scans, end the loop
            if (int(laserCloudCornerLast->points[j].intensity) < (closestPointScanID - NEARBY_SCAN))
              break;

            double pointSqDis =
                (laserCloudCornerLast->points[j].x - pointSel.x) * (laserCloudCornerLast->points[j].x - pointSel.x) +
                (laserCloudCornerLast->points[j].y - pointSel.y) * (laserCloudCornerLast->points[j].y - pointSel.y) +
                (laserCloudCornerLast->points[j].z - pointSel.z) * (laserCloudCornerLast->points[j].z - pointSel.z);

            if (pointSqDis < minPointSqDis2)
            {
              // find nearer point
              minPointSqDis2 = pointSqDis;
              minPointInd2 = j;
            }
          }
        }
        if (minPointInd2 >= 0)  // both closestPointInd and minPointInd2 is valid
        {
          // 当前点和上一帧最近点和次临近点
          Eigen::Vector3d curr_point(cornerPointsSharp->points[i].x, cornerPointsSharp->points[i].y,
                                     cornerPointsSharp->points[i].z);
          Eigen::Vector3d last_point_a(laserCloudCornerLast->points[closestPointInd].x,
                                       laserCloudCornerLast->points[closestPointInd].y,
                                       laserCloudCornerLast->points[closestPointInd].z);
          Eigen::Vector3d last_point_b(laserCloudCornerLast->points[minPointInd2].x,
                                       laserCloudCornerLast->points[minPointInd2].y,
                                       laserCloudCornerLast->points[minPointInd2].z);

          double s;
          if (DISTORTION)
            s = (cornerPointsSharp->points[i].intensity - int(cornerPointsSharp->points[i].intensity)) / SCAN_PERIOD;
          else
            s = 1.0;
          ceres::CostFunction* cost_function = LidarEdgeFactor::Create(curr_point, last_point_a, last_point_b, s);
          // if (corner_correspondence < 20) {
          //     std::cout << "Corner feature No." << corner_correspondence << ", curr_point: " << curr_point << ",
          //     last_point_a: " << last_point_a << ", last_point_b: " << last_point_b;
          // } // TODO: remove this later (test only code)
          // 构建残差，para_q, para_t
          problem.AddResidualBlock(cost_function, loss_function, para_q, para_t);
          corner_correspondence++;
        }
      }

      // find correspondence for plane features
      // 处理平面点
      for (int i = 0; i < surfPointsFlatNum; ++i)
      {
        // 把当前点云转换到起始位置
        TransformToStart(&(surfPointsFlat->points[i]), &pointSel);
        // kd-tree
        kdtreeSurfLast->nearestKSearch(pointSel, 1, pointSearchInd, pointSearchSqDis);

        // 当前帧激光点在上一帧的临近点，次临近点和次临近点，对应论文
        int closestPointInd = -1, minPointInd2 = -1, minPointInd3 = -1;
        // 最邻近点小于阈值
        if (pointSearchSqDis[0] < DISTANCE_SQ_THRESHOLD)
        {
          // 上一帧对应的激光点
          closestPointInd = pointSearchInd[0];

          // get closest point's scan ID
          int closestPointScanID = int(laserCloudSurfLast->points[closestPointInd].intensity);
          double minPointSqDis2 = DISTANCE_SQ_THRESHOLD, minPointSqDis3 = DISTANCE_SQ_THRESHOLD;

          // search in the direction of increasing scan line
          for (int j = closestPointInd + 1; j < (int)laserCloudSurfLast->points.size(); ++j)
          {
            // if not in nearby scans, end the loop
            if (int(laserCloudSurfLast->points[j].intensity) > (closestPointScanID + NEARBY_SCAN))
              break;

            double pointSqDis =
                (laserCloudSurfLast->points[j].x - pointSel.x) * (laserCloudSurfLast->points[j].x - pointSel.x) +
                (laserCloudSurfLast->points[j].y - pointSel.y) * (laserCloudSurfLast->points[j].y - pointSel.y) +
                (laserCloudSurfLast->points[j].z - pointSel.z) * (laserCloudSurfLast->points[j].z - pointSel.z);

            // if in the same or lower scan line
            if (int(laserCloudSurfLast->points[j].intensity) <= closestPointScanID && pointSqDis < minPointSqDis2)
            {
              minPointSqDis2 = pointSqDis;
              minPointInd2 = j;
            }
            // if in the higher scan line
            else if (int(laserCloudSurfLast->points[j].intensity) > closestPointScanID && pointSqDis < minPointSqDis3)
            {
              minPointSqDis3 = pointSqDis;
              minPointInd3 = j;
            }
          }

          // search in the direction of decreasing scan line
          for (int j = closestPointInd - 1; j >= 0; --j)
          {
            // if not in nearby scans, end the loop
            if (int(laserCloudSurfLast->points[j].intensity) < (closestPointScanID - NEARBY_SCAN))
              break;

            double pointSqDis =
                (laserCloudSurfLast->points[j].x - pointSel.x) * (laserCloudSurfLast->points[j].x - pointSel.x) +
                (laserCloudSurfLast->points[j].y - pointSel.y) * (laserCloudSurfLast->points[j].y - pointSel.y) +
                (laserCloudSurfLast->points[j].z - pointSel.z) * (laserCloudSurfLast->points[j].z - pointSel.z);

            // if in the same or higher scan line
            if (int(laserCloudSurfLast->points[j].intensity) >= closestPointScanID && pointSqDis < minPointSqDis2)
            {
              minPointSqDis2 = pointSqDis;
              minPointInd2 = j;
            }
            else if (int(laserCloudSurfLast->points[j].intensity) < closestPointScanID && pointSqDis < minPointSqDis3)
            {
              // find nearer point
              minPointSqDis3 = pointSqDis;
              minPointInd3 = j;
            }
          }

          // 平面点对应的上一帧三个激光点
          if (minPointInd2 >= 0 && minPointInd3 >= 0)
          {
            Eigen::Vector3d curr_point(surfPointsFlat->points[i].x, surfPointsFlat->points[i].y,
                                       surfPointsFlat->points[i].z);
            Eigen::Vector3d last_point_a(laserCloudSurfLast->points[closestPointInd].x,
                                         laserCloudSurfLast->points[closestPointInd].y,
                                         laserCloudSurfLast->points[closestPointInd].z);
            Eigen::Vector3d last_point_b(laserCloudSurfLast->points[minPointInd2].x,
                                         laserCloudSurfLast->points[minPointInd2].y,
                                         laserCloudSurfLast->points[minPointInd2].z);
            Eigen::Vector3d last_point_c(laserCloudSurfLast->points[minPointInd3].x,
                                         laserCloudSurfLast->points[minPointInd3].y,
                                         laserCloudSurfLast->points[minPointInd3].z);

            double s;
            if (DISTORTION)
              s = (surfPointsFlat->points[i].intensity - int(surfPointsFlat->points[i].intensity)) / SCAN_PERIOD;
            else
              s = 1.0;
            // ceres的代价函数 
            ceres::CostFunction* cost_function =
                LidarPlaneFactor::Create(curr_point, last_point_a, last_point_b, last_point_c, s);
            problem.AddResidualBlock(cost_function, loss_function, para_q, para_t);
            plane_correspondence++;
          }
        }
      }

      if (verbose_level > 1)
      {
        ROS_INFO("coner_correspondance %d, plane_correspondence %d \n", corner_correspondence, plane_correspondence);
        ROS_INFO("data association time %f ms \n", t_data.toc());
      }

      if ((corner_correspondence + plane_correspondence) < 10)
      {
        ROS_INFO("less correspondence! *************************************************\n");
      }

      TicToc t_solver;
      // 配置并运行求解器
      ceres::Solver::Options options;
      // 配置增量方程的解法
      options.linear_solver_type = ceres::DENSE_QR;
      options.max_num_iterations = 4;
      // 不输出消息
      options.minimizer_progress_to_stdout = false;
      // 优化信息
      ceres::Solver::Summary summary;
      // 求解
      ceres::Solve(options, &problem, &summary);
      // ROS_INFO("full report: \n%s", summary.FullReport().c_str());
      // ROS_INFO("raw: q_last_curr: w = %f x = %f, y = %f, z = %f", para_q[3], para_q[0], para_q[1], para_q[2]);
      if (verbose_level > 1)
      {
        ROS_INFO("solver time %f ms \n", t_solver.toc());
      }
    }

    if (verbose_level > 1)
    {
      ROS_INFO("optimization twice time %f \n", t_opt.toc());
    }
    
    // para_q，para_t对应了q_w_curr，t_w_curr
    t_w_curr = t_w_curr + q_w_curr * t_last_curr;
    q_w_curr = q_w_curr * q_last_curr;
  }

  // ROS_INFO("raw: q_last_curr: w = %f x = %f, y = %f, z = %f", para_q[3], para_q[0], para_q[1], para_q[2]);
  // ROS_INFO("q_last_curr: w = %f x = %f, y = %f, z = %f", q_last_curr.w(), q_last_curr.x(), q_last_curr.y(),
  // q_last_curr.z()); ROS_INFO("q_w_curr: w = %f x = %f, y = %f, z = %f", q_w_curr.w(), q_w_curr.x(), q_w_curr.y(),
  // q_w_curr.z());

  // ROS_INFO("t_last_curr: x = %f, y = %f, z = %f", t_last_curr.x(), t_last_curr.y(), t_last_curr.z());
  // ROS_INFO("t_w_curr: x = %f, y = %f, z = %f", t_w_curr.x(), t_w_curr.y(), t_w_curr.z());

  // transform corner features and plane features to the scan end point
  if (0)
  {
    int cornerPointsLessSharpNum = cornerPointsLessSharp->points.size();
    for (int i = 0; i < cornerPointsLessSharpNum; i++)
    {
      TransformToEnd(&cornerPointsLessSharp->points[i], &cornerPointsLessSharp->points[i]);
    }

    int surfPointsLessFlatNum = surfPointsLessFlat->points.size();
    for (int i = 0; i < surfPointsLessFlatNum; i++)
    {
      TransformToEnd(&surfPointsLessFlat->points[i], &surfPointsLessFlat->points[i]);
    }

    int laserCloudFullResNum = laserCloudFullRes->points.size();
    for (int i = 0; i < laserCloudFullResNum; i++)
    {
      TransformToEnd(&laserCloudFullRes->points[i], &laserCloudFullRes->points[i]);
    }
  }

  pcl::PointCloud<PointType>::Ptr laserCloudTemp = cornerPointsLessSharp;
  cornerPointsLessSharp = laserCloudCornerLast;
  laserCloudCornerLast = laserCloudTemp;

  laserCloudTemp = surfPointsLessFlat;
  surfPointsLessFlat = laserCloudSurfLast;
  laserCloudSurfLast = laserCloudTemp;

  laserCloudCornerLastNum = laserCloudCornerLast->points.size();
  laserCloudSurfLastNum = laserCloudSurfLast->points.size();

  // std::cout << "the size of corner last is " << laserCloudCornerLastNum << ", and the size of surf last is " <<
  // laserCloudSurfLastNum << '\n';

  // 下一次的kd-tree准备
  kdtreeCornerLast->setInputCloud(laserCloudCornerLast);
  kdtreeSurfLast->setInputCloud(laserCloudSurfLast);

  if (verbose_level > 1)
  {
    ROS_INFO("whole laserOdometry time %f ms \n \n", t_whole.toc());
    if (t_whole.toc() > 100)
      ROS_WARN("odometry process over 100ms");
  }

  frameCount++;
}

// 发布激光里程计
void LaserOdometry::publish()
{
  TicToc t_pub;

  // publish odometry
  nav_msgs::Odometry laserOdometry;
  laserOdometry.header.frame_id = "map";
  laserOdometry.child_frame_id = "laser_odom";
  laserOdometry.header.stamp = ros::Time::now();
  laserOdometry.pose.pose.orientation.x = q_w_curr.x();
  laserOdometry.pose.pose.orientation.y = q_w_curr.y();
  laserOdometry.pose.pose.orientation.z = q_w_curr.z();
  laserOdometry.pose.pose.orientation.w = q_w_curr.w();
  laserOdometry.pose.pose.position.x = t_w_curr.x();
  laserOdometry.pose.pose.position.y = t_w_curr.y();
  laserOdometry.pose.pose.position.z = t_w_curr.z();
  pubLaserOdometry.publish(laserOdometry);

  ROS_INFO("lidar_odometry: q = %.4f, %.4f, %.4f, %.4f; t = %.4f, %.4f, %.4f", q_last_curr.x(), q_last_curr.y(),
           q_last_curr.z(), q_last_curr.w(), t_last_curr.x(), t_last_curr.y(), t_last_curr.z());

  // vloam_tf->cam0_init_eigen_LOT_cam0_last.translation() = t_w_curr;
  // vloam_tf->cam0_init_eigen_LOT_cam0_last.linear() = q_w_curr.normalized().toRotationMatrix();

  // frame2frame estimation recording
  vloam_tf->base_prev_LOT_base_curr.setOrigin(tf2::Vector3(t_last_curr.x(), t_last_curr.y(), t_last_curr.z()));
  vloam_tf->base_prev_LOT_base_curr.setRotation(
      tf2::Quaternion(q_last_curr.x(), q_last_curr.y(), q_last_curr.z(), q_last_curr.w()));
  vloam_tf->cam0_curr_LOT_cam0_prev =
      vloam_tf->base_T_cam0.inverse() * vloam_tf->base_prev_LOT_base_curr.inverse() * vloam_tf->base_T_cam0;

  // world2frame estimation
  vloam_tf->world_LOT_base_last.setOrigin(tf2::Vector3(t_w_curr.x(), t_w_curr.y(), t_w_curr.z()));
  vloam_tf->world_LOT_base_last.setRotation(tf2::Quaternion(q_w_curr.x(), q_w_curr.y(), q_w_curr.z(), q_w_curr.w()));

  geometry_msgs::PoseStamped laserPose;
  laserPose.header = laserOdometry.header;
  laserPose.pose = laserOdometry.pose.pose;
  laserPath.header.stamp = laserOdometry.header.stamp;
  laserPath.poses.push_back(laserPose);
  laserPath.header.frame_id = "map";
  pubLaserPath.publish(laserPath);

  // if (frameCount % mapping_skip_frame == 0) // no need to publish
  // {
  //     frameCount = 0;

  //     sensor_msgs::PointCloud2 laserCloudCornerLast2;
  //     pcl::toROSMsg(*laserCloudCornerLast, laserCloudCornerLast2);
  //     laserCloudCornerLast2.header.stamp = ros::Time::now();
  //     laserCloudCornerLast2.header.frame_id = "velo";
  //     pubLaserCloudCornerLast.publish(laserCloudCornerLast2);

  //     sensor_msgs::PointCloud2 laserCloudSurfLast2;
  //     pcl::toROSMsg(*laserCloudSurfLast, laserCloudSurfLast2);
  //     laserCloudSurfLast2.header.stamp = ros::Time::now();
  //     laserCloudSurfLast2.header.frame_id = "velo";
  //     pubLaserCloudSurfLast.publish(laserCloudSurfLast2);

  //     sensor_msgs::PointCloud2 laserCloudFullRes3;
  //     pcl::toROSMsg(*laserCloudFullRes, laserCloudFullRes3);
  //     laserCloudFullRes3.header.stamp = ros::Time::now();
  //     laserCloudFullRes3.header.frame_id = "velo";
  //     pubLaserCloudFullRes.publish(laserCloudFullRes3);
  // }

  if (verbose_level > 1)
  {
    ROS_INFO("publication time %f ms \n", t_pub.toc());
  }
}

// 输出里程计得到的当前帧和上一帧间位置转换关系
void LaserOdometry::output(Eigen::Quaterniond& q_w_curr_, Eigen::Vector3d& t_w_curr_,
                           pcl::PointCloud<PointType>::Ptr& laserCloudCornerLast_,
                           pcl::PointCloud<PointType>::Ptr& laserCloudSurfLast_,
                           pcl::PointCloud<PointType>::Ptr& laserCloudFullRes_, bool& skip_frame)
{
  q_w_curr_ = q_w_curr;
  t_w_curr_ = t_w_curr;

  if (frameCount % mapping_skip_frame == 0)
  {
    laserCloudCornerLast_ = laserCloudCornerLast;
    laserCloudSurfLast_ = laserCloudSurfLast;
    laserCloudFullRes_ = laserCloudFullRes;
    skip_frame = false;
  }  // TODO: check the meaning of skip frame.
  else
  {
    skip_frame = true;
  }
}
}  // namespace vloam