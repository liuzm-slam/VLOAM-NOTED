#include <sstream>
#include <string>
// #include <fstream>
#include <actionlib/server/simple_action_server.h>
#include <ceres/ceres.h>
#include <ceres/loss_function.h>
#include <ceres/rotation.h>
#include <cv_bridge/cv_bridge.h>
#include <geometry_msgs/TransformStamped.h>
#include <lidar_odometry_mapping/lidar_odometry_mapping.h>
#include <lidar_odometry_mapping/tic_toc.h>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <ros/package.h>
#include <ros/ros.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/String.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2_eigen/tf2_eigen.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <visual_odometry/visual_odometry.h>
#include <vloam_main/vloam_mainAction.h>
#include <vloam_main/vloam_mainFeedback.h>
#include <vloam_main/vloam_mainResult.h>
#include <vloam_tf/vloam_tf.h>

#include <cstdlib>
#include <eigen3/Eigen/Dense>
#include <iostream>
#include <opencv4/opencv2/opencv.hpp>

// parameters from launch file
double rosbag_rate;
bool visualize_depth, publish_point_cloud, detach_VO_LO, save_traj;
int start_frame, end_frame;  // inclusive interval

typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::CameraInfo,
                                                        sensor_msgs::PointCloud2>
    MySyncPolicy;
std::shared_ptr<message_filters::Subscriber<sensor_msgs::Image>> sub_image00_ptr;
std::shared_ptr<message_filters::Subscriber<sensor_msgs::CameraInfo>> sub_camera00_ptr;
std::shared_ptr<message_filters::Subscriber<sensor_msgs::PointCloud2>> sub_velodyne_ptr;

// actionlib服务请求的功能外，还可以实时获取服务器执行任务的进度状态，以及强制中断服务的功能
// 进行定义sctionlib
typedef actionlib::SimpleActionServer<vloam_main::vloam_mainAction> Server;
vloam_main::vloam_mainFeedback feedback;
vloam_main::vloam_mainResult result;

int count, i, j;  // TODO: check if count will overflow

std::shared_ptr<vloam::VisualOdometry> VO;
cv_bridge::CvImagePtr cv_ptr;
std::shared_ptr<vloam::LidarOdometryMapping> LOAM;
pcl::PointCloud<pcl::PointXYZ> point_cloud_pcl;
std::shared_ptr<vloam::VloamTF> vloam_tf;

std::string seq, cmd;
std::ostringstream ss;
int sys_ret;

// output for evaluation
std::string VOFileName;
std::string LOFileName;
std::string MOFileName;
FILE* LOFilePtr = NULL;
FILE* VOFilePtr = NULL;
FILE* MOFilePtr = NULL;

void init(const vloam_main::vloam_mainGoalConstPtr& goal)
{
  // section 1, initialize file pointers
  if (save_traj)
  {
    ss.clear();
    ss.str("");
    ss << std::setw(4) << std::setfill('0') << goal->seq;
    seq = std::string(ss.str());

    VOFileName = ros::package::getPath("vloam_main") + "/results/" + goal->date + +"_drive_" + seq;
    LOFileName = ros::package::getPath("vloam_main") + "/results/" + goal->date + +"_drive_" + seq;
    MOFileName = ros::package::getPath("vloam_main") + "/results/" + goal->date + +"_drive_" + seq;

    cmd = "mkdir -p " + VOFileName;
    sys_ret = system(cmd.c_str());
    cmd = "mkdir -p " + LOFileName;
    sys_ret = system(cmd.c_str());
    cmd = "mkdir -p " + MOFileName;
    sys_ret = system(cmd.c_str());

    VOFileName += "/VO" + std::to_string(detach_VO_LO) + ".txt";
    LOFileName += "/LO" + std::to_string(detach_VO_LO) + ".txt";
    MOFileName += "/MO" + std::to_string(detach_VO_LO) + ".txt";

    VOFilePtr = fopen(VOFileName.c_str(), "w");
    LOFilePtr = fopen(LOFileName.c_str(), "w");
    MOFilePtr = fopen(MOFileName.c_str(), "w");

    if (VOFilePtr == NULL or LOFilePtr == NULL or MOFilePtr == NULL)
    {
      ROS_INFO("FilePtr == NULL");
      ROS_BREAK();
    }
    ROS_INFO("\n LO file path; %s\n", LOFileName.c_str());
  }

  // section 2, prepare for a new set of estimation
  count = 0;

  vloam_tf = std::make_shared<vloam::VloamTF>();
  VO = std::make_shared<vloam::VisualOdometry>();
  LOAM = std::make_shared<vloam::LidarOdometryMapping>();

  vloam_tf->init();
  VO->init(vloam_tf);
  LOAM->init(vloam_tf);
}

/**
 * @description: 调用ros时间同步函数对三种数据进行时间同步操作
 * @param image_msg 输入图像
 * @param camera_info_msg 图像的标定数据
 * @param point_cloud_msg 输入的激光点云
 * @return 
 */
void callback(const sensor_msgs::Image::ConstPtr& image_msg, const sensor_msgs::CameraInfo::ConstPtr& camera_info_msg,
              const sensor_msgs::PointCloud2::ConstPtr& point_cloud_msg)
{
  static tf2_ros::StaticTransformBroadcaster static_broadcaster;
  static tf2_ros::TransformBroadcaster dynamic_broadcaster;

  // 初始值为0
  i = count % 2;

  VO->reset();
  LOAM->reset();

  // Section 1: Process Image // takes ~34ms 处理图像
  cv_ptr = cv_bridge::toCvCopy(image_msg, sensor_msgs::image_encodings::MONO8);
  // 对图像进行处理，使用光流法或者描述子对相邻两帧图像匹配
  VO->processImage(cv_ptr->image);

  // Section 2: Process Static Transformations and Camera Intrinsics 处理静态tf,camera_info_msg
  if (count == 0)
  {
    vloam_tf->processStaticTransform();
    VO->setUpPointCloud(camera_info_msg);
  }

  // Section 3: Process Point Cloud // takes ~2.6ms 处理点云数据
  // 转换点云数据到pcl格式
  pcl::fromROSMsg(*point_cloud_msg, point_cloud_pcl);  // optimization can be applied if pcl library is not necessarily
  // ROS_INFO("point cloud width=%d, height=%d", point_cloud_pcl.width, point_cloud_pcl.height); // typical output
  // "point cloud width=122270, height=1053676" // TODO: check why height is so large
  // 处理激光点云，把点云数据投影到相机坐标系下并且可视化深度图
  VO->processPointCloud(point_cloud_msg, point_cloud_pcl, visualize_depth, publish_point_cloud);

  // Section 4: Solve and Publish VO
  if (count > 0)
  {
    // ceres求解，得到相机特征的重投影误差，得到相机计算的（R，t），处理过程相当于一个RGBD相机，
    // 激光雷达计算深度，图像进行匹配
    VO->solveNlsAll();  // result is cam0_curr_T_cam0_last, f2f odometry
                        // VO->solveNls2dOnly();
                        // VO->solveRANSAC();
  }
  // 视觉计算的机器人位姿转换关系
  vloam_tf->VO2VeloAndBase(VO->cam0_curr_T_cam0_last);                             // transform f2f VO to world VO
  // 使用tf2对视觉位姿转换关系广播出来
  vloam_tf->dynamic_broadcaster.sendTransform(vloam_tf->world_stamped_VOtf_base);  // publish for visualization // can
                                                                                   // be commented out
  // 发布视觉里程计
  VO->publish();                                                                   // publish nav_msgs::odometry

  // Section 5: Solve and Publish LO MO
  // LO中使用的VO计算得到的机器人位姿转换关系（通过tf2），作为一个先验值
  // 对输入的原始点云进行处理，得到边缘点的点云和平面点的点云
  LOAM->scanRegistrationIO(point_cloud_pcl);
  // 激光里程计计算连续两帧激光数据的位姿
  LOAM->laserOdometryIO();
  // 当前帧与submap匹配获得机器人在连续时刻的位置变换关系
  LOAM->laserMappingIO();

  // Section 6, save odoms
  if (save_traj and count >= start_frame and count <= end_frame)
  {
    vloam_tf->VO2Cam0StartFrame(VOFilePtr, count - start_frame);
    vloam_tf->LO2Cam0StartFrame(LOFilePtr, count - start_frame);
    vloam_tf->MO2Cam0StartFrame(MOFilePtr, count - start_frame);
  }

  ++count;
  ROS_INFO("total count = %d", count);
}

void execute(const vloam_main::vloam_mainGoalConstPtr& goal, Server* as)
{
  result.loading_finished = false;

  // section1, generate new message filter and tf_buffer
  // 调用ROS时间同步函数对订阅数据进行时间同步
  message_filters::Synchronizer<MySyncPolicy> sync(MySyncPolicy(20), *sub_image00_ptr, *sub_camera00_ptr,
                                                   *sub_velodyne_ptr);
  sync.setInterMessageLowerBound(ros::Duration(0.09));
  sync.registerCallback(boost::bind(&callback, _1, _2, _3));

  // section 2, initialize all pointers
  init(goal);

  // section 3, play the bag
  start_frame = goal->start_frame;
  end_frame = goal->end_frame;

  ss.clear();
  ss.str("");
  ss << std::setw(4) << std::setfill('0') << goal->seq;
  seq = std::string(ss.str());
  cmd = "rosbag play " + ros::package::getPath("vloam_main") + "/bags/kitti_" + goal->date + "_drive_" + seq +
        "_synced.bag -d 2 -r " + std::to_string(rosbag_rate);
  // TODO: add one more entry of goal for different dataset type:
  // In kitti2bag, kitti_types = ["raw_synced", "odom_color", "odom_gray"]
  // https://github.com/tomas789/kitti2bag/blob/bf0d46c49a77f5d5500621934ccd617d18cf776b/kitti2bag/kitti2bag.py#L264
  ROS_INFO("The command is %s", cmd.c_str());
  sys_ret = system(cmd.c_str());

  // section 4, save files
  if (save_traj)
  {
    fclose(LOFilePtr);
    fclose(VOFilePtr);
    fclose(MOFilePtr);
    ROS_INFO("\n LO, VO, MO saved\n\n");
  }

  result.loading_finished = true;
  as->setSucceeded(result);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "vloam_main_node");

  ros::NodeHandle nh_private = ros::NodeHandle("~");
  nh_private.getParam("rosbag_rate", rosbag_rate);
  nh_private.getParam("visualize_depth", visualize_depth);
  nh_private.getParam("publish_point_cloud", publish_point_cloud);
  nh_private.getParam("save_traj", save_traj);
  if (!ros::param::get("detach_VO_LO", detach_VO_LO))
    ROS_BREAK();

  ros::NodeHandle nh;

  // 订阅ROS图像信息
  sub_image00_ptr =
      std::make_shared<message_filters::Subscriber<sensor_msgs::Image>>(nh, "/kitti/camera_gray_left/image_raw", 10);
  // 订阅相机的参数
  sub_camera00_ptr = std::make_shared<message_filters::Subscriber<sensor_msgs::CameraInfo>>(
      nh, "/kitti/camera_gray_left/camera_info", 10);
  // 订阅激光雷达
  sub_velodyne_ptr = std::make_shared<message_filters::Subscriber<sensor_msgs::PointCloud2>>(
      nh, "/kitti/velo/pointcloud", 10);  // TODO: change the buffer size to potentially reduce the data lost

  // 初始化类对象，并定义action名称为“load_small_dataset_action_server”，绑定execute为任务回调函数，并传入参数server，
  // 方便在回调函数中使用对象接口，false为不在初始化时，自动启动
  Server server(nh, "load_small_dataset_action_server", boost::bind(&execute, _1, &server), false);
  // 开始server
  server.start();

  ros::Rate loop_rate(100);
  while (ros::ok())
  {
    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}