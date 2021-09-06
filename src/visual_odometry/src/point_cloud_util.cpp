#include <visual_odometry/point_cloud_util.h>

namespace vloam
{
/**
 * @description: 加载标定文件
 * @param calib_cam_to_cam_file_path 相机间的转换关系路径名称
 * @param calib_velo_to_cam_file_path 激光雷达到相机的转换关系路径名称
 * @return 
 */
/* 
calib_cam_to_cam.txt文件的内容格式
- S_xx：1x2 矫正前的图像xx的大小
- K_xx：3x3 矫正前摄像机xx的校准矩阵
- D_xx：1x5 矫正前摄像头xx的失真向量
- R_xx：3x3 （外部）的旋转矩阵(从相机0到相机xx)
- T_xx：3x1 （外部）的平移矢量(从相机0到相机xx)
- S_rect_xx：1x2 矫正后的图像xx的大小
- R_rect_xx：3x3 纠正旋转矩阵(使图像平面共面)
- P_rect_xx：3x4 矫正后的投影矩阵

xx：00,01,02,03 代表相机的编号，0表示左边灰度相机，1右边灰度相机，2左边彩色相机，3右边彩色相机。
 */
void PointCloudUtil::loadTransformations(const std::string& calib_cam_to_cam_file_path,
                                         const std::string& calib_velo_to_cam_file_path)
{
  std::string line;
  std::string delim = " ";
  float value;
  std::string curr;

  std::fstream input1(calib_velo_to_cam_file_path.c_str(), std::ios::in);
  // 读取失败
  if (!input1.good())
  {
    std::cerr << "Could not read file: " << calib_velo_to_cam_file_path << std::endl;
    exit(EXIT_FAILURE);
  }

  // 把激光雷达到相机间的标定结果输入到line中
  while (getline(input1, line))
  {  // read data from file object and put it into string.
    // std::cout << line << "\n"; //print the data of the string
    int start = 0;
    int end = line.find(delim);
    // 把line开始到结束位置的字符串赋值到curr中
    curr = line.substr(start, end - start);
    // std::cout << curr << ", ";
    // 旋转
    if (curr == "R:")
    {
      int index = 0;
      // std::string::npos字符串结束位置
      while (end != std::string::npos)
      {
        start = end + delim.length();
        end = line.find(delim, start);
        curr = line.substr(start, end - start);
        // 对数据进行类型转换这里把字符串转换成float型
        // 使用boost:: lexical_cast。如果转换发生了意外，
        // lexical_cast会抛出一个bad_lexical_cast异常，因此程序中需要对其进行捕捉。
        value = boost::lexical_cast<float>(line.substr(start, end - start));
        // std::cout << value << ", ";
        cam_T_velo(index / 3, index % 3) = value;
        ++index;
      }
    }
    // 平移
    if (curr == "T:")
    {
      int index = 0;
      while (end != std::string::npos)
      {
        start = end + delim.length();
        end = line.find(delim, start);
        curr = line.substr(start, end - start);
        value = boost::lexical_cast<float>(line.substr(start, end - start));
        // std::cout << value << ", ";
        cam_T_velo(index, 3) = value;
        ++index;
      }
    }
  }
  cam_T_velo(3, 3) = 1;

  if (print_result)
    std::cout << "cam_T_velo = \n" << cam_T_velo << "\n" << std::endl;

  std::fstream input2(calib_cam_to_cam_file_path.c_str(), std::ios::in);
  if (!input2.good())
  {
    std::cerr << "Could not read file: " << calib_cam_to_cam_file_path << std::endl;
    exit(EXIT_FAILURE);
  }

  while (getline(input2, line))
  {  // read data from file object and put it into string.
    // std::cout << line << "\n"; //print the data of the string
    int start = 0;
    int end = line.find(delim);
    curr = line.substr(start, end - start);
    // std::cout << curr << ", ";
    // TODO: if not just choosing camera 0, check R_0x T_0x for an extra transformation,
    // TODO: and R_00 is an identity matrix, T_00 is (almost) zero matrix
    // - R_rect_00：3x3 纠正旋转矩阵(使图像平面共面)
    if (curr == "R_rect_00:")
    {
      int index = 0;
      while (end != std::string::npos)
      {
        start = end + delim.length();
        end = line.find(delim, start);
        curr = line.substr(start, end - start);
        value = boost::lexical_cast<float>(line.substr(start, end - start));
        // std::cout << value << ", ";
        rect0_T_cam(index / 3, index % 3) = value;
        ++index;
      }
      rect0_T_cam(3, 3) = 1;
    }
    // P_rect_00 3x4 矫正后的投影矩阵，投影矩阵可以得到相机内参
    if (curr == "P_rect_00:")
    {
      int index = 0;
      while (end != std::string::npos)
      {
        start = end + delim.length();
        end = line.find(delim, start);
        curr = line.substr(start, end - start);
        value = boost::lexical_cast<float>(line.substr(start, end - start));
        // std::cout << value << ", ";
        P_rect0(index / 4, index % 4) = value;
        ++index;
      }
    }
  }

  if (print_result)
  {
    std::cout << "rect0_T_cam = \n" << rect0_T_cam << "\n" << std::endl;
    std::cout << "P_rect0 = \n" << P_rect0 << "\n" << std::endl;
  }

  input1.close();
  input2.close();
}

void PointCloudUtil::loadPointCloud(const std::string& bin_file_path)
{
  int32_t num = 1000000;  // about 10 times larger than the real point cloud size
  float* data = (float*)malloc(num * sizeof(float));

  // pointers
  float* px = data + 0;
  float* py = data + 1;
  float* pz = data + 2;

  // load point cloud
  FILE* stream;
  stream = fopen(bin_file_path.c_str(), "rb");
  num = fread(data, sizeof(float), num, stream) / 4;
  point_cloud_3d_tilde = Eigen::MatrixXf::Ones(num, 4);  // TODO: check if this affect performance
  // std::cout << num << std::endl;
  for (int32_t i = 0; i < num; i++)
  {
    point_cloud_3d_tilde(i, 0) = *px;
    point_cloud_3d_tilde(i, 1) = *py;
    point_cloud_3d_tilde(i, 2) = *pz;
    px += 4;
    py += 4;
    pz += 4;
  }

  fclose(stream);
  free(data);
}

/**
 * @description: 对激光点云进行投影操作，把激光重激光雷达坐标系转换到相机坐标系然后筛选
 * @param 
 * @return 
 */
void PointCloudUtil::projectPointCloud()
{
  // Projection
  // Eigen::MatrixXf point_cloud_3d_tilde原始采集的激光点n*4
  // Eigen::Matrix4f cam_T_velo 相机到激光的转换矩阵
  // Eigen::Matrix4f rect0_T_cam 3x3 纠正旋转矩阵(使图像平面共面)因为（3，3）= 1 所以为4*4矩阵
  // Eigen::MatrixXf P_rect0 3x4 矫正后的投影矩阵 4*4矩阵
  // Eigen::MatrixXf point_cloud_3d转换到相机坐标系下的点
  Eigen::MatrixXf point_cloud_3d =
      point_cloud_3d_tilde * cam_T_velo.transpose() * rect0_T_cam.transpose() *
      P_rect0.transpose();  // shape = point_cloud_3d_tilde.rows(), 3; in rect cam0 coordinate; last column is the depth

  // Screen out back points
  // 防止出现激光点为负的情况，对激光点进行筛选
  Eigen::VectorXi select_front =
      (point_cloud_3d.col(2).array() > 0.1).cast<int>();  // depth should not just be positive, but greater than a
                                                          // threshold
  Eigen::MatrixXf point_cloud_3d_front(select_front.sum(), 3);
  int i, j = 0;
  for (i = 0; i < point_cloud_3d.rows(); ++i)
  {
    if (select_front(i))
    {
      point_cloud_3d_front.row(j) = point_cloud_3d.row(i);
      ++j;
    }
  }

  // From homogeneous to normal coordiante, last column is still depth
  // 从齐次坐标到正则坐标，最后一列还是深度
  // eigen array单个元素操作
  // point_cloud_2d.leftCols(2).array().colwise()对第二列元素进行操作
  // point_cloud_2d.col(2).array()左边第二列的元素操作
  point_cloud_2d = point_cloud_3d_front;
  point_cloud_2d.leftCols(2) =
      point_cloud_2d.leftCols(2).array().colwise() * Eigen::inverse(point_cloud_2d.col(2).array());
}

void PointCloudUtil::printPointCloud2dStats() const
{
  std::cout << "min x = " << point_cloud_2d.col(0).minCoeff() << std::endl;
  std::cout << "max x = " << point_cloud_2d.col(0).maxCoeff() << std::endl;
  std::cout << "mean x = " << point_cloud_2d.col(0).mean() << std::endl;
  std::cout << "std x = "
            << std::sqrt((point_cloud_2d.col(0).array() - point_cloud_2d.col(0).mean()).square().sum() /
                         (point_cloud_2d.rows() - 1))
            << std::endl;

  std::cout << "\nmin y = " << point_cloud_2d.col(1).minCoeff() << std::endl;
  std::cout << "max y = " << point_cloud_2d.col(1).maxCoeff() << std::endl;
  std::cout << "mean y = " << point_cloud_2d.col(1).mean() << std::endl;
  std::cout << "std y = "
            << std::sqrt((point_cloud_2d.col(1).array() - point_cloud_2d.col(1).mean()).square().sum() /
                         (point_cloud_2d.rows() - 1))
            << std::endl;

  std::cout << "\nmin depth = " << point_cloud_2d.col(2).minCoeff() << std::endl;
  std::cout << "max depth = " << point_cloud_2d.col(2).maxCoeff() << std::endl;
  std::cout << "mean depth = " << point_cloud_2d.col(2).mean() << std::endl;
  std::cout << "std depth = "
            << std::sqrt((point_cloud_2d.col(2).array() - point_cloud_2d.col(2).mean()).square().sum() /
                         (point_cloud_2d.rows() - 1))
            << "\n"
            << std::endl;
  // min depth = 0.100293, max depth = 78.3935
}

/**
 * @description: 在相机视野范围内对激光数据处理
 *               对数据进行下采样，并且把激光点投影到相机视野范围内的图像中
 * @param {*}
 * @return {*}
 */
void PointCloudUtil::downsamplePointCloud()
{
  // const int IMG_HEIGHT = 375;
  // const int IMG_WIDTH = 1242;
  // visual_odometry中downsample_grid_size = 5
  // 激光投影到相机平面时的投影大小（new_width，new_height）
  const int new_width = std::ceil(static_cast<float>(IMG_WIDTH) / static_cast<float>(downsample_grid_size));
  const int new_height = std::ceil(static_cast<float>(IMG_HEIGHT) / static_cast<float>(downsample_grid_size));
  
  // bucket_x 点云x轴坐标值
  bucket_x = Eigen::MatrixXf::Zero(new_width, new_height);  // TODO: check if these lines are costly
  // bucket_y点云y轴坐标值
  bucket_y = Eigen::MatrixXf::Zero(new_width, new_height);
  // bucket_depth点云深度
  bucket_depth = Eigen::MatrixXf::Zero(new_width, new_height);
  // 用来判断当前图像平面是否被占用
  bucket_count = Eigen::MatrixXi::Zero(new_width, new_height);

  int index_x, index_y, i, j, global_count = 0;
  for (i = 0; i < point_cloud_2d.rows(); ++i)
  {
    // 在bucket上的坐标
    index_x = static_cast<int>(point_cloud_2d(i, 0) / downsample_grid_size);
    index_y = static_cast<int>(point_cloud_2d(i, 1) / downsample_grid_size);
    if (index_x >= 0 and index_x < new_width and index_y >= 0 and index_y < new_height)
    {
      // bucket_count没用被占用
      if (bucket_count(index_x, index_y) == 0)
      {
        bucket_x(index_x, index_y) = point_cloud_2d(i, 0); // x坐标在图像平面
        bucket_y(index_x, index_y) = point_cloud_2d(i, 1); // y坐标在图像平面
        bucket_depth(index_x, index_y) = point_cloud_2d(i, 2); // 当前激光点深度
        ++global_count; // 激光点数
      }
      // bucket_count位置被占用
      else
      {  // incremental averaging -> TODO: check better averaging method
        bucket_x(index_x, index_y) +=
            (point_cloud_2d(i, 0) - bucket_x(index_x, index_y)) / bucket_count(index_x, index_y);
        bucket_y(index_x, index_y) +=
            (point_cloud_2d(i, 1) - bucket_y(index_x, index_y)) / bucket_count(index_x, index_y);
        bucket_depth(index_x, index_y) +=
            (point_cloud_2d(i, 2) - bucket_depth(index_x, index_y)) / bucket_count(index_x, index_y);
      }
      ++bucket_count(index_x, index_y);
    }
  }

  if (print_result)
    std::cout << "Point number in FOV after downsample = " << global_count << "\n" << std::endl;
  // grid size = 5 => print \approx 9.5k => ~10 times less points

  // 把下采样和投影后的激光点坐标（在下采样图像上的坐标点）和深度放在point_cloud_2d_dnsp中
  point_cloud_2d_dnsp = Eigen::MatrixXf(global_count, 3);  // TODO: check if this affect performance
  for (i = 0; i < new_width; ++i)
  {
    for (j = 0; j < new_height; ++j)
    {
      if (bucket_count(i, j) > 0)
      {
        --global_count;
        point_cloud_2d_dnsp(global_count, 0) = bucket_x(i, j);
        point_cloud_2d_dnsp(global_count, 1) = bucket_y(i, j);
        point_cloud_2d_dnsp(global_count, 2) = bucket_depth(i, j);
      }
    }
  }
  assert(global_count == 0);
}

/**
 * @description: 对点云可视化
 * @param {string} image_file_path 图片路径
 * @param {bool} select_downsampled 下采样方式
 * @return {*}
 */
void PointCloudUtil::visualizePointCloud(const std::string image_file_path, const bool select_downsampled)
{
  image_with_point_cloud = cv::imread(image_file_path, cv::IMREAD_GRAYSCALE);
  // cv::Mat point_cloud_2d_image(IMG_HEIGHT, IMG_WIDTH, CV_8UC3, cv::Scalar(0, 0, 0));
  // TODO: remove points outside of Cam0 FOV -> will be done in downsampling
  assert(image_with_point_cloud.size().height == IMG_HEIGHT);
  assert(image_with_point_cloud.size().width == IMG_WIDTH);

  // 对图像进行转换
  cv::cvtColor(image_with_point_cloud, image_with_point_cloud, cv::COLOR_GRAY2BGR);

  // 选择下采样类型，获得的激光点类型
  Eigen::MatrixXf& point_cloud_2d_ = (select_downsampled) ? point_cloud_2d_dnsp : point_cloud_2d;

  float depth, depth_min = 0.1f, depth_max = 50.0f, ratio;
  int i = 0;
  for (i = 0; i < point_cloud_2d_.rows(); ++i)
  {
    depth = point_cloud_2d_(i, 2);
    ratio = std::max(std::min((depth - depth_min) / (depth_max - depth_min), 1.0f), 0.0f);
    if (ratio < 0.5)
    {
      cv::circle(image_with_point_cloud, cv::Point(point_cloud_2d_(i, 0), point_cloud_2d_(i, 1)),  // x, y
                 1, cv::Scalar(0, 255 * ratio * 2, 255 * (1 - ratio * 2)), cv::FILLED, cv::LINE_8);
    }
    else
    {
      cv::circle(image_with_point_cloud, cv::Point(point_cloud_2d_(i, 0), point_cloud_2d_(i, 1)), 1,
                 cv::Scalar(255 * (ratio - 0.5) * 2, 255 * (1 - (ratio - 0.5) * 2), 0), cv::FILLED, cv::LINE_8);
    }
  }

  cv::namedWindow("Display Kitti Image With Depth", cv::WINDOW_AUTOSIZE);
  cv::imshow("Display Kitti Image With Depth", image_with_point_cloud);

  cv::waitKey(0);  // wait for key to be pressed
}

// float triangleArea(const Eigen::Vector2f& A, const Eigen::Vector2f& B, const Eigen::Vector2f& C) {

// }

/**
 * @description: 
 * @param {float} x
 * @param {float} y
 * @param {int} searching_radius
 * @return {*}
 */
float PointCloudUtil::queryDepth(const float x, const float y, const int searching_radius) const
{
  // grid size and searching radius are respectively recommended to be 5 and 2
  // 网格大小和搜索半径分别推荐为5和2
  assert(std::ceil(static_cast<float>(IMG_WIDTH) / static_cast<float>(downsample_grid_size)) == bucket_x.rows());
  assert(std::ceil(static_cast<float>(IMG_HEIGHT) / static_cast<float>(downsample_grid_size)) == bucket_x.cols());

  // float x = static_cast<float>(c);
  // float y = static_cast<float>(IMG_HEIGHT - r);
  int index_x = static_cast<int>(x / downsample_grid_size);
  int index_y = static_cast<int>(y / downsample_grid_size);
  const int new_width = bucket_x.rows();  // cautious, bucket axis0 is x, axis1 is y => different from image array
  const int new_height = bucket_x.cols();

  // select all neighbors in a certain local block
  int index_x_, index_y_;
  std::vector<Eigen::Vector4f> neighbors;
  Eigen::Vector4f neighbor;
  for (index_x_ = index_x - searching_radius; index_x_ <= index_x + searching_radius; ++index_x_)
  {
    for (index_y_ = index_y - searching_radius; index_y_ <= index_y + searching_radius; ++index_y_)
    {
      if (index_x_ >= 0 and index_x_ < new_width and index_y_ >= 0 and index_y_ < new_height and
          bucket_count(index_x_, index_y_) > 0)
      {
        neighbor(0) = bucket_x(index_x_, index_y_);
        neighbor(1) = bucket_y(index_x_, index_y_);
        neighbor(2) = bucket_depth(index_x_, index_y_);
        neighbor(3) = std::sqrt(std::pow(x - neighbor(0), 2) + std::pow(y - neighbor(1), 2));
        neighbors.push_back(neighbor);
        // std::cout << neighbor.transpose() << std::endl;
      }
    }
  }

  // edge case, no enough neighbors
  if (neighbors.size() < 10)
    return -1.0f;  // a fixed unrealistic value representing query failure

  // sort the vector; better ways can be quick select and heapify
  std::sort(neighbors.begin(), neighbors.end(),
            [&](const Eigen::Vector4f& n1, const Eigen::Vector4f& n2) -> bool { return n1(3) < n2(3); });

  // // Condition to be satisfied:  point x,y should be inside of the n0, n1 and n2 triangle
  // Eigen::Vector2f n0to1 = neighbors[1].head(2) - neighbors[0].head(2);
  // Eigen::Vector2f n0to2 = neighbors[2].head(2) - neighbors[0].head(2);
  // Eigen::Vector2f n0toP;
  // n0toP << x - neighbors[0](0), y - neighbors[0](1);
  // Eigen::Vector2f n1to0 = neighbors[0].head(2) - neighbors[1].head(2);
  // Eigen::Vector2f n1to2 = neighbors[2].head(2) - neighbors[1].head(2);
  // Eigen::Vector2f n1toP;
  // n1toP << x - neighbors[1](0), y - neighbors[1](1);
  // Eigen::Vector2f n2to0 = neighbors[0].head(2) - neighbors[2].head(2);
  // Eigen::Vector2f n2to1 = neighbors[1].head(2) - neighbors[2].head(2);
  // Eigen::Vector2f n2toP;
  // n2toP << x - neighbors[2](0), y - neighbors[2](1);

  // if ((n0to1*n0to2) * (n0to1*n0toP) < 0 or )

  // float area_012 =

  // float z = (neighbors[0](2) + neighbors[1](2) + neighbors[2](2))/3.0f;  // TODO: weighted distance -> Done? need to
  // test

  // std::cout << neighbors[0].head(3).transpose() << std::endl;
  // std::cout << neighbors[1].head(3).transpose() << std::endl;
  // std::cout << neighbors[2].head(3).transpose() << "\n" << std::endl;

  // float depth_max = std::max({
  //     neighbors[0](2),
  //     neighbors[1](2),
  //     neighbors[2](2)
  // });
  // float depth_min = std::min({
  //     neighbors[0](2),
  //     neighbors[1](2),
  //     neighbors[2](2)
  // });
  // if (depth_max - depth_min > 1.0)
  //     return -2.0f;

  float z = (neighbors[0](2) * neighbors[1](3) * neighbors[2](3) + neighbors[1](2) * neighbors[0](3) * neighbors[2](3) +
             neighbors[2](2) * neighbors[0](3) * neighbors[1](3)) /
            (0.0001f + neighbors[1](3) * neighbors[2](3) + neighbors[0](3) * neighbors[2](3) +
             neighbors[0](3) * neighbors[1](3));  // TODO: weighted distance -> Done? need to test
  assert(z > 0);
  return z;
  // // naive averaging is already providing good estimation => maybe just check the current bucket_depth is also a fast
  // and good estimation

  // // select 3 nearest neighbor
  // Eigen::Vector3f n1 = neighbors[0]; // n1 here is the \hat{X}_i^{k−1} in eq.10 in DEMO paper
  // n1(0) *= n1(2); // go back to 3d coordinate
  // n1(1) *= n1(2); // go back to 3d coordinate
  // Eigen::Vector3f n2 = neighbors[1];
  // n2(0) *= n2(2);
  // n2(1) *= n2(2);
  // Eigen::Vector3f n3 = neighbors[2];
  // n3(0) *= n3(2);
  // n3(1) *= n3(2);

  // // calculate depth
  // Eigen::Vector3f cp = (n1 - n3).cross(n1 - n2);
  // float z = n1.dot(cp) / (0.0001f + x*cp(0) + y*cp(1) + cp(2));
  // // assert(z > 0); // can't guarantee
  // return z;
}

/**
 * @description: 
 * @param {int} event
 * @param {int} x
 * @param {int} y
 * @return {*}
 */
void PointCloudUtil::visualizeDepthCallBack(int event, int x, int y)
{
  if (event == cv::EVENT_LBUTTONDOWN)
  {
    std::cout << "Left button of the mouse is clicked - position (" << x << ", " << y << "), and the depth is "
              << PointCloudUtil::queryDepth(x, y) << std::endl;
  }
}

/**
 * @description: 
 * @param {int} ev
 * @param {int} x
 * @param {int} y
 * @param {void*} obj
 * @return {*}
 */
void visualizeDepthOnMouse(int ev, int x, int y, int, void* obj)
{
  PointCloudUtil* pcu = static_cast<PointCloudUtil*>(obj);
  if (pcu)
    pcu->visualizeDepthCallBack(ev, x, y);
}

void PointCloudUtil::visualizeDepth(const std::string image_file_path)
{
  // 读入灰度图 cv::IMREAD_GRAYSCALE
  image_with_depth = cv::imread(image_file_path, cv::IMREAD_GRAYSCALE);
  // 转换图像
  cv::cvtColor(image_with_depth, image_with_depth, cv::COLOR_GRAY2BGR);

  int x, y;
  float depth, depth_min = 0.1f, depth_max = 50.0f, ratio;
  for (x = 0; x < IMG_WIDTH; x += 3)
  {  // += 3 to make the visualization sparse
    for (y = 0; y < IMG_HEIGHT; y += 3)
    {
      depth = PointCloudUtil::queryDepth(static_cast<float>(x), static_cast<float>(y));
      if (depth > 0)
      {
        ratio = std::max(std::min((depth - depth_min) / (depth_max - depth_min), 1.0f), 0.0f);
        if (ratio < 0.5)
        {
          cv::circle(image_with_depth, cv::Point(x, y),  // x, y
                     1, cv::Scalar(0, 255 * ratio * 2, 255 * (1 - ratio * 2)), cv::FILLED, cv::LINE_8);
        }
        else
        {
          cv::circle(image_with_depth, cv::Point(x, y), 1,
                     cv::Scalar(255 * (ratio - 0.5) * 2, 255 * (1 - (ratio - 0.5) * 2), 0), cv::FILLED, cv::LINE_8);
        }
      }
      // else {
      //     cv::circle(
      //         image_with_depth,
      //         cv::Point(x, y), // x, y
      //         1,
      //         cv::Scalar(255, 0, 255),
      //         cv::FILLED,
      //         cv::LINE_8
      //     );
      // }
    }
  }

  cv::namedWindow("Display Kitti Sample Image With Depth Estimation", cv::WINDOW_AUTOSIZE);
  cv::setMouseCallback("Display Kitti Sample Image With Depth Estimation", visualizeDepthOnMouse, this);
  cv::imshow("Display Kitti Sample Image With Depth Estimation", image_with_depth);
  // cv::imwrite(ros::package::getPath("visual_odometry") + "/figures/gray_image_with_depth_3nn_plane.png", image2);
  cv::waitKey(0);  // wait for key to be pressed
}

cv::Mat PointCloudUtil::visualizeDepth(const cv::Mat& gray_image)
{
  image_with_depth = gray_image.clone();
  cv::cvtColor(image_with_depth, image_with_depth, cv::COLOR_GRAY2BGR);
  int x, y;
  float depth, depth_min = 0.1f, depth_max = 50.0f, ratio;
  for (x = 0; x < IMG_WIDTH; x += 3)
  {  // += 3 to make the visualization sparse
    for (y = 0; y < IMG_HEIGHT; y += 3)
    {
      depth = PointCloudUtil::queryDepth(static_cast<float>(x), static_cast<float>(y));
      if (depth > 0)
      {
        ratio = std::max(std::min((depth - depth_min) / (depth_max - depth_min), 1.0f), 0.0f);
        if (ratio < 0.5)
        {
          cv::circle(image_with_depth, cv::Point(x, y),  // x, y
                     1, cv::Scalar(0, 255 * ratio * 2, 255 * (1 - ratio * 2)), cv::FILLED, cv::LINE_8);
        }
        else
        {
          cv::circle(image_with_depth, cv::Point(x, y), 1,
                     cv::Scalar(255 * (ratio - 0.5) * 2, 255 * (1 - (ratio - 0.5) * 2), 0), cv::FILLED, cv::LINE_8);
        }
      }
    }
  }

  // cv::namedWindow("Display Kitti Sample Image With Depth Estimation", cv::WINDOW_AUTOSIZE);
  // cv::imshow("Display Kitti Sample Image With Depth Estimation", image_with_depth);
  // cv::waitKey(1); // wait for 1ms

  return image_with_depth;
}
}  // namespace vloam