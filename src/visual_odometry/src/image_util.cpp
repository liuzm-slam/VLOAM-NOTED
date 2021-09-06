#include <visual_odometry/image_util.h>

namespace vloam
{
  /* 在图像配准应用中，速度比较：SIFT<SURF<BRISK<FREAK<ORB，
  在对有较大模糊的图像配准时，BRISK算法在其中表现最为出色 
  */
std::vector<cv::KeyPoint> ImageUtil::detKeypoints(cv::Mat& img)
{
  std::vector<cv::KeyPoint> keypoints;

  if (print_result)
    time = (double)cv::getTickCount();

  // shi-tomasi角点
  if (detector_type == DetectorType::ShiTomasi)
  {
    // 计算协方差矩阵时的窗口大小
    int block_size = 5;
    /*      
     用于计算每个像素邻域上的导数协变矩阵的平均块的大小
     double max_overlap = 0.0; // 最大限度。 两个特征之间允许的重叠百分比
     double min_distance = (1.0 - max_overlap) * block_size;  
     size of an average block for computing a derivative covariation matrix over each pixel neighborhood
     double max_overlap = 0.0; // max. permissible overlap between two features in %
     double min_distance = (1.0 - max_overlap) * block_size; 
    */
    // 对于初选出的角点而言，如果在其周围minDistance范围内存在其他更强角点，则将此角点删除  
    double min_distance = block_size * 1.5;
    // int maxCorners = img.rows * img.cols / std::max(1.0, min_distance); // max. num. of keypoints
    // 角点数目最大值，如果实际检测的角点超过此值，则只返回前maxCorners个强角点
    int maxCorners = 1024;

    // 角点的品质因子
    double quality_level = 0.03;  // minimal accepted quality of image corners

    // Harris角点检测需要的k值
    double k = 0.04;

    // 保存检测出的角点
    std::vector<cv::Point2f> corners;
    /**
     * img:8位或32位浮点型输入图像，单通道；
     * corners:保存检测出的角点,位置点向量，保存的是检测到的角点的坐标;
     * maxCorners：角点数目最大值，如果实际检测的角点超过此值，则只返回前maxCorners个强角点,
     * 检测到的角点的质量等级，角点特征值小于qualityLevel*最大特征值的点将被舍弃;
     * qualityLevel：角点的品质因子,以像素为单位;
     * minDistance：对于初选出的角点而言，如果在其周围minDistance范围内存在其他更强角点，则将此角点删除;
     * cv::Mat()：指定感兴趣区，如不需在整幅图上寻找角点，则用此参数指定ROI,
     * 指定检测区域，若检测整幅图像，mask置为空Mat()
     * block_size：计算协方差矩阵时的窗口大小
     * false:useHarrisDetector：指示是否使用Harris角点检测，如不指定，则计算shi-tomasi角点
     * k:harrisK：Harris角点检测需要的k值,留给Harris角点检测算子用的中间参数，
     * 一般取经验值0.04~0.06。第八个参数为false时，该参数不起作用
     * 
    **/
    // 对整张图进行shi-tomasi角点查找，结果缓存到corners坐标  
    cv::goodFeaturesToTrack(img, corners, maxCorners, quality_level, min_distance, cv::Mat(), block_size, false, k);

    // add corners to result vector
    for (auto it = corners.begin(); it != corners.end(); ++it)
    {
      cv::KeyPoint new_keypoint;
      // 关键点的点坐标
      new_keypoint.pt = cv::Point2f((*it).x, (*it).y);
      // 该关键点邻域直径大小
      new_keypoint.size = block_size;
      // keypoints关键点
      keypoints.push_back(new_keypoint);
    }
  }
  else if (detector_type == DetectorType::FAST)
  {
    int threshold = 100;
    /**
     * applying FAST key point detector  
     * Fast特征提取函数
     * threshold指的是中心像素与周围像素强度的差的阈值
     * nonmaxSuppression是为了去除特征点聚集的情况
     * DetectorType提供了三种不同的邻域类型
    **/
    cv::FAST(img, keypoints, threshold, true);
  }
  else
  {
    cv::Ptr<cv::FeatureDetector> detector;
    // BRISK算法中构造了图像金字塔进行多尺度表达，因此具有较好的旋转不变性、尺度不变性，较好的鲁棒性等。
    if (detector_type == DetectorType::BRISK)
    {
      // BRISK关键子检测
      detector = cv::BRISK::create();
    }
  /** 
  static Ptr<ORB> cv::ORB::create (
        int     nfeatures = 500,    //The maximum number of features to retain.
        float   scaleFactor = 1.2f,
                                    //金字塔抽取比，大于1。scaleFactor==2表示经典金字塔，每一层的像素都比上一层少4倍，
                                    //但如此大的尺度因子会显著降低特征匹配得分。
                                    //另一方面，过于接近1的比例因素将意味着要覆盖一定的比例范围，
                                    //你将需要更多的金字塔级别，因此速度将受到影响。
        int     nlevels = 8,        //金字塔等级的数量。
                                    //最小级别的线性大小等于input_image_linear_size/pow(scaleFactor, nlevels).
                                    //输入图像线性大小/功率(缩放因子，非线性)。
        int     edgeThreshold = 31,  //这是未检测到特征的边框大小。它应该大致匹配patchSize参数.
        int     firstLevel = 0,     //It should be 0 in the current implementation.
        int     WTA_K = 2,          //产生oriented BRIEF描述子的每个元素的点数。
        int     scoreType = ORB::HARRIS_SCORE,
                                    //默认的HARRIS_SCORE意味着HARRIS算法用于对特征进行排序
                                    //(分数被写入KeyPoint::score，并用于保留最佳特征)；
                                    //FAST_SCORE是参数的另一个值，它会产生稍微不太稳定的关键点，
                                    //但是计算起来要快一点。
        int     patchSize = 31,     //oriented BRIEF描述符使用的补丁大小。
                                    //在较小的金字塔层上，被特征覆盖的感知图像区域会更大。
        int     fastThreshold = 20
   )

    nfeatures ：最多提取的特征点的数量；
    scaleFactor ： 金字塔图像之间的尺度参数，类似于SIFT中的?；
    nlevels： 高斯金字塔的层数；
    edgeThreshold ：边缘阈值，这个值主要是根据后面的patchSize来定的，靠近边缘edgeThreshold以内的像素是不检测特征点的。
    firstLevel-：看过SIFT都知道，我们可以指定第一层的索引值，这里默认为0。
    WET_K ： 用于产生BIREF描述子的点对的个数，一般为2个，也可以设置为3个或4个，那么这时候描述子之间的距离计算就不能用汉明距离了，而是应该用一个变种。OpenCV中，如果设置WET_K = 2，则选用点对就只有2个点，匹配的时候距离参数选择NORM_HAMMING，如果WET_K设置为3或4，则BIREF描述子会选择3个或4个点，那么后面匹配的时候应该选择的距离参数为NORM_HAMMING2。
    scoreType ：用于对特征点进行排序的算法，你可以选择HARRIS_SCORE，也可以选择FAST_SCORE，但是它也只是比前者快一点点而已。
    patchSize ：用于计算BIREF描述子的特征点邻域大小。 
    **/
    else if (detector_type == DetectorType::ORB)
    {
      int num_features = 2000;
      float scaleFactor = 1.2f;
      int nlevels = 8;
      int edgeThreshold = 31;
      int firstLevel = 0;
      int WTA_K = 2;
      cv::ORB::ScoreType scoreType = cv::ORB::FAST_SCORE;
      int patchSize = 31;
      int fastThreshold = 20;
      // 提取orb特征
      detector = cv::ORB::create(num_features, scaleFactor, nlevels, edgeThreshold, firstLevel, WTA_K, scoreType,
                                 patchSize, fastThreshold);
    }
    else if (detector_type == DetectorType::AKAZE)
      detector = cv::AKAZE::create();
    else if (detector_type == DetectorType::SIFT)
      detector = cv::SIFT::create();
    else
    {
      std::cerr << "Detector is not implemented" << std::endl;
      /**
       * 首先介绍一下：
       * exit(0): 正常执行程序并退出程序。
       * exit(1): 非正常执行导致退出程序。
       * stdlib.h头文件中 定义了两个变量：
       * #define EXIT_SUCCESS 0
       * #define EXIT_FAILURE 1
       * exit(EXIT_SUCCESS)  : 代表安全退出。
       * exit(EXIT_FAILURE) ： 代表异常退出。
      **/
      exit(EXIT_FAILURE);
    }

    // 找到了图像的关键点
    detector->detect(img, keypoints);
  }

  if (print_result)
  {
    time = ((double)cv::getTickCount() - time) / cv::getTickFrequency();
    std::cout << DetectorType_str[static_cast<int>(detector_type)] + "detection with n=" << keypoints.size()
              << " keypoints in " << 1000 * time / 1.0 << " ms" << std::endl;
  }

  if (visualize_result)
  {
    // std::vector<cv::KeyPoint> fake_keypoints;
    // fake_keypoints.push_back(keypoints[0]);
    // std::cout << "fake keypoints 0: " << keypoints[0].pt.x << ", " << keypoints[0].pt.y << std::endl;

    // 把图像img的信息全部拷贝到img_keypoints
    img_keypoints = img.clone();
    // cv::drawKeypoints(img, fake_keypoints, img_keypoints, cv::Scalar::all(-1),
    // cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);
    // 把图像和关键点在屏幕上显示出来
    cv::drawKeypoints(img, keypoints, img_keypoints, cv::Scalar::all(-1), cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);
    std::string window_name = DetectorType_str[static_cast<int>(detector_type)] + " Detector Results";
    cv::namedWindow(window_name, 6);
    cv::imshow(window_name, img_keypoints);
    cv::waitKey(0);
  }

  // 返回关键点
  return keypoints;
}

  /**
   * @brief keypoints_after_NMS取额定数量的关键点，大于额定值时取强度最多的max_bucket_keypoints个点
   * @param keypoints关键帧
   * @param bucket_width
   * @param bucket_height
   * @param max_total_keypoints最大关键点数量
   * @return keypoints_after_NMS取额定数量的关键点
   */
std::vector<cv::KeyPoint> ImageUtil::keyPointsNMS(  // TODO: check if opencv detector minDistance helps here
    std::vector<cv::KeyPoint>&& keypoints,
    const int bucket_width,   // width for horizontal direction in image plane => x, col
    const int bucket_height,  // height for vertical direction in image plane => y, row
    const int max_total_keypoints)
{
  // std::ceil向上取整
  const int bucket_shape_x = std::ceil(static_cast<float>(IMG_WIDTH) / static_cast<float>(bucket_width));    // 13
  const int bucket_shape_y = std::ceil(static_cast<float>(IMG_HEIGHT) / static_cast<float>(bucket_height));  // 4

  const int max_bucket_keypoints = max_total_keypoints / (bucket_shape_x * bucket_shape_y);  // 7

  std::vector<std::vector<std::vector<cv::KeyPoint>>> bucket(
      bucket_shape_x, std::vector<std::vector<cv::KeyPoint>>(bucket_shape_y, std::vector<cv::KeyPoint>()));

  // put all keypoints into buckets
  for (const auto& keypoint : keypoints)
  {
    bucket[static_cast<int>(keypoint.pt.x / static_cast<float>(bucket_width))]
          [static_cast<int>(keypoint.pt.y / static_cast<float>(bucket_height))]
              .push_back(keypoint);
  }

  // keypoints_after_NMS取额定数量的关键点，大于额定值时取强度最多的max_bucket_keypoints个点
  std::vector<cv::KeyPoint> keypoints_after_NMS;
  // 强制容器把它的容量改为至少n
  keypoints_after_NMS.reserve(max_total_keypoints);
  // 响应程度，代表该点强壮大小—response代表着该关键点how good，更确切的说，是该点角点的程度 
  auto keypoints_sort = [](const cv::KeyPoint& kp0, const cv::KeyPoint& kp1) { return kp0.response > kp1.response; };

  // iterate all bucket, sort and put keypoints with top response to the return
  // keypoints_after_NMS取额定数量的关键点，大于额定值时取强度最多的max_bucket_keypoints个点
  int col, row;
  for (col = 0; col < bucket_shape_x; ++col)
  {
    for (row = 0; row < bucket_shape_y; ++row)
    {
      if (bucket[col][row].empty())
        continue;

      // 小于最大点数量就拷贝到keypoints_after_NMS
      if (bucket[col][row].size() <= max_bucket_keypoints)
      {
        // std::copy是stl的拷贝函数，把bucket的元素拷贝到keypoints_after_NMS中
        std::copy(bucket[col][row].begin(), bucket[col][row].end(), std::back_inserter(keypoints_after_NMS));
        continue;
      }

      // 根据关键点的健壮强度来排序
      std::sort(bucket[col][row].begin(), bucket[col][row].end(),
                keypoints_sort);  // ascending order of keypoint response
      // 当大于最大关键点数量的时候，就只取健壮的最大关键点数量的值
      std::copy(bucket[col][row].begin(), bucket[col][row].begin() + max_bucket_keypoints,
                std::back_inserter(keypoints_after_NMS));
    }
  }

  // keypoints_after_NMS取额定数量的关键点
  return keypoints_after_NMS;
}

// 保存关键点图像
void ImageUtil::saveKeypointsImage(const std::string file_name)
{
  if (!img_keypoints.data)
  {
    printf("No keypoints data \n");
    return;
  }
  cv::imwrite(ros::package::getPath("visual_odometry") + "/figures/" + file_name + ".png", img_keypoints);
}

/**
 * @brief 根据图像和关键点提取描述子
 * @param keypoints 关键点
 * @param img 图像
 * @return descriptors描述子
**/
cv::Mat ImageUtil::descKeypoints(std::vector<cv::KeyPoint>& keypoints, cv::Mat& img)
{
  // 描述子
  cv::Mat descriptors;

  // 提取描述子
  cv::Ptr<cv::DescriptorExtractor> extractor;
  if (descriptor_type == DescriptorType::BRISK)
  {
    int threshold = 30;          // FAST/AGAST detection threshold score. AGAST（FAST的加速版）检测的阈值，阈值越大检测到的点越小
    int octaves = 3;             // detection octaves (use 0 to do single scale) 金字塔的层数
    float pattern_scale = 1.0f;  // apply this scale to the pattern used for sampling the neighbourhood of a keypoint.

    // 提取BRISK描述子
    extractor = cv::BRISK::create(threshold, octaves, pattern_scale);
  }
  // else if (descriptor_type == DescriptorType::BRIEF) {
  //     extractor = cv::xfeatures2d::BriefDescriptorExtractor::create();
  // }
  else if (descriptor_type == DescriptorType::ORB)
  {
    // 提取ORB描述子
    extractor = cv::ORB::create();
  }
  // else if (descriptor_type == DescriptorType::FREAK) {
  //     extractor = cv::xfeatures2d::FREAK::create();
  // }
  else if (descriptor_type == DescriptorType::AKAZE)
  {
    // 提取AKAZE描述子
    extractor = cv::AKAZE::create();
  }
  else if (descriptor_type == DescriptorType::SIFT)
  {
    // 提取SIFT描述子
    extractor = cv::SIFT::create();
  }
  else
  {
    std::cerr << "Decscriptor is not implemented" << std::endl;
    // 异常退出
    exit(EXIT_FAILURE);
  }

  if (print_result)
    time = (double)cv::getTickCount();

  // 提取描述子
  extractor->compute(img, keypoints, descriptors);

  if (print_result)
  {
    time = ((double)cv::getTickCount() - time) / cv::getTickFrequency();
    std::cout << DescriptorType_str[static_cast<int>(descriptor_type)] << " descriptor extraction in "
              << 1000 * time / 1.0 << " ms" << std::endl;
  }

  // 返回描述子
  return descriptors;
}

/**
 * @brief 匹配描述子
 * @param descriptors0
 * @param descriptors1
 * @return matches匹配结果
 */
std::vector<cv::DMatch> ImageUtil::matchDescriptors(cv::Mat& descriptors0, cv::Mat& descriptors1)
{
  std::vector<cv::DMatch> matches;
  bool crossCheck = (selector_type == SelectType::NN);
  cv::Ptr<cv::DescriptorMatcher> matcher;

  // Brute-Force-Match暴力匹配算法
  // Reference: https://docs.opencv.org/master/dc/dc3/tutorial_py_matcher.html
  if (matcher_type == MatcherType::BF)
  {
    int normType;
    if (descriptor_type == DescriptorType::AKAZE or descriptor_type == DescriptorType::BRISK or
        descriptor_type == DescriptorType::ORB)
    {
      // 汉明距离
      normType = cv::NORM_HAMMING;
    }
    else if (descriptor_type == DescriptorType::SIFT)
    {
      // 根据欧氏距离判断，比NORM_L2更准确
      normType = cv::NORM_L2;
    }
    else
    {
      std::cerr << "Decscriptor is not implemented" << std::endl;
    }
    matcher = cv::BFMatcher::create(normType, crossCheck);
  }
  // 快速最近邻逼近搜索FLANN
  else if (matcher_type == MatcherType::FLANN)
  {
    if (descriptors0.type() != CV_32F)
    {  // OpenCV 错误解决方法：由于当前 OpenCV 中的错误，将二进制描述符转换为浮点数
       // OpenCV bug workaround : convert binary descriptors to floating point due to a bug in current OpenCV
       // implementation
      descriptors0.convertTo(descriptors0, CV_32F);
    }
    if (descriptors1.type() != CV_32F)
    {  // OpenCV bug workaround : convert binary descriptors to floating point due to a bug in current OpenCV
       // implementation
      descriptors1.convertTo(descriptors1, CV_32F);
    }

    matcher = cv::DescriptorMatcher::create(cv::DescriptorMatcher::FLANNBASED);
  }

  if (print_result)
    time = (double)cv::getTickCount();

  if (selector_type == SelectType::NN)
  {
    // 对描述子进行匹配
    matcher->match(descriptors0, descriptors1, matches);

    if (print_result)
    {
      time = ((double)cv::getTickCount() - time) / cv::getTickFrequency();
      std::cout << " (NN) with n=" << matches.size() << " matches in " << 1000 * time / 1.0 << " ms" << std::endl;
    }
  }
  // knnMatch不同的匹配方式
  else if (selector_type == SelectType::KNN)
  {  // k nearest neighbors (k=2)
    // double t = (double)cv::getTickCount();

    std::vector<std::vector<cv::DMatch>> knn_matches;
    matcher->knnMatch(descriptors0, descriptors1, knn_matches, 2);
    for (const auto& knn_match : knn_matches)
    {
      // 对匹配结果进行筛选
      if (knn_match[0].distance < 0.8 * knn_match[1].distance)
      {
        matches.push_back(knn_match[0]);
      }
    }

    if (print_result)
    {
      time = ((double)cv::getTickCount() - time) / cv::getTickFrequency();
      std::cout << " (KNN) with n=" << matches.size() << " matches in " << 1000 * time / 1.0 << " ms" << std::endl;
      std::cout << "KNN matches = " << knn_matches.size() << ", qualified matches = " << matches.size()
                << ", discard ratio = " << (float)(knn_matches.size() - matches.size()) / (float)knn_matches.size()
                << std::endl;
    }
  }

  if (print_result)
    std::cout << "MATCH KEYPOINT DESCRIPTORS done, and the number of matches is " << matches.size() << std::endl;

  // 返回描述子的匹配结果
  return matches;
}

// 可视化鼠标左键按到的位置
void ImageUtil::visualizeMatchesCallBack(int event, int x, int y)
{
  // 左键点击EVENT_LBUTTONDOWN
  if (event == cv::EVENT_LBUTTONDOWN)
  {
    std::cout << "Left button of the mouse is clicked - position (" << x << ", " << y << ")" << std::endl;
  }
}

void visualizeMatchesOnMouse(int ev, int x, int y, int, void* obj)
{
  ImageUtil* iu = static_cast<ImageUtil*>(obj);
  if (iu)
    iu->visualizeMatchesCallBack(ev, x, y);
}

// 对匹配结果进行可视化
cv::Mat ImageUtil::visualizeMatches(const cv::Mat& image0, const cv::Mat& image1,
                                    const std::vector<cv::KeyPoint>& keypoints0,
                                    const std::vector<cv::KeyPoint>& keypoints1, const std::vector<cv::DMatch>& matches)
{
  std::vector<cv::DMatch> matches_dnsp;
  const int stride = std::ceil(static_cast<float>(matches.size()) / 100.0f);  // at most 100 points

  int prev_pt_x, prev_pt_y, curr_pt_x, curr_pt_y;
  for (int i = 0; i < matches.size(); i += stride)
  {
    prev_pt_x = keypoints0[matches[i].queryIdx].pt.x;
    prev_pt_y = keypoints0[matches[i].queryIdx].pt.y;
    curr_pt_x = keypoints1[matches[i].trainIdx].pt.x;
    curr_pt_y = keypoints1[matches[i].trainIdx].pt.y;
    if (remove_VO_outlier > 0)
    {
      if (std::pow(prev_pt_x - curr_pt_x, 2) + std::pow(prev_pt_y - curr_pt_y, 2) >
          remove_VO_outlier * remove_VO_outlier)
        continue;
    }
    matches_dnsp.push_back(matches[i]);
  }

  cv::Mat matchImg = image1.clone();
  cv::drawMatches(image0, keypoints0, image1, keypoints1, matches_dnsp, matchImg, cv::Scalar::all(-1),
                  cv::Scalar::all(-1), std::vector<char>(), cv::DrawMatchesFlags::DEFAULT);

  // std::string windowName = "Matching keypoints between two camera images";
  // cv::namedWindow(windowName, 7);
  // cv::setMouseCallback(windowName, visualizeMatchesOnMouse, this);
  // cv::imshow(windowName, matchImg);
  // std::cout << "Press key to continue to next image" << std::endl;
  // cv::waitKey(0); // wait for key to be pressed
  // ROS_INFO("image showed");

  return matchImg;
}

/**
 * @description: 使用光流法对连续两帧图像进行位置估计
 * @param image0 上一帧图像
 * @param image1 当前帧图像
 * @param keypoints0 上一帧的关键点
 * @return keypoints0_2f上一帧的关键点，keypoints1_2f当前帧根据光流法得到的关键点，optical_flow_status光流法找到的光流
 */
std::tuple<std::vector<cv::Point2f>, std::vector<cv::Point2f>, std::vector<uchar>> ImageUtil::calculateOpticalFlow(
    const cv::Mat& image0, const cv::Mat& image1, const std::vector<cv::KeyPoint>& keypoints0)
{
  std::vector<cv::Point2f> keypoints0_2f;
  std::vector<cv::Point2f> keypoints1_2f;
  std::vector<uchar> optical_flow_status;

  /* 
    transform keypoints to points_2f
    // 把图像上的关键点放到keypoints0_2f（keypoints0 to keypoints0_2f）
    const_iterator : cbegin，cend 迭代器
    std::back_inserter，back_inserter : 返回尾部插入型迭代器，内部会调用容器的push_back()方法来将数据插入容器的尾部 
    std::transform在指定的范围内应用于给定的操作，并将结果存储在指定的另一个范围内
  */
  std::transform(keypoints0.cbegin(), keypoints0.cend(), std::back_inserter(keypoints0_2f),
                 [](const cv::KeyPoint& keypoint) { return keypoint.pt; });

  // calculate optical flow
  std::vector<float> err;
  // cv::TermCriteria一种迭代求解结构
  // cv::TermCriteria::COUNT达到最大迭代次数,cv::TermCriteria::EPS达到精度
  // TermCriteria::COUNT + TermCriteria::EPS //以上两种同时作为判定条件
  // 通过初始化的criteria后，就可以作为变量提供给opencv需要迭代的算法中作为其中一个参数。
  cv::TermCriteria criteria = cv::TermCriteria((cv::TermCriteria::COUNT) + (cv::TermCriteria::EPS), 10, 0.03);
  
  /*   
    void cv::calcOpticalFlowPyrLK	(	
    InputArray 	prevImg,
    InputArray 	nextImg,
    InputArray 	prevPts,
    InputOutputArray 	nextPts,
    OutputArray 	status,
    OutputArray 	err,
    Size 	winSize = Size(21, 21),
    int 	maxLevel = 3,
    TermCriteria 	criteria = TermCriteria(TermCriteria::COUNT+TermCriteria::EPS, 30, 0.01),
    int 	flags = 0,
    double 	minEigThreshold = 1e-4 
    )	 
    使用具有金字塔的迭代Lucas-Kanade方法计算稀疏特征集的光流。
    参数：

    prevImg ：buildOpticalFlowPyramid构造的第一个8位输入图像或金字塔。
    nextImg ：与prevImg相同大小和相同类型的第二个输入图像或金字塔
    prevPts ：需要找到流的2D点的矢量(vector of 2D points for which the flow needs to be found;);点坐标必须是单精度浮点数。
    nextPts ：输出二维点的矢量（具有单精度浮点坐标），包含第二图像中输入特征的计算新位置;当传递OPTFLOW_USE_INITIAL_FLOW标志时，向量必须与输入中的大小相同。
    status ：输出状态向量（无符号字符）;如果找到相应特征的流，则向量的每个元素设置为1，否则设置为0。
    err ：输出错误的矢量; 向量的每个元素都设置为相应特征的错误，错误度量的类型可以在flags参数中设置; 如果未找到流，则未定义错误（使用status参数查找此类情况）。
    winSize ：每个金字塔等级的搜索窗口的winSize大小。
    maxLevel ：基于0的最大金字塔等级数;如果设置为0，则不使用金字塔（单级），如果设置为1，则使用两个级别，依此类推;如果将金字塔传递给输入，那么算法将使用与金字塔一样多的级别，但不超过maxLevel。
    criteria ：参数，指定迭代搜索算法的终止条件（在指定的最大迭代次数criteria.maxCount之后或当搜索窗口移动小于criteria.epsilon时）。
    flags ：操作标志：
    OPTFLOW_USE_INITIAL_FLOW使用初始估计，存储在nextPts中;如果未设置标志，则将prevPts复制到nextPts并将其视为初始估计。
    OPTFLOW_LK_GET_MIN_EIGENVALS使用最小特征值作为误差测量（参见minEigThreshold描述）;如果没有设置标志，则将原稿周围的色块和移动点之间的L1距离除以窗口中的像素数，用作误差测量。
    minEigThreshold ：算法计算光流方程的2x2正常矩阵的最小特征值，除以窗口中的像素数;如果此值小于minEigThreshold，则过滤掉相应的功能并且不处理其流程，因此它允许删除坏点并获得性能提升。
    该函数实现了金字塔中Lucas-Kanade光流的稀疏迭代版本。
  */	
  // 通过光流法求解当前帧和上一帧间的运动关系
  // keypoints1_2f根据光流法得到的对应的关键点keypoints1_2f
  cv::calcOpticalFlowPyrLK(image0, image1, keypoints0_2f, keypoints1_2f, optical_flow_status, err, cv::Size(15, 15), 2,
                           criteria);

  // 输出光流法找到的关键点数
  ROS_INFO("Optical Flow: total candidates = %ld",
           std::count(optical_flow_status.cbegin(), optical_flow_status.cend(), 1));

  return std::make_tuple(std::cref(keypoints0_2f), std::cref(keypoints1_2f), std::cref(optical_flow_status));
}

// 对输入的图像和二维点间画连线
cv::Mat ImageUtil::visualizeOpticalFlow(const cv::Mat& image1, const std::vector<cv::Point2f>& keypoints0_2f,
                                        const std::vector<cv::Point2f>& keypoints1_2f,
                                        const std::vector<uchar>& optical_flow_status)
{
  // Create some random colors
  // 转换
  cv::Mat image1_color = image1.clone();
  cv::cvtColor(image1_color, image1_color, cv::COLOR_GRAY2BGR);
  cv::RNG rng; // 产生随机数
  cv::Scalar color; // 画线的颜色cv::Scalar(v1, v2, v3, v4)的这四个参数就依次是BGRA，即蓝、绿、红和透明度。
  int r, g, b, j;
  for (j = 0; j < keypoints0_2f.size(); ++j)
  {
    // Select good points
    if (optical_flow_status[j] == 1)
    {
      // draw the tracks
      r = rng.uniform(0, 256);
      g = rng.uniform(0, 256);
      b = rng.uniform(0, 256);
      color = cv::Scalar(r, g, b);
      /* 
        cv::line：画线
        第一个参数img：要划的线所在的图像;
        第二个参数pt1：直线起点
        第二个参数pt2：直线终点
        第三个参数color：直线的颜色 e.g:Scalor(0,0,255)
        第四个参数thickness=1：线条粗细
       */
      cv::line(image1_color, keypoints1_2f[j], keypoints0_2f[j], color, 2);
      // 函数 cv::circle 绘制具有给定中心和半径的空心或实心圆。
      cv::circle(image1_color, keypoints1_2f[j], 3, color, -1);
    }
  }

  return image1_color;
}

// 对输入的图像和关键点对应的匹配关系画匹配示意图
cv::Mat ImageUtil::visualizeOpticalFlow(const cv::Mat& image1, const std::vector<cv::KeyPoint>& keypoints0,
                                        const std::vector<cv::KeyPoint>& keypoints1,
                                        const std::vector<cv::DMatch>& matches)
{
  // Create some random colors
  cv::Mat image1_color = image1.clone();
  cv::cvtColor(image1_color, image1_color, cv::COLOR_GRAY2BGR);
  cv::RNG rng;
  cv::Scalar color;
  int r, g, b, j;
  for (const auto match : matches)
  {
    // draw the tracks
    r = rng.uniform(0, 256);
    g = rng.uniform(0, 256);
    b = rng.uniform(0, 256);
    color = cv::Scalar(r, g, b);
    cv::line(image1_color, keypoints1[match.trainIdx].pt, keypoints0[match.queryIdx].pt, color, 2);
    cv::circle(image1_color, keypoints1[match.trainIdx].pt, 3, color, -1);
  }

  return image1_color;
}
}  // namespace vloam