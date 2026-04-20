#include "localization_sc.hpp"

#include <pcl_conversions/pcl_conversions.h>
#include <tf2_ros/transform_broadcaster.h>

#include <rclcpp/rclcpp.hpp>

using namespace std::chrono_literals;

FastLioLocalizationScQn::FastLioLocalizationScQn(const std::string &node_name)
    : Node(node_name) {
  // -----------------------
  // 1. ROS2 参数
  // -----------------------
  std::string saved_map_path;
  double map_match_hz;
  MapMatcherConfig mm_config;
  auto &gc = mm_config.gicp_config_;
  auto &qc = mm_config.quatro_config_;

  // declare/get parameters
  this->declare_parameter<std::string>("basic.map_frame", "map");
  this->get_parameter("basic.map_frame", map_frame_);
  this->declare_parameter<std::string>("basic.saved_map",
                                       "/home/mason/kitti.bag");
  this->get_parameter("basic.saved_map", saved_map_path);
  this->declare_parameter<double>("basic.map_match_hz", 1.0);
  this->get_parameter("basic.map_match_hz", map_match_hz);
  this->declare_parameter<double>("basic.visualize_voxel_size", 1.0);
  this->get_parameter("basic.visualize_voxel_size", voxel_res_);
  this->declare_parameter<double>("keyframe.keyframe_threshold", 1.0);
  this->get_parameter("keyframe.keyframe_threshold", keyframe_dist_thr_);
  this->declare_parameter<int>("keyframe.num_submap_keyframes", 5);
  this->get_parameter("keyframe.num_submap_keyframes",
                      mm_config.num_submap_keyframes_);
  this->declare_parameter<double>(
      "match.scancontext_max_correspondence_distance", 15.0);
  this->get_parameter("match.scancontext_max_correspondence_distance",
                      mm_config.scancontext_max_correspondence_distance_);
  this->declare_parameter<double>("match.quatro_nano_gicp_voxel_resolution",
                                  0.3);
  this->get_parameter("match.quatro_nano_gicp_voxel_resolution",
                      mm_config.voxel_res_);
  this->declare_parameter<int>("nano_gicp.thread_number", 0);
  this->get_parameter("nano_gicp.thread_number", gc.nano_thread_number_);
  this->declare_parameter<double>("nano_gicp/icp_score_threshold", 10.0);
  this->get_parameter("nano_gicp/icp_score_threshold", gc.icp_score_thr_);
  this->declare_parameter<int>("nano_gicp/correspondences_number", 15);
  this->get_parameter("nano_gicp/correspondences_number",
                      gc.nano_correspondences_number_);
  this->declare_parameter<double>("nano_gicp/max_correspondence_distance",
                                  0.01);
  this->get_parameter("nano_gicp/max_correspondence_distance",
                      gc.max_corr_dist_);
  this->declare_parameter<int>("nano_gicp/max_iter", 32);
  this->get_parameter("nano_gicp/max_iter", gc.nano_max_iter_);
  this->declare_parameter<double>("nano_gicp/transformation_epsilon", 0.01);
  this->get_parameter("nano_gicp/transformation_epsilon",
                      gc.transformation_epsilon_);
  this->declare_parameter<double>("nano_gicp/euclidean_fitness_epsilon", 0.01);
  this->get_parameter("nano_gicp/euclidean_fitness_epsilon",
                      gc.euclidean_fitness_epsilon_);
  this->declare_parameter<int>("nano_gicp/ransac/max_iter", 5);
  this->get_parameter("nano_gicp/ransac/max_iter", gc.nano_ransac_max_iter_);
  this->declare_parameter<double>(
      "nano_gicp/ransac/outlier_rejection_threshold", 1.0);
  this->get_parameter("nano_gicp/ransac/outlier_rejection_threshold",
                      gc.ransac_outlier_rejection_threshold_);
  this->declare_parameter<bool>("quatro.enable", false);
  this->get_parameter("quatro.enable", mm_config.enable_quatro_);
  this->declare_parameter<bool>("quatro.optimize_matching", true);
  this->get_parameter("quatro.optimize_matching", qc.use_optimized_matching_);
  this->declare_parameter<double>("quatro.distance_threshold", 30.0);
  this->get_parameter("quatro.distance_threshold",
                      qc.quatro_distance_threshold_);
  this->declare_parameter<int>("quatro/max_correspondences", 200);
  this->get_parameter("quatro/max_correspondences", qc.quatro_max_num_corres_);
  this->declare_parameter<double>("quatro/fpfh_normal_radius", 0.02);
  this->get_parameter("quatro/fpfh_normal_radius", qc.fpfh_normal_radius_);
  this->declare_parameter<double>("quatro/fpfh_radius", 0.04);
  this->get_parameter("quatro/fpfh_radius", qc.fpfh_radius_);
  this->declare_parameter<bool>("quatro.estimating_scale", false);
  this->get_parameter("quatro.estimating_scale", qc.estimat_scale_);
  this->declare_parameter<double>("quatro/noise_bound", 0.25);
  this->get_parameter("quatro/noise_bound", qc.noise_bound_);
  this->declare_parameter<double>("quatro/rotation/gnc_factor", 0.25);
  this->get_parameter("quatro/rotation/gnc_factor", qc.rot_gnc_factor_);
  this->declare_parameter<double>("quatro/rotation/rot_cost_diff_threshold",
                                  0.25);
  this->get_parameter("quatro/rotation/rot_cost_diff_threshold",
                      qc.rot_cost_diff_thr_);
  this->declare_parameter<int>("quatro/rotation/num_max_iter", 50);
  this->get_parameter("quatro/rotation/num_max_iter", qc.quatro_max_iter_);

  // -----------------------
  // 2. Initialize map matcher
  // -----------------------
  map_matcher_ = std::make_shared<MapMatcher>(mm_config);

  // -----------------------
  // 3. Load saved map
  // -----------------------
  loadMap(saved_map_path);

  // -----------------------
  // 4. ROS2 publishers
  // -----------------------
  odom_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
      "/ori_odom_localization", 10);
  path_pub_ =
      this->create_publisher<nav_msgs::msg::Path>("/ori_path_localization", 10);
  corrected_odom_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
      "/corrected_odom_localization", 10);
  corrected_path_pub_ = this->create_publisher<nav_msgs::msg::Path>(
      "/corrected_path_localization", 10);
  corrected_current_pcd_pub_ =
      this->create_publisher<sensor_msgs::msg::PointCloud2>(
          "/corrected_current_pcd_localization", 10);
  map_match_pub_ = this->create_publisher<visualization_msgs::msg::Marker>(
      "/map_match_localization", 10);
  realtime_pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
      "/pose_stamped_localization", 10);
  saved_map_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
      "/saved_map_localization", 10);
  debug_src_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
      "/src_localization", 10);
  debug_dst_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
      "/dst_localization", 10);
  debug_coarse_aligned_pub_ =
      this->create_publisher<sensor_msgs::msg::PointCloud2>(
          "/coarse_aligned_quatro_localization", 10);
  debug_fine_aligned_pub_ =
      this->create_publisher<sensor_msgs::msg::PointCloud2>(
          "/fine_aligned_nano_gicp_localization", 10);
  map_to_odom_pub_ =
      this->create_publisher<nav_msgs::msg::Odometry>("/map_to_odom", 10);
  localization_odom_pub_ =
      this->create_publisher<nav_msgs::msg::Odometry>("/localization", 10);
  /* subscribers */
  sub_odom_ =
      std::make_shared<message_filters::Subscriber<nav_msgs::msg::Odometry>>();
  sub_pcd_ = std::make_shared<
      message_filters::Subscriber<sensor_msgs::msg::PointCloud2>>();

  sub_odom_->subscribe(this, "/Odometry");
  sub_pcd_->subscribe(this, "/cloud_registered");

  sub_odom_pcd_sync_ =
      std::make_shared<message_filters::Synchronizer<odom_pcd_sync_pol>>(
          odom_pcd_sync_pol(10), *sub_odom_, *sub_pcd_);

  sub_odom_pcd_sync_->registerCallback(
      std::bind(&FastLioLocalizationScQn::odomPcdCallback, this,
                std::placeholders::_1, std::placeholders::_2));

  // -----------------------
  // 5. Timer
  // -----------------------
  match_timer_ = this->create_wall_timer(
      std::chrono::duration<double>(1.0 / map_match_hz),
      std::bind(&FastLioLocalizationScQn::matchingTimerFunc, this));

  // -----------------------
  // 6. TF Broadcaster
  // -----------------------
  broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

  RCLCPP_WARN(this->get_logger(), "FastLioLocalizationScQn node started...");
}
// ----------------------
// odom+pcd 同步回调
// ----------------------
void FastLioLocalizationScQn::odomPcdCallback(
    const nav_msgs::msg::Odometry::ConstSharedPtr &odom_msg,
    const sensor_msgs::msg::PointCloud2::ConstSharedPtr &pcd_msg) {
  PosePcd current_frame(*odom_msg, *pcd_msg, current_keyframe_idx_);

  // 1. 实时 pose = last TF * odom
  current_frame.pose_corrected_eig_ =
      last_corrected_TF_ * current_frame.pose_eig_;
  geometry_msgs::msg::PoseStamped current_pose_stamped =
      poseEigToPoseStamped(current_frame.pose_corrected_eig_, map_frame_);
  realtime_pose_pub_->publish(current_pose_stamped);

  nav_msgs::msg::Odometry corrected_odom;
  corrected_odom.header.stamp = odom_msg->header.stamp;
  corrected_odom.header.frame_id = map_frame_;
  corrected_odom.child_frame_id = odom_msg->child_frame_id;

  corrected_odom.pose.pose = current_pose_stamped.pose;
  corrected_odom.twist = odom_msg->twist;

  // 设置协方差
  for (int i = 0; i < 36; i++) corrected_odom.pose.covariance[i] = 0.0;
  corrected_odom.pose.covariance[0] = 0.01;
  corrected_odom.pose.covariance[7] = 0.01;
  corrected_odom.pose.covariance[14] = 0.01;
  corrected_odom.pose.covariance[21] = 0.01;
  corrected_odom.pose.covariance[28] = 0.01;
  corrected_odom.pose.covariance[35] = 0.01;

  for (int i = 0; i < 36; i++)
    corrected_odom.twist.covariance[i] = odom_msg->twist.covariance[i];

  localization_odom_pub_->publish(corrected_odom);

  // 发布点云
  corrected_current_pcd_pub_->publish(pclToPclRos(
      transformPcd(current_frame.pcd_, current_frame.pose_corrected_eig_),
      map_frame_));

  if (!is_initialized_) {
    std::lock_guard<std::mutex> lock(keyframes_mutex_);
    last_keyframe_ = current_frame;
    current_keyframe_idx_++;

    {
      std::lock_guard<std::mutex> lock2(vis_mutex_);
      updateOdomsAndPaths(current_frame);
    }

    publishMapToOdom(this->now());
    is_initialized_ = true;
    return;
  }

  // 检查 keyframe
  //   if (checkIfKeyframe(current_frame, last_keyframe_)) {
  std::lock_guard<std::mutex> lock(keyframes_mutex_);
  last_keyframe_ = current_frame;
  current_keyframe_idx_++;

  std::lock_guard<std::mutex> lock2(vis_mutex_);
  updateOdomsAndPaths(current_frame);
  //   }
}

// ----------------------
// map match 定时器函数
// ----------------------
void FastLioLocalizationScQn::matchingTimerFunc() {
  if (!is_initialized_) return;

  auto t1 = high_resolution_clock::now();

  PosePcd last_keyframe_copy;
  {
    std::lock_guard<std::mutex> lock(keyframes_mutex_);
    last_keyframe_copy = last_keyframe_;
    last_keyframe_.processed_ = true;
  }
  if (last_keyframe_copy.idx_ == 0 || last_keyframe_copy.processed_) return;

  int closest_keyframe_idx = map_matcher_->fetchClosestKeyframeIdx(
      last_keyframe_copy, saved_map_from_bag_);
  if (closest_keyframe_idx < 0) return;

  const RegistrationOutput &reg_output = map_matcher_->performMapMatcher(
      last_keyframe_copy, saved_map_from_bag_, closest_keyframe_idx);

  if (reg_output.is_valid_) {
    RCLCPP_INFO(this->get_logger(), "Map matching accepted. Score: %.3f",
                reg_output.score_);
    last_corrected_TF_ = reg_output.pose_between_eig_ * last_corrected_TF_;
    Eigen::Matrix4d TFed_pose =
        reg_output.pose_between_eig_ * last_keyframe_copy.pose_corrected_eig_;

    std::lock_guard<std::mutex> lock(vis_mutex_);
    corrected_odoms_.points[last_keyframe_copy.idx_] =
        pcl::PointXYZ(TFed_pose(0, 3), TFed_pose(1, 3), TFed_pose(2, 3));
    corrected_odom_path_.poses[last_keyframe_copy.idx_] =
        poseEigToPoseStamped(TFed_pose, map_frame_);
    matched_pairs_xyz_.push_back(
        {corrected_odoms_.points[last_keyframe_copy.idx_],
         raw_odoms_.points[last_keyframe_copy.idx_]});
    map_match_pub_->publish(getMatchMarker(matched_pairs_xyz_));
    publishMapToOdom(this->now());
  }

  // 发布 debug 点云
  debug_src_pub_->publish(
      pclToPclRos(map_matcher_->getSourceCloud(), map_frame_));
  debug_dst_pub_->publish(
      pclToPclRos(map_matcher_->getTargetCloud(), map_frame_));
  debug_coarse_aligned_pub_->publish(
      pclToPclRos(map_matcher_->getCoarseAlignedCloud(), map_frame_));
  debug_fine_aligned_pub_->publish(
      pclToPclRos(map_matcher_->getFinalAlignedCloud(), map_frame_));

  std::lock_guard<std::mutex> lock(vis_mutex_);
  corrected_odom_pub_->publish(pclToPclRos(corrected_odoms_, map_frame_));
  corrected_path_pub_->publish(corrected_odom_path_);
  odom_pub_->publish(pclToPclRos(raw_odoms_, map_frame_));
  path_pub_->publish(raw_odom_path_);

  if (saved_map_vis_switch_ && saved_map_pub_->get_subscription_count() > 0) {
    saved_map_pub_->publish(pclToPclRos(saved_map_pcd_, map_frame_));
    saved_map_vis_switch_ = false;
  }
  if (!saved_map_vis_switch_ && saved_map_pub_->get_subscription_count() == 0) {
    saved_map_vis_switch_ = true;
  }

  auto t2 = high_resolution_clock::now();
  RCLCPP_INFO(this->get_logger(), "Matching: %.1fms",
              duration_cast<microseconds>(t2 - t1).count() / 1e3);
}

// ----------------------
// 发布 map->odom
// ----------------------
void FastLioLocalizationScQn::publishMapToOdom(const rclcpp::Time &stamp) {
  nav_msgs::msg::Odometry map_to_odom;

  // header
  map_to_odom.header.stamp = stamp;
  map_to_odom.header.frame_id = map_frame_;
  map_to_odom.child_frame_id = "odom";

  // 将Eigen变换矩阵转换为Pose消息
  Eigen::Affine3d affine(last_corrected_TF_.cast<double>());
  geometry_msgs::msg::Pose pose_msg = tf2::toMsg(affine);

  map_to_odom.pose.pose = pose_msg;

  // 设置协方差矩阵
  for (int i = 0; i < 36; i++) {
    map_to_odom.pose.covariance[i] = 0.0;
  }

  // 设置对角线元素为较小的不确定性
  map_to_odom.pose.covariance[0] = 0.01;   // x
  map_to_odom.pose.covariance[7] = 0.01;   // y
  map_to_odom.pose.covariance[14] = 0.01;  // z
  map_to_odom.pose.covariance[21] = 0.01;  // roll
  map_to_odom.pose.covariance[28] = 0.01;  // pitch
  map_to_odom.pose.covariance[35] = 0.01;  // yaw

  // 发布
  map_to_odom_pub_->publish(map_to_odom);

  RCLCPP_INFO(
      this->get_logger(), "Published map->odom transform: [%.3f, %.3f, %.3f]",
      map_to_odom.pose.pose.position.x, map_to_odom.pose.pose.position.y,
      map_to_odom.pose.pose.position.z);
}

// ----------------------
// 更新 odom 和 path
// ----------------------
void FastLioLocalizationScQn::updateOdomsAndPaths(const PosePcd &pose_pcd_in) {
  raw_odoms_.points.emplace_back(pose_pcd_in.pose_eig_(0, 3),
                                 pose_pcd_in.pose_eig_(1, 3),
                                 pose_pcd_in.pose_eig_(2, 3));
  corrected_odoms_.points.emplace_back(pose_pcd_in.pose_corrected_eig_(0, 3),
                                       pose_pcd_in.pose_corrected_eig_(1, 3),
                                       pose_pcd_in.pose_corrected_eig_(2, 3));

  raw_odom_path_.poses.emplace_back(
      poseEigToPoseStamped(pose_pcd_in.pose_eig_, map_frame_));
  corrected_odom_path_.poses.emplace_back(
      poseEigToPoseStamped(pose_pcd_in.pose_corrected_eig_, map_frame_));
}

// ----------------------
// 匹配 marker
// ----------------------
visualization_msgs::msg::Marker FastLioLocalizationScQn::getMatchMarker(
    const std::vector<std::pair<pcl::PointXYZ, pcl::PointXYZ>>
        &match_xyz_pairs) {
  visualization_msgs::msg::Marker edges;
  edges.type = visualization_msgs::msg::Marker::LINE_LIST;
  edges.scale.x = 0.2f;
  edges.header.frame_id = map_frame_;
  edges.pose.orientation.w = 1.0f;
  edges.color.r = 1.0f;
  edges.color.g = 1.0f;
  edges.color.b = 1.0f;
  edges.color.a = 1.0f;

  for (const auto &p : match_xyz_pairs) {
    geometry_msgs::msg::Point pt1, pt2;
    pt1.x = p.first.x;
    pt1.y = p.first.y;
    pt1.z = p.first.z;
    pt2.x = p.second.x;
    pt2.y = p.second.y;
    pt2.z = p.second.z;
    edges.points.push_back(pt1);
    edges.points.push_back(pt2);
  }
  return edges;
}

// ----------------------
// 判断 keyframe
// ----------------------
bool FastLioLocalizationScQn::checkIfKeyframe(const PosePcd &pose_pcd_in,
                                              const PosePcd &latest_pose_pcd) {
  Eigen::Vector3d diff = latest_pose_pcd.pose_corrected_eig_.block<3, 1>(0, 3) -
                         pose_pcd_in.pose_corrected_eig_.block<3, 1>(0, 3);
  return diff.norm() > keyframe_dist_thr_;
}

// ----------------------
// 加载 map
// ----------------------
void FastLioLocalizationScQn::loadMap(const std::string &saved_map_path) {
  // 创建 rosbag2 reader
  rosbag2_cpp::Reader reader;
  reader.open(saved_map_path);  // ROS 2 bag 文件通常是 *.db3

  std::vector<sensor_msgs::msg::PointCloud2> load_pcd_vec;
  std::vector<geometry_msgs::msg::PoseStamped> load_pose_vec;

  // rclcpp 序列化对象，用于反序列化消息
  rclcpp::Serialization<sensor_msgs::msg::PointCloud2> pcd_serializer;
  rclcpp::Serialization<geometry_msgs::msg::PoseStamped> pose_serializer;

  // 遍历 bag 中的所有消息
  while (reader.has_next()) {
    auto bag_msg = reader.read_next();

    if (bag_msg->topic_name == "/keyframe_pcd") {
      sensor_msgs::msg::PointCloud2 msg;
      rclcpp::SerializedMessage serialized_msg(*bag_msg->serialized_data);
      pcd_serializer.deserialize_message(&serialized_msg, &msg);
      load_pcd_vec.push_back(msg);
    } else if (bag_msg->topic_name == "/keyframe_pose") {
      geometry_msgs::msg::PoseStamped msg;
      rclcpp::SerializedMessage serialized_msg(*bag_msg->serialized_data);
      pose_serializer.deserialize_message(&serialized_msg, &msg);
      load_pose_vec.push_back(msg);
    }
  }

  if (load_pcd_vec.size() != load_pose_vec.size()) {
    RCLCPP_ERROR(this->get_logger(), "WRONG BAG FILE!!!!!");
    return;
  }

  // 将 Pose 和 PCD 结合成 saved_map_from_bag_
  for (size_t i = 0; i < load_pose_vec.size(); ++i) {
    saved_map_from_bag_.push_back(
        PosePcdReduced(load_pose_vec[i], load_pcd_vec[i], i));

    // transform 点云到全局坐标
    saved_map_pcd_ += transformPcd(saved_map_from_bag_[i].pcd_,
                                   saved_map_from_bag_[i].pose_eig_);

    // 更新 Scan Context，用于 loop candidate detection
    map_matcher_->updateScancontext(saved_map_from_bag_[i].pcd_);
  }

  // voxelize 降采样
  saved_map_pcd_ = *voxelizePcd(saved_map_pcd_, voxel_res_);

  RCLCPP_INFO(this->get_logger(), "Loaded map with %zu keyframes",
              load_pose_vec.size());
}