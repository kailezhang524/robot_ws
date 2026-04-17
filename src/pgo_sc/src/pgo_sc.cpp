#include "pgo_sc.hpp"

#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>

#include <filesystem>
#include <fstream>
#include <iomanip>
#include <sstream>

using namespace std::chrono_literals;
namespace fs = std::filesystem;
PgoSc::PgoSc()
    : Node("pgo_sc"),
      is_initialized_(false),
      loop_added_flag_(false),
      loop_added_flag_vis_(false) {
  ////// ROS2 params
  double loop_update_hz, vis_hz;
  LoopClosureConfig lc_config;
  auto &gc = lc_config.gicp_config_;
  auto &qc = lc_config.quatro_config_;

  /* declare parameters */
  this->declare_parameter<std::string>("basic.map_frame", "odom");
  this->declare_parameter<double>("basic.loop_update_hz", 1.0);
  this->declare_parameter<double>("basic.vis_hz", 0.5);
  this->declare_parameter<double>("save_voxel_resolution", 0.3);
  this->declare_parameter<double>("quatro_nano_gicp_voxel_resolution", 0.3);

  this->declare_parameter<double>("keyframe.keyframe_threshold", 1.0);
  this->declare_parameter<int>("keyframe.nusubmap_keyframes", 5);
  this->declare_parameter<bool>("keyframe.enable_submap_matching", false);

  this->declare_parameter<double>("scancontext_max_correspondence_distance",
                                  35.0);

  this->declare_parameter<int>("nano_gicp.thread_number", 0);
  this->declare_parameter<double>("nano_gicp.icp_score_threshold", 10.0);
  this->declare_parameter<int>("nano_gicp.correspondences_number", 15);
  this->declare_parameter<double>("nano_gicp.max_correspondence_distance",
                                  0.01);
  this->declare_parameter<int>("nano_gicp.max_iter", 32);
  this->declare_parameter<double>("nano_gicp.transformation_epsilon", 0.01);
  this->declare_parameter<double>("nano_gicp.euclidean_fitness_epsilon", 0.01);
  this->declare_parameter<int>("nano_gicp.ransac.max_iter", 5);
  this->declare_parameter<double>(
      "nano_gicp.ransac.outlier_rejection_threshold", 1.0);

  this->declare_parameter<bool>("quatro.enable", false);
  this->declare_parameter<bool>("quatro.optimize_matching", true);
  this->declare_parameter<double>("quatro.distance_threshold", 30.0);
  this->declare_parameter<int>("quatro.max_nucorrespondences", 200);
  this->declare_parameter<double>("quatro.fpfh_normal_radius", 0.3);
  this->declare_parameter<double>("quatro.fpfh_radius", 0.5);
  this->declare_parameter<bool>("quatro.estimating_scale", false);
  this->declare_parameter<double>("quatro.noise_bound", 0.3);
  this->declare_parameter<double>("quatro.rotation.gnc_factor", 1.4);
  this->declare_parameter<double>("quatro.rotation.rot_cost_diff_threshold",
                                  0.0001);
  this->declare_parameter<int>("quatro.rotation.numax_iter", 50);

  this->declare_parameter<bool>("result.save_map_bag", false);
  this->declare_parameter<bool>("result.save_map_pcd", false);
  this->declare_parameter<bool>("result.save_in_kitti_format", false);
  this->declare_parameter<std::string>("result.seq_name", "");

  /* get parameters */
  this->get_parameter("basic.map_frame", map_frame_);
  this->get_parameter("basic.loop_update_hz", loop_update_hz);
  this->get_parameter("basic.vis_hz", vis_hz);
  this->get_parameter("save_voxel_resolution", voxel_res_);
  this->get_parameter("quatro_nano_gicp_voxel_resolution",
                      lc_config.voxel_res_);

  this->get_parameter("keyframe.keyframe_threshold", keyframe_thr_);
  this->get_parameter("keyframe.nusubmap_keyframes",
                      lc_config.num_submap_keyframes_);
  this->get_parameter("keyframe.enable_submap_matching",
                      lc_config.enable_submap_matching_);

  this->get_parameter("scancontext_max_correspondence_distance",
                      lc_config.scancontext_max_correspondence_distance_);

  this->get_parameter("nano_gicp.thread_number", gc.nano_thread_number_);
  this->get_parameter("nano_gicp.icp_score_threshold", gc.icp_score_thr_);
  this->get_parameter("nano_gicp.correspondences_number",
                      gc.nano_correspondences_number_);
  this->get_parameter("nano_gicp.max_correspondence_distance",
                      gc.max_corr_dist_);
  this->get_parameter("nano_gicp.max_iter", gc.nano_max_iter_);
  this->get_parameter("nano_gicp.transformation_epsilon",
                      gc.transformation_epsilon_);
  this->get_parameter("nano_gicp.euclidean_fitness_epsilon",
                      gc.euclidean_fitness_epsilon_);
  this->get_parameter("nano_gicp.ransac.max_iter", gc.nano_ransac_max_iter_);
  this->get_parameter("nano_gicp.ransac.outlier_rejection_threshold",
                      gc.ransac_outlier_rejection_threshold_);

  this->get_parameter("quatro.enable", lc_config.enable_quatro_);
  this->get_parameter("quatro.optimize_matching", qc.use_optimized_matching_);
  this->get_parameter("quatro.distance_threshold",
                      qc.quatro_distance_threshold_);
  this->get_parameter("quatro.max_nucorrespondences",
                      qc.quatro_max_num_corres_);
  this->get_parameter("quatro.fpfh_normal_radius", qc.fpfh_normal_radius_);
  this->get_parameter("quatro.fpfh_radius", qc.fpfh_radius_);
  this->get_parameter("quatro.estimating_scale", qc.estimat_scale_);
  this->get_parameter("quatro.noise_bound", qc.noise_bound_);
  this->get_parameter("quatro.rotation.gnc_factor", qc.rot_gnc_factor_);
  this->get_parameter("quatro.rotation.rot_cost_diff_threshold",
                      qc.rot_cost_diff_thr_);
  this->get_parameter("quatro.rotation.numax_iter", qc.quatro_max_iter_);

  this->get_parameter("result.save_map_bag", save_map_bag_);
  this->get_parameter("result.save_map_pcd", save_map_pcd_);
  this->get_parameter("result.save_in_kitti_format", save_in_kitti_format_);
  this->get_parameter("result.seq_name", seq_name_);

  /* loop closure */
  loop_closure_ = std::make_shared<LoopClosure>(lc_config);

  /* Initialization of GTSAM */
  gtsam::ISAM2Params isam_params;
  isam_params.relinearizeThreshold = 0.01;
  isam_params.relinearizeSkip = 1;
  isam_handler_ = std::make_shared<gtsam::ISAM2>(isam_params);

  /* ROS things */
  odom_path_.header.frame_id = map_frame_;
  corrected_path_.header.frame_id = map_frame_;
  package_path_ = ament_index_cpp::get_package_share_directory("pgo_sc");

  /* QoS */
  auto qos_latched =
      rclcpp::QoS(rclcpp::KeepLast(10)).reliable().transient_local();
  auto qos_normal = rclcpp::QoS(rclcpp::KeepLast(10));

  /* publishers */
  broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
  odom_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
      "/ori_odom", qos_latched);
  path_pub_ =
      this->create_publisher<nav_msgs::msg::Path>("/ori_path", qos_latched);
  corrected_odom_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
      "/corrected_odom", qos_latched);
  corrected_path_pub_ = this->create_publisher<nav_msgs::msg::Path>(
      "/corrected_path", qos_latched);
  corrected_pcd_map_pub_ =
      this->create_publisher<sensor_msgs::msg::PointCloud2>("/corrected_map",
                                                            qos_latched);
  corrected_current_pcd_pub_ =
      this->create_publisher<sensor_msgs::msg::PointCloud2>(
          "/corrected_current_pcd", qos_latched);
  loop_detection_pub_ = this->create_publisher<visualization_msgs::msg::Marker>(
      "/loop_detection", qos_latched);
  realtime_pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
      "/pose_stamped", qos_normal);
  debug_src_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
      "/src", qos_latched);
  debug_dst_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
      "/dst", qos_latched);
  debug_coarse_aligned_pub_ =
      this->create_publisher<sensor_msgs::msg::PointCloud2>(
          "/coarse_aligned_quatro", qos_latched);
  debug_fine_aligned_pub_ =
      this->create_publisher<sensor_msgs::msg::PointCloud2>(
          "/fine_aligned_nano_gicp", qos_latched);
  odom_to_baselink_pub_ =
      this->create_publisher<nav_msgs::msg::Odometry>("/slam", qos_normal);

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

  sub_odom_pcd_sync_->registerCallback(std::bind(&PgoSc::odomPcdCallback, this,
                                                 std::placeholders::_1,
                                                 std::placeholders::_2));

  sub_save_flag_ = this->create_subscription<std_msgs::msg::String>(
      "/save_dir", rclcpp::QoS(1),
      std::bind(&PgoSc::saveFlagCallback, this, std::placeholders::_1));

  /* timers */
  loop_timer_ = this->create_wall_timer(
      std::chrono::duration<double>(1.0 / loop_update_hz),
      std::bind(&PgoSc::loopTimerFunc, this));

  vis_timer_ =
      this->create_wall_timer(std::chrono::duration<double>(1.0 / vis_hz),
                              std::bind(&PgoSc::visTimerFunc, this));

  RCLCPP_INFO(this->get_logger(), "Main class, starting node...");
}

// ====================== odom + pcd callback ======================
// 功能：前端位姿估计 + 关键帧管理 + 后端图优化
void PgoSc::odomPcdCallback(
    const nav_msgs::msg::Odometry::ConstSharedPtr &odom_msg,
    const sensor_msgs::msg::PointCloud2::ConstSharedPtr &pcd_msg) {
  // 保存上一帧 odom，用于计算 delta
  Eigen::Matrix4d last_odom_tf = current_frame_.pose_eig_;
  current_frame_ =
      PosePcd(*odom_msg, *pcd_msg, current_keyframe_idx_);  // 构建当前帧

  auto t1 = std::chrono::high_resolution_clock::now();

  // ==== 1. 实时位姿计算 ====
  {
    std::lock_guard<std::mutex> lock(realtime_pose_mutex_);
    odom_delta_ =
        odom_delta_ * last_odom_tf.inverse() * current_frame_.pose_eig_;
    current_frame_.pose_corrected_eig_ = last_corrected_pose_ * odom_delta_;

    // ROS2 发布 PoseStamped
    geometry_msgs::msg::PoseStamped pose_msg =
        poseEigToPoseStamped(current_frame_.pose_corrected_eig_, map_frame_);
    realtime_pose_pub_->publish(pose_msg);
    geometry_msgs::msg::TransformStamped tf_msg;
    tf_msg.header.stamp = this->now();
    tf_msg.header.frame_id = map_frame_;
    tf_msg.child_frame_id = "robot";
    tf_msg.transform = poseEigToROSTf(current_frame_.pose_corrected_eig_);

    broadcaster_->sendTransform(tf_msg);
  }
  // 发布当前校正点云
  corrected_current_pcd_pub_->publish(pclToPclRos(
      transformPcd(current_frame_.pcd_, current_frame_.pose_corrected_eig_),
      map_frame_));

  // ==== 初始化关键帧（只执行一次） ====
  if (!is_initialized_) {
    keyframes_.push_back(current_frame_);
    updateOdomsAndPaths(current_frame_);

    // 添加 PriorFactor 到图
    auto variance_vector =
        (gtsam::Vector(6) << 1e-4, 1e-4, 1e-4, 1e-2, 1e-2, 1e-2).finished();
    auto prior_noise = gtsam::noiseModel::Diagonal::Variances(variance_vector);
    gtsam_graph_.add(gtsam::PriorFactor<gtsam::Pose3>(
        0, poseEigToGtsamPose(current_frame_.pose_eig_), prior_noise));
    init_esti_.insert(current_keyframe_idx_,
                      poseEigToGtsamPose(current_frame_.pose_eig_));
    current_keyframe_idx_++;

    // 更新 ScanContext
    loop_closure_->updateScancontext(current_frame_.pcd_);

    is_initialized_ = true;
    return;  // 初始化完成直接返回
  }

  // ==== 2. 判断是否是新关键帧 ====
  auto t2 = std::chrono::high_resolution_clock::now();
  if (checkIfKeyframe(current_frame_, keyframes_.back())) {
    {
      std::lock_guard<std::mutex> lock(keyframes_mutex_);
      keyframes_.push_back(current_frame_);
    }

    // ==== 3. 添加到图优化 ====
    auto variance_vector =
        (gtsam::Vector(6) << 1e-4, 1e-4, 1e-4, 1e-2, 1e-2, 1e-2).finished();
    auto odom_noise = gtsam::noiseModel::Diagonal::Variances(variance_vector);

    gtsam::Pose3 pose_from = poseEigToGtsamPose(
        keyframes_[current_keyframe_idx_ - 1].pose_corrected_eig_);
    gtsam::Pose3 pose_to =
        poseEigToGtsamPose(current_frame_.pose_corrected_eig_);
    {
      std::lock_guard<std::mutex> lock(graph_mutex_);
      gtsam_graph_.add(gtsam::BetweenFactor<gtsam::Pose3>(
          current_keyframe_idx_ - 1, current_keyframe_idx_,
          pose_from.between(pose_to), odom_noise));
      init_esti_.insert(current_keyframe_idx_, pose_to);
    }

    current_keyframe_idx_++;

    // ==== 更新 ScanContext ====
    loop_closure_->updateScancontext(current_frame_.pcd_);

    // ==== 4. 更新可视化 ====
    auto t3 = std::chrono::high_resolution_clock::now();
    {
      std::lock_guard<std::mutex> lock(vis_mutex_);
      updateOdomsAndPaths(current_frame_);
    }

    // ==== 5. 图优化（ISAM2） ====
    auto t4 = std::chrono::high_resolution_clock::now();
    {
      std::lock_guard<std::mutex> lock(graph_mutex_);
      isam_handler_->update(gtsam_graph_, init_esti_);
      isam_handler_->update();

      if (loop_added_flag_) {
        // 处理回环
        isam_handler_->update();
      }

      gtsam_graph_.resize(0);
      init_esti_.clear();
    }

    // ==== 6. 更新校正位姿 ====
    auto t5 = std::chrono::high_resolution_clock::now();
    {
      std::lock_guard<std::mutex> lock(realtime_pose_mutex_);
      corrected_esti_ = isam_handler_->calculateEstimate();
      last_corrected_pose_ = gtsamPoseToPoseEig(
          corrected_esti_.at<gtsam::Pose3>(corrected_esti_.size() - 1));
      odom_delta_ = Eigen::Matrix4d::Identity();
    }

    if (loop_added_flag_) {
      std::lock_guard<std::mutex> lock(keyframes_mutex_);
      for (size_t i = 0; i < corrected_esti_.size(); ++i) {
        keyframes_[i].pose_corrected_eig_ =
            gtsamPoseToPoseEig(corrected_esti_.at<gtsam::Pose3>(i));
      }
      loop_added_flag_ = false;
    }

    // ==== 7. ROS2 中可以省略详细时间统计，保留简单日志 ====
    auto t6 = std::chrono::high_resolution_clock::now();
    RCLCPP_INFO(
        this->get_logger(), "Keyframe added, total time: %.1f ms",
        std::chrono::duration_cast<std::chrono::milliseconds>(t6 - t1).count());
  }
}
// ====================== loop closure timer ======================
void PgoSc::loopTimerFunc() {
  if (!is_initialized_ || keyframes_.empty()) return;

  auto &latest_keyframe = keyframes_.back();
  if (latest_keyframe.processed_) return;
  latest_keyframe.processed_ = true;

  auto t1 = std::chrono::high_resolution_clock::now();

  // ==== 1. 查找候选关键帧进行 loop closure ====
  int closest_keyframe_idx =
      loop_closure_->fetchCandidateKeyframeIdx(latest_keyframe, keyframes_);
  if (closest_keyframe_idx < 0) return;

  RegistrationOutput reg_output = loop_closure_->performLoopClosure(
      latest_keyframe, keyframes_, closest_keyframe_idx);

  // ==== 2. 如果 loop closure 有效，加入图优化 ====
  if (reg_output.is_valid_) {
    RCLCPP_INFO(this->get_logger(),
                "\033[1;32mLoop closure accepted. Score: %.3f\033[0m",
                reg_output.score_);

    gtsam::Pose3 pose_from = poseEigToGtsamPose(
        reg_output.pose_between_eig_ * latest_keyframe.pose_corrected_eig_);
    gtsam::Pose3 pose_to = poseEigToGtsamPose(
        keyframes_[closest_keyframe_idx].pose_corrected_eig_);

    auto variance_vector =
        (gtsam::Vector(6) << reg_output.score_, reg_output.score_,
         reg_output.score_, reg_output.score_, reg_output.score_,
         reg_output.score_)
            .finished();

    auto loop_noise = gtsam::noiseModel::Diagonal::Variances(variance_vector);

    {
      std::lock_guard<std::mutex> lock(graph_mutex_);
      gtsam_graph_.add(gtsam::BetweenFactor<gtsam::Pose3>(
          latest_keyframe.idx_, closest_keyframe_idx,
          pose_from.between(pose_to), loop_noise));
    }

    // 保存 loop closure 信息用于可视化
    loop_idx_pairs_.push_back({latest_keyframe.idx_, closest_keyframe_idx});
    loop_added_flag_vis_ = true;
    loop_added_flag_ = true;
  } else {
    RCLCPP_WARN(this->get_logger(), "Loop closure rejected. Score: %.3f",
                reg_output.score_);
  }

  auto t2 = std::chrono::high_resolution_clock::now();

  // ==== 3. 发布调试点云（debug） ====
  debug_src_pub_->publish(
      pclToPclRos(loop_closure_->getSourceCloud(), map_frame_));
  debug_dst_pub_->publish(
      pclToPclRos(loop_closure_->getTargetCloud(), map_frame_));
  debug_fine_aligned_pub_->publish(
      pclToPclRos(loop_closure_->getFinalAlignedCloud(), map_frame_));
  debug_coarse_aligned_pub_->publish(
      pclToPclRos(loop_closure_->getCoarseAlignedCloud(), map_frame_));

  // ==== 4. 输出循环耗时信息 ====
  RCLCPP_INFO(
      this->get_logger(), "loop: %.1f ms",
      std::chrono::duration_cast<std::chrono::microseconds>(t2 - t1).count() /
          1e3);
}

// ====================== visualization timer ======================
void PgoSc::visTimerFunc() {
  if (!is_initialized_) return;

  auto tv1 = std::chrono::high_resolution_clock::now();

  //// 1. 如果 loop closure 更新，处理可视化数据
  if (loop_added_flag_vis_) {
    gtsam::Values corrected_esti_copied;
    pcl::PointCloud<pcl::PointXYZ> corrected_odoms;
    nav_msgs::msg::Path corrected_path;  // ROS2 使用 msg 命名空间

    // 拷贝实时估计，避免多线程冲突
    {
      std::lock_guard<std::mutex> lock(realtime_pose_mutex_);
      corrected_esti_copied = corrected_esti_;
    }

    // 遍历估计结果生成校正轨迹
    for (size_t i = 0; i < corrected_esti_copied.size(); ++i) {
      gtsam::Pose3 pose_ = corrected_esti_copied.at<gtsam::Pose3>(i);
      corrected_odoms.points.emplace_back(pose_.translation().x(),
                                          pose_.translation().y(),
                                          pose_.translation().z());
      corrected_path.poses.push_back(gtsamPoseToPoseStamped(pose_, map_frame_));
    }

    // 发布 loop closure 约束 Marker
    if (!loop_idx_pairs_.empty()) {
      loop_detection_pub_->publish(getLoopMarkers(corrected_esti_copied));
    }

    // 更新全局可视化数据
    {
      std::lock_guard<std::mutex> lock(vis_mutex_);
      corrected_odoms_ = corrected_odoms;
      corrected_path_.poses = corrected_path.poses;
    }

    loop_added_flag_vis_ = false;
  }

  //// 2. 发布 odom、路径
  {
    std::lock_guard<std::mutex> lock(vis_mutex_);
    odom_pub_->publish(pclToPclRos(odoms_, map_frame_));
    path_pub_->publish(odom_path_);
    corrected_odom_pub_->publish(pclToPclRos(corrected_odoms_, map_frame_));
    corrected_path_pub_->publish(corrected_path_);
  }

  //// 3. 发布全局地图
  if (global_map_vis_switch_ &&
      corrected_pcd_map_pub_->get_subscription_count() >
          0)  // ROS2 获取订阅者数量
  {
    pcl::PointCloud<PointType>::Ptr corrected_map(
        new pcl::PointCloud<PointType>());
    corrected_map->reserve(keyframes_[0].pcd_.size() * keyframes_.size());

    {
      std::lock_guard<std::mutex> lock(keyframes_mutex_);
      for (size_t i = 0; i < keyframes_.size(); ++i) {
        *corrected_map +=
            transformPcd(keyframes_[i].pcd_, keyframes_[i].pose_corrected_eig_);
      }
    }

    const auto &voxelized_map = voxelizePcd(corrected_map, voxel_res_);
    corrected_pcd_map_pub_->publish(pclToPclRos(*voxelized_map, map_frame_));
    global_map_vis_switch_ = false;
  }

  if (!global_map_vis_switch_ &&
      corrected_pcd_map_pub_->get_subscription_count() == 0) {
    global_map_vis_switch_ = true;
  }

  auto tv2 = std::chrono::high_resolution_clock::now();
  // ROS2 日志输出
  RCLCPP_INFO(
      this->get_logger(), "vis: %.1f ms",
      std::chrono::duration_cast<std::chrono::microseconds>(tv2 - tv1).count() /
          1e3);
}

// ====================== save callback ======================
void PgoSc::saveFlagCallback(const std_msgs::msg::String::SharedPtr msg) {
  // 获取保存目录，如果 msg->data 为空，则使用默认 package_path_
  std::string save_dir = !msg->data.empty() ? msg->data : package_path_;

  std::string seq_directory = save_dir + "/" + seq_name_;
  std::string scans_directory = seq_directory + "/scans";

  //// 1. 保存点云和位姿，KITTI / TUM 格式
  if (save_in_kitti_format_) {
    RCLCPP_INFO(this->get_logger(),
                "\033[32;1mScans are saved in %s, following the KITTI and TUM "
                "format\033[0m",
                scans_directory.c_str());

    // 删除已有目录并创建新目录
    if (fs::exists(seq_directory)) {
      fs::remove_all(seq_directory);
    }
    fs::create_directories(scans_directory);

    std::ofstream kitti_pose_file(seq_directory + "/poses_kitti.txt");
    std::ofstream tum_pose_file(seq_directory + "/poses_tum.txt");
    tum_pose_file << "#timestamp x y z qx qy qz qw\n";

    // 遍历关键帧保存点云和位姿
    {
      std::lock_guard<std::mutex> lock(keyframes_mutex_);
      for (size_t i = 0; i < keyframes_.size(); ++i) {
        // 保存 PCD
        std::stringstream ss_;
        ss_ << scans_directory << "/" << std::setw(6) << std::setfill('0') << i
            << ".pcd";
        RCLCPP_INFO(this->get_logger(), "Saving %s...", ss_.str().c_str());
        pcl::io::savePCDFileASCII<PointType>(ss_.str(), keyframes_[i].pcd_);

        // 保存 KITTI 格式位姿
        const auto &pose_ = keyframes_[i].pose_corrected_eig_;
        kitti_pose_file << pose_(0, 0) << " " << pose_(0, 1) << " "
                        << pose_(0, 2) << " " << pose_(0, 3) << " "
                        << pose_(1, 0) << " " << pose_(1, 1) << " "
                        << pose_(1, 2) << " " << pose_(1, 3) << " "
                        << pose_(2, 0) << " " << pose_(2, 1) << " "
                        << pose_(2, 2) << " " << pose_(2, 3) << "\n";

        // 保存 TUM 格式位姿
        const auto &lidar_optim_pose_ =
            poseEigToPoseStamped(keyframes_[i].pose_corrected_eig_);
        tum_pose_file << std::fixed << std::setprecision(8)
                      << keyframes_[i].timestamp_ << " "
                      << lidar_optim_pose_.pose.position.x << " "
                      << lidar_optim_pose_.pose.position.y << " "
                      << lidar_optim_pose_.pose.position.z << " "
                      << lidar_optim_pose_.pose.orientation.x << " "
                      << lidar_optim_pose_.pose.orientation.y << " "
                      << lidar_optim_pose_.pose.orientation.z << " "
                      << lidar_optim_pose_.pose.orientation.w << "\n";
      }
    }

    kitti_pose_file.close();
    tum_pose_file.close();
    RCLCPP_INFO(
        this->get_logger(),
        "\033[32;1mScans and poses saved in .pcd and KITTI format\033[0m");
  }

  //// 2. 保存 rosbag (ROS2 不再直接使用 rosbag，需改用 rosbag2 API)
  if (save_map_bag_) {
    RCLCPP_WARN(
        this->get_logger(),
        "save_map_bag_ not implemented in ROS2; use rosbag2 API instead.");
    // ROS1: 使用 rosbag::Bag
    // ROS2: rosbag2_cpp::Writer 可实现类似功能
  }

  //// 3. 保存累积地图 PCD
  if (save_map_pcd_) {
    pcl::PointCloud<PointType>::Ptr corrected_map(
        new pcl::PointCloud<PointType>());
    corrected_map->reserve(keyframes_[0].pcd_.size() * keyframes_.size());

    {
      std::lock_guard<std::mutex> lock(keyframes_mutex_);
      for (size_t i = 0; i < keyframes_.size(); ++i) {
        *corrected_map +=
            transformPcd(keyframes_[i].pcd_, keyframes_[i].pose_corrected_eig_);
      }
    }

    const auto &voxelized_map = voxelizePcd(corrected_map, voxel_res_);
    pcl::io::savePCDFileASCII<PointType>(
        seq_directory + "/" + seq_name_ + "_map.pcd", *voxelized_map);
    RCLCPP_INFO(this->get_logger(),
                "\033[32;1mAccumulated map cloud saved in .pcd format\033[0m");
  }
}

// ====================== Destructor ======================
PgoSc::~PgoSc() {
  if (save_map_bag_) {
    rosbag2_cpp::Writer writer;
    writer.open(package_path_ + "/result_bag");

    writer.create_topic({"/keyframe_pcd", "sensor_msgs/msg/PointCloud2",
                         rmw_get_serialization_format(), ""});

    writer.create_topic({"/keyframe_pose", "geometry_msgs/msg/PoseStamped",
                         rmw_get_serialization_format(), ""});

    rclcpp::Serialization<sensor_msgs::msg::PointCloud2> cloud_serializer;
    rclcpp::Serialization<geometry_msgs::msg::PoseStamped> pose_serializer;

    {
      std::lock_guard<std::mutex> lock(keyframes_mutex_);

      for (size_t i = 0; i < keyframes_.size(); ++i) {
        auto cloud_msg = pclToPclRos(keyframes_[i].pcd_, map_frame_);
        auto pose_msg =
            poseEigToPoseStamped(keyframes_[i].pose_corrected_eig_, map_frame_);

        const auto stamp_ns = static_cast<rcutils_time_point_value_t>(
            keyframes_[i].timestamp_ * 1e9);

        rclcpp::Time stamp(stamp_ns);
        cloud_msg.header.stamp = stamp;
        pose_msg.header.stamp = stamp;

        auto serialized_cloud = std::make_shared<rclcpp::SerializedMessage>();
        auto serialized_pose = std::make_shared<rclcpp::SerializedMessage>();

        cloud_serializer.serialize_message(&cloud_msg, serialized_cloud.get());
        pose_serializer.serialize_message(&pose_msg, serialized_pose.get());

        auto bag_cloud_msg =
            std::make_shared<rosbag2_storage::SerializedBagMessage>();
        bag_cloud_msg->topic_name = "/keyframe_pcd";
        bag_cloud_msg->time_stamp = stamp_ns;
        bag_cloud_msg->serialized_data = std::shared_ptr<rcutils_uint8_array_t>(
            new rcutils_uint8_array_t, [](rcutils_uint8_array_t *msg) {
              rcutils_uint8_array_fini(msg);
              delete msg;
            });
        *bag_cloud_msg->serialized_data =
            serialized_cloud->release_rcl_serialized_message();

        auto bag_pose_msg =
            std::make_shared<rosbag2_storage::SerializedBagMessage>();
        bag_pose_msg->topic_name = "/keyframe_pose";
        bag_pose_msg->time_stamp = stamp_ns;
        bag_pose_msg->serialized_data = std::shared_ptr<rcutils_uint8_array_t>(
            new rcutils_uint8_array_t, [](rcutils_uint8_array_t *msg) {
              rcutils_uint8_array_fini(msg);
              delete msg;
            });
        *bag_pose_msg->serialized_data =
            serialized_pose->release_rcl_serialized_message();

        writer.write(bag_cloud_msg);
        writer.write(bag_pose_msg);
      }
    }

    RCLCPP_INFO(this->get_logger(),
                "\033[36;1mResult saved in ROS2 bag format!!!\033[0m");
  }

  //// 2. 保存累积地图 PCD
  if (save_map_pcd_) {
    pcl::PointCloud<PointType>::Ptr corrected_map(
        new pcl::PointCloud<PointType>());
    corrected_map->reserve(keyframes_[0].pcd_.size() * keyframes_.size());

    {
      std::lock_guard<std::mutex> lock(keyframes_mutex_);
      for (size_t i = 0; i < keyframes_.size(); ++i) {
        *corrected_map +=
            transformPcd(keyframes_[i].pcd_, keyframes_[i].pose_corrected_eig_);
      }
    }

    const auto &voxelized_map = voxelizePcd(corrected_map, voxel_res_);
    pcl::io::savePCDFileASCII<PointType>(package_path_ + "/result.pcd",
                                         *voxelized_map);

    RCLCPP_INFO(this->get_logger(),
                "\033[32;1mResult saved in .pcd format!!!\033[0m");
  }
}
// ====================== Keyframe Utilities ======================
void PgoSc::updateOdomsAndPaths(const PosePcd &pose_pcd_in) {
  odoms_.points.emplace_back(pose_pcd_in.pose_eig_(0, 3),
                             pose_pcd_in.pose_eig_(1, 3),
                             pose_pcd_in.pose_eig_(2, 3));
  corrected_odoms_.points.emplace_back(pose_pcd_in.pose_corrected_eig_(0, 3),
                                       pose_pcd_in.pose_corrected_eig_(1, 3),
                                       pose_pcd_in.pose_corrected_eig_(2, 3));
  odom_path_.poses.push_back(
      poseEigToPoseStamped(pose_pcd_in.pose_eig_, map_frame_));
  corrected_path_.poses.push_back(
      poseEigToPoseStamped(pose_pcd_in.pose_corrected_eig_, map_frame_));
}

visualization_msgs::msg::Marker PgoSc::getLoopMarkers(
    const gtsam::Values &corrected_esti_in) {
  visualization_msgs::msg::Marker edges;
  edges.type = visualization_msgs::msg::Marker::LINE_LIST;
  edges.scale.x = 0.12f;
  edges.header.frame_id = map_frame_;
  edges.pose.orientation.w = 1.0f;
  edges.color.r = edges.color.g = edges.color.b = edges.color.a = 1.0f;

  for (auto &pair : loop_idx_pairs_) {
    if (pair.first >= corrected_esti_in.size() ||
        pair.second >= corrected_esti_in.size())
      continue;

    gtsam::Pose3 p1 = corrected_esti_in.at<gtsam::Pose3>(pair.first);
    gtsam::Pose3 p2 = corrected_esti_in.at<gtsam::Pose3>(pair.second);
    geometry_msgs::msg::Point pt1, pt2;
    pt1.x = p1.translation().x();
    pt1.y = p1.translation().y();
    pt1.z = p1.translation().z();
    pt2.x = p2.translation().x();
    pt2.y = p2.translation().y();
    pt2.z = p2.translation().z();
    edges.points.push_back(pt1);
    edges.points.push_back(pt2);
  }
  return edges;
}

bool PgoSc::checkIfKeyframe(const PosePcd &pose_pcd_in,
                            const PosePcd &latest_pose_pcd) {
  return keyframe_thr_ <
         (latest_pose_pcd.pose_corrected_eig_.block<3, 1>(0, 3) -
          pose_pcd_in.pose_corrected_eig_.block<3, 1>(0, 3))
             .norm();
}