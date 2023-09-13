/**
 * @file aim_assistant_component.hpp
 * @author Toyozo Shimada (shimada@aisl.cs.tut.ac.jp)
 * @brief Detect poles with PCL.
 * @version ???
 * @date 2023-99-99
 *
 * @copyright Copyright (c) 2023
 *
 */
#pragma once

#include <rclcpp/rclcpp.hpp>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <iostream>
#include <numeric>
#include <optional>
#include <queue>
#include <random>
#include <string>
#include <type_traits>

#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/point_cloud.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud_conversion.hpp>

#include <geometry_msgs/msg/point32.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>

#include <std_msgs/msg/color_rgba.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/u_int16.hpp>

#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <nav_msgs/msg/odometry.hpp>

#include "aim_assistant_msgs/msg/point_cloud_array.hpp"
#include "aim_assistant_msgs/msg/pole.hpp"
#include "aim_assistant_msgs/msg/pole_array.hpp"

#include <jsk_rviz_plugin_msgs/msg/overlay_text.hpp>

#include "field_info.hpp"
#include "low_pass_filter.hpp"
#include "kalman_filter.hpp"
#include "vector2.hpp"

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/utils.h>
#include <tf2_eigen/tf2_eigen.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

#include <pcl/common/transforms.h>
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_cylinder.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl_conversions/pcl_conversions.h>

#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

#include "aim_assistant_exceptions.hpp"

// #include <message_filters/subscriber.h>
// #include <message_filters/time_synchronizer.h>
// #include <message_filters/

#include "Eigen/Dense"

namespace abu2023 {

struct Pole {
  float center_x, center_y, radius;
  uint16_t type = 0;
  uint16_t id = 11;
  float height = 0;
  float error_from_ideal = 0;
  cpp_general::Vector2 error_from_ideal_vec = cpp_general::Vector2(0, 0);
  float score = 0;
  bool is_lost = false;
  bool is_interpolated_pole = false;

  Pole() = default;
  constexpr Pole(float x, float y, float r) : center_x(x), center_y(y), radius(r) {}
  constexpr Pole(cpp_general::Vector2 v, float r) : Pole(v.x, v.y, r) {}

  cpp_general::Vector2 get_center_vec() { return cpp_general::Vector2(center_x, center_y); }

  visualization_msgs::msg::Marker as_visualization_msgs(rclcpp::Time stamp, std::string frame_id = "map",
                                                        size_t id = 0) {
    visualization_msgs::msg::Marker msg;
    msg.action = visualization_msgs::msg::Marker::ADD;
    msg.type = visualization_msgs::msg::Marker::CYLINDER;
    msg.header.frame_id = frame_id;
    msg.header.stamp = stamp;
    msg.color.a = 1.0;
    msg.color.r = 0.0;
    msg.color.g = 1.0;
    msg.color.b = 0.0;
    msg.pose.orientation.x = 0.0;
    msg.pose.orientation.y = 0.0;
    msg.pose.orientation.z = 0.0;
    msg.pose.orientation.w = 1.0;
    msg.pose.position.x = center_x;
    msg.pose.position.y = center_y;
    msg.pose.position.z = 0.0;
    msg.scale.x = 2.0 * radius;
    msg.scale.y = 2.0 * radius;
    msg.scale.z = 2.0;
    msg.id = id;
    return msg;
  }

  aim_assistant_msgs::msg::Pole as_aim_assistant_msgs() {
    // assert(id < 11);
    aim_assistant_msgs::msg::Pole msg;
    msg.id = assert_id(id);
    msg.type = type;
    msg.point.x = center_x;
    msg.point.y = center_y;
    msg.point.z = 0.0;
    return msg;
  }

  constexpr uint8_t assert_id(int _id) {
    assert(_id < 11);
    return _id;
  }
};

constexpr size_t POLE_NUM = 11;

using SegModelType = std::pair<pcl::ModelCoefficients::Ptr, double>;
using Point3d = pcl::PointXYZ;
using Point2d = pcl::PointXY;
using Cloud3d = pcl::PointCloud<Point3d>;
using Cloud2d = pcl::PointCloud<Point2d>;

using PCLPtrCluster = std::vector<Cloud3d::Ptr>;
using PoleClusters = std::array<std::vector<Pole>, POLE_NUM>;
using PoleCluster = std::vector<Pole>;

// #define NO_TRANSFORM_DEBUG
// #define mid_70_BASED_PUBLISH
// #define VELODYNE_BASED_PUBLISH

class AimAssistant : public rclcpp::Node {
private:
  double MAX_DETECT_RANGE_X;
  double MIN_DETECT_RANGE_X;
  double MAX_DETECT_RANGE_Y;
  double MIN_DETECT_RANGE_Y;
  double MAX_DETECT_RANGE_Z;
  double MIN_DETECT_RANGE_Z;
  std::vector<std::vector<double>> DETECT_X_RANGE;
  std::vector<std::vector<double>> DETECT_Y_RANGE;
  std::vector<std::vector<double>> DETECT_Z_RANGE;

  std::vector<std::vector<double>> VLP16_DETECT_X_RANGE;
  std::vector<std::vector<double>> VLP16_DETECT_Y_RANGE;
  std::vector<std::vector<double>> VLP16_DETECT_Z_RANGE;

  double ROR_SEARCH_RADIUS;
  int ROR_MIN_NEIGHBORS;

  double VGF_LEAF_SIZE;

  double EUCLIDEAN_CLUSTERING_CLS_TOR;
  size_t EUCLIDEAN_CLUSTERING_CLS_MIN;
  size_t EUCLIDEAN_CLUSTERING_CLS_MAX;

  size_t SEG_CYL_K_NEAREST_NEIGHBORS;
  size_t SEG_CYL_MAX_ITERATIONS;
  double SEG_CYL_NORMAL_DISTANCE_WEIGHT;
  double SEG_CYL_DISTANCE_THRESHOLD;
  std::vector<double> SEG_CYL_RADIUS_LIM;

  size_t SEG_CIR_K_NEAREST_NEIGHBORS;
  size_t SEG_CIR_MAX_ITERATIONS;
  double SEG_CIR_NORMAL_DISTANCE_WEIGHT;
  double SEG_CIR_DISTANCE_THRESHOLD;
  std::vector<double> SEG_CIR_RADIUS_LIM;

  double MATCH_DISTANCE_THRSHOLD;

  size_t POLES_BUFFER_MAX_SIZE;

  size_t main_process_count_ = 0;
  Cloud3d tmp_cloud_buffer_;

  field_info field_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> listener_;
  bool adjust_type2_base_ = false;

  Cloud3d main_cloud_buffer_; // 本命 自己位置依存
  Cloud3d sub_cloud_buffer_;  // サブ 自己位置非依存

  Cloud3d vlp_cloud_;
  Cloud3d vlp_2d_cloud_;

  cpp_general::Vector2 current_velocity_vector_ = {0, 0};

  std::queue<std::array<Pole, POLE_NUM>> detected_poles_buffer_;
  std::array<Pole, POLE_NUM> detected_pole_archive_;
  std::array<Pole, POLE_NUM> ref_ideal_poles_;
  std::array<Pole, POLE_NUM> dbg_detected_pole_;

  std::shared_ptr<jsk_rviz_plugin_msgs::msg::OverlayText> dbg_detection_info_;
  std::string dbg_detection_info_text_;

  std_msgs::msg::UInt16 current_target_pole_id_;

  rclcpp::Publisher<aim_assistant_msgs::msg::PoleArray>::SharedPtr target_pole_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr dbg_mid_70_map_based_cloud_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr dbg_vlp_map_based_cloud_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr dbg_poles_pub_;
  rclcpp::Publisher<jsk_rviz_plugin_msgs::msg::OverlayText>::SharedPtr dbg_detection_info_pub_;
  // rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr dbg_converted_cloud_pub_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr dbg_diff_from_ideal_pub_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr dbg_filtered_diff_from_ideal_pub_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr dbg_kf_diff_from_ideal_pub_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr dbg_current_abs_vel_pub_;

  // rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr dbg_poles_from_velodyne_pub_;

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr mid_70_cloud_sub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr mid_360_cloud_sub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr velodyne_cloud_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr localization_sub_;

  rclcpp::TimerBase::SharedPtr livox_detection_process_timer_;
  rclcpp::TimerBase::SharedPtr publisher_timer_callback_;

  pcl::PassThrough<pcl::PointXYZ> pass_filter_;
  pcl::VoxelGrid<pcl::PCLPointCloud2> vgf_filter_;
  pcl::RadiusOutlierRemoval<pcl::PointXYZ> ror_filter_;

  // pole correction filters
  enum class CorrectionMethod { TWO_POINT_AVG, KALMAN_FILTER, LOWPASS_FILTER, NO_FILTERING };

  std::unordered_map<std::string, CorrectionMethod> pole_correction_method_map_ = {
      {"two_point_avg", CorrectionMethod::TWO_POINT_AVG},
      {"kalman_filter", CorrectionMethod::KALMAN_FILTER},
      {"lowpass_filter", CorrectionMethod::LOWPASS_FILTER},
      {"no_filtering", CorrectionMethod::NO_FILTERING}};

  CorrectionMethod pole_correction_method_;
  std::array<KalmanFilter<double, 2, 2, 2>, POLE_NUM> pole_kf_filter_;

  const double lpf_x_gain_ = 0.05;
  const double lpf_y_gain_ = 0.05;
  std::array<std::array<cpp_general::LowPassFilter, 2>, POLE_NUM> pole_lp_filter_ = {
      {{cpp_general::LowPassFilter(lpf_x_gain_, 2.0), cpp_general::LowPassFilter(lpf_y_gain_, 2.0)},
       {cpp_general::LowPassFilter(lpf_x_gain_, 2.0), cpp_general::LowPassFilter(lpf_y_gain_, 2.0)},
       {cpp_general::LowPassFilter(lpf_x_gain_, 2.0), cpp_general::LowPassFilter(lpf_y_gain_, 2.0)},
       {cpp_general::LowPassFilter(lpf_x_gain_, 2.0), cpp_general::LowPassFilter(lpf_y_gain_, 2.0)},
       {cpp_general::LowPassFilter(lpf_x_gain_, 2.0), cpp_general::LowPassFilter(lpf_y_gain_, 2.0)},
       {cpp_general::LowPassFilter(lpf_x_gain_, 2.0), cpp_general::LowPassFilter(lpf_y_gain_, 2.0)},
       {cpp_general::LowPassFilter(lpf_x_gain_, 2.0), cpp_general::LowPassFilter(lpf_y_gain_, 2.0)},
       {cpp_general::LowPassFilter(lpf_x_gain_, 2.0), cpp_general::LowPassFilter(lpf_y_gain_, 2.0)},
       {cpp_general::LowPassFilter(lpf_x_gain_, 2.0), cpp_general::LowPassFilter(lpf_y_gain_, 2.0)},
       {cpp_general::LowPassFilter(lpf_x_gain_, 2.0), cpp_general::LowPassFilter(lpf_y_gain_, 2.0)},
       {cpp_general::LowPassFilter(lpf_x_gain_, 2.0), cpp_general::LowPassFilter(lpf_y_gain_, 2.0)}}};
  std::array<Pole, POLE_NUM> kf_filtered_pre_pole_array_;
  // KalmanFilter<double, 2, 2, 2> kf_;
  // Pole kf_filtered_pre_pole_;

  geometry_msgs::msg::Quaternion set_orientation(double x, double y, double z, double w) {
    geometry_msgs::msg::Quaternion q;
    q.x = x;
    q.y = y;
    q.z = z;
    q.w = w;
    return q;
  }

  geometry_msgs::msg::Point set_position(double x, double y, double z) {
    geometry_msgs::msg::Point p;
    p.x = x;
    p.y = y;
    p.z = z;
    return p;
  }

  geometry_msgs::msg::Vector3 set_scale(double x, double y, double z) {
    geometry_msgs::msg::Vector3 v;
    v.x = x;
    v.y = y;
    v.z = z;
    return v;
  }

  std_msgs::msg::ColorRGBA set_color(double a, double r, double g, double b) {
    std_msgs::msg::ColorRGBA color;
    color.a = a;
    color.r = r;
    color.g = g;
    color.b = b;
    return color;
  }

  // 自分を補間可能な他のポールの集合
  std::array<std::vector<std::vector<size_t>>, POLE_NUM> interpolatable_pole_id_ = {
      std::vector<std::vector<size_t>>{{1, 2}, {5, 6, 9, 10}},
      std::vector<std::vector<size_t>>{{0, 2}, {4, 10}},
      std::vector<std::vector<size_t>>{{0, 1}, {3, 7, 8, 10}},
      std::vector<std::vector<size_t>>{{4, 5}, {2, 7, 8, 10}},
      std::vector<std::vector<size_t>>{{3, 5}, {1, 10}},
      std::vector<std::vector<size_t>>{{3, 4}, {0, 6, 9, 10}},
      std::vector<std::vector<size_t>>{{0, 5, 9, 10}},
      std::vector<std::vector<size_t>>{{2, 3, 8, 10}},
      std::vector<std::vector<size_t>>{{2, 3, 7, 10}},
      std::vector<std::vector<size_t>>{{0, 5, 6, 10}},
      std::vector<std::vector<size_t>>{{0, 6, 9, 5}, {2, 3, 7, 8}, {1, 4}}};

  template <typename T> void swap(T &a, T &b) {
    T tmp;
    tmp = a;
    a = b;
    b = tmp;
  }

public:
  AimAssistant(const rclcpp::NodeOptions &options) : AimAssistant("", options) {}
  AimAssistant(const std::string &name_space = "", const rclcpp::NodeOptions &options = rclcpp::NodeOptions());

  /*=====================================================================================================*/
  /*====================================DEFINITION OF MEMBER FUNCTION====================================*/
  /*=====================================================================================================*/

  /*==================DEFINITION FOR POINT CLOUD CALLBACKS==================*/

  /**
   * @brief Preprocess for livox cloud. Transform to map-frame, Store in to main_cloud_buffer.
   *
   * @param source_cloud [in] Subscribed cloud.
   * @param output_cloud [out] cloud buffer
   */
  void store_in_buffer(sensor_msgs::msg::PointCloud2 &input_cloud, std::string source_frame, Cloud3d &main_cloud_buf,
                       Cloud3d &sub_cloud_buf);

  /**
   * @brief Preprocess for vlp16 cloud. Transform to map-frame, crop range, filtering.
   *
   * @param input_cloud [in] sensor_msgs::msg::PointCloud2
   * @param output_pcl_cloud [out] result preprocess (3d)
   * @param output_pcl_cloud_2d [out] result preprocess (2d). Compressed to 2d.
   * @return true successfully transformed
   * @return false failed to transform
   */
  bool VLP_preprocess(sensor_msgs::msg::PointCloud2 &input_cloud, Cloud3d &output_pcl_cloud,
                      Cloud3d &output_pcl_cloud_2d);

  /*==================END OF DEFINITION FOR POINT CLOUD CALLBACKS==================*/

  /*==================DEFINITION FOR POINT CLOUD UTILITIES==================*/

  /**
   * @brief Passthrough fileter to rangecut, radius_outlier_
   *
   * @param input_cloud
   */
  void filtering(Cloud3d &input_cloud, std::vector<std::vector<double>> &X_RANGE,
                 std::vector<std::vector<double>> &Y_RANGE, std::vector<std::vector<double>> &Z_RANGE);

  /**
   * @brief Pass through filtering. This expected to be used instead of rangecut.
   *
   * @param source_cloud
   */
  void pass_through(Cloud3d &source_cloud, std::vector<double> x_lim, std::vector<double> y_lim,
                    std::vector<double> z_lim);

  /**
   * @brief Radius Outlier Removal. Remove outliers from cloud.
   *
   * @param source_cloud
   * @param search_radius
   * @param min_neighbors_in_search_radius
   */
  void radius_outlier_removal(Cloud3d &source_cloud);

  /**
   * @brief Voxel Grid Filtering
   *
   * @param source_cloud [in]
   * @param leaf_size [in] default to 0.01
   */
  void VGF(Cloud3d &source_cloud);

  /**
   * @brief Convert pcl cloud frame.
   *
   * @param from_frame [in] frame before transform
   * @param to_frame [in] frame after transform
   * @param input_pcl_cloud [in] target pcl cloud
   * @param output_pcl_cloud [out] result internally stored in this cloud.
   * @param tf_buffer [in] buffer
   * @param base_time [in] Base time of transform. This time is required to be strictly correct.
   * @return true successfully transformed
   * @return false failed to transform
   */
  bool transformPointCloud(const std::string &from_frame, const std::string &to_frame, const Cloud3d &input_pcl_cloud,
                           Cloud3d &output_pcl_cloud);

  /**
   * @brief Convert pcl cloud frame.
   *
   * @param from_frame [in] frame before transform
   * @param to_frame [in] frame after transform
   * @param input_pcl_cloud [in] target pcl cloud
   * @param output_pcl_cloud [out] result internally stored in this cloud.
   * @param tf_buffer [in] buffer
   * @param base_time [in] Base time of transform. This time is required to be strictly correct.
   * @return true successfully transformed
   * @return false failed to transform
   */
  bool convert_cloud_frame(sensor_msgs::msg::PointCloud2 &source_cloud, std::string from_frame, std::string to_frame);

  /**
   * @brief Compress dimension to a particular axis. (x,y,z)
   *
   * @param input_pcl_cloud
   * @param output_pcl_cloud2d
   * @param to_axis [in] Eigen::Vector3i
   */
  void compress_dimension(Cloud3d &input_pcl_cloud, Cloud3d &output_pcl_cloud2d, Eigen::Vector3i to_axis);

  /*==================END OF DEFINITION FOR POINT CLOUD UTILITIES==================*/

  /*==================DEFINITION FOR POINT CLOUD CLUSTERING==================*/

  /**
   * @brief Clustering by euclidean 3d distance
   *
   * @param input_cloud [in]
   * @param clusters [out] aim_assistant_msgs/msg/PointCloudArray
   */
  PCLPtrCluster euclidean_clustering(Cloud3d &source_cloud);

  /**
   * @brief Euclidean clustering using 2d-cloud.
   *
   * @param input_pcl_cloud_2d [in] 2d pcl_cloud
   * @param input_pcl_cloud_3d [in] 3d(original) pcl_cloud
   * @return PCLPtrCluster std::vector<std::shared_ptr<Cloud3>>
   */
  PCLPtrCluster euclidean_clustering(Cloud3d &source_pcl_cloud_2d, Cloud3d &source_pcl_cloud_3d);

  /*==================END OF DEFINITION FOR POINT CLOUD CLUSTERING==================*/

  /*==================DEFINITION FOR POINT CLOUD SEGMENTATION==================*/

  /**
   * @brief Pole estimation by circle2d fitting.
   *
   * @param input_clusters [in] PCLPtrCluster. the result of clustering.
   * @return PoleCluster candidates of pole.
   */
  PoleCluster estimate_pole(PCLPtrCluster &input_clusters);

  /**
   * @brief Circle2d fitting as pole-detection. using RANSAC algorithms. Currently, the method doesn't using plane
   * extraction.
   *
   * @param input_cloud
   * @return SegModelType pair of circle coef and inliers (x, y, r)
   */
  std::optional<SegModelType> circle2d_fitting(std::shared_ptr<Cloud3d> &source_cloud_ptr);

  /*==================END OF DEFINITION FOR POINT CLOUD SEGMENTATION==================*/

  /*==================DEFINITION FOR POLE MATCHING==================*/

  /**
   * @brief Matching pole-candidate to ideal.
   *
   * @param candidate [in] std::vector<Circle>
   * @return clusters Result of matching. The coordinates bases on "map".
   */
  std::array<Pole, POLE_NUM> matching_poles(PoleCluster &candidate);

  /**
   * @brief Ignoring duplication by relative pose of pole.
   *
   * @param candidate
   */
  void ignore_duplication(PoleCluster &candidate);

  /**
   * @brief Filtering detected pole position in three way.
   *
   * - two_point_avg
   * - kalman_fitler
   * - lowpass_filter
   * @param pr_pole_array [in/out] detected_poles archive, actually publish this value
   * @param cr_pole_array [in/out] poles detected in this time. this is unpublished data
   */
  void filter_pole_pos(std::array<Pole, POLE_NUM> &pr_pole_array, std::array<Pole, POLE_NUM> &cr_pole_array);

  /**
   * @brief Filtering by two-point-average. Correct current pole data from 1-frame-old data.
   *
   * @param pr_pole_array [in/out] 1-frame-old pole_array
   * @param cr_pole_array [in/out] current pole_array
   */
  void two_point_average_filter(std::array<Pole, POLE_NUM> &pr_pole_array, std::array<Pole, POLE_NUM> &cr_pole_array);

  /**
   * @brief Interpolation line
   *
   * @param interpolation_line
   * @return true
   * @return false
   */
  bool is_vertical_line_interpolation(const std::vector<size_t> &interpolation_line);

  /**
   * @brief Interpolate lost pole using already detected pole.
   *
   * @param pole_array
   */
  void interpolate_pole(std::array<Pole, POLE_NUM> &pole_array);

  /*==================END OF DEFINITION FOR POLE MATCHING==================*/

  /*==================DEFINITION FOR OTHERS==================*/

  double diff_from_ideal(Pole &detected_pole);

  /**
   * @brief Visualize pole array
   *
   * @param pole_array
   * @param marker_array
   */
  void visualize_pole_array(aim_assistant_msgs::msg::PoleArray &pole_array,
                            visualization_msgs::msg::MarkerArray &marker_array);

  /*==================END OF DEFINITION FOR OTHERS==================*/
};
} // namespace abu2023=