#include "../include/aim_assistant/aim_assistant_component.hpp"

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>

using namespace abu2023;

AimAssistant::AimAssistant(const std::string &name_space, const rclcpp::NodeOptions &options)
    : Node("aim_assistant_node", name_space, options) {
  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
  listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
  using namespace std::chrono_literals;
  using namespace cpp_general;
  dbg_detection_info_ = std::make_shared<jsk_rviz_plugin_msgs::msg::OverlayText>();
  current_target_pole_id_.data = POLE_NUM;

  // Pole init_pole(Vector2(0.0, 0.0), 0.0, 0.0, 0.0);
  // Pole init_pole(Vector2(0.0, 0.0), 0.0);
  // init_pole.is_lost = true;
  // detected_pole_archive_.fill(init_pole);

  for (size_t i = 0; i < POLE_NUM; ++i) {
    auto [ideal_pole_center, type] = field_.POLE.at(i);
    Pole init_pole(ideal_pole_center, (i == 10) ? 0.075 : 0.05);
    init_pole.type = type;
    init_pole.id = i;
    // init_pole.is_lost = true;
    ref_ideal_poles_.at(i) = init_pole;
    detected_pole_archive_.at(i) = init_pole;
    kf_filtered_pre_pole_array_.at(i) = init_pole;
  }

  declare_parameter("publish_period", 10);
  int publish_period = get_parameter("publish_period").as_int();

  declare_parameter("mid_70_detection_process_period", 300);
  int mid_70_detection_process_period = get_parameter("mid_70_detection_process_period").as_int();

  declare_parameter("adjust_type2_base", false);
  adjust_type2_base_ = get_parameter("adjust_type2_base").as_bool();

  /**detection range**/
  declare_parameter("detect_range_params.max_detection_range_x", 25.0);
  MAX_DETECT_RANGE_X = get_parameter("detect_range_params.max_detection_range_x").as_double();

  declare_parameter("detect_range_params.min_detection_range_x", -25.0);
  MIN_DETECT_RANGE_X = get_parameter("detect_range_params.min_detection_range_x").as_double();

  declare_parameter("detect_range_params.max_detection_range_y", 25.0);
  MAX_DETECT_RANGE_Y = get_parameter("detect_range_params.max_detection_range_y").as_double();

  declare_parameter("detect_range_params.min_detection_range_y", -25.0);
  MIN_DETECT_RANGE_Y = get_parameter("detect_range_params.min_detection_range_y").as_double();

  declare_parameter("detect_range_params.max_detection_range_z", 10.0);
  MAX_DETECT_RANGE_Z = get_parameter("detect_range_params.max_detection_range_z").as_double();

  declare_parameter("detect_range_params.min_detection_range_z", -1.0);
  MIN_DETECT_RANGE_Z = get_parameter("detect_range_params.min_detection_range_z").as_double();

  declare_parameter("detect_range_params.detect_margin", 0.25); // set half size
  double detection_margin = get_parameter("detect_range_params.detect_margin").as_double();

  declare_parameter("detect_range_params.type1_z_range", std::vector<double>{0.4, 1.0});
  std::vector<double> TYPE1_Z_RANGE = get_parameter("detect_range_params.type1_z_range").as_double_array();

  declare_parameter("detect_range_params.type2_z_range", std::vector<double>{0.6, 1.2});
  std::vector<double> TYPE2_Z_RANGE = get_parameter("detect_range_params.type2_z_range").as_double_array();

  declare_parameter("detect_range_params.type3_z_range", std::vector<double>{1.2, 2.0});
  std::vector<double> TYPE3_Z_RANGE = get_parameter("detect_range_params.type3_z_range").as_double_array();

  for (size_t i = 0; i < POLE_NUM; ++i) {
    auto [ideal_pole_pos, type] = field_.POLE.at(i);
    DETECT_X_RANGE.push_back({ideal_pole_pos.x - detection_margin, ideal_pole_pos.x + detection_margin});
    DETECT_Y_RANGE.push_back({ideal_pole_pos.y - detection_margin, ideal_pole_pos.y + detection_margin});
    if (type == 3) {
      DETECT_Z_RANGE.push_back(TYPE3_Z_RANGE);
    } else if (type == 2) {
      DETECT_Z_RANGE.push_back(TYPE2_Z_RANGE);
    } else {
      DETECT_Z_RANGE.push_back(TYPE1_Z_RANGE);
    }
  }

  // for vlp16
  declare_parameter("vlp16_detection_range_params.section_1_x", std::vector<double>{1.0, 3.0});
  VLP16_DETECT_X_RANGE.push_back(get_parameter("vlp16_detection_range_params.section_1_x").as_double_array());

  declare_parameter("vlp16_detection_range_params.section_1_y", std::vector<double>{1.0, 3.0});
  VLP16_DETECT_Y_RANGE.push_back(get_parameter("vlp16_detection_range_params.section_1_y").as_double_array());

  declare_parameter("vlp16_detection_range_params.section_1_z", std::vector<double>{1.0, 3.0});
  VLP16_DETECT_Z_RANGE.push_back(get_parameter("vlp16_detection_range_params.section_1_z").as_double_array());

  declare_parameter("vlp16_detection_range_params.section_2_x", std::vector<double>{1.0, 3.0});
  VLP16_DETECT_X_RANGE.push_back(get_parameter("vlp16_detection_range_params.section_2_x").as_double_array());

  declare_parameter("vlp16_detection_range_params.section_2_y", std::vector<double>{1.0, 3.0});
  VLP16_DETECT_Y_RANGE.push_back(get_parameter("vlp16_detection_range_params.section_2_y").as_double_array());

  declare_parameter("vlp16_detection_range_params.section_2_z", std::vector<double>{1.0, 3.0});
  VLP16_DETECT_Z_RANGE.push_back(get_parameter("vlp16_detection_range_params.section_2_z").as_double_array());

  /**other pcl params**/

  declare_parameter("filter_params.radius_outlier_removal.search_radius", 1.0);
  ROR_SEARCH_RADIUS = get_parameter("filter_params.radius_outlier_removal.search_radius").as_double();

  declare_parameter("filter_params.radius_outlier_removal.min_neighbors_in_search_radius", 1);
  ROR_MIN_NEIGHBORS = get_parameter("filter_params.radius_outlier_removal.min_neighbors_in_search_radius").as_int();

  declare_parameter("filter_params.VGF.leaf_size", 0.01);
  VGF_LEAF_SIZE = get_parameter("filter_params.VGF.leaf_size").as_double();

  declare_parameter("euclidean_clustering_params.cluster_tolerance", 0.1);
  EUCLIDEAN_CLUSTERING_CLS_TOR = get_parameter("euclidean_clustering_params.cluster_tolerance").as_double();

  declare_parameter("euclidean_clustering_params.min_cluster_size", 50);
  EUCLIDEAN_CLUSTERING_CLS_MIN = get_parameter("euclidean_clustering_params.min_cluster_size").as_int();

  declare_parameter("euclidean_clustering_params.max_cluster_size", 1000);
  EUCLIDEAN_CLUSTERING_CLS_MAX = get_parameter("euclidean_clustering_params.max_cluster_size").as_int();

  declare_parameter("cylinder_fitting_params.k_nearest_neighbors", 100);
  SEG_CYL_K_NEAREST_NEIGHBORS = get_parameter("cylinder_fitting_params.k_nearest_neighbors").as_int();

  declare_parameter("cylinder_fitting_params.max_iterations", 1000);
  SEG_CYL_MAX_ITERATIONS = get_parameter("cylinder_fitting_params.max_iterations").as_int();

  declare_parameter("cylinder_fitting_params.normal_distance_weight", 0.1);
  SEG_CYL_NORMAL_DISTANCE_WEIGHT = get_parameter("cylinder_fitting_params.normal_distance_weight").as_double();

  declare_parameter("cylinder_fitting_params.distance_threshold", 0.05);
  SEG_CYL_DISTANCE_THRESHOLD = get_parameter("cylinder_fitting_params.distance_threshold").as_double();

  declare_parameter("cylinder_fitting_params.radius_limits", std::vector<double>{0.1, 0.15});
  SEG_CYL_RADIUS_LIM = get_parameter("cylinder_fitting_params.radius_limits").as_double_array();

  declare_parameter("circle2d_fitting_params.k_nearest_neighbors", 100);
  SEG_CIR_K_NEAREST_NEIGHBORS = get_parameter("circle2d_fitting_params.k_nearest_neighbors").as_int();

  declare_parameter("circle2d_fitting_params.max_iterations", 1000);
  SEG_CIR_MAX_ITERATIONS = get_parameter("circle2d_fitting_params.max_iterations").as_int();

  declare_parameter("circle2d_fitting_params.normal_distance_weight", 0.1);
  SEG_CIR_NORMAL_DISTANCE_WEIGHT = get_parameter("circle2d_fitting_params.normal_distance_weight").as_double();

  declare_parameter("circle2d_fitting_params.distance_threshold", 0.05);
  SEG_CIR_DISTANCE_THRESHOLD = get_parameter("circle2d_fitting_params.distance_threshold").as_double();

  declare_parameter("circle2d_fitting_params.radius_limits", std::vector<double>{0.1, 0.15});
  SEG_CIR_RADIUS_LIM = get_parameter("circle2d_fitting_params.radius_limits").as_double_array();

  declare_parameter("pole_matching_params.distance_threshold", 0.1);
  MATCH_DISTANCE_THRSHOLD = get_parameter("pole_matching_params.distance_threshold").as_double();

  declare_parameter("pole_correction_params.poles_buffer_max_size", 10);
  POLES_BUFFER_MAX_SIZE = get_parameter("pole_correction_params.poles_buffer_max_size").as_int();

  // kf
  declare_parameter("pole_correction_params.correction_method", "two_point_avg");
  std::string correction_method = get_parameter("pole_correction_params.correction_method").as_string();

  declare_parameter("pole_correction_params.kf_q", std::vector<double>{0.01, 0.01});
  std::vector<double> kf_q = get_parameter("pole_correction_params.kf_q").as_double_array();

  declare_parameter("pole_correction_params.kf_r", std::vector<double>{0.0001, 0.0001});
  std::vector<double> kf_r = get_parameter("pole_correction_params.kf_r").as_double_array();

  declare_parameter("pole_correction_params.kf_init_p", std::vector<double>{0.8, 0.8});
  std::vector<double> kf_init_p = get_parameter("pole_correction_params.kf_init_p").as_double_array();

  /**filter settings**/
  // method settings
  if (pole_correction_method_map_.find(correction_method) == pole_correction_method_map_.end()) {
    RCLCPP_WARN(this->get_logger(),
                "AimAssistant : Invalid Correction method is chosen. Currently TwoPointAvg Filter is set to default.");
    correction_method = "two_point_avg";
  }

  pole_correction_method_ = pole_correction_method_map_.at(correction_method);

  // kalmanfilter settings
  for (auto &kf : pole_kf_filter_) {
    kf.F = Eigen::Vector2d::Ones().asDiagonal();
    kf.G = Eigen::Matrix2d::Zero();
    kf.H = Eigen::Vector2d::Ones().asDiagonal();

    Eigen::Vector2d q_vec;
    q_vec << kf_q[0], kf_q[1];
    kf.Q = q_vec.asDiagonal();

    Eigen::Vector2d r_vec;
    r_vec << kf_r[0], kf_r[1];
    kf.R = r_vec.asDiagonal();

    Eigen::Vector2d x0;
    x0 << detected_pole_archive_.at(10).center_x, detected_pole_archive_.at(10).center_y;
    Eigen::Vector2d init_p;
    init_p << kf_init_p.at(0), kf_init_p.at(1);
    Eigen::Matrix<double, 2, 2> p0 = init_p.asDiagonal();

    kf.reset(x0, p0);
  }

  /**generate publisher entity**/
  for (size_t i = 0; i < POLE_NUM; ++i) {
    pole_lp_filter_.at(i).at(0).reset(detected_pole_archive_.at(i).center_x);
    pole_lp_filter_.at(i).at(1).reset(detected_pole_archive_.at(i).center_y);
  }

  target_pole_pub_ =
      this->create_publisher<aim_assistant_msgs::msg::PoleArray>("aim_assistant_3d/target_poles", rclcpp::QoS(10));
  dbg_poles_pub_ =
      this->create_publisher<visualization_msgs::msg::MarkerArray>("aim_assistant_3d/dbg_poles", rclcpp::QoS(10));
  dbg_mid_70_map_based_cloud_pub_ =
      this->create_publisher<sensor_msgs::msg::PointCloud2>("aim_assistant_3d/dbg_corrected_cloud2", rclcpp::QoS(10));
  dbg_vlp_map_based_cloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
      "aim_assistant_3d/dbg_corrected_vlp_cloud2", rclcpp::QoS(10));
  dbg_detection_info_pub_ = this->create_publisher<jsk_rviz_plugin_msgs::msg::OverlayText>(
      "aim_assistant_3d/detection_info", rclcpp::QoS(10));
  dbg_diff_from_ideal_pub_ =
      this->create_publisher<std_msgs::msg::Float32>("aim_assistant_3d/dbg/diff_from_ideal", rclcpp::QoS(10));
  dbg_filtered_diff_from_ideal_pub_ =
      this->create_publisher<std_msgs::msg::Float32>("aim_assistant_3d/dbg/filtered_diff_from_ideal", rclcpp::QoS(10));
  dbg_kf_diff_from_ideal_pub_ =
      this->create_publisher<std_msgs::msg::Float32>("aim_assistant_3d/dbg/kf_diff_from_ideal", rclcpp::QoS(10));
  dbg_current_abs_vel_pub_ =
      this->create_publisher<std_msgs::msg::Float32>("aim_assistant_3d/dbg/current_abs_velocity", rclcpp::QoS(10));
  // dbg_aim_arrow_pub_ =
  //     this->create_publisher<visualization_msgs::msg::Marker>("aim_assistant_3d/dbg_aim_arrow", rclcpp::QoS(10));
  // dbg_cloud2_pub_ =
  //     this->create_publisher<sensor_msgs::msg::PointCloud2>("aim_assistant_3d/dbg_cloud2", rclcpp::QoS(10));
  // dbg_poles_from_velodyne_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
  //     "aim_assistant_3d/dbg_poles_from_vlp", rclcpp::QoS(10));
  // dbg_converted_cloud_pub_ =
  //     this->create_publisher<sensor_msgs::msg::PointCloud2>("aim_assistant_3d/dbg_converted_cloud",
  //     rclcpp::QoS(10));

  mid_70_cloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      "/livox/mid_70", rclcpp::SensorDataQoS(), [&](const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
        if (adjust_type2_base_) {
          store_in_buffer(*msg, "calib_aim_assistant_laser", main_cloud_buffer_, sub_cloud_buffer_);
        } else {
          store_in_buffer(*msg, "aim_assistant_laser", main_cloud_buffer_, sub_cloud_buffer_);
        }
      });

  mid_360_cloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      "/livox/mid_360", rclcpp::SensorDataQoS(), [&](const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
        if (adjust_type2_base_) {
          store_in_buffer(*msg, "calib_aim_assistant_laser", main_cloud_buffer_, sub_cloud_buffer_);
        } else {
          store_in_buffer(*msg, "aim_assistant_laser", main_cloud_buffer_, sub_cloud_buffer_);
        }
      });

  // only calc pole pose. doesn't publish
  rclcpp::QoS velodyne_qos{rclcpp::KeepLast(1)};
  velodyne_cloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      "/velodyne_points", velodyne_qos, [this](const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
        return;
        if (msg->data.size() < 10)
          return;

        sensor_msgs::msg::PointCloud2 target_cloud;
        Cloud3d target_pcl_cloud;
        Cloud3d target_pcl_cloud_2d;

        try {
          if (!VLP_preprocess(*msg, target_pcl_cloud, target_pcl_cloud_2d)) {
            return;
          }
        } catch (EmptyPointCloudException &e) {
          RCLCPP_WARN(this->get_logger(), "%s", e.what());
          return;
        }

        PCLPtrCluster clusters = euclidean_clustering(target_pcl_cloud_2d, target_pcl_cloud);

        PoleCluster poles = estimate_pole(clusters);

        std::array<Pole, POLE_NUM> current_detected_poles = matching_poles(poles);

        // two_point_average_filter(detected_pole_archive_, current_detected_poles);
      });

  localization_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "/ER/localization_result", rclcpp::QoS(10).best_effort(), [&](const nav_msgs::msg::Odometry::SharedPtr msg) {
        current_velocity_vector_.x = msg->twist.twist.linear.x;
        current_velocity_vector_.y = msg->twist.twist.linear.y;

        std_msgs::msg::Float32 current_velocity_msg;

        current_velocity_msg.data = current_velocity_vector_.norm();
        dbg_current_abs_vel_pub_->publish(current_velocity_msg);
      });

  livox_detection_process_timer_ =
      this->create_wall_timer(std::chrono::milliseconds(mid_70_detection_process_period), [&]() {
        // ここで、収集された点群データは全てmap座標系であることに注意
        rclcpp::Time start_time = this->get_clock()->now();
        if (main_cloud_buffer_.points.size() < 10) {
          main_cloud_buffer_.clear();
          sub_cloud_buffer_.clear();
          return;
        }

        try {
          filtering(main_cloud_buffer_, DETECT_X_RANGE, DETECT_Y_RANGE, DETECT_Z_RANGE);
        } catch (EmptyPointCloudException &e) {
          RCLCPP_WARN(this->get_logger(), "%s", e.what());
          return;
        }

        sensor_msgs::msg::PointCloud2 input_cloud;
        pcl::toROSMsg(main_cloud_buffer_, input_cloud);
        input_cloud.header.frame_id = "map";
        input_cloud.header.stamp = this->get_clock()->now();

        dbg_mid_70_map_based_cloud_pub_->publish(input_cloud);

        tmp_cloud_buffer_ = main_cloud_buffer_;

        PCLPtrCluster clusters = euclidean_clustering(main_cloud_buffer_);

        PoleCluster poles = estimate_pole(clusters);

        std::array<Pole, POLE_NUM> current_detected_poles = matching_poles(poles);

        // interpolate_pole(current_detected_poles);

        dbg_detected_pole_ = current_detected_poles;

        std_msgs::msg::Float32 diff_from_ideal_msg;
        if (!dbg_detected_pole_.at(10).is_lost) {
          diff_from_ideal_msg.data = diff_from_ideal(dbg_detected_pole_.at(10));
        } else {
          diff_from_ideal_msg.data = 0.0;
        }

        dbg_diff_from_ideal_pub_->publish(diff_from_ideal_msg);

        filter_pole_pos(detected_pole_archive_, current_detected_poles);

        std_msgs::msg::Float32 filtered_diff_from_ideal_msg;
        filtered_diff_from_ideal_msg.data = diff_from_ideal(detected_pole_archive_.at(10));
        dbg_filtered_diff_from_ideal_pub_->publish(filtered_diff_from_ideal_msg);

        // for type2 based adjustment
        // if (adjust_type2_base_) {
        //   cpp_general::Vector2 pole6_pos = detected_pole_archive_.at(6).get_center_vec();
        //   cpp_general::Vector2 pole7_pos = detected_pole_archive_.at(7).get_center_vec();
        //   cpp_general::Vector2 pole7_diff_from_ideal = (ref_ideal_poles_.at(7).get_center_vec() - pole7_pos);
        //   RCLCPP_INFO(this->get_logger(), "pole6, 7 tilt : %lf, pole7 diff from ideal : (x,y) = (%lf, %lf)",
        //               (pole7_pos - pole6_pos).angle(), pole7_diff_from_ideal.x, pole7_diff_from_ideal.y);
        // }
        // clear buffer
        main_cloud_buffer_.clear();
        sub_cloud_buffer_.clear();
      });

  // publish poles as aim_assistant_msgs and visualization_msgs
  publisher_timer_callback_ = this->create_wall_timer(std::chrono::milliseconds(publish_period), [&]() {
    visualization_msgs::msg::MarkerArray pole_marker_array;
    aim_assistant_msgs::msg::PoleArray output_pole_msg;
    std_msgs::msg::Header header;
    header.frame_id = "map";
    header.stamp = this->get_clock()->now();
    size_t id = 0;
    for (auto &pole : detected_pole_archive_) {
      if (!pole.is_lost) {
        aim_assistant_msgs::msg::Pole tmp_pole = pole.as_aim_assistant_msgs();
        pole_marker_array.markers.push_back(pole.as_visualization_msgs(this->get_clock()->now(), "map", id++));
        tmp_pole.header = header;
        output_pole_msg.poles.push_back(tmp_pole);
      }
    }
    target_pole_pub_->publish(output_pole_msg);
    dbg_poles_pub_->publish(pole_marker_array);
  });
}

/*=========================================================================================================*/
/*====================================IMPLEMENTATION OF MEMBER FUNCTION====================================*/
/*=========================================================================================================*/

/*==================POINT CLOUD CALLBACKS==================*/

/**
 * @brief Preprocess for livox cloud. Transform to map-frame, Store in to main_cloud_buffer.
 *
 * @param source_cloud [in] Subscribed cloud.
 * @param output_cloud [out] cloud buffer
 */
void AimAssistant::store_in_buffer(sensor_msgs::msg::PointCloud2 &input_cloud, std::string source_frame,
                                   Cloud3d &main_cloud_buf, Cloud3d &sub_cloud_buf) {
  Cloud3d::Ptr tmp_pcl_cloud = std::make_shared<Cloud3d>();
  Cloud3d::Ptr ignore_nan_cloud = std::make_shared<Cloud3d>();
  // Cloud3d::Ptr cloud_removed_nan = std::make_shared<Cloud3d>();
  pcl::fromROSMsg(input_cloud, *ignore_nan_cloud);

  pcl::Indices ind;
  pcl::removeNaNFromPointCloud(*ignore_nan_cloud, *ignore_nan_cloud, ind);

  // ignore_nan_cloud->header.frame_id = "aim_assistant_laser";
  sensor_msgs::msg::PointCloud2 tmp_cloud2;
  tmp_cloud2.header.frame_id = source_frame;
  pcl::toROSMsg(*ignore_nan_cloud, tmp_cloud2);
  if (!convert_cloud_frame(tmp_cloud2, source_frame, "map")) {
    RCLCPP_WARN(this->get_logger(), "livox convert failed");
    return;
  };
  pcl::fromROSMsg(tmp_cloud2, *tmp_pcl_cloud);
  main_cloud_buf += *tmp_pcl_cloud;

  pcl::fromROSMsg(input_cloud, *tmp_pcl_cloud); // aim_assistant_laser frame (completely relative)
  sub_cloud_buf += *tmp_pcl_cloud;
}

/**
 * @brief Preprocess for vlp16 cloud. Transform to map-frame, crop range, filtering.
 *
 * @param input_cloud [in] sensor_msgs::msg::PointCloud2
 * @param output_pcl_cloud [out] result preprocess (3d)
 * @param output_pcl_cloud_2d [out] result preprocess (2d). Compressed to 2d.
 * @return true successfully transformed
 * @return false failed to transform
 */
bool AimAssistant::VLP_preprocess(sensor_msgs::msg::PointCloud2 &input_cloud, Cloud3d &output_pcl_cloud,
                                  Cloud3d &output_pcl_cloud_2d) {
  Cloud3d::Ptr cloud_with_nan = std::make_shared<Cloud3d>();
  Cloud3d::Ptr cloud_removed_nan = std::make_shared<Cloud3d>();
  sensor_msgs::msg::PointCloud2 ignore_nan_cloud;
  pcl::fromROSMsg(input_cloud, *cloud_with_nan);

  pcl::Indices nan_idx;
  pcl::removeNaNFromPointCloud(*cloud_with_nan, *cloud_removed_nan, nan_idx); // remove nan

  if (!transformPointCloud("vlp16_laser", "map", *cloud_removed_nan, output_pcl_cloud)) {
    return false;
  }

  sensor_msgs::msg::PointCloud2::UniquePtr converted_cloud = std::make_unique<sensor_msgs::msg::PointCloud2>();
  pcl::toROSMsg(output_pcl_cloud, *converted_cloud);
  converted_cloud->header.frame_id = "map";
  converted_cloud->header.stamp = this->get_clock()->now();
  dbg_vlp_map_based_cloud_pub_->publish(std::move(converted_cloud));

  filtering(output_pcl_cloud, DETECT_X_RANGE, DETECT_Y_RANGE, DETECT_Z_RANGE); // filtering

  Eigen::Vector3i axis(0, 0, 1);
  compress_dimension(output_pcl_cloud, output_pcl_cloud_2d, axis); // delete z
  return true;
}

/*==================POINT CLOUD CALLBACKS END==================*/

/*==================POINT CLOUD UTILITIES==================*/

/**
 * @brief Passthrough fileter to rangecut, radius_outlier_
 *
 * @param input_cloud
 */
void AimAssistant::filtering(Cloud3d &input_cloud, std::vector<std::vector<double>> &X_RANGE,
                             std::vector<std::vector<double>> &Y_RANGE, std::vector<std::vector<double>> &Z_RANGE) {
  if (input_cloud.empty()) {
    throw EmptyPointCloudException("[filtering error] empty cloud");
  }
  Cloud3d::Ptr filtered_cloud = std::make_shared<Cloud3d>();
  for (size_t i = 0; i < X_RANGE.size(); ++i) {
    Cloud3d tmp_pcl_cloud;
    tmp_pcl_cloud = input_cloud;
    pass_through(tmp_pcl_cloud, X_RANGE.at(i), Y_RANGE.at(i), Z_RANGE.at(i));
    *filtered_cloud += tmp_pcl_cloud;
  }

  input_cloud = *filtered_cloud;

  radius_outlier_removal(input_cloud);
  // VGF(input_cloud);
}

/**
 * @brief Pass through filtering. This expected to be used instead of rangecut.
 *
 * @param source_cloud
 */
void AimAssistant::pass_through(Cloud3d &source_cloud, std::vector<double> x_lim, std::vector<double> y_lim,
                                std::vector<double> z_lim) {
  Cloud3d::Ptr filtered_pcl_cloud = std::make_shared<Cloud3d>();
  *filtered_pcl_cloud = source_cloud;
  pass_filter_.setInputCloud(filtered_pcl_cloud);
  pass_filter_.setFilterFieldName("x");
  pass_filter_.setFilterLimits(x_lim.at(0), x_lim.at(1));
  pass_filter_.filter(*filtered_pcl_cloud);

  pass_filter_.setInputCloud(filtered_pcl_cloud);
  pass_filter_.setFilterFieldName("y");
  pass_filter_.setFilterLimits(y_lim.at(0), y_lim.at(1));
  pass_filter_.filter(*filtered_pcl_cloud);

  pass_filter_.setInputCloud(filtered_pcl_cloud);
  pass_filter_.setFilterFieldName("z");
  pass_filter_.setFilterLimits(z_lim.at(0), z_lim.at(1));
  pass_filter_.filter(source_cloud);
}

/**
 * @brief Radius Outlier Removal. Remove outliers from cloud.
 *
 * @param source_cloud
 * @param search_radius
 * @param min_neighbors_in_search_radius
 */
void AimAssistant::radius_outlier_removal(Cloud3d &source_cloud) {
  Cloud3d::Ptr tmp_pcl_cloud = std::make_shared<Cloud3d>();
  *tmp_pcl_cloud = source_cloud;
  ror_filter_.setInputCloud(tmp_pcl_cloud);
  ror_filter_.setRadiusSearch(ROR_SEARCH_RADIUS);
  ror_filter_.setMinNeighborsInRadius(ROR_MIN_NEIGHBORS);
  Cloud3d::Ptr cloud_filtered = std::make_shared<Cloud3d>();
  ror_filter_.filter(source_cloud);
}

/**
 * @brief Voxel Grid Filtering
 *
 * @param source_cloud [in]
 * @param leaf_size [in] default to 0.01
 */
void AimAssistant::VGF(Cloud3d &source_cloud) {
  // double leaf_size = 0.03;
  if (VGF_LEAF_SIZE == 0)
    return;

  pcl::PCLPointCloud2::Ptr tmp_pcl_cloud = std::make_shared<pcl::PCLPointCloud2>();
  pcl::toPCLPointCloud2(source_cloud, *tmp_pcl_cloud);

  vgf_filter_.setInputCloud(tmp_pcl_cloud);
  vgf_filter_.setLeafSize(VGF_LEAF_SIZE, VGF_LEAF_SIZE, VGF_LEAF_SIZE);
  pcl::PCLPointCloud2Ptr pcl_cloud_filtered = std::make_shared<pcl::PCLPointCloud2>();
  vgf_filter_.filter(*pcl_cloud_filtered);

  pcl::fromPCLPointCloud2(*pcl_cloud_filtered, source_cloud);
}

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
bool AimAssistant::transformPointCloud(const std::string &from_frame, const std::string &to_frame,
                                       const Cloud3d &input_pcl_cloud, Cloud3d &output_pcl_cloud) {
  if (input_pcl_cloud.header.frame_id == to_frame) {
    output_pcl_cloud = input_pcl_cloud;
    return true;
  }

  geometry_msgs::msg::TransformStamped transform;
  try {
    transform =
        tf_buffer_->lookupTransform(to_frame, from_frame, pcl_conversions::fromPCL(input_pcl_cloud.header.stamp),
                                    rclcpp::Duration::from_seconds(1.0));
  } catch (tf2::LookupException &e) {
    RCLCPP_WARN(this->get_logger(), "%s", e.what());
    return false;
  } catch (tf2::ExtrapolationException &e) {
    RCLCPP_WARN(this->get_logger(), "%s", e.what());
    return false;
  }

  Eigen::Affine3f affine_conversion(tf2::transformToEigen(transform).affine().cast<float>());

  pcl::transformPointCloud(input_pcl_cloud, output_pcl_cloud, affine_conversion);
  return true;
}

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
bool AimAssistant::convert_cloud_frame(sensor_msgs::msg::PointCloud2 &source_cloud, std::string from_frame,
                                       std::string to_frame) {
  geometry_msgs::msg::TransformStamped look_up_tf;
  try {
    look_up_tf = tf_buffer_->lookupTransform(to_frame, from_frame, rclcpp::Time(0));
  } catch (tf2::LookupException &e) {
    RCLCPP_WARN(this->get_logger(), "%s", e.what());
    return false;
  } catch (tf2::ExtrapolationException &e) {
    RCLCPP_WARN(this->get_logger(), "%s", e.what());
    return false;
  }
  Eigen::Affine3f affine_conversion(tf2::transformToEigen(look_up_tf).affine().cast<float>());

  // Cloud3d::Ptr input_pcl_cloud(new Cloud3d());
  // Cloud3d::Ptr transformed_pcl_cloud(new Cloud3d());
  Cloud3d::Ptr input_pcl_cloud = std::make_shared<Cloud3d>();
  Cloud3d::Ptr transformed_pcl_cloud = std::make_shared<Cloud3d>();
  pcl::fromROSMsg(source_cloud, *input_pcl_cloud);
  pcl::transformPointCloud(*input_pcl_cloud, *transformed_pcl_cloud, affine_conversion);
  pcl::toROSMsg(*transformed_pcl_cloud, source_cloud);
  source_cloud.header.frame_id = to_frame;
  source_cloud.header.stamp = this->get_clock()->now();
  return true;
}

/**
 * @brief Compress dimension to a particular axis. (x,y,z)
 *
 * @param input_pcl_cloud
 * @param output_pcl_cloud2d
 * @param to_axis [in] Eigen::Vector3i
 */
void AimAssistant::compress_dimension(Cloud3d &input_pcl_cloud, Cloud3d &output_pcl_cloud2d, Eigen::Vector3i to_axis) {
  if (input_pcl_cloud.empty()) {
    throw EmptyPointCloudException("Empty cloud input to compress dimension.");
  }

  Eigen::Matrix4f dim_compress_mat = Eigen::Matrix4f::Identity(4, 4);
  for (size_t i = 0; i < 3; ++i) {
    if (to_axis(i, 0) > 0) {
      dim_compress_mat(i, i) = 0;
      break;
    }
  }
  pcl::transformPointCloud(input_pcl_cloud, output_pcl_cloud2d, dim_compress_mat);
}

/*==================POINT CLOUD UTILITIES END==================*/

/*==================POINT CLOUD CLUSTERING==================*/

/**
 * @brief Clustering by euclidean 3d distance
 *
 * @param input_cloud [in]
 * @param clusters [out] aim_assistant_msgs/msg/PointCloudArray
 */
PCLPtrCluster AimAssistant::euclidean_clustering(Cloud3d &source_cloud) {
  PCLPtrCluster pcl_clusters = std::vector<std::shared_ptr<Cloud3d>>();
  std::vector<pcl::PointIndices> cluster_indices;
  pcl::EuclideanClusterExtraction<Point3d> clustering;
  typename pcl::search::KdTree<Point3d>::Ptr tree = std::make_shared<pcl::search::KdTree<Point3d>>();
  tree->setInputCloud(std::make_shared<Cloud3d>(source_cloud));
  clustering.setClusterTolerance(EUCLIDEAN_CLUSTERING_CLS_TOR); // 許容範囲
  clustering.setMinClusterSize(EUCLIDEAN_CLUSTERING_CLS_MIN);
  clustering.setMaxClusterSize(EUCLIDEAN_CLUSTERING_CLS_MAX);
  clustering.setSearchMethod(tree);
  clustering.setInputCloud(std::make_shared<Cloud3d>(source_cloud));
  clustering.extract(cluster_indices); // clusteringはここで実行される。クラスタ結果はインデックスで返されるぽい

  std::for_each(cluster_indices.begin(), cluster_indices.end(), [&](auto cluster) {
    Cloud3d output_pcl_cloud;
    output_pcl_cloud.width = cluster.indices.size();
    output_pcl_cloud.height = 1;
    output_pcl_cloud.is_dense = false;
    output_pcl_cloud.points.resize(output_pcl_cloud.width * output_pcl_cloud.height);
    for (size_t i = 0; i < cluster.indices.size(); ++i) {
      pcl::PointXYZ p;
      p.x = source_cloud.points[cluster.indices[i]].x;
      p.y = source_cloud.points[cluster.indices[i]].y;
      p.z = source_cloud.points[cluster.indices[i]].z;
      output_pcl_cloud.points.at(i) = p;
    }
    pcl_clusters.push_back(std::make_shared<Cloud3d>(output_pcl_cloud));
  });

  return pcl_clusters;
}

/**
 * @brief Euclidean clustering using 2d-cloud.
 *
 * @param input_pcl_cloud_2d [in] 2d pcl_cloud
 * @param input_pcl_cloud_3d [in] 3d(original) pcl_cloud
 * @return PCLPtrCluster std::vector<std::shared_ptr<Cloud3>>
 */
PCLPtrCluster AimAssistant::euclidean_clustering(Cloud3d &source_pcl_cloud_2d, Cloud3d &source_pcl_cloud_3d) {
  PCLPtrCluster pcl_clusters = std::vector<std::shared_ptr<Cloud3d>>();
  std::vector<pcl::PointIndices> cluster_indices;
  pcl::EuclideanClusterExtraction<Point3d> clustering;
  typename pcl::search::KdTree<Point3d>::Ptr tree = std::make_shared<pcl::search::KdTree<Point3d>>();
  tree->setInputCloud(std::make_shared<Cloud3d>(source_pcl_cloud_2d));
  clustering.setClusterTolerance(EUCLIDEAN_CLUSTERING_CLS_TOR);
  clustering.setMinClusterSize(EUCLIDEAN_CLUSTERING_CLS_MIN);
  clustering.setMaxClusterSize(EUCLIDEAN_CLUSTERING_CLS_MAX);
  clustering.setSearchMethod(tree);
  clustering.setInputCloud(std::make_shared<Cloud3d>(source_pcl_cloud_2d));
  clustering.extract(cluster_indices);

  std::for_each(cluster_indices.begin(), cluster_indices.end(), [&](auto cluster) {
    Cloud3d output_pcl_cloud;
    output_pcl_cloud.width = cluster.indices.size();
    output_pcl_cloud.height = 1;
    output_pcl_cloud.is_dense = false;
    output_pcl_cloud.points.resize(output_pcl_cloud.width * output_pcl_cloud.height);
    for (size_t i = 0; i < cluster.indices.size(); ++i) {
      pcl::PointXYZ p;
      p.x = source_pcl_cloud_3d.points[cluster.indices[i]].x;
      p.y = source_pcl_cloud_3d.points[cluster.indices[i]].y;
      p.z = source_pcl_cloud_3d.points[cluster.indices[i]].z;
      output_pcl_cloud.points.at(i) = p;
    }
    pcl_clusters.push_back(std::make_shared<Cloud3d>(output_pcl_cloud));
  });

  return pcl_clusters;
}

/*==================POINT CLOUD CLUSTERING END==================*/

/*==================POINT CLOUD SEGMENTATION==================*/

/**
 * @brief Pole estimation by circle2d fitting.
 *
 * @param input_clusters [in] PCLPtrCluster. the result of clustering.
 * @return PoleCluster candidates of pole.
 */
PoleCluster AimAssistant::estimate_pole(PCLPtrCluster &input_clusters) {
  PoleCluster poles;

  std::for_each(input_clusters.begin(), input_clusters.end(), [&](auto cluster) {
    auto circle = circle2d_fitting(cluster);
    if (circle.has_value()) {
      auto [circle_coef, score] = *circle;
      if (circle_coef->values.at(2) > SEG_CIR_RADIUS_LIM.at(0) - 0.01 &&
          circle_coef->values.at(2) < SEG_CIR_RADIUS_LIM.at(1) + 0.01) {
        Vector2 pole_center_vec(circle_coef->values.at(0), circle_coef->values.at(1));
        Pole pole(pole_center_vec, circle_coef->values.at(2));
        pole.score = score;
        poles.push_back(pole);
      }
    }
  });

  return poles;
}

/**
 * @brief Circle2d fitting as pole-detection. using RANSAC algorithms. Currently, the method doesn't using plane
 * extraction.
 *
 * @param input_cloud
 * @return SegModelType pair of circle coef and inliers (x, y, r)
 */
std::optional<SegModelType> AimAssistant::circle2d_fitting(std::shared_ptr<Cloud3d> &source_cloud_ptr) {
  pcl::NormalEstimation<Point3d, pcl::Normal> ne;
  pcl::search::KdTree<Point3d>::Ptr tree = std::make_shared<pcl::search::KdTree<Point3d>>();
  pcl::SACSegmentationFromNormals<Point3d, pcl::Normal> seg;
  pcl::ModelCoefficients::Ptr coefficients_circle = std::make_shared<pcl::ModelCoefficients>();
  pcl::PointIndices::Ptr inliers_circle = std::make_shared<pcl::PointIndices>();

  // 法線ベクトルを求める
  pcl::PointCloud<pcl::Normal>::Ptr cloud_normals = std::make_shared<pcl::PointCloud<pcl::Normal>>();
  ne.setSearchMethod(tree);
  ne.setInputCloud(source_cloud_ptr);
  ne.setKSearch(SEG_CIR_K_NEAREST_NEIGHBORS); // k近傍点
  ne.compute(*cloud_normals);

  Eigen::Vector3f axis = Eigen::Vector3f(0.0, 0.0, 1.0);
  seg.setOptimizeCoefficients(true);
  seg.setAxis(axis);
  seg.setModelType(pcl::SACMODEL_CIRCLE2D);
  seg.setMethodType(pcl::SAC_RANSAC);
  seg.setNormalDistanceWeight(SEG_CIR_NORMAL_DISTANCE_WEIGHT);
  seg.setMaxIterations(SEG_CIR_MAX_ITERATIONS);
  seg.setDistanceThreshold(SEG_CIR_DISTANCE_THRESHOLD);
  seg.setRadiusLimits(SEG_CIR_RADIUS_LIM.at(0), SEG_CIR_RADIUS_LIM.at(1));
  seg.setInputCloud(source_cloud_ptr);
  seg.setInputNormals(cloud_normals);

  seg.segment(*inliers_circle, *coefficients_circle);

  SegModelType ret = {coefficients_circle, inliers_circle->indices.size() / source_cloud_ptr->points.size()};

  if (ret.first->values.empty()) {
    return std::nullopt;
  } else {
    return ret;
  }
}

/*==================POINT CLOUD SEGMENTATION END==================*/

/*==================POLE MATCHING==================*/

/**
 * @brief Matching pole-candidate to ideal.
 *
 * @param candidate [in] std::vector<Circle>
 * @return clusters Result of matching. The coordinates bases on "map".
 */
std::array<Pole, POLE_NUM> AimAssistant::matching_poles(PoleCluster &candidate) {
  using namespace cpp_general;
  std::array<Pole, POLE_NUM> current_detected_pole_array;
  ignore_duplication(candidate);

  // #ifndef UNIMPLEMENTED
  std_msgs::msg::Header pole_header;
  pole_header.frame_id = "map";
  pole_header.stamp = this->get_clock()->now();

  PoleClusters clusters;

  for (auto &pole : candidate) {
    Vector2 map_base_center_vec = pole.get_center_vec();
    std::array<double, POLE_NUM> distance_from_ideal_poles;
    std::array<Vector2, POLE_NUM> diff_vector_from_ideal_poles;
    size_t distance_arr_idx = 0;

    std::for_each(field_.POLE.begin(), field_.POLE.end(), [&](auto ideal_pole) {
      auto [ideal_pole_pos, ideal_pole_type] = ideal_pole;
      double dist = Vector2::distance(map_base_center_vec, ideal_pole_pos);
      diff_vector_from_ideal_poles.at(distance_arr_idx) = map_base_center_vec - ideal_pole_pos;
      distance_from_ideal_poles.at(distance_arr_idx++) = dist;
    });

    size_t matching_result_idx =
        std::distance(distance_from_ideal_poles.begin(),
                      std::min_element(distance_from_ideal_poles.begin(), distance_from_ideal_poles.end()));

    // 理想位置との距離が閾値以上なら省く
    if (distance_from_ideal_poles.at(matching_result_idx) < MATCH_DISTANCE_THRSHOLD) {
      pole.error_from_ideal = distance_from_ideal_poles.at(matching_result_idx);
      pole.error_from_ideal_vec = diff_vector_from_ideal_poles.at(matching_result_idx);
      pole.type = std::get<1>(field_.POLE.at(matching_result_idx));
      pole.id = matching_result_idx;

      pole.radius = (pole.type == 3) ? 0.075 : 0.05;

      clusters.at(matching_result_idx).push_back(pole);
    }
  }

  for (size_t i = 0; i < clusters.size(); ++i) {
    if (!clusters.at(i).empty()) {
      std::sort(clusters.at(i).begin(), clusters.at(i).end(), [](auto a, auto b) { return a.score > b.score; });
      current_detected_pole_array.at(i) = clusters.at(i).front();
    } else {
      Pole lost_pole(0, 0, 0);
      lost_pole.is_lost = true;
      current_detected_pole_array.at(i) = lost_pole;
    }
  }

  return current_detected_pole_array;
}

/**
 * @brief Ignoring duplication by relative pose of pole.
 *
 * @param candidate
 */
void AimAssistant::ignore_duplication(PoleCluster &candidate) {
  using namespace cpp_general;
  PoleCluster filtered_candidate;
  Pole tmp_pole;

  std::vector<bool> skip_idx;
  skip_idx.resize(candidate.size());
  skip_idx.assign(candidate.size(), false);

  for (size_t i = 0; i < candidate.size(); ++i) {
    if (!skip_idx.at(i)) {
      for (size_t j = 0; j < candidate.size(); ++j) {
        if (!skip_idx.at(j) && i != j) {
          if (Vector2::distance(candidate.at(i).get_center_vec(), candidate.at(j).get_center_vec()) <
              MATCH_DISTANCE_THRSHOLD) {
            if (candidate.at(j).score < candidate.at(i).score) {
              skip_idx.at(j) = true;
            } else {
              skip_idx.at(i) = true;
              break;
            }
          }
        }
      }
    }
  }

  for (size_t i = 0; i < candidate.size(); ++i) {
    if (!skip_idx.at(i)) {
      filtered_candidate.push_back(candidate.at(i));
    }
  }

  candidate = filtered_candidate;
}

/**
 * @brief Filtering detected pole position in three way.
 *
 * - two_point_avg
 * - kalman_fitler
 * - lowpass_filter
 * @param pr_pole_array [in/out] detected_poles archive, actually publish this value
 * @param cr_pole_array [in/out] poles detected in this time. this is unpublished data
 */
void AimAssistant::filter_pole_pos(std::array<Pole, POLE_NUM> &pr_pole_array,
                                   std::array<Pole, POLE_NUM> &cr_pole_array) {

  std::function<std::pair<double, double>(Pole &, Pole &, size_t)> correction_func;

  switch (pole_correction_method_) {
  case CorrectionMethod::TWO_POINT_AVG:
    correction_func = [&](Pole &pr, Pole &cr, size_t id) -> std::pair<double, double> {
      if (cr.is_lost) {
        // return {pr.center_x, pr.center_y};
        // return ideal
        return {ref_ideal_poles_.at(id).center_x, ref_ideal_poles_.at(id).center_y};
      }
      cpp_general::Vector2 ref = (pr.get_center_vec() + cr.get_center_vec()) / 2.0;
      return {ref.x, ref.y};
    };
    break;

  case CorrectionMethod::KALMAN_FILTER:
    correction_func = [&](Pole &pr, Pole &cr, size_t id) -> std::pair<double, double> {
      Eigen::Vector2d u = Eigen::Vector2d::Zero();
      Eigen::Vector2d z;
      if (cr.is_lost) {
        z << pr.center_x, pr.center_y;
      } else {
        z << cr.center_x, cr.center_y;
      }
      auto estimated_pole_pos = pole_kf_filter_.at(id).filtering(u, z);
      return {estimated_pole_pos(0), estimated_pole_pos(1)};
    };
    break;

  case CorrectionMethod::LOWPASS_FILTER:
    correction_func = [&](Pole &pr, Pole &cr, size_t id) -> std::pair<double, double> {
      if (cr.is_lost) {
        return {pr.center_x, pr.center_y};
      }

      double estimated_x = pole_lp_filter_.at(id).at(0).filtering(cr.center_x);
      double estimated_y = pole_lp_filter_.at(id).at(1).filtering(cr.center_y);

      return {estimated_x, estimated_y};
    };
    break;

  case CorrectionMethod::NO_FILTERING:
    correction_func = [&](Pole &pr, Pole &cr, size_t id) -> std::pair<double, double> {
      if (cr.is_lost) {
        return {pr.center_x, pr.center_y};
      }
      return {cr.center_x, cr.center_y};
    };
    break;
  }

  for (size_t i = 0; i < POLE_NUM; ++i) {
    Pole cr_pole = cr_pole_array.at(i);
    Pole pr_pole = pr_pole_array.at(i);
    auto [estimated_pole_pos_x, estimated_pole_pos_y] = correction_func(pr_pole, cr_pole, i);
    pr_pole_array.at(i).center_x = estimated_pole_pos_x;
    pr_pole_array.at(i).center_y = estimated_pole_pos_y;
  }
}

/**
 * @brief Filtering by two-point-average. Correct current pole data from 1-frame-old data.
 *
 * @param pr_pole_array [in/out] 1-frame-old pole_array
 * @param cr_pole_array [in/out] current pole_array
 */
void AimAssistant::two_point_average_filter(std::array<Pole, POLE_NUM> &pr_pole_array,
                                            std::array<Pole, POLE_NUM> &cr_pole_array) {
  for (size_t i = 0; i < POLE_NUM; ++i) {
    Pole cr_pole = cr_pole_array.at(i);
    Pole pr_pole = pr_pole_array.at(i);
    if (!pr_pole.is_lost && !cr_pole.is_lost) {
      Vector2 ref = (cr_pole.get_center_vec() + pr_pole.get_center_vec()) / 2.0;
      cr_pole.center_x = ref.x;
      cr_pole.center_y = ref.y;
      pr_pole_array.at(i) = cr_pole;
      cr_pole_array.at(i) = cr_pole;
    } else if (!pr_pole.is_lost) {
      cr_pole_array.at(i) = pr_pole;
    } else {
      pr_pole_array.at(i) = cr_pole;
    }
  }
}

/**
 * @brief Interpolation line
 *
 * @param interpolation_line
 * @return true
 * @return false
 */
bool AimAssistant::is_vertical_line_interpolation(const std::vector<size_t> &interpolation_line) {
  size_t judge_cnt = 0;
  for (const auto &p : interpolation_line) {
    if (p == 1 || p == 4 || p == 10) {
      judge_cnt++;
    }
  }
  return (judge_cnt > 1);
}

/**
 * @brief Interpolate lost pole using already detected pole.
 *
 * @param pole_array
 */
void AimAssistant::interpolate_pole(std::array<Pole, POLE_NUM> &pole_array) {
  using namespace cpp_general;
  for (size_t i = 0; i < POLE_NUM; ++i) {
    if (pole_array.at(i).is_lost) {
      if (i == 10) {
        RCLCPP_INFO(this->get_logger(), "type3 pole is lost");
      }
      std::vector<size_t> interpolate_vector_line = {};
      for (const auto interpolate_line : interpolatable_pole_id_.at(i)) {
        // 補間用ラインから死んでるやつを省く
        std::vector<size_t> tmp_line = interpolate_line;
        std::vector<size_t>::iterator itr = tmp_line.begin();
        while (itr != tmp_line.end()) {
          if (pole_array.at(*itr).is_lost || pole_array.at(*itr).is_interpolated_pole) {
            itr = tmp_line.erase(itr);
          } else {
            itr++;
          }
        }

        if (tmp_line.size() >= 2) {
          interpolate_vector_line = tmp_line;
          // RCLCPP_INFO(this->get_logger(), "Pole %d is interpolatable.",i);
          break;
        }
      }

      if (!interpolate_vector_line.empty()) {
        Pole min_pole;
        Pole max_pole;
        // RCLCPP_INFO(this->get_logger(), "===============================");
        // for(const auto &id : interpolate_vector_line){
        //   RCLCPP_INFO(this->get_logger(), "id : %d", id);
        // }
        // RCLCPP_INFO(this->get_logger(), "===============================");
        // ここ縦補間かどうかで分けなきゃいけない
        // この時点ですでに二個以上の補間用ポールが揃ってる
        size_t tmp_min;
        size_t tmp_max;
        if (is_vertical_line_interpolation(interpolate_vector_line)) {
          // RCLCPP_INFO(this->get_logger(), "vertical line interpolation");
          // 縦補間
          min_pole = pole_array.at(*std::min_element(
              interpolate_vector_line.begin(), interpolate_vector_line.end(),
              [&](const size_t &a, const size_t &b) { return pole_array.at(a).center_y < pole_array.at(b).center_y; }));

          max_pole = pole_array.at(*std::max_element(interpolate_vector_line.begin(), interpolate_vector_line.end(),
                                                     [&](const size_t &a, const size_t &b) {
                                                       return pole_array.at(a).center_y < pole_array.at(b).center_y;
                                                       // --csv --qos-history keep_all --qos-reliability reliable >
                                                       // output.csv
                                                     }));
          // tmp_min = *std::min_element(interpolate_vector_line.begin(), interpolate_vector_line.end(),
          //                                            [&](const size_t &a, const size_t &b) {
          //                                              return pole_array.at(a).center_y <
          //                                              pole_array.at(b).center_y;
          //                                            });

          // tmp_max = *std::max_element(interpolate_vector_line.begin(), interpolate_vector_line.end(),
          //                                            [&](const size_t &a, const size_t &b) {
          //                                              return pole_array.at(a).center_y <
          //                                              pole_array.at(b).center_y;
          //  });
        } else {
          // RCLCPP_INFO(this->get_logger(), "side line interpolation");
          // 横・斜め補間
          min_pole = pole_array.at(*std::min_element(
              interpolate_vector_line.begin(), interpolate_vector_line.end(),
              [&](const size_t &a, const size_t &b) { return pole_array.at(a).center_x < pole_array.at(b).center_x; }));

          max_pole = pole_array.at(*std::max_element(
              interpolate_vector_line.begin(), interpolate_vector_line.end(),
              [&](const size_t &a, const size_t &b) { return pole_array.at(a).center_x < pole_array.at(b).center_x; }));
          // tmp_min = *std::min_element(interpolate_vector_line.begin(), interpolate_vector_line.end(),
          //                                            [&](const size_t &a, const size_t &b) {
          //                                              return pole_array.at(a).center_x <
          //                                              pole_array.at(b).center_x;
          //                                            });

          // tmp_max = *std::max_element(interpolate_vector_line.begin(), interpolate_vector_line.end(),
          //                                            [&](const size_t &a, const size_t &b) {
          //                                              return pole_array.at(a).center_x <
          //                                              pole_array.at(b).center_x;
          //                                            });
        }

        // RCLCPP_INFO(this->get_logger(), "min_id : %d, max_id : %d", tmp_min, tmp_max);
        // RCLCPP_INFO(this->get_logger(), "min(%lf, %lf), max(%lf, %lf)", min_pole.center_x, min_pole.center_y,
        // max_pole.center_x, max_pole.center_y);

        // 大幅にずれたときのために一応理想位置との差分をとっとくros2 topic echo /YOUR_TOPIC your_msgs/msg/YourMsgType
        Vector2 diff_avg = (min_pole.error_from_ideal_vec + max_pole.error_from_ideal_vec) / 2.0;
        Vector2 interpolate_target = std::get<0>(field_.POLE.at(i)) + diff_avg; // 大まかに合わせる
        Vector2 base_vector = min_pole.get_center_vec();
        Vector2 sub_vector = max_pole.get_center_vec();
        Pole base_pole = min_pole;
        Pole sub_pole = max_pole;

        // if (i == 10) {
        //   // RCLCPP_INFO(this->get_logger(), "min_pole id %d, max_pole id %d", min_pole.id, max_pole.id);
        // }

        // 補間対象がminより小さいなら
        // これも縦横で分けなきゃだめ
        double target_axis = 0.0;
        double min_axis = 0.0;
        if (is_vertical_line_interpolation(interpolate_vector_line)) {
          target_axis = interpolate_target.y;
          min_axis = min_pole.center_y;
        } else {
          target_axis = interpolate_target.x;
          min_axis = min_pole.center_x;
        }

        if (target_axis < min_axis) {
          swap(base_vector, sub_vector);
          swap(base_pole, sub_pole);
        }

        // RCLCPP_INFO(this->get_logger(), "Interpolation base-vector (x, y) = (%lf, %lf)", base_vector.x,
        // base_vector.y);

        Vector2 interpolate_dir = (sub_vector - base_vector) / ((sub_vector - base_vector).norm());
        // 補間基準ポールと補間対象との理想距離
        double ideal_distance =
            Vector2::distance(std::get<0>(field_.POLE.at(base_pole.id)), std::get<0>(field_.POLE.at(i)));
        interpolate_dir *= ideal_distance;

        Vector2 interpolate_result_vector = base_vector + interpolate_dir;

        Pole pole(interpolate_result_vector.x, interpolate_result_vector.y,
                  ((std::get<1>(field_.POLE.at(i)) == 3) ? 0.075 : 0.05));
        pole.type = std::get<1>(field_.POLE.at(i));
        pole.id = i;
        pole.is_interpolated_pole = true;

        // 補間ポールをarrayに入れる
        pole_array.at(i) = pole;
      } else {
        // RCLCPP_INFO(this->get_logger(), "line empty , cannot interpolate");
      }
    }
  }

  // RCLCPP_INFO(this->get_logger(), "type 3 pole interpolation status %d", pole_array.at(10).is_interpolated_pole);
}

/*==================POLE MATCHING END==================*/

/*==================OTHERS==================*/

double AimAssistant::diff_from_ideal(Pole &detected_pole) {
  size_t id = detected_pole.id;
  return (detected_pole.get_center_vec() - std::get<0>(field_.POLE.at(id))).norm();
}

/**
 * @brief Visualize pole array
 *
 * @param pole_array
 * @param marker_array
 */
void AimAssistant::visualize_pole_array(aim_assistant_msgs::msg::PoleArray &pole_array,
                                        visualization_msgs::msg::MarkerArray &marker_array) {
  visualization_msgs::msg::Marker pole_marker;
  pole_marker.header.frame_id = "map";
  pole_marker.header.stamp = this->get_clock()->now();
  pole_marker.ns = "detected_pole";
  pole_marker.action = visualization_msgs::msg::Marker::ADD;
  pole_marker.type = visualization_msgs::msg::Marker::CYLINDER;
  pole_marker.color = set_color(1.0, 0.0, 1.0, 0.0);
  pole_marker.pose.orientation = set_orientation(0.0, 0.0, 0.0, 1.0);
  for (auto &pole : pole_array.poles) {
    pole_marker.id++;
    if (dbg_detected_pole_.at(pole.id).is_interpolated_pole) {
      pole_marker.color = set_color(1.0, 1.0, 0.0, 0.0);
    }
    pole_marker.pose.position.x = pole.point.x;
    pole_marker.pose.position.y = pole.point.y;
    pole_marker.pose.position.z = 0.0;
    pole_marker.scale.x = pole_marker.scale.y = (pole.type == 3) ? 0.15 : 0.1;
    pole_marker.scale.z = 3.0;
    marker_array.markers.push_back(pole_marker);
  }
}

/*==================OTHERS END==================*/

RCLCPP_COMPONENTS_REGISTER_NODE(abu2023::AimAssistant)