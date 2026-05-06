#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/vector3_stamped.hpp>
#include <geometry_msgs/msg/wrench_stamped.hpp>
#include <geometry_msgs/msg/quaternion_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>

#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <Eigen/Eigenvalues>

#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>

#include <mutex>
#include <array>
#include <chrono>
#include <cctype>
#include <deque>
#include <string>
#include <algorithm>
#include <cmath>
#include <limits>

using namespace std::chrono_literals;

class RvizVisual : public rclcpp::Node
{
public:
  struct NormalMemoryState
  {
    Eigen::Matrix3d l_n = Eigen::Matrix3d::Zero();
    Eigen::Vector3d n_est = Eigen::Vector3d::Zero();
    bool initialized = false;
  };

  struct HistorySample
  {
    rclcpp::Time stamp{0, 0, RCL_ROS_TIME};
    geometry_msgs::msg::Point origin;
    geometry_msgs::msg::Quaternion orientation;
  };

  struct TrajectorySample
  {
    rclcpp::Time stamp{0, 0, RCL_ROS_TIME};
    geometry_msgs::msg::Point point;
  };

  RvizVisual()
  : Node("rviz_visual")
  {
    // -------------------------
    // Parameters
    // -------------------------
    parent_frame_     = this->declare_parameter<std::string>("parent_frame", "world");
    cf_frame_         = this->declare_parameter<std::string>("cf_frame", "crazyflie");
    cmd_drone_frame_  = this->declare_parameter<std::string>("cmd_drone_frame", "cmd_drone");
    cmd_ee_frame_     = this->declare_parameter<std::string>("cmd_ee_frame", "cmd_end_effector");
    cmd_active_frame_ = this->declare_parameter<std::string>("cmd_active_frame", "cmd_active");
    ee_frame_         = this->declare_parameter<std::string>("ee_frame", "end_effector");

    pose_topic_   = this->declare_parameter<std::string>("pose_topic", "/crazyflie/out/pose");
    vel_topic_    = this->declare_parameter<std::string>("vel_topic", "/crazyflie/out/vel");
    acc_topic_    = this->declare_parameter<std::string>("acc_topic", "/crazyflie/out/acc");

    ee_pose_topic_ = this->declare_parameter<std::string>("ee_pose_topic", "/crazyflie/out/EE_pose");
    ee_vel_topic_  = this->declare_parameter<std::string>("ee_vel_topic", "/crazyflie/out/EE_velocity");
    ee_acc_topic_  = this->declare_parameter<std::string>("ee_acc_topic", "/crazyflie/out/EE_acceleration");

    cmd_drone_topic_ = this->declare_parameter<std::string>(
      "cmd_drone_topic", "/crazyflie/debug/cmd_drone");
    cmd_ee_topic_ = this->declare_parameter<std::string>(
      "cmd_ee_topic", "/crazyflie/debug/cmd_ee");
    cmd_active_topic_ = this->declare_parameter<std::string>(
      "cmd_active_topic", "/crazyflie/debug/cmd_active");

    mob_topic_2nd_order_ = this->declare_parameter<std::string>(
      "mob_topic_2nd_order", "/crazyflie/out/ee_applied_mob_2nd");
    mob_topic_consistency_ = this->declare_parameter<std::string>(
      "mob_topic_consistency", "/crazyflie/out/ee_applied_mob_2nd_tau");

    contact_force_topic_ = this->declare_parameter<std::string>(
      "contact_force_topic", "/crazyflie/out/EE_contact_force_filt");

    contact_frame_quat_topic_pure_ = this->declare_parameter<std::string>(
      "contact_frame_quat_topic_pure", "/estimated_contact_frame_quat_pure");
    contact_frame_quat_topic_ = this->declare_parameter<std::string>(
      "contact_frame_quat_topic", "/estimated_contact_frame_quat");
    normal_debug_metrics_topic_pure_ = this->declare_parameter<std::string>(
      "normal_debug_metrics_topic_pure", "/normal_vector/debug_metrics_pure");
    normal_debug_metrics_topic_ = this->declare_parameter<std::string>(
      "normal_debug_metrics_topic", "/normal_vector/debug_metrics");

    est_contact_frame_ = this->declare_parameter<std::string>(
      "est_contact_frame", "estimated_contact_frame");

    environment_type_ = normalizeEnvironmentType(
      this->declare_parameter<std::string>("environment.type", "cylinder"));
    environment_marker_topic_ = this->declare_parameter<std::string>(
      "environment.marker_topic", "/rviz/environment");
    wind_indicator_enable_ = this->declare_parameter<bool>("wind_indicator.enable", true);
    wind_indicator_topic_ = this->declare_parameter<std::string>(
      "wind_indicator.wind_topic", "/crazyflie/in/wind");
    wind_indicator_marker_topic_ = this->declare_parameter<std::string>(
      "wind_indicator.marker_topic", "/rviz/wind_indicator");
    wind_indicator_origin_x_ = this->declare_parameter<double>("wind_indicator.origin.x", -0.05);
    wind_indicator_origin_y_ = this->declare_parameter<double>("wind_indicator.origin.y", 0.0);
    wind_indicator_origin_z_ = this->declare_parameter<double>("wind_indicator.origin.z", 0.0);
    wind_indicator_pole_height_ = this->declare_parameter<double>("wind_indicator.pole_height", 0.08);
    wind_indicator_mast_height_ = this->declare_parameter<double>("wind_indicator.mast_height", 0.34);
    wind_indicator_bend_gain_ = this->declare_parameter<double>("wind_indicator.bend_gain", 4.0);
    wind_indicator_max_bend_ = this->declare_parameter<double>("wind_indicator.max_bend", 0.14);
    wind_indicator_flutter_gain_ = this->declare_parameter<double>("wind_indicator.flutter_gain", 0.35);
    wind_indicator_flutter_hz_ = this->declare_parameter<double>("wind_indicator.flutter_hz", 6.0);

    cylinder_pos_x_ = this->declare_parameter<double>("cylinder.pos.x", 1.5);
    cylinder_pos_y_ = this->declare_parameter<double>("cylinder.pos.y", 0.0);
    cylinder_pos_z_ = this->declare_parameter<double>("cylinder.pos.z", 0.0);
    cylinder_radius_ = this->declare_parameter<double>("cylinder.radius", 1.0);
    cylinder_half_height_ = this->declare_parameter<double>("cylinder.half_height", 10.0);
    cylinder_rgba_ = this->declare_parameter<std::vector<double>>(
      "cylinder.rgba", std::vector<double>{0.75, 0.93, 0.75, 0.25});
    if (cylinder_rgba_.size() != 4) {
      RCLCPP_WARN(
        this->get_logger(),
        "cylinder.rgba must have size 4. Falling back to [0.75, 0.93, 0.75, 0.25].");
      cylinder_rgba_ = {0.75, 0.93, 0.75, 0.25};
    }
    wall_pos_x_ = this->declare_parameter<double>("wall.pos.x", 0.2);
    wall_pos_y_ = this->declare_parameter<double>("wall.pos.y", 0.0);
    wall_pos_z_ = this->declare_parameter<double>("wall.pos.z", 0.0);
    wall_size_x_ = this->declare_parameter<double>("wall.size.x", 0.1);
    wall_size_y_ = this->declare_parameter<double>("wall.size.y", 2.5);
    wall_size_z_ = this->declare_parameter<double>("wall.size.z", 2.5);
    wall_rgba_ = this->declare_parameter<std::vector<double>>(
      "wall.rgba", std::vector<double>{0.75, 0.93, 0.75, 0.25});
    if (wall_rgba_.size() != 4) {
      RCLCPP_WARN(
        this->get_logger(),
        "wall.rgba must have size 4. Falling back to [0.75, 0.93, 0.75, 0.25].");
      wall_rgba_ = {0.75, 0.93, 0.75, 0.25};
    }

    force_scale_         = this->declare_parameter<double>("force_scale", 10.0);
    contact_force_scale_ = this->declare_parameter<double>("contact_force_scale", 10.0);
    contact_force_lpf_omega_ = this->declare_parameter<double>("contact_force_lpf_omega", 12.0);
    normal_scale_        = this->declare_parameter<double>("normal_scale", 0.2);
    ke_normal_velocity_epsilon_ = this->declare_parameter<double>(
      "ke_normal_velocity_epsilon", 0.005);
    ke_normal_force_epsilon_ = this->declare_parameter<double>(
      "ke_normal_force_epsilon", 0.005);
    ke_raw_normal_beta_n_ = this->declare_parameter<double>(
      "ke_raw_normal.beta_n", 0.3);
    ke_raw_normal_sigma_n_ = this->declare_parameter<double>(
      "ke_raw_normal.sigma_n", 0.3);
    ke_projected_normal_beta_n_ = this->declare_parameter<double>(
      "ke_projected_normal.beta_n", 0.3);
    ke_projected_normal_sigma_n_ = this->declare_parameter<double>(
      "ke_projected_normal.sigma_n", 0.3);
    history_axis_scale_  = this->declare_parameter<double>("history_axis_scale", 0.08);
    history_sample_period_ = this->declare_parameter<double>("history_sample_period", 1.5);
    history_duration_    = this->declare_parameter<double>("history_duration", 30.0);

    drone_vel_scale_ = this->declare_parameter<double>("drone_vel_scale", 1.0);
    drone_acc_scale_ = this->declare_parameter<double>("drone_acc_scale", 1.0);
    ee_vel_scale_    = this->declare_parameter<double>("ee_vel_scale", 1.0);
    ee_acc_scale_    = this->declare_parameter<double>("ee_acc_scale", 1.0);

    arrow_shaft_diam_ = this->declare_parameter<double>("arrow_shaft_diam", 0.01);
    arrow_head_diam_  = this->declare_parameter<double>("arrow_head_diam", 0.02);
    arrow_head_len_   = this->declare_parameter<double>("arrow_head_len", 0.04);

    publish_hz_ = this->declare_parameter<double>("publish_hz", 20.0);

    // -------------------------
    // TF broadcaster
    // -------------------------
    tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

    // -------------------------
    // Subscribers
    // -------------------------
    sub_pose_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
      pose_topic_, 10,
      std::bind(&RvizVisual::cb_pose, this, std::placeholders::_1));

    sub_vel_ = this->create_subscription<geometry_msgs::msg::Vector3Stamped>(
      vel_topic_, 10,
      std::bind(&RvizVisual::cb_vel, this, std::placeholders::_1));

    sub_acc_ = this->create_subscription<geometry_msgs::msg::Vector3Stamped>(
      acc_topic_, 10,
      std::bind(&RvizVisual::cb_acc, this, std::placeholders::_1));

    sub_ee_pose_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
      ee_pose_topic_, 10,
      std::bind(&RvizVisual::cb_ee_pose, this, std::placeholders::_1));

    sub_ee_vel_ = this->create_subscription<geometry_msgs::msg::Vector3Stamped>(
      ee_vel_topic_, 10,
      std::bind(&RvizVisual::cb_ee_vel, this, std::placeholders::_1));

    sub_ee_acc_ = this->create_subscription<geometry_msgs::msg::Vector3Stamped>(
      ee_acc_topic_, 10,
      std::bind(&RvizVisual::cb_ee_acc, this, std::placeholders::_1));

    sub_cmd_drone_pose_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
      cmd_drone_topic_, 10,
      std::bind(&RvizVisual::cb_cmd_drone_pose, this, std::placeholders::_1));

    sub_cmd_ee_pose_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
      cmd_ee_topic_, 10,
      std::bind(&RvizVisual::cb_cmd_ee_pose, this, std::placeholders::_1));

    sub_cmd_active_pose_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
      cmd_active_topic_, 10,
      std::bind(&RvizVisual::cb_cmd_active_pose, this, std::placeholders::_1));

    sub_mob_2nd_order_ = this->create_subscription<geometry_msgs::msg::WrenchStamped>(
      mob_topic_2nd_order_, 10,
      std::bind(&RvizVisual::cb_mob_2nd_order, this, std::placeholders::_1));
    sub_mob_consistency_ = this->create_subscription<geometry_msgs::msg::WrenchStamped>(
      mob_topic_consistency_, 10,
      std::bind(&RvizVisual::cb_mob_consistency, this, std::placeholders::_1));

    sub_contact_force_ = this->create_subscription<geometry_msgs::msg::WrenchStamped>(
      contact_force_topic_, 10,
      std::bind(&RvizVisual::cb_contact_force, this, std::placeholders::_1));

    sub_contact_frame_quat_pure_ = this->create_subscription<geometry_msgs::msg::QuaternionStamped>(
      contact_frame_quat_topic_pure_, 10,
      std::bind(&RvizVisual::cb_contact_frame_quat_pure, this, std::placeholders::_1));
    sub_contact_frame_quat_ = this->create_subscription<geometry_msgs::msg::QuaternionStamped>(
      contact_frame_quat_topic_, 10,
      std::bind(&RvizVisual::cb_contact_frame_quat, this, std::placeholders::_1));
    sub_normal_debug_metrics_pure_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
      normal_debug_metrics_topic_pure_, 10,
      std::bind(&RvizVisual::cb_normal_debug_metrics_pure, this, std::placeholders::_1));
    sub_normal_debug_metrics_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
      normal_debug_metrics_topic_, 10,
      std::bind(&RvizVisual::cb_normal_debug_metrics, this, std::placeholders::_1));
    sub_wind_indicator_ = this->create_subscription<geometry_msgs::msg::Vector3Stamped>(
      wind_indicator_topic_, 10,
      std::bind(&RvizVisual::cb_wind_indicator, this, std::placeholders::_1));

    // -------------------------
    // Publishers (Markers)
    // -------------------------
    pub_mob_2nd_order_arrow_ = this->create_publisher<visualization_msgs::msg::Marker>(
      "/rviz/drone_external_force_mob_2nd_order", 10);
    pub_mob_consistency_arrow_ = this->create_publisher<visualization_msgs::msg::Marker>(
      "/rviz/drone_external_force_mob_2nd_tau", 10);

    pub_contact_arrow_ = this->create_publisher<visualization_msgs::msg::Marker>(
      "/rviz/EE_contact_force", 10);
    pub_estimated_normal_pure_arrow_ = this->create_publisher<visualization_msgs::msg::Marker>(
      "/rviz/estimated_contact_normal_pure", 10);
    pub_estimated_normal_arrow_ = this->create_publisher<visualization_msgs::msg::Marker>(
      "/rviz/estimated_contact_normal_ke", 10);
    pub_true_normal_arrow_ = this->create_publisher<visualization_msgs::msg::Marker>(
      "/rviz/true_contact_normal", 10);
    pub_ke_force_normal_raw_arrow_ = this->create_publisher<visualization_msgs::msg::Marker>(
      "/rviz/ke_force_normal_raw", 10);
    pub_ke_force_normal_projected_arrow_ = this->create_publisher<visualization_msgs::msg::Marker>(
      "/rviz/ke_force_normal_projected", 10);
    pub_ke_gamma_normal_arrow_ = this->create_publisher<visualization_msgs::msg::Marker>(
      "/rviz/ke_gamma_normal", 10);
    pub_force_based_normal_arrow_ = this->create_publisher<visualization_msgs::msg::Marker>(
      "/rviz/force_based_normal", 10);
    pub_lf_only_normal_arrow_ = this->create_publisher<visualization_msgs::msg::Marker>(
      "/rviz/lf_only_estimated_normal", 10);
    pub_lv_only_normal_arrow_ = this->create_publisher<visualization_msgs::msg::Marker>(
      "/rviz/lv_only_estimated_normal", 10);
    pub_contact_history_markers_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
      "/rviz/contact_frame_history", 10);

    pub_drone_vel_arrow_ = this->create_publisher<visualization_msgs::msg::Marker>(
      "/rviz/drone_velocity", 10);

    pub_drone_acc_arrow_ = this->create_publisher<visualization_msgs::msg::Marker>(
      "/rviz/drone_acceleration", 10);

    pub_ee_vel_arrow_ = this->create_publisher<visualization_msgs::msg::Marker>(
      "/rviz/EE_velocity", 10);

    pub_ee_acc_arrow_ = this->create_publisher<visualization_msgs::msg::Marker>(
      "/rviz/EE_acceleration", 10);
    pub_environment_marker_ = this->create_publisher<visualization_msgs::msg::Marker>(
      environment_marker_topic_, 10);
    pub_wind_indicator_markers_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
      wind_indicator_marker_topic_, 10);

    // -------------------------
    // Timer
    // -------------------------
    const auto period = std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::duration<double>(1.0 / std::max(1e-6, publish_hz_)));

    timer_ = this->create_wall_timer(
      period, std::bind(&RvizVisual::loop_publish, this));

    RCLCPP_INFO(this->get_logger(), "rviz_visual started.");
    RCLCPP_INFO(this->get_logger(), "Sub drone: %s | %s | %s",
                pose_topic_.c_str(), vel_topic_.c_str(), acc_topic_.c_str());
    RCLCPP_INFO(this->get_logger(), "Sub EE: %s | %s | %s",
                ee_pose_topic_.c_str(), ee_vel_topic_.c_str(), ee_acc_topic_.c_str());
    RCLCPP_INFO(this->get_logger(), "Sub cmd pose: %s | %s | %s",
                cmd_drone_topic_.c_str(), cmd_ee_topic_.c_str(), cmd_active_topic_.c_str());
    RCLCPP_INFO(this->get_logger(), "Sub external/contact: %s | %s | %s",
                mob_topic_2nd_order_.c_str(),
                mob_topic_consistency_.c_str(), contact_force_topic_.c_str());
  }

private:
  // =========================
  // Callbacks
  // =========================
  void cb_pose(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lk(mtx_);
    pose_ = *msg;
    have_pose_ = true;
  }

  void cb_vel(const geometry_msgs::msg::Vector3Stamped::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lk(mtx_);
    vel_ = *msg;
    have_vel_ = true;
  }

  void cb_acc(const geometry_msgs::msg::Vector3Stamped::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lk(mtx_);
    acc_ = *msg;
    have_acc_ = true;
  }

  void cb_ee_pose(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lk(mtx_);
    ee_pose_ = *msg;
    have_ee_pose_ = true;
  }

  void cb_ee_vel(const geometry_msgs::msg::Vector3Stamped::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lk(mtx_);
    ee_vel_ = *msg;
    have_ee_vel_ = true;
  }

  void cb_ee_acc(const geometry_msgs::msg::Vector3Stamped::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lk(mtx_);
    ee_acc_ = *msg;
    have_ee_acc_ = true;
  }

  void cb_cmd_drone_pose(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lk(mtx_);
    cmd_drone_pose_ = *msg;
    have_cmd_drone_pose_ = true;
  }

  void cb_cmd_ee_pose(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lk(mtx_);
    cmd_ee_pose_ = *msg;
    have_cmd_ee_pose_ = true;
  }

  void cb_cmd_active_pose(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lk(mtx_);
    cmd_active_pose_ = *msg;
    have_cmd_active_pose_ = true;
  }

  void cb_mob_2nd_order(const geometry_msgs::msg::WrenchStamped::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lk(mtx_);
    mob_force_2nd_order_[0] = static_cast<float>(msg->wrench.force.x);
    mob_force_2nd_order_[1] = static_cast<float>(msg->wrench.force.y);
    mob_force_2nd_order_[2] = static_cast<float>(msg->wrench.force.z);
    have_mob_force_2nd_order_ = true;
  }

  void cb_mob_consistency(const geometry_msgs::msg::WrenchStamped::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lk(mtx_);
    mob_force_consistency_[0] = static_cast<float>(msg->wrench.force.x);
    mob_force_consistency_[1] = static_cast<float>(msg->wrench.force.y);
    mob_force_consistency_[2] = static_cast<float>(msg->wrench.force.z);
    have_mob_force_consistency_ = true;
  }

  void cb_contact_force(const geometry_msgs::msg::WrenchStamped::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lk(mtx_);
    const std::array<float, 3> raw_force = {
      static_cast<float>(msg->wrench.force.x),
      static_cast<float>(msg->wrench.force.y),
      static_cast<float>(msg->wrench.force.z)};
    rclcpp::Time sample_time = msg->header.stamp;
    if (sample_time.nanoseconds() == 0) {
      sample_time = this->now();
    }

    if (!have_contact_filter_state_) {
      contact_F_ = raw_force;
      have_contact_filter_state_ = true;
    } else {
      const double dt = (sample_time - last_contact_force_update_time_).seconds();
      const double clamped_dt = std::max(0.0, dt);
      const double alpha =
        1.0 - std::exp(-std::max(0.0, contact_force_lpf_omega_) * clamped_dt);
      for (size_t i = 0; i < contact_F_.size(); ++i) {
        contact_F_[i] += static_cast<float>(alpha * (raw_force[i] - contact_F_[i]));
      }
    }

    last_contact_force_update_time_ = sample_time;
    have_contact_ = true;
    contact_frame_id_ = msg->header.frame_id;
  }

  static bool extractEstimatedNormalWorld(
    const std_msgs::msg::Float64MultiArray::SharedPtr msg,
    std::array<float, 3> & normal_out)
  {
    if (msg->data.size() < 31) {
      return false;
    }

    normal_out = {
      static_cast<float>(msg->data[28]),
      static_cast<float>(msg->data[29]),
      static_cast<float>(msg->data[30])};
    return isFiniteVector(normal_out) && vectorNorm(normal_out) > 1e-9f;
  }

  void cb_contact_frame_quat_pure(const geometry_msgs::msg::QuaternionStamped::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lk(mtx_);
    contact_q_pure_ = msg->quaternion;
    have_contact_q_pure_ = true;
  }

  void cb_contact_frame_quat(const geometry_msgs::msg::QuaternionStamped::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lk(mtx_);
    contact_q_ = msg->quaternion;
    have_contact_q_ = true;
  }

  void cb_normal_debug_metrics_pure(const std_msgs::msg::Float64MultiArray::SharedPtr msg)
  {
    std::array<float, 3> estimated_normal{0.0f, 0.0f, 0.0f};
    const bool have_estimated_normal = extractEstimatedNormalWorld(msg, estimated_normal);

    std::lock_guard<std::mutex> lk(mtx_);
    estimated_normal_pure_ = estimated_normal;
    have_estimated_normal_pure_ = have_estimated_normal;
  }

  void cb_normal_debug_metrics(const std_msgs::msg::Float64MultiArray::SharedPtr msg)
  {
    if (msg->data.size() < 34) {
      return;
    }

    const std::array<float, 3> n_geo{
      static_cast<float>(msg->data[28]),
      static_cast<float>(msg->data[29]),
      static_cast<float>(msg->data[30])};
    const bool have_n_geo =
      isFiniteVector(n_geo) && vectorNorm(n_geo) > 1e-9;

    std::array<float, 3> n_f{
      static_cast<float>(msg->data[31]),
      static_cast<float>(msg->data[32]),
      static_cast<float>(msg->data[33])};
    const bool have_n_f = isFiniteVector(n_f) && vectorNorm(n_f) > 1e-9;

    std::array<float, 3> n_projected{0.0f, 0.0f, 0.0f};
    bool have_n_projected = false;
    if (msg->data.size() >= 37) {
      n_projected = std::array<float, 3>{
        static_cast<float>(msg->data[34]),
        static_cast<float>(msg->data[35]),
        static_cast<float>(msg->data[36])};
      have_n_projected = isFiniteVector(n_projected) && vectorNorm(n_projected) > 1e-9f;
    }

    std::array<float, 3> n_gamma{0.0f, 0.0f, 0.0f};
    bool have_n_gamma = false;
    if (msg->data.size() >= 46) {
      n_gamma = std::array<float, 3>{
        static_cast<float>(msg->data[43]),
        static_cast<float>(msg->data[44]),
        static_cast<float>(msg->data[45])};
      have_n_gamma = isFiniteVector(n_gamma) && vectorNorm(n_gamma) > 1e-9f;
    }

    std::array<float, 3> n_v{0.0f, 0.0f, 0.0f};
    bool have_n_v = false;
    if (msg->data.size() >= 19) {
      Eigen::Matrix3d l_v_bar = Eigen::Matrix3d::Zero();
      bool finite_l_v = true;
      int idx = 10;
      for (int r = 0; r < 3; ++r) {
        for (int c = 0; c < 3; ++c) {
          const double value = msg->data[idx++];
          finite_l_v = finite_l_v && std::isfinite(value);
          l_v_bar(r, c) = value;
        }
      }

      if (finite_l_v) {
        const auto n_v_eigen = velocityEvidenceNormal(l_v_bar);
        n_v = {
          static_cast<float>(n_v_eigen.x()),
          static_cast<float>(n_v_eigen.y()),
          static_cast<float>(n_v_eigen.z())};
        have_n_v = isFiniteVector(n_v) && vectorNorm(n_v) > 1e-9;
        if (have_n_v && have_n_geo && dot(n_v, n_geo) < 0.0f) {
          n_v = scaleVector(n_v, -1.0f);
        }
      }
    }

    std::lock_guard<std::mutex> lk(mtx_);
    estimated_normal_k1_ = n_geo;
    have_estimated_normal_k1_ = have_n_geo;
    force_based_normal_ = n_f;
    have_force_based_normal_ = have_n_f;
    ke_force_normal_raw_ = n_f;
    have_ke_force_normal_raw_ = have_n_f;
    ke_force_normal_projected_ = n_projected;
    have_ke_force_normal_projected_ = have_n_projected;
    ke_gamma_normal_ = n_gamma;
    have_ke_gamma_normal_ = have_n_gamma;
    lf_only_normal_ = n_f;
    have_lf_only_normal_ = have_n_f;
    lv_only_normal_ = n_v;
    have_lv_only_normal_ = have_n_v;
  }

  void cb_wind_indicator(const geometry_msgs::msg::Vector3Stamped::SharedPtr msg)
  {
    if (!msg) {
      return;
    }

    std::lock_guard<std::mutex> lk(mtx_);
    wind_indicator_force_ = {
      static_cast<float>(msg->vector.x),
      static_cast<float>(msg->vector.y),
      static_cast<float>(msg->vector.z)};
    have_wind_indicator_force_ = true;
  }

  // =========================
  // Marker helper
  // =========================
  static bool isFiniteVector(const std::array<float, 3> & v)
  {
    return std::isfinite(v[0]) && std::isfinite(v[1]) && std::isfinite(v[2]);
  }

  static float dot(const std::array<float, 3> & a, const std::array<float, 3> & b)
  {
    return a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
  }

  static float vectorNorm(const std::array<float, 3> & v)
  {
    return std::sqrt(dot(v, v));
  }

  static std::array<float, 3> scaleVector(const std::array<float, 3> & v, float scale)
  {
    return {scale * v[0], scale * v[1], scale * v[2]};
  }

  static std::array<float, 3> subtractVector(
    const std::array<float, 3> & a,
    const std::array<float, 3> & b)
  {
    return {a[0] - b[0], a[1] - b[1], a[2] - b[2]};
  }

  static std::array<float, 3> normalizeVector(const std::array<float, 3> & v)
  {
    const float norm = vectorNorm(v);
    if (!(std::isfinite(norm) && norm > 1e-9f)) {
      return {0.0f, 0.0f, 0.0f};
    }
    return scaleVector(v, 1.0f / norm);
  }

  static Eigen::Vector3d toEigen(const std::array<float, 3> & v)
  {
    return Eigen::Vector3d(v[0], v[1], v[2]);
  }

  static std::array<float, 3> toArray(const Eigen::Vector3d & v)
  {
    return {
      static_cast<float>(v.x()),
      static_cast<float>(v.y()),
      static_cast<float>(v.z())};
  }

  static Eigen::Vector3d principalEigenvector(const Eigen::Matrix3d & mat)
  {
    const Eigen::Matrix3d sym = 0.5 * (mat + mat.transpose());
    if (sym.norm() < 1e-12) {
      return Eigen::Vector3d::Zero();
    }

    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> solver(sym);
    if (solver.info() != Eigen::Success) {
      return Eigen::Vector3d::Zero();
    }

    Eigen::Vector3d vec = solver.eigenvectors().col(2);
    const double norm = vec.norm();
    if (!(std::isfinite(norm) && norm > 1e-12)) {
      return Eigen::Vector3d::Zero();
    }
    return vec / norm;
  }

  static bool updateNormalMemory(
    NormalMemoryState & state,
    const std::array<float, 3> & candidate_in,
    double dt,
    double beta_n,
    double sigma_n,
    std::array<float, 3> & normal_out)
  {
    if (!(std::isfinite(dt) && dt > 0.0)) {
      return false;
    }

    Eigen::Vector3d candidate = toEigen(candidate_in);
    const double candidate_norm = candidate.norm();
    if (!(std::isfinite(candidate_norm) && candidate_norm > 1e-12)) {
      return false;
    }
    candidate /= candidate_norm;

    if (state.initialized && state.n_est.norm() > 1e-12 && state.n_est.dot(candidate) < 0.0) {
      candidate = -candidate;
    }

    Eigen::Matrix3d l_n_dot = -beta_n * state.l_n;
    l_n_dot += sigma_n * (candidate * candidate.transpose());
    state.l_n += dt * l_n_dot;

    Eigen::Vector3d n_est = principalEigenvector(state.l_n);
    if (n_est.norm() < 1e-12) {
      n_est = candidate;
    }
    if (n_est.dot(candidate) < 0.0) {
      n_est = -n_est;
    }

    state.n_est = n_est;
    state.initialized = true;
    normal_out = toArray(n_est);
    return true;
  }

  bool computeKeForceNormals(
    const std::array<float, 3> & force_world,
    const std::array<float, 3> & ee_vel_world,
    std::array<float, 3> & raw_normal_out,
    bool & have_raw_out,
    std::array<float, 3> & projected_normal_out,
    bool & have_projected_out) const
  {
    have_raw_out = false;
    have_projected_out = false;

    if (!isFiniteVector(force_world) || !isFiniteVector(ee_vel_world)) {
      return false;
    }

    const float force_norm = vectorNorm(force_world);
    if (!(std::isfinite(force_norm) && force_norm > static_cast<float>(ke_normal_force_epsilon_))) {
      return false;
    }

    raw_normal_out = normalizeVector(force_world);
    have_raw_out = vectorNorm(raw_normal_out) > 1e-9f;

    const float vel_norm = vectorNorm(ee_vel_world);
    if (!(std::isfinite(vel_norm) && vel_norm > static_cast<float>(ke_normal_velocity_epsilon_))) {
      projected_normal_out = raw_normal_out;
      have_projected_out = have_raw_out;
      return have_raw_out;
    }

    const std::array<float, 3> w_s = scaleVector(ee_vel_world, 1.0f / vel_norm);
    const std::array<float, 3> projected_force = subtractVector(
      force_world,
      scaleVector(w_s, dot(force_world, w_s)));
    const float projected_force_norm = vectorNorm(projected_force);
    if (!(std::isfinite(projected_force_norm) &&
      projected_force_norm > static_cast<float>(ke_normal_force_epsilon_)))
    {
      projected_normal_out = raw_normal_out;
      have_projected_out = have_raw_out;
      return have_raw_out;
    }

    projected_normal_out = normalizeVector(projected_force);
    have_projected_out = vectorNorm(projected_normal_out) > 1e-9f;
    if (have_raw_out && have_projected_out && dot(projected_normal_out, raw_normal_out) < 0.0f) {
      projected_normal_out = scaleVector(projected_normal_out, -1.0f);
    }
    return have_raw_out || have_projected_out;
  }

  static Eigen::Vector3d velocityEvidenceNormal(const Eigen::Matrix3d & l_v)
  {
    const Eigen::Matrix3d sym = 0.5 * (l_v + l_v.transpose());
    if (sym.norm() < 1e-12) {
      return Eigen::Vector3d::Constant(std::numeric_limits<double>::quiet_NaN());
    }

    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> solver(sym);
    if (solver.info() != Eigen::Success) {
      return Eigen::Vector3d::Constant(std::numeric_limits<double>::quiet_NaN());
    }

    Eigen::Vector3d n_v = solver.eigenvectors().col(0);
    const double norm = n_v.norm();
    if (!(std::isfinite(norm) && norm > 1e-12)) {
      return Eigen::Vector3d::Constant(std::numeric_limits<double>::quiet_NaN());
    }
    return n_v / norm;
  }

  static bool resolveNormalWorld(
    const std::array<float, 3> & metrics_normal,
    bool have_metrics_normal,
    const geometry_msgs::msg::Quaternion & quat,
    bool have_quat,
    std::array<float, 3> & normal_world_out)
  {
    if (have_metrics_normal && isFiniteVector(metrics_normal) && vectorNorm(metrics_normal) > 1e-9f) {
      normal_world_out = metrics_normal;
      return true;
    }

    if (!have_quat) {
      return false;
    }

    tf2::Quaternion q_wc(quat.x, quat.y, quat.z, quat.w);
    if (!std::isfinite(q_wc.x()) || !std::isfinite(q_wc.y()) ||
        !std::isfinite(q_wc.z()) || !std::isfinite(q_wc.w())) {
      return false;
    }
    q_wc.normalize();

    tf2::Matrix3x3 r_wc(q_wc);
    normal_world_out = {
      static_cast<float>(r_wc[0][0]),
      static_cast<float>(r_wc[1][0]),
      static_cast<float>(r_wc[2][0])};
    return isFiniteVector(normal_world_out) && vectorNorm(normal_world_out) > 1e-9f;
  }

  visualization_msgs::msg::Marker make_arrow_marker(
    const std::string &ns,
    int id,
    const std::string &frame_id,
    const rclcpp::Time &stamp,
    double x0, double y0, double z0,
    double vx, double vy, double vz,
    double scale,
    double r, double g, double b) const
  {
    visualization_msgs::msg::Marker mk;
    mk.header.stamp = stamp;
    mk.header.frame_id = frame_id;
    mk.ns = ns;
    mk.id = id;
    mk.type = visualization_msgs::msg::Marker::ARROW;
    mk.action = visualization_msgs::msg::Marker::ADD;

    geometry_msgs::msg::Point p0, p1;
    p0.x = x0; p0.y = y0; p0.z = z0;
    p1.x = x0 + scale * vx;
    p1.y = y0 + scale * vy;
    p1.z = z0 + scale * vz;

    mk.points = {p0, p1};
    mk.scale.x = arrow_shaft_diam_;
    mk.scale.y = arrow_head_diam_;
    mk.scale.z = arrow_head_len_;

    mk.color.a = 1.0;
    mk.color.r = r;
    mk.color.g = g;
    mk.color.b = b;

    mk.lifetime = rclcpp::Duration::from_seconds(0.2);
    return mk;
  }

  visualization_msgs::msg::Marker make_arrow_marker_with_dims(
    const std::string &ns,
    int id,
    const std::string &frame_id,
    const rclcpp::Time &stamp,
    double x0, double y0, double z0,
    double vx, double vy, double vz,
    double scale,
    double shaft_diam,
    double head_diam,
    double head_len,
    double r, double g, double b) const
  {
    auto mk = make_arrow_marker(
      ns, id, frame_id, stamp, x0, y0, z0, vx, vy, vz, scale, r, g, b);
    mk.scale.x = shaft_diam;
    mk.scale.y = head_diam;
    mk.scale.z = head_len;
    return mk;
  }

  visualization_msgs::msg::Marker make_delete_marker(
    const std::string & ns,
    int id,
    const std::string & frame_id,
    const rclcpp::Time & stamp) const
  {
    visualization_msgs::msg::Marker mk;
    mk.header.stamp = stamp;
    mk.header.frame_id = frame_id;
    mk.ns = ns;
    mk.id = id;
    mk.action = visualization_msgs::msg::Marker::DELETE;
    return mk;
  }

  visualization_msgs::msg::Marker make_line_list_marker(
    const std::string & ns,
    int id,
    const std::string & frame_id,
    const rclcpp::Time & stamp) const
  {
    visualization_msgs::msg::Marker mk;
    mk.header.stamp = stamp;
    mk.header.frame_id = frame_id;
    mk.ns = ns;
    mk.id = id;
    mk.type = visualization_msgs::msg::Marker::LINE_LIST;
    mk.action = visualization_msgs::msg::Marker::ADD;
    mk.scale.x = 0.004;
    mk.color.a = 1.0;
    return mk;
  }

  visualization_msgs::msg::Marker make_line_strip_marker(
    const std::string & ns,
    int id,
    const std::string & frame_id,
    const rclcpp::Time & stamp) const
  {
    visualization_msgs::msg::Marker mk;
    mk.header.stamp = stamp;
    mk.header.frame_id = frame_id;
    mk.ns = ns;
    mk.id = id;
    mk.type = visualization_msgs::msg::Marker::LINE_STRIP;
    mk.action = visualization_msgs::msg::Marker::ADD;
    mk.scale.x = 0.006;
    mk.color.a = 1.0;
    return mk;
  }

  visualization_msgs::msg::Marker make_cylinder_marker(
    const std::string & ns,
    int id,
    const std::string & frame_id,
    const rclcpp::Time & stamp) const
  {
    visualization_msgs::msg::Marker mk;
    mk.header.stamp = stamp;
    mk.header.frame_id = frame_id;
    mk.ns = ns;
    mk.id = id;
    mk.type = visualization_msgs::msg::Marker::CYLINDER;
    mk.action = visualization_msgs::msg::Marker::ADD;

    mk.pose.position.x = cylinder_pos_x_;
    mk.pose.position.y = cylinder_pos_y_;
    mk.pose.position.z = cylinder_pos_z_ + cylinder_half_height_;
    mk.pose.orientation.w = 1.0;

    mk.scale.x = 2.0 * cylinder_radius_;
    mk.scale.y = 2.0 * cylinder_radius_;
    mk.scale.z = 2.0 * cylinder_half_height_;

    mk.color.r = static_cast<float>(cylinder_rgba_[0]);
    mk.color.g = static_cast<float>(cylinder_rgba_[1]);
    mk.color.b = static_cast<float>(cylinder_rgba_[2]);
    mk.color.a = static_cast<float>(cylinder_rgba_[3]);
    mk.lifetime = rclcpp::Duration::from_seconds(0.0);
    return mk;
  }

  visualization_msgs::msg::Marker make_wall_marker(
    const std::string & ns,
    int id,
    const std::string & frame_id,
    const rclcpp::Time & stamp) const
  {
    visualization_msgs::msg::Marker mk;
    mk.header.stamp = stamp;
    mk.header.frame_id = frame_id;
    mk.ns = ns;
    mk.id = id;
    mk.type = visualization_msgs::msg::Marker::CUBE;
    mk.action = visualization_msgs::msg::Marker::ADD;

    mk.pose.position.x = wall_pos_x_;
    mk.pose.position.y = wall_pos_y_;
    mk.pose.position.z = wall_pos_z_ + wall_size_z_;
    mk.pose.orientation.w = 1.0;

    mk.scale.x = 2.0 * wall_size_x_;
    mk.scale.y = 2.0 * wall_size_y_;
    mk.scale.z = 2.0 * wall_size_z_;

    mk.color.r = static_cast<float>(wall_rgba_[0]);
    mk.color.g = static_cast<float>(wall_rgba_[1]);
    mk.color.b = static_cast<float>(wall_rgba_[2]);
    mk.color.a = static_cast<float>(wall_rgba_[3]);
    mk.lifetime = rclcpp::Duration::from_seconds(0.0);
    return mk;
  }

  visualization_msgs::msg::Marker make_environment_marker(
    const std::string & ns,
    int id,
    const std::string & frame_id,
    const rclcpp::Time & stamp) const
  {
    if (environment_type_ == "wall") {
      return make_wall_marker(ns, id, frame_id, stamp);
    }
    return make_cylinder_marker(ns, id, frame_id, stamp);
  }

  visualization_msgs::msg::MarkerArray makeWindIndicatorMarkerArray(
    const rclcpp::Time & stamp,
    const std::array<float, 3> & wind_force,
    bool have_wind_force) const
  {
    visualization_msgs::msg::MarkerArray out;

    if (!wind_indicator_enable_) {
      out.markers.push_back(make_delete_marker("wind_indicator_base", 0, parent_frame_, stamp));
      out.markers.push_back(make_delete_marker("wind_indicator_mast", 0, parent_frame_, stamp));
      out.markers.push_back(make_delete_marker("wind_indicator_tip", 0, parent_frame_, stamp));
      out.markers.push_back(make_delete_marker("wind_indicator_arrow", 0, parent_frame_, stamp));
      return out;
    }

    const double pole_height = std::max(1e-3, wind_indicator_pole_height_);
    const double mast_height = std::max(1e-3, wind_indicator_mast_height_);
    const double wx = have_wind_force ? static_cast<double>(wind_force[0]) : 0.0;
    const double wy = have_wind_force ? static_cast<double>(wind_force[1]) : 0.0;
    const double wind_xy_norm = std::hypot(wx, wy);
    const bool wind_active = wind_xy_norm > 1e-9;
    const double dir_x = wind_active ? wx / wind_xy_norm : 0.0;
    const double dir_y = wind_active ? wy / wind_xy_norm : 0.0;
    const double bend = std::clamp(
      wind_indicator_bend_gain_ * wind_xy_norm,
      0.0,
      std::max(0.0, wind_indicator_max_bend_));
    const double t = stamp.seconds();
    constexpr double kPi = 3.14159265358979323846;
    const double flutter =
      wind_active ?
      wind_indicator_flutter_gain_ * bend *
      std::sin(2.0 * kPi * wind_indicator_flutter_hz_ * t) :
      0.0;

    visualization_msgs::msg::Marker base;
    base.header.stamp = stamp;
    base.header.frame_id = parent_frame_;
    base.ns = "wind_indicator_base";
    base.id = 0;
    base.type = visualization_msgs::msg::Marker::CYLINDER;
    base.action = visualization_msgs::msg::Marker::ADD;
    base.pose.position.x = wind_indicator_origin_x_;
    base.pose.position.y = wind_indicator_origin_y_;
    base.pose.position.z = wind_indicator_origin_z_ + 0.5 * pole_height;
    base.pose.orientation.w = 1.0;
    base.scale.x = 0.014;
    base.scale.y = 0.014;
    base.scale.z = pole_height;
    base.color.r = 0.08f;
    base.color.g = 0.08f;
    base.color.b = 0.08f;
    base.color.a = 1.0f;
    base.lifetime = rclcpp::Duration::from_seconds(0.0);
    out.markers.push_back(base);

    visualization_msgs::msg::Marker mast;
    mast.header.stamp = stamp;
    mast.header.frame_id = parent_frame_;
    mast.ns = "wind_indicator_mast";
    mast.id = 0;
    mast.type = visualization_msgs::msg::Marker::LINE_STRIP;
    mast.action = visualization_msgs::msg::Marker::ADD;
    mast.scale.x = 0.010;
    mast.color.r = 0.05f;
    mast.color.g = 0.25f;
    mast.color.b = 0.90f;
    mast.color.a = 1.0f;
    mast.lifetime = rclcpp::Duration::from_seconds(0.0);

    constexpr int kSegments = 8;
    geometry_msgs::msg::Point tip_point;
    for (int i = 0; i <= kSegments; ++i) {
      const double s = static_cast<double>(i) / static_cast<double>(kSegments);
      const double eased = s * s;
      const double side = flutter * std::sin(kPi * s);

      geometry_msgs::msg::Point p;
      p.x = wind_indicator_origin_x_ + bend * eased * dir_x - side * dir_y;
      p.y = wind_indicator_origin_y_ + bend * eased * dir_y + side * dir_x;
      p.z = wind_indicator_origin_z_ + pole_height + mast_height * s;
      mast.points.push_back(p);
      tip_point = p;
    }
    out.markers.push_back(mast);

    visualization_msgs::msg::Marker tip;
    tip.header.stamp = stamp;
    tip.header.frame_id = parent_frame_;
    tip.ns = "wind_indicator_tip";
    tip.id = 0;
    tip.type = visualization_msgs::msg::Marker::SPHERE;
    tip.action = visualization_msgs::msg::Marker::ADD;
    tip.pose.position = tip_point;
    tip.pose.orientation.w = 1.0;
    tip.scale.x = 0.026;
    tip.scale.y = 0.026;
    tip.scale.z = 0.026;
    tip.color.r = 0.05f;
    tip.color.g = 0.25f;
    tip.color.b = 0.90f;
    tip.color.a = 1.0f;
    tip.lifetime = rclcpp::Duration::from_seconds(0.0);
    out.markers.push_back(tip);

    if (wind_active) {
      auto arrow = make_arrow_marker_with_dims(
        "wind_indicator_arrow", 0, parent_frame_, stamp,
        wind_indicator_origin_x_,
        wind_indicator_origin_y_,
        wind_indicator_origin_z_ + pole_height + mast_height + 0.05,
        dir_x, dir_y, 0.0,
        std::min(0.18, 0.06 + 4.0 * wind_xy_norm),
        0.006, 0.014, 0.030,
        0.10, 0.45, 1.00);
      arrow.lifetime = rclcpp::Duration::from_seconds(0.0);
      out.markers.push_back(arrow);
    } else {
      out.markers.push_back(make_delete_marker("wind_indicator_arrow", 0, parent_frame_, stamp));
    }

    return out;
  }

  std::string normalizeEnvironmentType(std::string type) const
  {
    std::transform(type.begin(), type.end(), type.begin(), [](unsigned char c) {
      return static_cast<char>(std::tolower(c));
    });
    if (type != "wall" && type != "cylinder") {
      RCLCPP_WARN(
        this->get_logger(),
        "Unknown environment.type '%s'. Falling back to 'cylinder'.",
        type.c_str());
      return "cylinder";
    }
    return type;
  }

  bool computeTrueNormalAtEE(
    const geometry_msgs::msg::PoseStamped & ee_pose,
    double & nx,
    double & ny,
    double & nz) const
  {
    (void)ee_pose;
    if (environment_type_ == "wall") {
      nx = 1.0;
      ny = 0.0;
      nz = 0.0;
      return true;
    }

    const double dx = ee_pose.pose.position.x - cylinder_pos_x_;
    const double dy = ee_pose.pose.position.y - cylinder_pos_y_;
    const double radial_norm = std::sqrt(dx * dx + dy * dy);
    if (!(std::isfinite(radial_norm) && radial_norm > 1e-9)) {
      nx = 0.0;
      ny = 0.0;
      nz = 0.0;
      return false;
    }

    nx = -dx / radial_norm;
    ny = -dy / radial_norm;
    nz = 0.0;
    return true;
  }

  void pushHistorySample(
    const geometry_msgs::msg::PoseStamped & ee_pose,
    const geometry_msgs::msg::Quaternion & contact_q,
    const rclcpp::Time & stamp)
  {
    std::lock_guard<std::mutex> lk(history_mtx_);
    const double sample_period = std::max(1e-3, history_sample_period_);
    const double keep_duration = std::max(sample_period, history_duration_);

    while (!contact_frame_history_.empty()) {
      const double age = (stamp - contact_frame_history_.front().stamp).seconds();
      if (age <= keep_duration) {
        break;
      }
      contact_frame_history_.pop_front();
    }

    if (
      last_history_sample_time_.nanoseconds() > 0 &&
      (stamp - last_history_sample_time_).seconds() < sample_period)
    {
      return;
    }

    HistorySample sample;
    sample.stamp = stamp;
    sample.origin.x = ee_pose.pose.position.x;
    sample.origin.y = ee_pose.pose.position.y;
    sample.origin.z = ee_pose.pose.position.z;
    sample.orientation = contact_q;

    contact_frame_history_.push_back(sample);
    last_history_sample_time_ = stamp;
  }

  void pushTrajectorySample(
    const geometry_msgs::msg::PoseStamped & ee_pose,
    const rclcpp::Time & stamp)
  {
    std::lock_guard<std::mutex> lk(history_mtx_);
    const double keep_duration = std::max(1e-3, history_duration_);

    while (!trajectory_history_.empty()) {
      const double age = (stamp - trajectory_history_.front().stamp).seconds();
      if (age <= keep_duration) {
        break;
      }
      trajectory_history_.pop_front();
    }

    TrajectorySample sample;
    sample.stamp = stamp;
    sample.point.x = ee_pose.pose.position.x;
    sample.point.y = ee_pose.pose.position.y;
    sample.point.z = ee_pose.pose.position.z;

    trajectory_history_.push_back(sample);
  }

  visualization_msgs::msg::MarkerArray makeContactHistoryMarkerArray(const rclcpp::Time & stamp)
  {
    std::deque<HistorySample> history_copy;
    std::deque<TrajectorySample> trajectory_copy;
    {
      std::lock_guard<std::mutex> lk(history_mtx_);
      history_copy = contact_frame_history_;
      trajectory_copy = trajectory_history_;
    }

    visualization_msgs::msg::MarkerArray out;
    if (history_copy.empty()) {
      {
        std::lock_guard<std::mutex> lk(history_mtx_);
        for (size_t i = 0; i < last_contact_frame_sample_count_; ++i) {
          out.markers.push_back(
            make_delete_marker(
              "contact_frame_history_x_segment", static_cast<int>(i), parent_frame_, stamp));
          out.markers.push_back(
            make_delete_marker(
              "contact_frame_history_y_segment", static_cast<int>(i), parent_frame_, stamp));
          out.markers.push_back(
            make_delete_marker(
              "contact_frame_history_z_segment", static_cast<int>(i), parent_frame_, stamp));
        }
        last_contact_frame_sample_count_ = 0;
        if (last_trajectory_marker_published_) {
          out.markers.push_back(
            make_delete_marker("ee_trajectory_history", 1000, parent_frame_, stamp));
          last_trajectory_marker_published_ = false;
        }
      }
      if (trajectory_copy.empty()) {
        return out;
      }
    }

    const double axis_len = std::max(1e-3, history_axis_scale_);
    size_t sample_count = 0;
    for (const auto & sample : history_copy) {
      tf2::Quaternion q(
        sample.orientation.x,
        sample.orientation.y,
        sample.orientation.z,
        sample.orientation.w);
      q.normalize();
      tf2::Matrix3x3 rot(q);

      geometry_msgs::msg::Point px = sample.origin;
      geometry_msgs::msg::Point py = sample.origin;
      geometry_msgs::msg::Point pz = sample.origin;

      const tf2::Vector3 ex = rot.getColumn(0);
      const tf2::Vector3 ey = rot.getColumn(1);
      const tf2::Vector3 ez = rot.getColumn(2);

      px.x += 2.0 * axis_len * ex.x();
      px.y += 2.0 * axis_len * ex.y();
      px.z += 2.0 * axis_len * ex.z();

      py.x += axis_len * ey.x();
      py.y += axis_len * ey.y();
      py.z += axis_len * ey.z();

      pz.x += axis_len * ez.x();
      pz.y += axis_len * ez.y();
      pz.z += axis_len * ez.z();

      const double age = std::max(0.0, (stamp - sample.stamp).seconds());
      const double age_ratio = std::clamp(age / std::max(1e-3, history_duration_), 0.0, 1.0);
      const double axis_alpha = 0.12 + 0.78 * (1.0 - age_ratio);

      auto x_marker = make_line_list_marker(
        "contact_frame_history_x_segment",
        static_cast<int>(sample_count),
        parent_frame_,
        stamp);
      auto y_marker = make_line_list_marker(
        "contact_frame_history_y_segment",
        static_cast<int>(sample_count),
        parent_frame_,
        stamp);
      auto z_marker = make_line_list_marker(
        "contact_frame_history_z_segment",
        static_cast<int>(sample_count),
        parent_frame_,
        stamp);

      x_marker.scale.x = 0.0045;
      x_marker.color.a = static_cast<float>(axis_alpha);
      x_marker.color.r = 1.0f;
      x_marker.color.g = 0.2f;
      x_marker.color.b = 0.2f;
      x_marker.points.push_back(sample.origin);
      x_marker.points.push_back(px);

      y_marker.scale.x = 0.0035;
      y_marker.color.a = static_cast<float>(0.9 * axis_alpha);
      y_marker.color.r = 0.2f;
      y_marker.color.g = 1.0f;
      y_marker.color.b = 0.2f;
      y_marker.points.push_back(sample.origin);
      y_marker.points.push_back(py);

      z_marker.scale.x = 0.0035;
      z_marker.color.a = static_cast<float>(0.9 * axis_alpha);
      z_marker.color.r = 0.2f;
      z_marker.color.g = 0.4f;
      z_marker.color.b = 1.0f;
      z_marker.points.push_back(sample.origin);
      z_marker.points.push_back(pz);

      out.markers.push_back(x_marker);
      out.markers.push_back(y_marker);
      out.markers.push_back(z_marker);
      ++sample_count;
    }

    if (trajectory_copy.size() >= 2) {
      auto traj_marker = make_line_strip_marker(
        "ee_trajectory_history", 1000, parent_frame_, stamp);
      traj_marker.scale.x = 0.010;
      traj_marker.color.a = 0.65f;
      traj_marker.color.r = 0.22f;
      traj_marker.color.g = 0.22f;
      traj_marker.color.b = 0.26f;
      for (const auto & sample : trajectory_copy) {
        traj_marker.points.push_back(sample.point);
      }
      out.markers.push_back(traj_marker);
    }

    {
      std::lock_guard<std::mutex> lk(history_mtx_);
      for (size_t i = sample_count; i < last_contact_frame_sample_count_; ++i) {
        out.markers.push_back(
          make_delete_marker(
            "contact_frame_history_x_segment", static_cast<int>(i), parent_frame_, stamp));
        out.markers.push_back(
          make_delete_marker(
            "contact_frame_history_y_segment", static_cast<int>(i), parent_frame_, stamp));
        out.markers.push_back(
          make_delete_marker(
            "contact_frame_history_z_segment", static_cast<int>(i), parent_frame_, stamp));
      }
      last_contact_frame_sample_count_ = sample_count;
      if (trajectory_copy.size() >= 2) {
        last_trajectory_marker_published_ = true;
      } else if (last_trajectory_marker_published_) {
        out.markers.push_back(
          make_delete_marker("ee_trajectory_history", 1000, parent_frame_, stamp));
        last_trajectory_marker_published_ = false;
      }
    }

    return out;
  }

  // =========================
  // Timer loop
  // =========================
  void loop_publish()
  {
    geometry_msgs::msg::PoseStamped pose, ee_pose;
    geometry_msgs::msg::Vector3Stamped vel, acc, ee_vel, ee_acc;
    geometry_msgs::msg::PoseStamped cmd_drone_pose, cmd_ee_pose, cmd_active_pose;
    std::array<float, 3> mob_force_2nd_order;
    std::array<float, 3> mob_force_consistency;
    std::array<float, 3> Fcontact;
    std::array<float, 3> estimated_normal_pure;
    std::array<float, 3> estimated_normal_k1;
    std::array<float, 3> force_based_normal;
    std::array<float, 3> ke_gamma_normal;
    std::array<float, 3> lf_only_normal;
    std::array<float, 3> lv_only_normal;
    std::array<float, 3> ke_force_normal_raw{0.0f, 0.0f, 0.0f};
    std::array<float, 3> ke_force_normal_projected{0.0f, 0.0f, 0.0f};
    std::array<float, 3> wind_indicator_force;
    geometry_msgs::msg::Quaternion contact_q_pure;
    geometry_msgs::msg::Quaternion contact_q;

    bool have_pose, have_vel, have_acc;
    bool have_ee_pose, have_ee_vel, have_ee_acc;
    bool have_cmd_drone_pose, have_cmd_ee_pose, have_cmd_active_pose;
    bool have_mob_force_2nd_order, have_mob_force_consistency;
    bool have_contact, have_contact_q;
    bool have_contact_q_pure;
    bool have_estimated_normal_pure, have_estimated_normal_k1;
    bool have_force_based_normal;
    bool have_ke_gamma_normal;
    bool have_lf_only_normal, have_lv_only_normal;
    bool have_ke_force_normal_raw = false;
    bool have_ke_force_normal_projected = false;
    bool have_wind_indicator_force;
    std::string contact_frame_id;

    {
      std::lock_guard<std::mutex> lk(mtx_);
      pose = pose_;
      vel = vel_;
      acc = acc_;
      ee_pose = ee_pose_;
      ee_vel = ee_vel_;
      ee_acc = ee_acc_;

      cmd_drone_pose = cmd_drone_pose_;
      cmd_ee_pose = cmd_ee_pose_;
      cmd_active_pose = cmd_active_pose_;

      mob_force_2nd_order = mob_force_2nd_order_;
      mob_force_consistency = mob_force_consistency_;
      Fcontact = contact_F_;
      estimated_normal_pure = estimated_normal_pure_;
      estimated_normal_k1 = estimated_normal_k1_;
      force_based_normal = force_based_normal_;
      ke_gamma_normal = ke_gamma_normal_;
      ke_force_normal_raw = ke_force_normal_raw_;
      ke_force_normal_projected = ke_force_normal_projected_;
      lf_only_normal = lf_only_normal_;
      lv_only_normal = lv_only_normal_;
      wind_indicator_force = wind_indicator_force_;
      contact_q_pure = contact_q_pure_;
      contact_q = contact_q_;

      have_pose = have_pose_;
      have_vel = have_vel_;
      have_acc = have_acc_;
      have_ee_pose = have_ee_pose_;
      have_ee_vel = have_ee_vel_;
      have_ee_acc = have_ee_acc_;

      have_cmd_drone_pose = have_cmd_drone_pose_;
      have_cmd_ee_pose = have_cmd_ee_pose_;
      have_cmd_active_pose = have_cmd_active_pose_;

      have_mob_force_2nd_order = have_mob_force_2nd_order_;
      have_mob_force_consistency = have_mob_force_consistency_;
      have_contact = have_contact_;
      have_contact_q_pure = have_contact_q_pure_;
      have_contact_q = have_contact_q_;
      have_estimated_normal_pure = have_estimated_normal_pure_;
      have_estimated_normal_k1 = have_estimated_normal_k1_;
      have_force_based_normal = have_force_based_normal_;
      have_ke_gamma_normal = have_ke_gamma_normal_;
      have_ke_force_normal_raw = have_ke_force_normal_raw_;
      have_ke_force_normal_projected = have_ke_force_normal_projected_;
      have_lf_only_normal = have_lf_only_normal_;
      have_lv_only_normal = have_lv_only_normal_;
      have_wind_indicator_force = have_wind_indicator_force_;
      contact_frame_id = contact_frame_id_;
    }

    const auto stamp = this->now();

    if (have_ee_pose) {
      pushTrajectorySample(ee_pose, stamp);
    }

    if (have_ee_pose && have_contact_q) {
      pushHistorySample(ee_pose, contact_q, stamp);
    }

    // 1) TF: world -> crazyflie
    if (have_pose) {
      geometry_msgs::msg::TransformStamped tf;
      tf.header.stamp = stamp;
      tf.header.frame_id = parent_frame_;
      tf.child_frame_id = cf_frame_;
      tf.transform.translation.x = pose.pose.position.x;
      tf.transform.translation.y = pose.pose.position.y;
      tf.transform.translation.z = pose.pose.position.z;
      tf.transform.rotation = pose.pose.orientation;
      tf_broadcaster_->sendTransform(tf);
    }

    // 2) TF: world -> cmd_drone
    if (have_cmd_drone_pose) {
      geometry_msgs::msg::TransformStamped tf;
      tf.header.stamp = stamp;
      tf.header.frame_id = parent_frame_;
      tf.child_frame_id = cmd_drone_frame_;
      tf.transform.translation.x = cmd_drone_pose.pose.position.x;
      tf.transform.translation.y = cmd_drone_pose.pose.position.y;
      tf.transform.translation.z = cmd_drone_pose.pose.position.z;
      tf.transform.rotation = cmd_drone_pose.pose.orientation;
      tf_broadcaster_->sendTransform(tf);
    }

    // 3) TF: world -> cmd_end_effector
    if (have_cmd_ee_pose) {
      geometry_msgs::msg::TransformStamped tf;
      tf.header.stamp = stamp;
      tf.header.frame_id = parent_frame_;
      tf.child_frame_id = cmd_ee_frame_;
      tf.transform.translation.x = cmd_ee_pose.pose.position.x;
      tf.transform.translation.y = cmd_ee_pose.pose.position.y;
      tf.transform.translation.z = cmd_ee_pose.pose.position.z;
      tf.transform.rotation = cmd_ee_pose.pose.orientation;
      tf_broadcaster_->sendTransform(tf);
    }

    // 4) TF: world -> cmd_active
    if (have_cmd_active_pose) {
      geometry_msgs::msg::TransformStamped tf;
      tf.header.stamp = stamp;
      tf.header.frame_id = parent_frame_;
      tf.child_frame_id = cmd_active_frame_;
      tf.transform.translation.x = cmd_active_pose.pose.position.x;
      tf.transform.translation.y = cmd_active_pose.pose.position.y;
      tf.transform.translation.z = cmd_active_pose.pose.position.z;
      tf.transform.rotation = cmd_active_pose.pose.orientation;
      tf_broadcaster_->sendTransform(tf);
    }

    // 5) TF: world -> end_effector
    if (have_ee_pose) {
      geometry_msgs::msg::TransformStamped tf;
      tf.header.stamp = stamp;
      tf.header.frame_id = parent_frame_;
      tf.child_frame_id = ee_frame_;
      tf.transform.translation.x = ee_pose.pose.position.x;
      tf.transform.translation.y = ee_pose.pose.position.y;
      tf.transform.translation.z = ee_pose.pose.position.z;
      tf.transform.rotation = ee_pose.pose.orientation;
      tf_broadcaster_->sendTransform(tf);
    }

    // 6) TF: world -> estimated_contact_frame
    if (have_ee_pose && have_contact_q) {
      geometry_msgs::msg::TransformStamped tf;
      tf.header.stamp = stamp;
      tf.header.frame_id = parent_frame_;
      tf.child_frame_id = est_contact_frame_;
      tf.transform.translation.x = ee_pose.pose.position.x;
      tf.transform.translation.y = ee_pose.pose.position.y;
      tf.transform.translation.z = ee_pose.pose.position.z;

      tf2::Quaternion q_wc(contact_q.x, contact_q.y, contact_q.z, contact_q.w);
      q_wc.normalize();
      tf.transform.rotation.x = q_wc.x();
      tf.transform.rotation.y = q_wc.y();
      tf.transform.rotation.z = q_wc.z();
      tf.transform.rotation.w = q_wc.w();
      tf_broadcaster_->sendTransform(tf);
    }

    const double source_shaft = 0.65 * arrow_shaft_diam_;
    const double source_head_diam = 0.75 * arrow_head_diam_;
    const double source_head_len = 0.80 * arrow_head_len_;
    const double normal_shaft = 1.80 * arrow_shaft_diam_;
    const double normal_head_diam = 1.80 * arrow_head_diam_;
    const double normal_head_len = 1.60 * arrow_head_len_;

    if (have_ee_pose && have_mob_force_2nd_order) {
      auto mk = make_arrow_marker_with_dims(
        "drone_external_force_mob_2nd_order", 0, parent_frame_, stamp,
        ee_pose.pose.position.x, ee_pose.pose.position.y, ee_pose.pose.position.z,
        static_cast<double>(mob_force_2nd_order[0]),
        static_cast<double>(mob_force_2nd_order[1]),
        static_cast<double>(mob_force_2nd_order[2]),
        force_scale_,
        source_shaft,
        source_head_diam,
        source_head_len,
        1.0, 0.8, 0.1);
      pub_mob_2nd_order_arrow_->publish(mk);
    }

    if (have_ee_pose && have_mob_force_consistency) {
      auto mk = make_arrow_marker_with_dims(
        "drone_external_force_mob_2nd_tau", 0, parent_frame_, stamp,
        ee_pose.pose.position.x, ee_pose.pose.position.y, ee_pose.pose.position.z,
        static_cast<double>(mob_force_consistency[0]),
        static_cast<double>(mob_force_consistency[1]),
        static_cast<double>(mob_force_consistency[2]),
        1.725 * force_scale_,
        source_shaft,
        source_head_diam,
        source_head_len,
        0.75, 0.20, 1.00);
      pub_mob_consistency_arrow_->publish(mk);
    }

    // 8) Marker: contact force
    if (have_ee_pose && have_contact) {
      auto mk = make_arrow_marker_with_dims(
        "contact_force", 0, parent_frame_, stamp,
        ee_pose.pose.position.x, ee_pose.pose.position.y, ee_pose.pose.position.z,
        static_cast<double>(Fcontact[0]), static_cast<double>(Fcontact[1]), static_cast<double>(Fcontact[2]),
        contact_force_scale_,
        source_shaft,
        source_head_diam,
        source_head_len,
        0.3, 0.3, 1.0);
      pub_contact_arrow_->publish(mk);
    }

    const bool have_normal_anchor = have_ee_pose || have_pose;
    const double normal_anchor_x =
      have_ee_pose ? ee_pose.pose.position.x : pose.pose.position.x;
    const double normal_anchor_y =
      have_ee_pose ? ee_pose.pose.position.y : pose.pose.position.y;
    const double normal_anchor_z =
      have_ee_pose ? ee_pose.pose.position.z : pose.pose.position.z;

    if (have_normal_anchor && have_ke_force_normal_raw) {
      auto mk = make_arrow_marker_with_dims(
        "ke_force_normal_raw", 0, parent_frame_, stamp,
        normal_anchor_x, normal_anchor_y, normal_anchor_z,
        static_cast<double>(ke_force_normal_raw[0]),
        static_cast<double>(ke_force_normal_raw[1]),
        static_cast<double>(ke_force_normal_raw[2]),
        1.02 * normal_scale_,
        normal_shaft,
        normal_head_diam,
        normal_head_len,
        0.95, 0.45, 0.05);
      pub_ke_force_normal_raw_arrow_->publish(mk);
    } else {
      pub_ke_force_normal_raw_arrow_->publish(
        make_delete_marker("ke_force_normal_raw", 0, parent_frame_, stamp));
    }

    if (have_normal_anchor && have_ke_force_normal_projected) {
      auto mk = make_arrow_marker_with_dims(
        "ke_force_normal_projected", 0, parent_frame_, stamp,
        normal_anchor_x, normal_anchor_y, normal_anchor_z,
        static_cast<double>(ke_force_normal_projected[0]),
        static_cast<double>(ke_force_normal_projected[1]),
        static_cast<double>(ke_force_normal_projected[2]),
        1.06 * normal_scale_,
        normal_shaft,
        normal_head_diam,
        normal_head_len,
        0.10, 0.55, 1.00);
      pub_ke_force_normal_projected_arrow_->publish(mk);
    } else {
      pub_ke_force_normal_projected_arrow_->publish(
        make_delete_marker("ke_force_normal_projected", 0, parent_frame_, stamp));
    }

    if (have_normal_anchor && have_ke_gamma_normal) {
      auto mk = make_arrow_marker_with_dims(
        "ke_gamma_normal", 0, parent_frame_, stamp,
        normal_anchor_x, normal_anchor_y, normal_anchor_z,
        static_cast<double>(ke_gamma_normal[0]),
        static_cast<double>(ke_gamma_normal[1]),
        static_cast<double>(ke_gamma_normal[2]),
        1.10 * normal_scale_,
        normal_shaft,
        normal_head_diam,
        normal_head_len,
        0.10, 0.85, 0.25);
      pub_ke_gamma_normal_arrow_->publish(mk);
    } else {
      pub_ke_gamma_normal_arrow_->publish(
        make_delete_marker("ke_gamma_normal", 0, parent_frame_, stamp));
    }

    if (have_ee_pose && have_force_based_normal) {
      auto mk = make_arrow_marker_with_dims(
        "force_based_normal", 0, parent_frame_, stamp,
        ee_pose.pose.position.x, ee_pose.pose.position.y, ee_pose.pose.position.z,
        static_cast<double>(force_based_normal[0]),
        static_cast<double>(force_based_normal[1]),
        static_cast<double>(force_based_normal[2]),
        0.98 * normal_scale_,
        normal_shaft,
        normal_head_diam,
        normal_head_len,
        1.00, 0.62, 0.05);
      pub_force_based_normal_arrow_->publish(mk);
    } else {
      pub_force_based_normal_arrow_->publish(
        make_delete_marker("force_based_normal", 0, parent_frame_, stamp));
    }

    if (have_ee_pose && have_lf_only_normal) {
      auto mk = make_arrow_marker_with_dims(
        "lf_only_estimated_normal", 0, parent_frame_, stamp,
        ee_pose.pose.position.x, ee_pose.pose.position.y, ee_pose.pose.position.z,
        static_cast<double>(lf_only_normal[0]),
        static_cast<double>(lf_only_normal[1]),
        static_cast<double>(lf_only_normal[2]),
        0.96 * normal_scale_,
        normal_shaft,
        normal_head_diam,
        normal_head_len,
        1.00, 0.42, 0.00);
      pub_lf_only_normal_arrow_->publish(mk);
    } else {
      pub_lf_only_normal_arrow_->publish(
        make_delete_marker("lf_only_estimated_normal", 0, parent_frame_, stamp));
    }

    if (have_ee_pose && have_lv_only_normal) {
      auto mk = make_arrow_marker_with_dims(
        "lv_only_estimated_normal", 0, parent_frame_, stamp,
        ee_pose.pose.position.x, ee_pose.pose.position.y, ee_pose.pose.position.z,
        static_cast<double>(lv_only_normal[0]),
        static_cast<double>(lv_only_normal[1]),
        static_cast<double>(lv_only_normal[2]),
        0.92 * normal_scale_,
        normal_shaft,
        normal_head_diam,
        normal_head_len,
        0.20, 0.75, 0.20);
      pub_lv_only_normal_arrow_->publish(mk);
    } else {
      pub_lv_only_normal_arrow_->publish(
        make_delete_marker("lv_only_estimated_normal", 0, parent_frame_, stamp));
    }

    // 9) Marker: estimated normal vectors, expressed directly in the world frame
    std::array<float, 3> estimated_normal_world{0.0f, 0.0f, 0.0f};
    if (
      have_ee_pose &&
      resolveNormalWorld(
        estimated_normal_pure, have_estimated_normal_pure,
        contact_q_pure, have_contact_q_pure,
        estimated_normal_world))
    {
      auto mk = make_arrow_marker_with_dims(
        "estimated_contact_normal_pure", 0, parent_frame_, stamp,
        ee_pose.pose.position.x, ee_pose.pose.position.y, ee_pose.pose.position.z,
        static_cast<double>(estimated_normal_world[0]),
        static_cast<double>(estimated_normal_world[1]),
        static_cast<double>(estimated_normal_world[2]),
        0.96 * normal_scale_,
        normal_shaft,
        normal_head_diam,
        normal_head_len,
        0.15, 0.45, 0.95);
      pub_estimated_normal_pure_arrow_->publish(mk);
    } else {
      pub_estimated_normal_pure_arrow_->publish(
        make_delete_marker("estimated_contact_normal_pure", 0, parent_frame_, stamp));
    }

    if (
      have_ee_pose &&
      resolveNormalWorld(
        estimated_normal_k1, have_estimated_normal_k1,
        contact_q, have_contact_q,
        estimated_normal_world))
    {
      auto mk = make_arrow_marker_with_dims(
        "estimated_contact_normal_ke", 0, parent_frame_, stamp,
        ee_pose.pose.position.x, ee_pose.pose.position.y, ee_pose.pose.position.z,
        static_cast<double>(estimated_normal_world[0]),
        static_cast<double>(estimated_normal_world[1]),
        static_cast<double>(estimated_normal_world[2]),
        1.00 * normal_scale_,
        normal_shaft,
        normal_head_diam,
        normal_head_len,
        0.00, 0.85, 0.95);
      pub_estimated_normal_arrow_->publish(mk);
    } else {
      pub_estimated_normal_arrow_->publish(
        make_delete_marker("estimated_contact_normal_ke", 0, parent_frame_, stamp));
    }

    // 9.5) Marker: true normal vector from the selected environment geometry
    if (have_ee_pose) {
      double nx = 0.0;
      double ny = 0.0;
      double nz = 0.0;
      if (computeTrueNormalAtEE(ee_pose, nx, ny, nz)) {
        auto mk = make_arrow_marker_with_dims(
          "true_contact_normal", 0, parent_frame_, stamp,
          ee_pose.pose.position.x, ee_pose.pose.position.y, ee_pose.pose.position.z,
          nx, ny, nz,
          1.50 * normal_scale_,
          source_shaft,
          source_head_diam,
          source_head_len,
          0.70, 0.10, 0.90);
        pub_true_normal_arrow_->publish(mk);
      } else {
        pub_true_normal_arrow_->publish(
          make_delete_marker("true_contact_normal", 0, parent_frame_, stamp));
      }
    }

    pub_contact_history_markers_->publish(makeContactHistoryMarkerArray(stamp));

    pub_environment_marker_->publish(
      make_environment_marker("environment", 0, parent_frame_, stamp));
    pub_wind_indicator_markers_->publish(
      makeWindIndicatorMarkerArray(stamp, wind_indicator_force, have_wind_indicator_force));

    // 10) Marker: drone velocity
    if (have_pose && have_vel) {
      auto mk = make_arrow_marker_with_dims(
        "drone_velocity", 0, parent_frame_, stamp,
        pose.pose.position.x, pose.pose.position.y, pose.pose.position.z,
        vel.vector.x, vel.vector.y, vel.vector.z,
        drone_vel_scale_,
        source_shaft,
        source_head_diam,
        source_head_len,
        1.0, 1.0, 0.2);
      pub_drone_vel_arrow_->publish(mk);
    }

    // 11) Marker: drone acceleration
    if (have_pose && have_acc) {
      auto mk = make_arrow_marker_with_dims(
        "drone_acceleration", 0, parent_frame_, stamp,
        pose.pose.position.x, pose.pose.position.y, pose.pose.position.z,
        acc.vector.x, acc.vector.y, acc.vector.z,
        drone_acc_scale_,
        source_shaft,
        source_head_diam,
        source_head_len,
        1.0, 0.5, 0.0);
      pub_drone_acc_arrow_->publish(mk);
    }

    // 12) Marker: EE velocity in world frame, anchored at the EE position
    if (have_ee_pose && have_ee_vel) {
      auto mk = make_arrow_marker_with_dims(
        "ee_velocity", 0, parent_frame_, stamp,
        ee_pose.pose.position.x, ee_pose.pose.position.y, ee_pose.pose.position.z,
        ee_vel.vector.x, ee_vel.vector.y, ee_vel.vector.z,
        1.80 * ee_vel_scale_,
        source_shaft,
        source_head_diam,
        source_head_len,
        0.15, 1.00, 0.15);
      pub_ee_vel_arrow_->publish(mk);
    }

    // 13) Marker: EE acceleration
    if (have_ee_pose && have_ee_acc) {
      auto mk = make_arrow_marker_with_dims(
        "ee_acceleration", 0, parent_frame_, stamp,
        ee_pose.pose.position.x, ee_pose.pose.position.y, ee_pose.pose.position.z,
        ee_acc.vector.x, ee_acc.vector.y, ee_acc.vector.z,
        ee_acc_scale_,
        source_shaft,
        source_head_diam,
        source_head_len,
        0.2, 1.0, 1.0);
      pub_ee_acc_arrow_->publish(mk);
    }
  }

private:
  // =========================
  // ROS
  // =========================
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr sub_pose_;
  rclcpp::Subscription<geometry_msgs::msg::Vector3Stamped>::SharedPtr sub_vel_;
  rclcpp::Subscription<geometry_msgs::msg::Vector3Stamped>::SharedPtr sub_acc_;

  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr sub_ee_pose_;
  rclcpp::Subscription<geometry_msgs::msg::Vector3Stamped>::SharedPtr sub_ee_vel_;
  rclcpp::Subscription<geometry_msgs::msg::Vector3Stamped>::SharedPtr sub_ee_acc_;

  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr sub_cmd_drone_pose_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr sub_cmd_ee_pose_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr sub_cmd_active_pose_;

  rclcpp::Subscription<geometry_msgs::msg::WrenchStamped>::SharedPtr sub_mob_2nd_order_;
  rclcpp::Subscription<geometry_msgs::msg::WrenchStamped>::SharedPtr sub_mob_consistency_;
  rclcpp::Subscription<geometry_msgs::msg::WrenchStamped>::SharedPtr sub_contact_force_;
  rclcpp::Subscription<geometry_msgs::msg::QuaternionStamped>::SharedPtr sub_contact_frame_quat_pure_;
  rclcpp::Subscription<geometry_msgs::msg::QuaternionStamped>::SharedPtr sub_contact_frame_quat_;
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr sub_normal_debug_metrics_pure_;
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr sub_normal_debug_metrics_;
  rclcpp::Subscription<geometry_msgs::msg::Vector3Stamped>::SharedPtr sub_wind_indicator_;

  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr pub_mob_2nd_order_arrow_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr pub_mob_consistency_arrow_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr pub_contact_arrow_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr pub_estimated_normal_pure_arrow_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr pub_estimated_normal_arrow_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr pub_true_normal_arrow_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr pub_ke_force_normal_raw_arrow_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr pub_ke_force_normal_projected_arrow_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr pub_ke_gamma_normal_arrow_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr pub_force_based_normal_arrow_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr pub_lf_only_normal_arrow_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr pub_lv_only_normal_arrow_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_contact_history_markers_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr pub_drone_vel_arrow_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr pub_drone_acc_arrow_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr pub_ee_vel_arrow_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr pub_ee_acc_arrow_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr pub_environment_marker_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_wind_indicator_markers_;

  rclcpp::TimerBase::SharedPtr timer_;

  // =========================
  // State
  // =========================
  std::mutex mtx_;

  geometry_msgs::msg::PoseStamped pose_;
  geometry_msgs::msg::Vector3Stamped vel_;
  geometry_msgs::msg::Vector3Stamped acc_;
  bool have_pose_{false};
  bool have_vel_{false};
  bool have_acc_{false};

  geometry_msgs::msg::PoseStamped ee_pose_;
  geometry_msgs::msg::Vector3Stamped ee_vel_;
  geometry_msgs::msg::Vector3Stamped ee_acc_;
  bool have_ee_pose_{false};
  bool have_ee_vel_{false};
  bool have_ee_acc_{false};

  geometry_msgs::msg::PoseStamped cmd_drone_pose_;
  geometry_msgs::msg::PoseStamped cmd_ee_pose_;
  geometry_msgs::msg::PoseStamped cmd_active_pose_;
  bool have_cmd_drone_pose_{false};
  bool have_cmd_ee_pose_{false};
  bool have_cmd_active_pose_{false};

  std::array<float, 3> mob_force_2nd_order_{0.0f, 0.0f, 0.0f};
  std::array<float, 3> mob_force_consistency_{0.0f, 0.0f, 0.0f};
  std::array<float, 3> wind_indicator_force_{0.0f, 0.0f, 0.0f};
  bool have_mob_force_2nd_order_{false};
  bool have_mob_force_consistency_{false};
  bool have_wind_indicator_force_{false};

  std::array<float, 3> contact_F_{0.0f, 0.0f, 0.0f};
  bool have_contact_{false};
  bool have_contact_filter_state_{false};
  std::string contact_frame_id_;

  std::array<float, 3> force_based_normal_{0.0f, 0.0f, 0.0f};
  std::array<float, 3> ke_gamma_normal_{0.0f, 0.0f, 0.0f};
  std::array<float, 3> ke_force_normal_raw_{0.0f, 0.0f, 0.0f};
  std::array<float, 3> ke_force_normal_projected_{0.0f, 0.0f, 0.0f};
  std::array<float, 3> estimated_normal_pure_{0.0f, 0.0f, 0.0f};
  std::array<float, 3> estimated_normal_k1_{0.0f, 0.0f, 0.0f};
  NormalMemoryState ke_raw_normal_memory_state_;
  NormalMemoryState ke_projected_normal_memory_state_;
  rclcpp::Time last_ke_memory_update_time_{0, 0, RCL_ROS_TIME};
  bool have_estimated_normal_pure_{false};
  bool have_estimated_normal_k1_{false};
  bool have_force_based_normal_{false};
  bool have_ke_gamma_normal_{false};
  bool have_ke_force_normal_raw_{false};
  bool have_ke_force_normal_projected_{false};
  std::array<float, 3> lf_only_normal_{0.0f, 0.0f, 0.0f};
  std::array<float, 3> lv_only_normal_{0.0f, 0.0f, 0.0f};
  bool have_lf_only_normal_{false};
  bool have_lv_only_normal_{false};

  geometry_msgs::msg::Quaternion contact_q_pure_;
  geometry_msgs::msg::Quaternion contact_q_;
  bool have_contact_q_pure_{false};
  bool have_contact_q_{false};

  std::mutex history_mtx_;
  std::deque<HistorySample> contact_frame_history_;
  std::deque<TrajectorySample> trajectory_history_;
  rclcpp::Time last_history_sample_time_{0, 0, RCL_ROS_TIME};
  size_t last_contact_frame_sample_count_{0};
  bool last_trajectory_marker_published_{false};

  // =========================
  // Params
  // =========================
  std::string parent_frame_;
  std::string cf_frame_;
  std::string cmd_drone_frame_;
  std::string cmd_ee_frame_;
  std::string cmd_active_frame_;
  std::string ee_frame_;

  std::string pose_topic_;
  std::string vel_topic_;
  std::string acc_topic_;

  std::string ee_pose_topic_;
  std::string ee_vel_topic_;
  std::string ee_acc_topic_;

  std::string cmd_drone_topic_;
  std::string cmd_ee_topic_;
  std::string cmd_active_topic_;

  std::string mob_topic_2nd_order_;
  std::string mob_topic_consistency_;
  std::string contact_force_topic_;
  std::string contact_frame_quat_topic_pure_;
  std::string contact_frame_quat_topic_;
  std::string normal_debug_metrics_topic_pure_;
  std::string normal_debug_metrics_topic_;
  std::string est_contact_frame_;
  std::string environment_type_{"cylinder"};
  std::string environment_marker_topic_;
  std::string wind_indicator_topic_;
  std::string wind_indicator_marker_topic_;

  double force_scale_{10.0};
  double contact_force_scale_{10.0};
  double contact_force_lpf_omega_{12.0};
  double normal_scale_{0.2};
  double ke_normal_velocity_epsilon_{0.005};
  double ke_normal_force_epsilon_{0.005};
  double ke_raw_normal_beta_n_{0.3};
  double ke_raw_normal_sigma_n_{0.3};
  double ke_projected_normal_beta_n_{0.3};
  double ke_projected_normal_sigma_n_{0.3};
  double history_axis_scale_{0.08};
  double history_sample_period_{0.5};
  double history_duration_{30.0};
  double drone_vel_scale_{1.0};
  double drone_acc_scale_{1.0};
  double ee_vel_scale_{1.0};
  double ee_acc_scale_{1.0};

  double arrow_shaft_diam_{0.01};
  double arrow_head_diam_{0.02};
  double arrow_head_len_{0.04};
  double publish_hz_{60.0};
  double cylinder_pos_x_{1.5};
  double cylinder_pos_y_{0.0};
  double cylinder_pos_z_{0.0};
  double cylinder_radius_{1.0};
  double cylinder_half_height_{10.0};
  std::vector<double> cylinder_rgba_{0.75, 0.93, 0.75, 0.85};
  double wall_pos_x_{0.2};
  double wall_pos_y_{0.0};
  double wall_pos_z_{0.0};
  double wall_size_x_{0.1};
  double wall_size_y_{2.5};
  double wall_size_z_{2.5};
  std::vector<double> wall_rgba_{0.75, 0.93, 0.75, 0.85};
  bool wind_indicator_enable_{true};
  double wind_indicator_origin_x_{-0.05};
  double wind_indicator_origin_y_{0.0};
  double wind_indicator_origin_z_{0.0};
  rclcpp::Time last_contact_force_update_time_{0, 0, RCL_ROS_TIME};
  double wind_indicator_pole_height_{0.08};
  double wind_indicator_mast_height_{0.34};
  double wind_indicator_bend_gain_{4.0};
  double wind_indicator_max_bend_{0.14};
  double wind_indicator_flutter_gain_{0.35};
  double wind_indicator_flutter_hz_{6.0};
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<RvizVisual>());
  rclcpp::shutdown();
  return 0;
}
