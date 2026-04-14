#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/vector3_stamped.hpp>
#include <geometry_msgs/msg/wrench_stamped.hpp>
#include <geometry_msgs/msg/quaternion_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>

#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>

#include <mutex>
#include <array>
#include <chrono>
#include <deque>
#include <string>
#include <algorithm>

using namespace std::chrono_literals;

class RvizVisual : public rclcpp::Node
{
public:
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

    drone_wrench_topic_ = this->declare_parameter<std::string>(
      "drone_wrench_topic", "/crazyflie/out/mob");
    mob_topic_2nd_order_ = this->declare_parameter<std::string>(
      "mob_topic_2nd_order", "/crazyflie/out/mob_2nd");
    mob_topic_consistency_ = this->declare_parameter<std::string>(
      "mob_topic_consistency", "/crazyflie/out/mob_2nd_tau");

    contact_force_topic_ = this->declare_parameter<std::string>(
      "contact_force_topic", "/crazyflie/out/EE_contact_force_filt");

    contact_frame_quat_topic_ = this->declare_parameter<std::string>(
      "contact_frame_quat_topic", "/estimated_contact_frame_quat");

    est_contact_frame_ = this->declare_parameter<std::string>(
      "est_contact_frame", "estimated_contact_frame");

    cylinder_marker_topic_ = this->declare_parameter<std::string>(
      "cylinder_marker_topic", "/rviz/environment_cylinder");
    cylinder_pos_x_ = this->declare_parameter<double>("cylinder.pos.x", 1.5);
    cylinder_pos_y_ = this->declare_parameter<double>("cylinder.pos.y", 0.0);
    cylinder_pos_z_ = this->declare_parameter<double>("cylinder.pos.z", 0.0);
    cylinder_radius_ = this->declare_parameter<double>("cylinder.radius", 1.0);
    cylinder_half_height_ = this->declare_parameter<double>("cylinder.half_height", 10.0);
    cylinder_rgba_ = this->declare_parameter<std::vector<double>>(
      "cylinder.rgba", std::vector<double>{0.75, 0.93, 0.75, 0.85});
    if (cylinder_rgba_.size() != 4) {
      RCLCPP_WARN(
        this->get_logger(),
        "cylinder.rgba must have size 4. Falling back to [0.75, 0.93, 0.75, 0.85].");
      cylinder_rgba_ = {0.75, 0.93, 0.75, 0.85};
    }

    force_scale_         = this->declare_parameter<double>("force_scale", 10.0);
    contact_force_scale_ = this->declare_parameter<double>("contact_force_scale", 10.0);
    normal_scale_        = this->declare_parameter<double>("normal_scale", 0.2);
    history_axis_scale_  = this->declare_parameter<double>("history_axis_scale", 0.08);
    history_sample_period_ = this->declare_parameter<double>("history_sample_period", 1.0);
    history_duration_    = this->declare_parameter<double>("history_duration", 15.0);

    drone_vel_scale_ = this->declare_parameter<double>("drone_vel_scale", 1.0);
    drone_acc_scale_ = this->declare_parameter<double>("drone_acc_scale", 1.0);
    ee_vel_scale_    = this->declare_parameter<double>("ee_vel_scale", 1.0);
    ee_acc_scale_    = this->declare_parameter<double>("ee_acc_scale", 1.0);

    arrow_shaft_diam_ = this->declare_parameter<double>("arrow_shaft_diam", 0.01);
    arrow_head_diam_  = this->declare_parameter<double>("arrow_head_diam", 0.02);
    arrow_head_len_   = this->declare_parameter<double>("arrow_head_len", 0.04);

    publish_hz_ = this->declare_parameter<double>("publish_hz", 60.0);

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

    sub_drone_wrench_ = this->create_subscription<geometry_msgs::msg::WrenchStamped>(
      drone_wrench_topic_, 10,
      std::bind(&RvizVisual::cb_drone_wrench, this, std::placeholders::_1));

    sub_mob_2nd_order_ = this->create_subscription<geometry_msgs::msg::WrenchStamped>(
      mob_topic_2nd_order_, 10,
      std::bind(&RvizVisual::cb_mob_2nd_order, this, std::placeholders::_1));
    sub_mob_consistency_ = this->create_subscription<geometry_msgs::msg::WrenchStamped>(
      mob_topic_consistency_, 10,
      std::bind(&RvizVisual::cb_mob_consistency, this, std::placeholders::_1));

    sub_contact_force_ = this->create_subscription<geometry_msgs::msg::WrenchStamped>(
      contact_force_topic_, 10,
      std::bind(&RvizVisual::cb_contact_force, this, std::placeholders::_1));

    sub_contact_frame_quat_ = this->create_subscription<geometry_msgs::msg::QuaternionStamped>(
      contact_frame_quat_topic_, 10,
      std::bind(&RvizVisual::cb_contact_frame_quat, this, std::placeholders::_1));

    // -------------------------
    // Publishers (Markers)
    // -------------------------
    pub_drone_ext_force_arrow_ = this->create_publisher<visualization_msgs::msg::Marker>(
      "/rviz/drone_external_force", 10);

    pub_mob_2nd_order_arrow_ = this->create_publisher<visualization_msgs::msg::Marker>(
      "/rviz/drone_external_force_mob_2nd_order", 10);
    pub_mob_consistency_arrow_ = this->create_publisher<visualization_msgs::msg::Marker>(
      "/rviz/drone_external_force_mob_2nd_tau", 10);

    pub_contact_arrow_ = this->create_publisher<visualization_msgs::msg::Marker>(
      "/rviz/EE_contact_force", 10);
    pub_estimated_normal_arrow_ = this->create_publisher<visualization_msgs::msg::Marker>(
      "/rviz/estimated_contact_normal", 10);
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
    pub_cylinder_marker_ = this->create_publisher<visualization_msgs::msg::Marker>(
      cylinder_marker_topic_, 10);

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
    RCLCPP_INFO(this->get_logger(), "Sub external/contact: %s | %s | %s | %s",
                drone_wrench_topic_.c_str(), mob_topic_2nd_order_.c_str(),
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

  void cb_drone_wrench(const geometry_msgs::msg::WrenchStamped::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lk(mtx_);
    drone_ext_force_[0] = static_cast<float>(msg->wrench.force.x);
    drone_ext_force_[1] = static_cast<float>(msg->wrench.force.y);
    drone_ext_force_[2] = static_cast<float>(msg->wrench.force.z);
    have_drone_ext_force_ = true;
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
    contact_F_[0] = static_cast<float>(msg->wrench.force.x);
    contact_F_[1] = static_cast<float>(msg->wrench.force.y);
    contact_F_[2] = static_cast<float>(msg->wrench.force.z);
    have_contact_ = true;
    contact_frame_id_ = msg->header.frame_id;
  }

  void cb_contact_frame_quat(const geometry_msgs::msg::QuaternionStamped::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lk(mtx_);
    contact_q_ = msg->quaternion;
    have_contact_q_ = true;
  }

  // =========================
  // Marker helper
  // =========================
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

      px.x += axis_len * ex.x();
      px.y += axis_len * ex.y();
      px.z += axis_len * ex.z();

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
      traj_marker.scale.x = 0.005;
      traj_marker.color.a = 0.32f;
      traj_marker.color.r = 1.0f;
      traj_marker.color.g = 1.0f;
      traj_marker.color.b = 1.0f;
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
    std::array<float, 3> drone_ext_force;
    std::array<float, 3> mob_force_2nd_order;
    std::array<float, 3> mob_force_consistency;
    std::array<float, 3> Fcontact;
    geometry_msgs::msg::Quaternion contact_q;

    bool have_pose, have_vel, have_acc;
    bool have_ee_pose, have_ee_vel, have_ee_acc;
    bool have_cmd_drone_pose, have_cmd_ee_pose, have_cmd_active_pose;
    bool have_drone_ext_force, have_mob_force_2nd_order, have_mob_force_consistency;
    bool have_contact, have_contact_q;
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

      drone_ext_force = drone_ext_force_;
      mob_force_2nd_order = mob_force_2nd_order_;
      mob_force_consistency = mob_force_consistency_;
      Fcontact = contact_F_;
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

      have_drone_ext_force = have_drone_ext_force_;
      have_mob_force_2nd_order = have_mob_force_2nd_order_;
      have_mob_force_consistency = have_mob_force_consistency_;
      have_contact = have_contact_;
      have_contact_q = have_contact_q_;
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

    // 7) Marker: drone external force
    if (have_ee_pose && have_drone_ext_force) {
      auto mk = make_arrow_marker(
        "drone_external_force", 0, parent_frame_, stamp,
        ee_pose.pose.position.x, ee_pose.pose.position.y, ee_pose.pose.position.z,
        static_cast<double>(drone_ext_force[0]),
        static_cast<double>(drone_ext_force[1]),
        static_cast<double>(drone_ext_force[2]),
        force_scale_, 1.0, 0.3, 0.3);
      pub_drone_ext_force_arrow_->publish(mk);
    }

    if (have_ee_pose && have_mob_force_2nd_order) {
      auto mk = make_arrow_marker(
        "drone_external_force_mob_2nd_order", 0, parent_frame_, stamp,
        ee_pose.pose.position.x, ee_pose.pose.position.y, ee_pose.pose.position.z,
        static_cast<double>(mob_force_2nd_order[0]),
        static_cast<double>(mob_force_2nd_order[1]),
        static_cast<double>(mob_force_2nd_order[2]),
        force_scale_, 1.0, 0.8, 0.1);
      pub_mob_2nd_order_arrow_->publish(mk);
    }

    if (have_ee_pose && have_mob_force_consistency) {
      auto mk = make_arrow_marker(
        "drone_external_force_mob_2nd_tau", 0, parent_frame_, stamp,
        ee_pose.pose.position.x, ee_pose.pose.position.y, ee_pose.pose.position.z,
        static_cast<double>(mob_force_consistency[0]),
        static_cast<double>(mob_force_consistency[1]),
        static_cast<double>(mob_force_consistency[2]),
        force_scale_, 0.7, 0.2, 1.0);
      pub_mob_consistency_arrow_->publish(mk);
    }

    // 8) Marker: contact force
    if (have_ee_pose && have_contact) {
      auto mk = make_arrow_marker(
        "contact_force", 0, parent_frame_, stamp,
        ee_pose.pose.position.x, ee_pose.pose.position.y, ee_pose.pose.position.z,
        static_cast<double>(Fcontact[0]), static_cast<double>(Fcontact[1]), static_cast<double>(Fcontact[2]),
        contact_force_scale_, 0.3, 0.3, 1.0);
      pub_contact_arrow_->publish(mk);
    }

    // 9) Marker: estimated normal vector (x-axis of estimated contact frame)
    if (have_ee_pose && have_contact_q) {
      tf2::Quaternion q_wc(contact_q.x, contact_q.y, contact_q.z, contact_q.w);
      q_wc.normalize();

      tf2::Matrix3x3 r_wc(q_wc);
      tf2::Vector3 n_w = r_wc.getColumn(0);

      auto mk = make_arrow_marker(
        "estimated_contact_normal", 0, parent_frame_, stamp,
        ee_pose.pose.position.x, ee_pose.pose.position.y, ee_pose.pose.position.z,
        n_w.x(), n_w.y(), n_w.z(),
        normal_scale_, 0.0, 0.9, 0.9);
      pub_estimated_normal_arrow_->publish(mk);
    }

    pub_contact_history_markers_->publish(makeContactHistoryMarkerArray(stamp));

    pub_cylinder_marker_->publish(
      make_cylinder_marker("environment_cylinder", 0, parent_frame_, stamp));

    // 10) Marker: drone velocity
    if (have_pose && have_vel) {
      auto mk = make_arrow_marker(
        "drone_velocity", 0, parent_frame_, stamp,
        pose.pose.position.x, pose.pose.position.y, pose.pose.position.z,
        vel.vector.x, vel.vector.y, vel.vector.z,
        drone_vel_scale_, 1.0, 1.0, 0.2);
      pub_drone_vel_arrow_->publish(mk);
    }

    // 11) Marker: drone acceleration
    if (have_pose && have_acc) {
      auto mk = make_arrow_marker(
        "drone_acceleration", 0, parent_frame_, stamp,
        pose.pose.position.x, pose.pose.position.y, pose.pose.position.z,
        acc.vector.x, acc.vector.y, acc.vector.z,
        drone_acc_scale_, 1.0, 0.5, 0.0);
      pub_drone_acc_arrow_->publish(mk);
    }

    // 12) Marker: EE velocity
    if (have_ee_pose && have_ee_vel) {
      auto mk = make_arrow_marker(
        "ee_velocity", 0, parent_frame_, stamp,
        ee_pose.pose.position.x, ee_pose.pose.position.y, ee_pose.pose.position.z,
        ee_vel.vector.x, ee_vel.vector.y, ee_vel.vector.z,
        ee_vel_scale_, 0.2, 1.0, 0.2);
      pub_ee_vel_arrow_->publish(mk);
    }

    // 13) Marker: EE acceleration
    if (have_ee_pose && have_ee_acc) {
      auto mk = make_arrow_marker(
        "ee_acceleration", 0, parent_frame_, stamp,
        ee_pose.pose.position.x, ee_pose.pose.position.y, ee_pose.pose.position.z,
        ee_acc.vector.x, ee_acc.vector.y, ee_acc.vector.z,
        ee_acc_scale_, 0.2, 1.0, 1.0);
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

  rclcpp::Subscription<geometry_msgs::msg::WrenchStamped>::SharedPtr sub_drone_wrench_;
  rclcpp::Subscription<geometry_msgs::msg::WrenchStamped>::SharedPtr sub_mob_2nd_order_;
  rclcpp::Subscription<geometry_msgs::msg::WrenchStamped>::SharedPtr sub_mob_consistency_;
  rclcpp::Subscription<geometry_msgs::msg::WrenchStamped>::SharedPtr sub_contact_force_;
  rclcpp::Subscription<geometry_msgs::msg::QuaternionStamped>::SharedPtr sub_contact_frame_quat_;

  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr pub_drone_ext_force_arrow_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr pub_mob_2nd_order_arrow_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr pub_mob_consistency_arrow_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr pub_contact_arrow_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr pub_estimated_normal_arrow_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_contact_history_markers_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr pub_drone_vel_arrow_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr pub_drone_acc_arrow_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr pub_ee_vel_arrow_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr pub_ee_acc_arrow_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr pub_cylinder_marker_;

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

  std::array<float, 3> drone_ext_force_{0.0f, 0.0f, 0.0f};
  std::array<float, 3> mob_force_2nd_order_{0.0f, 0.0f, 0.0f};
  std::array<float, 3> mob_force_consistency_{0.0f, 0.0f, 0.0f};
  bool have_drone_ext_force_{false};
  bool have_mob_force_2nd_order_{false};
  bool have_mob_force_consistency_{false};

  std::array<float, 3> contact_F_{0.0f, 0.0f, 0.0f};
  bool have_contact_{false};
  std::string contact_frame_id_;

  geometry_msgs::msg::Quaternion contact_q_;
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

  std::string drone_wrench_topic_;
  std::string mob_topic_2nd_order_;
  std::string mob_topic_consistency_;
  std::string contact_force_topic_;
  std::string contact_frame_quat_topic_;
  std::string est_contact_frame_;
  std::string cylinder_marker_topic_;

  double force_scale_{10.0};
  double contact_force_scale_{10.0};
  double normal_scale_{0.2};
  double history_axis_scale_{0.08};
  double history_sample_period_{0.5};
  double history_duration_{15.0};
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
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<RvizVisual>());
  rclcpp::shutdown();
  return 0;
}
