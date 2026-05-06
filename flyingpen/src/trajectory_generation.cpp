#include <rclcpp/rclcpp.hpp>

#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>

#include <geometry_msgs/msg/quaternion_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/vector3_stamped.hpp>


#include <array>
#include <algorithm>
#include <cctype>
#include <cmath>
#include <mutex>
#include <limits>
#include <string>
#include <vector>

#include <Eigen/Dense>

using namespace std::chrono_literals;

class TrajectoryGeneration : public rclcpp::Node
{
public:
  TrajectoryGeneration()
  : Node("trajectory_generation")
  {
    // --------------------------------------------------
    // Parameters
    // --------------------------------------------------
    reference_object_ = this->declare_parameter<std::string>(
      "reference_object", "drone");   // "drone" or "end_effector"

    yaw_align_kp_ = this->declare_parameter<double>(
      "yaw_align_kp", 3.0);

    normal_xy_min_ = this->declare_parameter<double>(
      "normal_xy_min", 1e-3);

    velocity_command_frame_ = normalizeFrameName(
      this->declare_parameter<std::string>("velocity_command_frame", "contact"));
    if (velocity_command_frame_ != "contact" && velocity_command_frame_ != "world") {
      RCLCPP_WARN(
        this->get_logger(),
        "Unknown velocity_command_frame '%s'. Fallback to 'contact'.",
        velocity_command_frame_.c_str());
      velocity_command_frame_ = "contact";
    }

    force_adm_kp_ = this->declare_parameter<double>(
      "force_adm_kp", 5.0);

    force_adm_kd_ = this->declare_parameter<double>(
      "force_adm_kd", 0.005);

    const auto requested_trajectory_type = this->declare_parameter<std::string>(
      "trajectory.type", "lissajous");
    trajectory_type_ = normalizeTrajectoryType(requested_trajectory_type);
    if (!isSupportedTrajectoryType(requested_trajectory_type)) {
      RCLCPP_WARN(this->get_logger(),
                  "Unknown trajectory.type '%s'. Fallback to lissajous.",
                  requested_trajectory_type.c_str());
    }
    trajectory_ramp_sec_ = this->declare_parameter<double>(
      "trajectory.ramp_sec", 2.0);

    lissajous_amp_y_ = this->declare_parameter<double>(
      "lissajous.amp_y", 0.40);
    lissajous_amp_z_ = this->declare_parameter<double>(
      "lissajous.amp_z", 0.2);
    lissajous_period_y_ = this->declare_parameter<double>(
      "lissajous.period_y", 20.0);
    lissajous_period_z_ = this->declare_parameter<double>(
      "lissajous.period_z", 10.0);

    circle_radius_ = this->declare_parameter<double>(
      "circle.radius", 0.30);
    circle_period_sec_ = this->declare_parameter<double>(
      "circle.period_sec", 20.0);
    circle_center_y_ = this->declare_parameter<double>(
      "circle.center_y", 0.0);
    circle_center_z_ = this->declare_parameter<double>(
      "circle.center_z", 0.0);
    circle_phase_rad_ = this->declare_parameter<double>(
      "circle.phase_rad", -0.5 * M_PI);

    const double default_square_side_length = this->declare_parameter<double>(
      "square.side_length", 0.40);
    square_parallel_length_ = this->declare_parameter<double>(
      "square.parallel_length", default_square_side_length);
    square_vertical_length_ = this->declare_parameter<double>(
      "square.vertical_length", default_square_side_length);
    square_period_sec_ = this->declare_parameter<double>(
      "square.period_sec", 20.0);
    square_center_y_ = this->declare_parameter<double>(
      "square.center_y", 0.0);
    square_center_z_ = this->declare_parameter<double>(
      "square.center_z", 0.0);
    square_phase_ = this->declare_parameter<double>(
      "square.phase", 0.0);

    constant_accel_y_ = this->declare_parameter<double>(
      "constant_accel.acc_y", 0.40);
    constant_accel_z_ = this->declare_parameter<double>(
      "constant_accel.acc_z", 0.0);

    auto ee_off = this->declare_parameter<std::vector<double>>(
      "end_effector_offset", {0.09, 0.0, 0.085});
    if (ee_off.size() != 3) {
      RCLCPP_WARN(this->get_logger(),
                  "end_effector_offset must be size 3. Fallback to [0,0,0].");
      ee_off = {0.0, 0.0, 0.0};
    }
    d_B_ = Eigen::Vector3d(ee_off[0], ee_off[1], ee_off[2]);

    // topics
    pose_topic_ = this->declare_parameter<std::string>(
      "pose_topic", "/crazyflie/out/pose");
    vel_topic_ = this->declare_parameter<std::string>(
      "vel_topic", "/crazyflie/out/vel");
    acc_topic_ = this->declare_parameter<std::string>(
      "acc_topic", "/crazyflie/out/acc");

    ee_pose_topic_ = this->declare_parameter<std::string>(
      "ee_pose_topic", "/crazyflie/out/EE_pose");
    ee_vel_topic_ = this->declare_parameter<std::string>(
      "ee_vel_topic", "/crazyflie/out/EE_velocity");
    ee_acc_topic_ = this->declare_parameter<std::string>(
      "ee_acc_topic", "/crazyflie/out/EE_acceleration");

    contact_frame_quat_topic_ = this->declare_parameter<std::string>(
      "contact_frame_quat_topic", "/estimated_contact_frame_quat");
    contact_force_x_topic_ = this->declare_parameter<std::string>(
      "contact_force_x_topic", "/su/contact_force_x");

    // --------------------------------------------------
    // Subscribers
    // --------------------------------------------------
    sub_keyboard_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
      "/su/keyboard_input", 10,
      std::bind(&TrajectoryGeneration::keyboardCb, this, std::placeholders::_1));

    sub_use_vel_mode_ = this->create_subscription<std_msgs::msg::Float32>(
      "su/use_vel_mode", 10,
      std::bind(&TrajectoryGeneration::useVelModeCb, this, std::placeholders::_1));

    sub_cmd_force_ = this->create_subscription<std_msgs::msg::Float32>(
      "su/cmd_force", 10,
      std::bind(&TrajectoryGeneration::cmdForceCb, this, std::placeholders::_1));

    sub_contact_frame_quat_ = this->create_subscription<geometry_msgs::msg::QuaternionStamped>(
      contact_frame_quat_topic_, 10,
      std::bind(&TrajectoryGeneration::contactFrameQuatCb, this, std::placeholders::_1));
    sub_contact_force_x_ = this->create_subscription<std_msgs::msg::Float32>(
      contact_force_x_topic_, 10,
      std::bind(&TrajectoryGeneration::contactForceXCb, this, std::placeholders::_1));

    sub_pose_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
      pose_topic_, 10,
      std::bind(&TrajectoryGeneration::poseCb, this, std::placeholders::_1));

    sub_vel_ = this->create_subscription<geometry_msgs::msg::Vector3Stamped>(
      vel_topic_, 10,
      std::bind(&TrajectoryGeneration::velCb, this, std::placeholders::_1));

    sub_acc_ = this->create_subscription<geometry_msgs::msg::Vector3Stamped>(
      acc_topic_, 10,
      std::bind(&TrajectoryGeneration::accCb, this, std::placeholders::_1));

    sub_ee_pose_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
      ee_pose_topic_, 10,
      std::bind(&TrajectoryGeneration::eePoseCb, this, std::placeholders::_1));

    sub_ee_vel_ = this->create_subscription<geometry_msgs::msg::Vector3Stamped>(
      ee_vel_topic_, 10,
      std::bind(&TrajectoryGeneration::eeVelCb, this, std::placeholders::_1));

    sub_ee_acc_ = this->create_subscription<geometry_msgs::msg::Vector3Stamped>(
      ee_acc_topic_, 10,
      std::bind(&TrajectoryGeneration::eeAccCb, this, std::placeholders::_1));

    // --------------------------------------------------
    // Publishers
    // --------------------------------------------------
    pub_pos_cmd_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(
      "/crazyflie/in/pos_cmd", 10);

    pub_force_lpf_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(
      "/su/force_lpf", 10);

    pub_contact_vel_cmd_ = this->create_publisher<geometry_msgs::msg::Vector3Stamped>(
      "/su/debug/contact_vel_cmd", 10);

    pub_contact_vel_actual_ = this->create_publisher<geometry_msgs::msg::Vector3Stamped>(
      "/su/debug/contact_vel_actual", 10);

    // new debug pose publishers
    pub_cmd_drone_pose_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
      "/crazyflie/debug/cmd_drone", 10);

    pub_cmd_ee_pose_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
      "/crazyflie/debug/cmd_ee", 10);

    pub_cmd_active_pose_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
      "/crazyflie/debug/cmd_active", 10);

    // --------------------------------------------------
    // Timers
    // --------------------------------------------------
    timer_ = this->create_wall_timer(
      10ms, std::bind(&TrajectoryGeneration::update, this)); // 100 Hz

    force_timer_ = this->create_wall_timer(
      10ms, std::bind(&TrajectoryGeneration::forceUpdate, this)); // 100 Hz

    RCLCPP_INFO(this->get_logger(), "trajectory_generation started");
    RCLCPP_INFO(this->get_logger(), "reference_object = %s", reference_object_.c_str());
    RCLCPP_INFO(this->get_logger(), "velocity_command_frame = %s", velocity_command_frame_.c_str());
    RCLCPP_INFO(this->get_logger(), "g-key trajectory.type = %s", trajectory_type_.c_str());
    RCLCPP_INFO(this->get_logger(), "EE offset d_B = [%.4f %.4f %.4f]",
                d_B_.x(), d_B_.y(), d_B_.z());
  }

private:
  // ==================================================
  // Utils
  // ==================================================
  static inline double wrap_pi(double a)
  {
    while (a > M_PI) a -= 2.0 * M_PI;
    while (a < -M_PI) a += 2.0 * M_PI;
    return a;
  }

  static Eigen::Quaterniond quatMsgToEigen(const geometry_msgs::msg::Quaternion &q)
  {
    return Eigen::Quaterniond(q.w, q.x, q.y, q.z);
  }

  static geometry_msgs::msg::Quaternion yawToQuatMsg(double yaw)
  {
    geometry_msgs::msg::Quaternion q;
    q.w = std::cos(0.5 * yaw);
    q.x = 0.0;
    q.y = 0.0;
    q.z = std::sin(0.5 * yaw);
    return q;
  }

  static inline void smoothstepQuintic(double tau, double T, double & s, double & s_dot)
  {
    if (T <= 1e-9) {
      s = 1.0;
      s_dot = 0.0;
      return;
    }

    const double t = std::clamp(tau / T, 0.0, 1.0);
    const double t2 = t * t;
    const double t3 = t2 * t;
    const double t4 = t3 * t;
    const double t5 = t4 * t;

    s = 10.0 * t3 - 15.0 * t4 + 6.0 * t5;
    s_dot = (30.0 * t2 - 60.0 * t3 + 30.0 * t4) / T;
  }

  static std::string normalizeTrajectoryType(std::string type)
  {
    std::transform(type.begin(), type.end(), type.begin(),
      [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
    if (type == "circle" || type == "circular") {
      return "circle";
    }
    if (type == "square") {
      return "square";
    }
    if (type == "constant_accel" || type == "const_accel" || type == "constant-accel") {
      return "constant_accel";
    }
    return "lissajous";
  }

  static std::string normalizeFrameName(std::string frame)
  {
    std::transform(frame.begin(), frame.end(), frame.begin(),
      [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
    return frame;
  }

  static bool isSupportedTrajectoryType(std::string type)
  {
    std::transform(type.begin(), type.end(), type.begin(),
      [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
    return type == "lissajous" || type == "circle" ||
      type == "circular" || type == "square" ||
      type == "constant_accel" || type == "const_accel" || type == "constant-accel";
  }

  static inline double positivePeriod(double period_sec)
  {
    return std::max(1e-3, period_sec);
  }

  void computeTrajectoryReference(
    double t, double & y_ref, double & z_ref, double & y_ref_dot, double & z_ref_dot) const
  {
    if (trajectory_type_ == "circle") {
      const double radius = std::max(0.0, circle_radius_);
      const double w = 2.0 * M_PI / positivePeriod(circle_period_sec_);
      const double theta = w * t + circle_phase_rad_;
      y_ref = circle_center_y_ + radius * std::cos(theta);
      z_ref = circle_center_z_ + radius * std::sin(theta);
      y_ref_dot = -radius * w * std::sin(theta);
      z_ref_dot = radius * w * std::cos(theta);
      return;
    }

    if (trajectory_type_ == "square") {
      const double parallel_length = std::max(0.0, square_parallel_length_);
      const double vertical_length = std::max(0.0, square_vertical_length_);
      const double half_parallel = 0.5 * parallel_length;
      const double half_vertical = 0.5 * vertical_length;
      const double period = positivePeriod(square_period_sec_);
      const double perimeter = 2.0 * (parallel_length + vertical_length);

      if (perimeter < 1e-9) {
        y_ref = square_center_y_;
        z_ref = square_center_z_;
        y_ref_dot = 0.0;
        z_ref_dot = 0.0;
        return;
      }

      const double speed = perimeter / period;
      double phase = std::fmod(t / period + square_phase_, 1.0);
      if (phase < 0.0) {
        phase += 1.0;
      }

      const double s = phase * perimeter;
      const double edge0_end = parallel_length;
      const double edge1_end = edge0_end + vertical_length;
      const double edge2_end = edge1_end + parallel_length;

      if (s < edge0_end) {
        y_ref = square_center_y_ - half_parallel + s;
        z_ref = square_center_z_ - half_vertical;
        y_ref_dot = speed;
        z_ref_dot = 0.0;
      } else if (s < edge1_end) {
        const double edge_s = s - edge0_end;
        y_ref = square_center_y_ + half_parallel;
        z_ref = square_center_z_ - half_vertical + edge_s;
        y_ref_dot = 0.0;
        z_ref_dot = speed;
      } else if (s < edge2_end) {
        const double edge_s = s - edge1_end;
        y_ref = square_center_y_ + half_parallel - edge_s;
        z_ref = square_center_z_ + half_vertical;
        y_ref_dot = -speed;
        z_ref_dot = 0.0;
      } else {
        const double edge_s = s - edge2_end;
        y_ref = square_center_y_ - half_parallel;
        z_ref = square_center_z_ + half_vertical - edge_s;
        y_ref_dot = 0.0;
        z_ref_dot = -speed;
      }
      return;
    }

    if (trajectory_type_ == "constant_accel") {
      y_ref = 0.5 * constant_accel_y_ * t * t;
      z_ref = 0.5 * constant_accel_z_ * t * t;
      y_ref_dot = constant_accel_y_ * t;
      z_ref_dot = constant_accel_z_ * t;
      return;
    }

    const double wy = 2.0 * M_PI / positivePeriod(lissajous_period_y_);
    const double wz = 2.0 * M_PI / positivePeriod(lissajous_period_z_);
    y_ref = lissajous_amp_y_ * std::sin(wy * t);
    z_ref = lissajous_amp_z_ * std::sin(wz * t);
    y_ref_dot = lissajous_amp_y_ * wy * std::cos(wy * t);
    z_ref_dot = lissajous_amp_z_ * wz * std::cos(wz * t);
  }

  // ==================================================
  // Reference state
  // ==================================================
  struct ReferenceState
  {
    Eigen::Vector3d pos_w = Eigen::Vector3d::Zero();
    Eigen::Vector3d vel_w = Eigen::Vector3d::Zero();
    Eigen::Vector3d acc_w = Eigen::Vector3d::Zero();
    double yaw_w = 0.0;
    bool valid = false;
  };

  ReferenceState getReferenceStateLocked() const
  {
    ReferenceState ref;
    const bool use_ee = (reference_object_ == "end_effector");

    if (!use_ee) {
      if (!(pose_received_ && vel_received_ && acc_received_)) {
        ref.valid = false;
        return ref;
      }

      ref.pos_w = Eigen::Vector3d(pose_w_[0], pose_w_[1], pose_w_[2]);
      ref.vel_w = Eigen::Vector3d(vel_w_[0], vel_w_[1], vel_w_[2]);
      ref.acc_w = Eigen::Vector3d(acc_w_[0], acc_w_[1], acc_w_[2]);
      ref.yaw_w = yaw_w_;
      ref.valid = true;
      return ref;
    }

    if (!(ee_pose_received_ && ee_vel_received_ && ee_acc_received_ && pose_received_)) {
      ref.valid = false;
      return ref;
    }

    ref.pos_w = Eigen::Vector3d(ee_pose_w_[0], ee_pose_w_[1], ee_pose_w_[2]);
    ref.vel_w = Eigen::Vector3d(ee_vel_w_[0], ee_vel_w_[1], ee_vel_w_[2]);
    ref.acc_w = Eigen::Vector3d(ee_acc_w_[0], ee_acc_w_[1], ee_acc_w_[2]);
    ref.yaw_w = yaw_w_;
    ref.valid = true;
    return ref;
  }

  // ==================================================
  // Debug command pose publishing
  // ==================================================
  void publishDebugCmdPoses(
    const Eigen::Vector3d &p_drone_des_w,
    const Eigen::Vector3d &p_ee_des_w,
    double yaw_des,
    const rclcpp::Time &stamp)
  {
    geometry_msgs::msg::PoseStamped msg_drone;
    msg_drone.header.stamp = stamp;
    msg_drone.header.frame_id = "world";
    msg_drone.pose.position.x = p_drone_des_w.x();
    msg_drone.pose.position.y = p_drone_des_w.y();
    msg_drone.pose.position.z = p_drone_des_w.z();
    msg_drone.pose.orientation = yawToQuatMsg(yaw_des);
    pub_cmd_drone_pose_->publish(msg_drone);

    geometry_msgs::msg::PoseStamped msg_ee;
    msg_ee.header.stamp = stamp;
    msg_ee.header.frame_id = "world";
    msg_ee.pose.position.x = p_ee_des_w.x();
    msg_ee.pose.position.y = p_ee_des_w.y();
    msg_ee.pose.position.z = p_ee_des_w.z();
    msg_ee.pose.orientation = yawToQuatMsg(yaw_des);
    pub_cmd_ee_pose_->publish(msg_ee);

    geometry_msgs::msg::PoseStamped msg_active;
    msg_active.header.stamp = stamp;
    msg_active.header.frame_id = "world";

    if (reference_object_ == "end_effector") {
      msg_active.pose.position.x = p_ee_des_w.x();
      msg_active.pose.position.y = p_ee_des_w.y();
      msg_active.pose.position.z = p_ee_des_w.z();
    } else {
      msg_active.pose.position.x = p_drone_des_w.x();
      msg_active.pose.position.y = p_drone_des_w.y();
      msg_active.pose.position.z = p_drone_des_w.z();
    }
    msg_active.pose.orientation = yawToQuatMsg(yaw_des);
    pub_cmd_active_pose_->publish(msg_active);
  }

  void publishContactVelocityDebug(
    const Eigen::Vector3d &v_cmd_c,
    const Eigen::Vector3d &v_act_c,
    const rclcpp::Time &stamp)
  {
    geometry_msgs::msg::Vector3Stamped cmd_msg;
    cmd_msg.header.stamp = stamp;
    cmd_msg.header.frame_id = "contact";
    cmd_msg.vector.x = v_cmd_c.x();
    cmd_msg.vector.y = v_cmd_c.y();
    cmd_msg.vector.z = v_cmd_c.z();
    pub_contact_vel_cmd_->publish(cmd_msg);

    geometry_msgs::msg::Vector3Stamped act_msg;
    act_msg.header.stamp = stamp;
    act_msg.header.frame_id = "contact";
    act_msg.vector.x = v_act_c.x();
    act_msg.vector.y = v_act_c.y();
    act_msg.vector.z = v_act_c.z();
    pub_contact_vel_actual_->publish(act_msg);
  }

  // ==================================================
  // Callbacks
  // ==================================================
  void keyboardCb(const std_msgs::msg::Float64MultiArray::SharedPtr msg)
  {
    if (msg->data.size() < 4) return;

    sp_in_[0] = msg->data[0];
    sp_in_[1] = msg->data[1];
    sp_in_[2] = msg->data[2];
    sp_in_yaw_ = msg->data[3];
    trajectory_cmd_enabled_ = (msg->data.size() >= 5 && msg->data[4] > 0.5);
    sp_received_ = true;
  }

  void useVelModeCb(const std_msgs::msg::Float32::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lk(force_mtx_);
    su_cmd_use_vel_mode_ = msg->data;
  }

  void cmdForceCb(const std_msgs::msg::Float32::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lk(force_mtx_);
    su_cmd_fx_ = msg->data;
  }

  void contactFrameQuatCb(const geometry_msgs::msg::QuaternionStamped::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lk(force_mtx_);
    const Eigen::Quaterniond q_wc = quatMsgToEigen(msg->quaternion).normalized();
    contact_R_C_ = q_wc.toRotationMatrix();
    contact_n_w_ = contact_R_C_.col(0);
    contact_frame_received_ = true;
  }

  void contactForceXCb(const std_msgs::msg::Float32::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lk(force_mtx_);
    contact_force_x_ = static_cast<double>(msg->data);
    contact_force_x_received_ = true;
  }

  void poseCb(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lk(state_mtx_);

    pose_w_[0] = msg->pose.position.x;
    pose_w_[1] = msg->pose.position.y;
    pose_w_[2] = msg->pose.position.z;

    const auto &q = msg->pose.orientation;
    q_WB_ = quatMsgToEigen(q).normalized();

    const double siny_cosp = 2.0 * (q.w * q.z + q.x * q.y);
    const double cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
    yaw_w_ = std::atan2(siny_cosp, cosy_cosp);

    pose_received_ = true;
  }

  void velCb(const geometry_msgs::msg::Vector3Stamped::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lk(state_mtx_);
    vel_w_[0] = msg->vector.x;
    vel_w_[1] = msg->vector.y;
    vel_w_[2] = msg->vector.z;
    vel_received_ = true;
  }

  void accCb(const geometry_msgs::msg::Vector3Stamped::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lk(state_mtx_);
    acc_w_[0] = msg->vector.x;
    acc_w_[1] = msg->vector.y;
    acc_w_[2] = msg->vector.z;
    acc_received_ = true;
  }

  void eePoseCb(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lk(state_mtx_);
    ee_pose_w_[0] = msg->pose.position.x;
    ee_pose_w_[1] = msg->pose.position.y;
    ee_pose_w_[2] = msg->pose.position.z;
    ee_pose_received_ = true;
  }

  void eeVelCb(const geometry_msgs::msg::Vector3Stamped::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lk(state_mtx_);
    ee_vel_w_[0] = msg->vector.x;
    ee_vel_w_[1] = msg->vector.y;
    ee_vel_w_[2] = msg->vector.z;
    ee_vel_received_ = true;
  }

  void eeAccCb(const geometry_msgs::msg::Vector3Stamped::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lk(state_mtx_);
    ee_acc_w_[0] = msg->vector.x;
    ee_acc_w_[1] = msg->vector.y;
    ee_acc_w_[2] = msg->vector.z;
    ee_acc_received_ = true;
  }

  // ==================================================
  // Publish controller output + debug command poses
  // ==================================================
  void publishOut()
  {
    Eigen::Quaterniond q_WB;
    bool pose_ok = false;
    {
      std::lock_guard<std::mutex> lk(state_mtx_);
      q_WB = q_WB_;
      pose_ok = pose_received_;
    }

    if (!pose_ok) {
      return;
    }

    const Eigen::Matrix3d R_WB = q_WB.toRotationMatrix();

    Eigen::Vector3d p_active_des_w(
      su_int_ref_pos_[0],
      su_int_ref_pos_[1],
      su_int_ref_pos_[2]);

    Eigen::Vector3d p_drone_des_w = Eigen::Vector3d::Zero();
    Eigen::Vector3d p_ee_des_w = Eigen::Vector3d::Zero();

    if (reference_object_ == "end_effector") {
      p_ee_des_w = p_active_des_w;
      p_drone_des_w = p_ee_des_w - R_WB * d_B_;
    } else {
      p_drone_des_w = p_active_des_w;
      p_ee_des_w = p_drone_des_w + R_WB * d_B_;
    }

    // controller input: always drone command
    std_msgs::msg::Float64MultiArray out;
    out.data.resize(4);
    out.data[0] = p_drone_des_w.x();
    out.data[1] = p_drone_des_w.y();
    out.data[2] = p_drone_des_w.z();
    out.data[3] = su_int_ref_yaw_;
    pub_pos_cmd_->publish(out);

    // debug poses
    publishDebugCmdPoses(
      p_drone_des_w,
      p_ee_des_w,
      su_int_ref_yaw_,
      this->now());
  }

  // ==================================================
  // Force update
  // ==================================================
  void forceUpdate()
  {
    float use_vel;
    {
      std::lock_guard<std::mutex> lk(force_mtx_);
      use_vel = su_cmd_use_vel_mode_;
    }

    if (use_vel <= 0.5f) {
      eF_state_initialized_ = false;
      return;
    }

    float su_cmd_fx_local = 0.0f;
    double contact_force_x_local = 0.0;
    bool contact_force_x_ok = false;
    {
      std::lock_guard<std::mutex> lk(force_mtx_);
      su_cmd_fx_local = su_cmd_fx_;
      contact_force_x_local = contact_force_x_;
      contact_force_x_ok = contact_force_x_received_;
    }

    if (!contact_force_x_ok) {
      eF_state_initialized_ = false;
      return;
    }

    const rclcpp::Time now = this->now();
    double dt = 0.0;
    if (eF_last_time_.nanoseconds() != 0) {
      dt = (now - eF_last_time_).seconds();
    }
    eF_last_time_ = now;

    if (dt <= 1e-5 || dt > 0.1) {
      eF_state_initialized_ = false;
      return;
    }

    std::array<double,3> eF{0.0,0.0,0.0};
    eF[0] = static_cast<double>(su_cmd_fx_local) - contact_force_x_local;

    if (!eF_state_initialized_) {
      eF_prev_ = eF;
      eF_dot_filt_prev_ = {0.0,0.0,0.0};

      {
        std::lock_guard<std::mutex> lk(force_mtx_);
        eF_ = eF;
        eF_dot_filt_ = {0.0,0.0,0.0};
      }

      eF_state_initialized_ = true;
      return;
    }

    std::array<double,3> eF_dot_raw{0.0,0.0,0.0};
    eF_dot_raw[0] = (eF[0] - eF_prev_[0]) / dt;

    const double wc = 3.0;
    const double alpha = wc * dt;
    const double a = std::clamp(alpha, 0.0, 1.0);

    std::array<double,3> eF_dot_filt{0.0,0.0,0.0};
    eF_dot_filt[0] = (1.0 - a) * eF_dot_filt_prev_[0] + a * eF_dot_raw[0];

    eF_prev_ = eF;
    eF_dot_filt_prev_ = eF_dot_filt;

    {
      std::lock_guard<std::mutex> lk(force_mtx_);
      eF_ = eF;
      eF_dot_filt_ = eF_dot_filt;
    }

    std_msgs::msg::Float64MultiArray lpf;
    lpf.data.resize(2);
    lpf.data[0] = eF_dot_raw[0];
    lpf.data[1] = eF_dot_filt[0];
    pub_force_lpf_->publish(lpf);
  }

  // ==================================================
  // Main update
  // ==================================================
  void update()
  {
    if (!sp_received_) {
      return;
    }

    const rclcpp::Time now = this->now();
    double dt = 0.0;
    if (last_time_.nanoseconds() != 0) {
      dt = (now - last_time_).seconds();
    }
    last_time_ = now;

    if (dt <= 1e-5 || dt > 0.1) {
      publishOut();
      return;
    }

    float use_vel_mode_local = 0.0f;
    {
      std::lock_guard<std::mutex> lk(force_mtx_);
      use_vel_mode_local = su_cmd_use_vel_mode_;
    }
    const bool vel_mode_on = (use_vel_mode_local > 0.5f);

    ReferenceState ref;
    {
      std::lock_guard<std::mutex> lk(state_mtx_);
      ref = getReferenceStateLocked();
    }

    if (!su_int_initialized_) {
      if (!vel_mode_on) {
        su_int_ref_pos_ = sp_in_;
        su_int_ref_yaw_ = sp_in_yaw_;
      } else {
        if (ref.valid) {
          su_int_ref_pos_[0] = ref.pos_w.x();
          su_int_ref_pos_[1] = ref.pos_w.y();
          su_int_ref_pos_[2] = ref.pos_w.z();
          su_int_ref_yaw_ = ref.yaw_w;
        } else {
          su_int_ref_pos_ = {0.0, 0.0, 0.0};
          su_int_ref_yaw_ = 0.0;
        }
      }

      su_int_initialized_ = true;
      su_vel_mode_prev_ = vel_mode_on;
      su_pos_base_valid_ = false;
    }

    if (vel_mode_on && !su_vel_mode_prev_) {
      if (ref.valid) {
        su_int_ref_pos_[0] = ref.pos_w.x();
        su_int_ref_pos_[1] = ref.pos_w.y();
        su_int_ref_pos_[2] = ref.pos_w.z();
        su_int_ref_yaw_ = ref.yaw_w;
      }
      su_pos_base_valid_ = false;
      trajectory_gain_ = 0.0;
      trajectory_phase_time_ = 0.0;
      trajectory_elapsed_sec_ = 0.0;
    }
    else if (!vel_mode_on && su_vel_mode_prev_) {
      su_pos_base_ = su_int_ref_pos_;
      su_yaw_base_ = su_int_ref_yaw_;
      su_pos_base_valid_ = true;
      trajectory_gain_ = 0.0;
      trajectory_elapsed_sec_ = 0.0;
    }
    su_vel_mode_prev_ = vel_mode_on;

    if (!vel_mode_on) {
      geometry_msgs::msg::Vector3Stamped nan_msg;
      nan_msg.header.stamp = now;
      nan_msg.header.frame_id = "contact";
      nan_msg.vector.x = std::numeric_limits<double>::quiet_NaN();
      nan_msg.vector.y = std::numeric_limits<double>::quiet_NaN();
      nan_msg.vector.z = std::numeric_limits<double>::quiet_NaN();
      pub_contact_vel_cmd_->publish(nan_msg);
      pub_contact_vel_actual_->publish(nan_msg);

      if (su_pos_base_valid_) {
        su_int_ref_pos_[0] = su_pos_base_[0] + sp_in_[0];
        su_int_ref_pos_[1] = su_pos_base_[1] + sp_in_[1];
        su_int_ref_pos_[2] = su_pos_base_[2] + sp_in_[2];
        su_int_ref_yaw_    = su_yaw_base_    + sp_in_yaw_;
      } else {
        su_int_ref_pos_ = sp_in_;
        su_int_ref_yaw_ = sp_in_yaw_;
      }
    }
    else {
      double vx_cmd   = sp_in_[0];
      double vy_cmd   = sp_in_[1];
      double vz_cmd   = sp_in_[2];
      double vyaw_cmd = sp_in_yaw_;

      const bool trajectory_target_on = vel_mode_on && trajectory_cmd_enabled_;
      const double ramp_sec = std::max(1e-3, trajectory_ramp_sec_);
      double trajectory_gain_dot = 0.0;
      if (trajectory_target_on) {
        trajectory_elapsed_sec_ = std::min(ramp_sec, trajectory_elapsed_sec_ + dt);
        smoothstepQuintic(
          trajectory_elapsed_sec_, ramp_sec, trajectory_gain_, trajectory_gain_dot);
        trajectory_phase_time_ += dt;
      } else {
        if (trajectory_type_ == "constant_accel") {
          trajectory_elapsed_sec_ = 0.0;
          trajectory_gain_ = 0.0;
          trajectory_gain_dot = 0.0;
          trajectory_phase_time_ = 0.0;
        } else {
        trajectory_elapsed_sec_ = std::max(0.0, trajectory_elapsed_sec_ - dt);
        smoothstepQuintic(
          trajectory_elapsed_sec_, ramp_sec, trajectory_gain_, trajectory_gain_dot);
        trajectory_gain_dot = -trajectory_gain_dot;
        if (trajectory_gain_ > 1e-6) {
          trajectory_phase_time_ += dt;
        } else {
          trajectory_phase_time_ = 0.0;
        }
        }
      }

      if (trajectory_gain_ > 1e-6) {
        double y_ref = 0.0;
        double z_ref = 0.0;
        double y_ref_dot = 0.0;
        double z_ref_dot = 0.0;
        computeTrajectoryReference(
          trajectory_phase_time_, y_ref, z_ref, y_ref_dot, z_ref_dot);
        vy_cmd += trajectory_gain_ * y_ref_dot + trajectory_gain_dot * y_ref;
        vz_cmd += trajectory_gain_ * z_ref_dot + trajectory_gain_dot * z_ref;
      }

      std::array<double,3> eF{0.0, 0.0, 0.0};
      std::array<double,3> eF_dot_filt{0.0, 0.0, 0.0};
      {
        std::lock_guard<std::mutex> lk(force_mtx_);
        eF = eF_;
        eF_dot_filt = eF_dot_filt_;
      }

      const double vel_adm_x = force_adm_kp_ * eF[0] + force_adm_kd_ * eF_dot_filt[0];
      vx_cmd += vel_adm_x;

      Eigen::Vector3d v_c(vx_cmd, vy_cmd, vz_cmd);
      Eigen::Vector3d v_w = v_c;

      Eigen::Matrix3d contact_rotation = Eigen::Matrix3d::Identity();
      Eigen::Vector3d contact_normal = Eigen::Vector3d::UnitX();
      bool contact_frame_ok = false;
      {
        std::lock_guard<std::mutex> lk(force_mtx_);
        contact_rotation = contact_R_C_;
        contact_normal = contact_n_w_;
        contact_frame_ok = contact_frame_received_;
      }

      const bool use_contact_command_frame = (velocity_command_frame_ == "contact");
      if (contact_frame_ok && use_contact_command_frame) {
        v_w = contact_rotation * v_c;
        Eigen::Vector3d v_act_w = ref.valid ? ref.vel_w : Eigen::Vector3d::Zero();
        Eigen::Vector3d v_act_c = contact_rotation.transpose() * v_act_w;
        publishContactVelocityDebug(v_c, v_act_c, now);
      } else if (!use_contact_command_frame) {
        Eigen::Vector3d v_act_w = ref.valid ? ref.vel_w : Eigen::Vector3d::Zero();
        publishContactVelocityDebug(v_w, v_act_w, now);
      } else {
        geometry_msgs::msg::Vector3Stamped nan_msg;
        nan_msg.header.stamp = now;
        nan_msg.header.frame_id = "contact";
        nan_msg.vector.x = std::numeric_limits<double>::quiet_NaN();
        nan_msg.vector.y = std::numeric_limits<double>::quiet_NaN();
        nan_msg.vector.z = std::numeric_limits<double>::quiet_NaN();
        pub_contact_vel_cmd_->publish(nan_msg);
        pub_contact_vel_actual_->publish(nan_msg);
      }

      su_int_ref_pos_[0] += v_w.x() * dt;
      su_int_ref_pos_[1] += v_w.y() * dt;
      su_int_ref_pos_[2] += v_w.z() * dt;

      double yaw_next = su_int_ref_yaw_ + vyaw_cmd * dt;
      const bool manual_yaw_cmd_active = std::abs(vyaw_cmd) > 1e-6;
      if (contact_frame_ok && use_contact_command_frame && !manual_yaw_cmd_active) {
        const Eigen::Vector3d n_w = contact_normal;
        const double nxy = std::hypot(n_w.x(), n_w.y());

        if (nxy > normal_xy_min_) {
          const double psi_align = std::atan2(n_w.y(), n_w.x());
          const double e_yaw = wrap_pi(psi_align - su_int_ref_yaw_);
          const double psi_dot = yaw_align_kp_ * e_yaw;
          yaw_next = su_int_ref_yaw_ + psi_dot * dt;
        }
      }
      su_int_ref_yaw_ = yaw_next;
    }

    publishOut();
  }

  // ==================================================
  // ROS interfaces
  // ==================================================
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr sub_keyboard_;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr sub_use_vel_mode_;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr sub_cmd_force_;
  rclcpp::Subscription<geometry_msgs::msg::QuaternionStamped>::SharedPtr sub_contact_frame_quat_;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr sub_contact_force_x_;

  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr sub_pose_;
  rclcpp::Subscription<geometry_msgs::msg::Vector3Stamped>::SharedPtr sub_vel_;
  rclcpp::Subscription<geometry_msgs::msg::Vector3Stamped>::SharedPtr sub_acc_;

  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr sub_ee_pose_;
  rclcpp::Subscription<geometry_msgs::msg::Vector3Stamped>::SharedPtr sub_ee_vel_;
  rclcpp::Subscription<geometry_msgs::msg::Vector3Stamped>::SharedPtr sub_ee_acc_;

  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr pub_pos_cmd_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr pub_force_lpf_;
  rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr pub_contact_vel_cmd_;
  rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr pub_contact_vel_actual_;

  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pub_cmd_drone_pose_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pub_cmd_ee_pose_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pub_cmd_active_pose_;

  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::TimerBase::SharedPtr force_timer_;

  // ==================================================
  // Parameters / config
  // ==================================================
  std::string reference_object_;
  double yaw_align_kp_{3.0};
  double normal_xy_min_{1e-3};
  std::string velocity_command_frame_{"contact"};

  double force_adm_kp_{5.0};
  double force_adm_kd_{0.005};
  std::string trajectory_type_{"lissajous"};
  double trajectory_ramp_sec_{2.0};
  double lissajous_amp_y_{0.40};
  double lissajous_amp_z_{0.4};
  double lissajous_period_y_{20.0};
  double lissajous_period_z_{10.0};
  double circle_radius_{0.30};
  double circle_period_sec_{20.0};
  double circle_center_y_{0.0};
  double circle_center_z_{0.0};
  double circle_phase_rad_{-0.5 * M_PI};
  double square_parallel_length_{0.40};
  double square_vertical_length_{0.40};
  double square_period_sec_{20.0};
  double square_center_y_{0.0};
  double square_center_z_{0.0};
  double square_phase_{0.0};
  double constant_accel_y_{0.40};
  double constant_accel_z_{0.0};

  Eigen::Vector3d d_B_{0.0, 0.0, 0.0};

  std::string pose_topic_;
  std::string vel_topic_;
  std::string acc_topic_;
  std::string ee_pose_topic_;
  std::string ee_vel_topic_;
  std::string ee_acc_topic_;
  std::string contact_frame_quat_topic_;
  std::string contact_force_x_topic_;

  std::array<double, 3> sp_in_{0.0, 0.0, 0.0};
  double sp_in_yaw_{0.0};
  bool trajectory_cmd_enabled_{false};
  bool sp_received_{false};

  std::array<double, 3> su_int_ref_pos_{0.0, 0.0, 0.0};
  double su_int_ref_yaw_{0.0};
  bool su_int_initialized_{false};

  bool su_vel_mode_prev_{false};
  std::array<double, 3> su_pos_base_{0.0, 0.0, 0.0};
  double su_yaw_base_{0.0};
  bool su_pos_base_valid_{false};
  double trajectory_gain_{0.0};
  double trajectory_phase_time_{0.0};
  double trajectory_elapsed_sec_{0.0};

  std::mutex force_mtx_;
  float su_cmd_use_vel_mode_{0.0f};
  float su_cmd_fx_{0.0f};
  Eigen::Matrix3d contact_R_C_{Eigen::Matrix3d::Identity()};
  Eigen::Vector3d contact_n_w_{Eigen::Vector3d::UnitX()};
  double contact_force_x_{0.0};
  bool contact_frame_received_{false};
  bool contact_force_x_received_{false};

  std::array<double,3> eF_{0.0, 0.0, 0.0};
  std::array<double,3> eF_dot_filt_{0.0, 0.0, 0.0};
  std::array<double,3> eF_prev_{0.0, 0.0, 0.0};
  std::array<double,3> eF_dot_filt_prev_{0.0, 0.0, 0.0};
  bool eF_state_initialized_{false};

  rclcpp::Time eF_last_time_;
  rclcpp::Time last_time_;

  std::mutex state_mtx_;

  std::array<double,3> pose_w_{0.0, 0.0, 0.0};
  std::array<double,3> vel_w_{0.0, 0.0, 0.0};
  std::array<double,3> acc_w_{0.0, 0.0, 0.0};
  double yaw_w_{0.0};
  Eigen::Quaterniond q_WB_{1.0, 0.0, 0.0, 0.0};

  std::array<double,3> ee_pose_w_{0.0, 0.0, 0.0};
  std::array<double,3> ee_vel_w_{0.0, 0.0, 0.0};
  std::array<double,3> ee_acc_w_{0.0, 0.0, 0.0};

  bool pose_received_{false};
  bool vel_received_{false};
  bool acc_received_{false};

  bool ee_pose_received_{false};
  bool ee_vel_received_{false};
  bool ee_acc_received_{false};
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TrajectoryGeneration>());
  rclcpp::shutdown();
  return 0;
}
