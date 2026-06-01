#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/quaternion_stamped.hpp>
#include <geometry_msgs/msg/vector3_stamped.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>

#include <Eigen/Dense>

#include <algorithm>
#include <array>
#include <cmath>
#include <cctype>
#include <limits>
#include <mutex>
#include <string>
#include <vector>

using namespace std::chrono_literals;

class TrajectoryGeneration : public rclcpp::Node
{
public:
  TrajectoryGeneration()
  : Node("trajectory_generation")
  {
    reference_object_ = declare_parameter<std::string>(
      "reference_object", "end_effector");
    command_embedding_mode_ = normalizeModeName(
      declare_parameter<std::string>(
        "command_embedding_mode", "position_loop_embedding"));
    force_control_mode_ = normalizeForceControlMode(
      declare_parameter<std::string>("force_control.mode", "paper"));
    manual_velocity_frame_ = normalizeFrameName(
      declare_parameter<std::string>("manual_velocity_frame", "contact"));
    yaw_strategy_ = normalizeYawStrategy(
      declare_parameter<std::string>("yaw.strategy", "normal_align"));

    yaw_align_kp_ = declare_parameter<double>("yaw.normal_align_kp", 3.0);
    normal_xy_min_ = declare_parameter<double>("yaw.normal_xy_min", 1e-3);
    cbar_tau_ = declare_parameter<double>("yaw.paper.cbar_tau", 0.25);
    epsilon_tau_c_ = declare_parameter<double>("yaw.paper.epsilon_tau_c", 1.0e-6);

    const double preload_k_f_legacy = declare_parameter<double>("preload.k_f", 5.0);
    const double preload_k_l_legacy = declare_parameter<double>("preload.k_l", 2.0);
    const double preload_v_n_max_legacy = declare_parameter<double>("preload.v_n_max", 0.20);
    paper_force_k_f_ = declare_parameter<double>(
      "force_control.paper.k_f", preload_k_f_legacy);
    paper_force_k_l_ = declare_parameter<double>(
      "force_control.paper.k_l", preload_k_l_legacy);
    paper_force_v_n_max_ = declare_parameter<double>(
      "force_control.paper.v_n_max", preload_v_n_max_legacy);
    legacy_force_kp_ = declare_parameter<double>("force_control.legacy.kp", 5.0);
    legacy_force_kd_ = declare_parameter<double>("force_control.legacy.kd", 0.005);
    legacy_force_v_n_max_ = declare_parameter<double>("force_control.legacy.v_n_max", 0.20);
    legacy_force_derivative_cutoff_rad_s_ = declare_parameter<double>(
      "force_control.legacy.derivative_cutoff_rad_s", 3.0);

    const auto requested_pattern_type = declare_parameter<std::string>(
      "pattern.type", "lissajous");
    pattern_type_ = normalizePatternType(requested_pattern_type);
    pattern_ramp_sec_ = declare_parameter<double>("pattern.ramp_sec", 1.0);
    pattern_speed_ = declare_parameter<double>("pattern.v_s", 0.25);

    lissajous_amp_1_ = declare_parameter<double>("lissajous.amp_1", 0.20);
    lissajous_amp_2_ = declare_parameter<double>("lissajous.amp_2", 0.00);
    lissajous_period_1_ = declare_parameter<double>("lissajous.period_1", 15.0);
    lissajous_period_2_ = declare_parameter<double>("lissajous.period_2", 10.0);

    circle_radius_ = declare_parameter<double>("circle.radius", 0.20);
    circle_period_sec_ = declare_parameter<double>("circle.period_sec", 7.0);
    circle_center_1_ = declare_parameter<double>("circle.center_1", 0.0);
    circle_center_2_ = declare_parameter<double>("circle.center_2", 0.0);
    circle_phase_rad_ = declare_parameter<double>("circle.phase_rad", -0.5 * M_PI);

    square_parallel_length_ = declare_parameter<double>("square.parallel_length", 0.40);
    square_vertical_length_ = declare_parameter<double>("square.vertical_length", 0.40);
    square_period_sec_ = declare_parameter<double>("square.period_sec", 20.0);
    square_center_1_ = declare_parameter<double>("square.center_1", 0.0);
    square_center_2_ = declare_parameter<double>("square.center_2", 0.0);
    square_phase_ = declare_parameter<double>("square.phase", 0.0);

    constant_speed_1_ = declare_parameter<double>("constant.speed_1", 0.10);
    constant_speed_2_ = declare_parameter<double>("constant.speed_2", 0.0);

    auto ee_off = declare_parameter<std::vector<double>>(
      "end_effector_offset", {0.09, 0.0, 0.085});
    if (ee_off.size() != 3) {
      RCLCPP_WARN(get_logger(),
        "end_effector_offset must have size 3. Falling back to [0, 0, 0].");
      ee_off = {0.0, 0.0, 0.0};
    }
    d_B_ = Eigen::Vector3d(ee_off[0], ee_off[1], ee_off[2]);

    pose_topic_ = declare_parameter<std::string>("pose_topic", "/crazyflie/out/pose");
    vel_topic_ = declare_parameter<std::string>("vel_topic", "/crazyflie/out/vel");
    acc_topic_ = declare_parameter<std::string>("acc_topic", "/crazyflie/out/acc");
    angvel_topic_ = declare_parameter<std::string>("angvel_topic", "/crazyflie/out/ang_vel");

    ee_pose_topic_ = declare_parameter<std::string>("ee_pose_topic", "/crazyflie/out/EE_pose");
    ee_vel_topic_ = declare_parameter<std::string>("ee_vel_topic", "/crazyflie/out/EE_velocity");
    ee_acc_topic_ = declare_parameter<std::string>("ee_acc_topic", "/crazyflie/out/EE_acceleration");

    contact_frame_quat_topic_ = declare_parameter<std::string>(
      "contact_frame_quat_topic", "/estimated_contact_frame_quat");
    contact_force_x_topic_ = declare_parameter<std::string>(
      "contact_force_x_topic", "/su/contact_force_x");
    normal_debug_metrics_topic_ = declare_parameter<std::string>(
      "normal_debug_metrics_topic", "/normal_vector/debug_metrics");

    sub_keyboard_ = create_subscription<std_msgs::msg::Float64MultiArray>(
      "/su/keyboard_input", 10,
      std::bind(&TrajectoryGeneration::keyboardCb, this, std::placeholders::_1));
    sub_use_vel_mode_ = create_subscription<std_msgs::msg::Float32>(
      "su/use_vel_mode", 10,
      std::bind(&TrajectoryGeneration::useVelModeCb, this, std::placeholders::_1));
    sub_cmd_force_ = create_subscription<std_msgs::msg::Float32>(
      "su/cmd_force", 10,
      std::bind(&TrajectoryGeneration::cmdForceCb, this, std::placeholders::_1));
    sub_contact_frame_quat_ = create_subscription<geometry_msgs::msg::QuaternionStamped>(
      contact_frame_quat_topic_, 10,
      std::bind(&TrajectoryGeneration::contactFrameQuatCb, this, std::placeholders::_1));
    sub_contact_force_x_ = create_subscription<std_msgs::msg::Float32>(
      contact_force_x_topic_, 10,
      std::bind(&TrajectoryGeneration::contactForceXCb, this, std::placeholders::_1));
    sub_normal_debug_metrics_ = create_subscription<std_msgs::msg::Float64MultiArray>(
      normal_debug_metrics_topic_, 10,
      std::bind(&TrajectoryGeneration::normalDebugMetricsCb, this, std::placeholders::_1));

    sub_pose_ = create_subscription<geometry_msgs::msg::PoseStamped>(
      pose_topic_, 10, std::bind(&TrajectoryGeneration::poseCb, this, std::placeholders::_1));
    sub_vel_ = create_subscription<geometry_msgs::msg::Vector3Stamped>(
      vel_topic_, 10, std::bind(&TrajectoryGeneration::velCb, this, std::placeholders::_1));
    sub_acc_ = create_subscription<geometry_msgs::msg::Vector3Stamped>(
      acc_topic_, 10, std::bind(&TrajectoryGeneration::accCb, this, std::placeholders::_1));
    sub_angvel_ = create_subscription<geometry_msgs::msg::Vector3Stamped>(
      angvel_topic_, 10,
      std::bind(&TrajectoryGeneration::angVelCb, this, std::placeholders::_1));

    sub_ee_pose_ = create_subscription<geometry_msgs::msg::PoseStamped>(
      ee_pose_topic_, 10, std::bind(&TrajectoryGeneration::eePoseCb, this, std::placeholders::_1));
    sub_ee_vel_ = create_subscription<geometry_msgs::msg::Vector3Stamped>(
      ee_vel_topic_, 10, std::bind(&TrajectoryGeneration::eeVelCb, this, std::placeholders::_1));
    sub_ee_acc_ = create_subscription<geometry_msgs::msg::Vector3Stamped>(
      ee_acc_topic_, 10, std::bind(&TrajectoryGeneration::eeAccCb, this, std::placeholders::_1));

    pub_pos_cmd_ = create_publisher<std_msgs::msg::Float64MultiArray>(
      "/crazyflie/in/pos_cmd", 10);
    pub_vel_cmd_ = create_publisher<std_msgs::msg::Float64MultiArray>(
      "/crazyflie/in/vel_cmd", 10);
    pub_force_lpf_ = create_publisher<std_msgs::msg::Float64MultiArray>(
      "/su/force_lpf", 10);
    pub_contact_vel_cmd_ = create_publisher<geometry_msgs::msg::Vector3Stamped>(
      "/su/debug/contact_vel_cmd", 10);
    pub_contact_vel_actual_ = create_publisher<geometry_msgs::msg::Vector3Stamped>(
      "/su/debug/contact_vel_actual", 10);
    pub_cmd_drone_pose_ = create_publisher<geometry_msgs::msg::PoseStamped>(
      "/crazyflie/debug/cmd_drone", 10);
    pub_cmd_ee_pose_ = create_publisher<geometry_msgs::msg::PoseStamped>(
      "/crazyflie/debug/cmd_ee", 10);
    pub_cmd_active_pose_ = create_publisher<geometry_msgs::msg::PoseStamped>(
      "/crazyflie/debug/cmd_active", 10);

    timer_ = create_wall_timer(10ms, std::bind(&TrajectoryGeneration::update, this));

    RCLCPP_INFO(get_logger(), "trajectory_generation started");
    RCLCPP_INFO(get_logger(), "reference_object = %s", reference_object_.c_str());
    RCLCPP_INFO(get_logger(), "command_embedding_mode = %s", command_embedding_mode_.c_str());
    RCLCPP_INFO(get_logger(), "force_control.mode = %s", force_control_mode_.c_str());
    RCLCPP_INFO(get_logger(), "yaw.strategy = %s", yaw_strategy_.c_str());
    RCLCPP_INFO(get_logger(), "pattern.type = %s", pattern_type_.c_str());
  }

private:
  struct ReferenceState
  {
    Eigen::Vector3d pos_w = Eigen::Vector3d::Zero();
    Eigen::Vector3d vel_w = Eigen::Vector3d::Zero();
    Eigen::Vector3d acc_w = Eigen::Vector3d::Zero();
    Eigen::Quaterniond q_wb{1.0, 0.0, 0.0, 0.0};
    Eigen::Vector3d angvel_b = Eigen::Vector3d::Zero();
    double yaw_w = 0.0;
    bool valid = false;
  };

  static double wrapPi(double value)
  {
    while (value > M_PI) value -= 2.0 * M_PI;
    while (value < -M_PI) value += 2.0 * M_PI;
    return value;
  }

  static double clamp01(double value)
  {
    return std::clamp(value, 0.0, 1.0);
  }

  static Eigen::Quaterniond quatMsgToEigen(const geometry_msgs::msg::Quaternion & q)
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

  static void smoothstepQuintic(double tau, double duration, double & s, double & s_dot)
  {
    if (duration <= 1e-9) {
      s = 1.0;
      s_dot = 0.0;
      return;
    }

    const double t = std::clamp(tau / duration, 0.0, 1.0);
    const double t2 = t * t;
    const double t3 = t2 * t;
    const double t4 = t3 * t;
    const double t5 = t4 * t;
    s = 10.0 * t3 - 15.0 * t4 + 6.0 * t5;
    s_dot = (30.0 * t2 - 60.0 * t3 + 30.0 * t4) / duration;
  }

  static std::string normalizeLower(std::string value)
  {
    std::transform(value.begin(), value.end(), value.begin(),
      [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
    return value;
  }

  static std::string normalizePatternType(std::string value)
  {
    value = normalizeLower(std::move(value));
    if (value == "circle" || value == "circular") {
      return "circle";
    }
    if (value == "square") {
      return "square";
    }
    if (value == "constant" || value == "constant_velocity") {
      return "constant";
    }
    return "lissajous";
  }

  static std::string normalizeFrameName(std::string value)
  {
    value = normalizeLower(std::move(value));
    if (value != "contact" && value != "world") {
      return "contact";
    }
    return value;
  }

  static std::string normalizeModeName(std::string value)
  {
    value = normalizeLower(std::move(value));
    if (
      value == "velocity_loop_direct" ||
      value == "velocity_direct" ||
      value == "direct")
    {
      return "velocity_loop_direct";
    }
    return "position_loop_embedding";
  }

  static std::string normalizeYawStrategy(std::string value)
  {
    value = normalizeLower(std::move(value));
    if (value == "paper" || value == "directional_consistency") {
      return "paper";
    }
    if (value == "none" || value == "manual") {
      return "none";
    }
    return "normal_align";
  }

  static std::string normalizeForceControlMode(std::string value)
  {
    value = normalizeLower(std::move(value));
    if (value == "legacy" || value == "previous" || value == "old") {
      return "legacy";
    }
    return "paper";
  }

  static double positivePeriod(double period_sec)
  {
    return std::max(1e-3, period_sec);
  }

  void computePatternState(
    double path_parameter,
    double & zeta_1,
    double & zeta_2,
    double & dzeta_1_ds,
    double & dzeta_2_ds) const
  {
    if (pattern_type_ == "circle") {
      const double radius = std::max(0.0, circle_radius_);
      const double w = 2.0 * M_PI / positivePeriod(circle_period_sec_);
      const double theta = w * path_parameter + circle_phase_rad_;
      zeta_1 = circle_center_1_ + radius * std::cos(theta);
      zeta_2 = circle_center_2_ + radius * std::sin(theta);
      dzeta_1_ds = -radius * w * std::sin(theta);
      dzeta_2_ds = radius * w * std::cos(theta);
      return;
    }

    if (pattern_type_ == "square") {
      const double parallel_length = std::max(0.0, square_parallel_length_);
      const double vertical_length = std::max(0.0, square_vertical_length_);
      const double half_parallel = 0.5 * parallel_length;
      const double half_vertical = 0.5 * vertical_length;
      const double period = positivePeriod(square_period_sec_);
      const double perimeter = 2.0 * (parallel_length + vertical_length);

      if (perimeter < 1e-9) {
        zeta_1 = square_center_1_;
        zeta_2 = square_center_2_;
        dzeta_1_ds = 0.0;
        dzeta_2_ds = 0.0;
        return;
      }

      const double speed = perimeter / period;
      double phase = std::fmod(path_parameter / period + square_phase_, 1.0);
      if (phase < 0.0) {
        phase += 1.0;
      }

      const double s = phase * perimeter;
      const double edge0_end = parallel_length;
      const double edge1_end = edge0_end + vertical_length;
      const double edge2_end = edge1_end + parallel_length;

      if (s < edge0_end) {
        zeta_1 = square_center_1_ - half_parallel + s;
        zeta_2 = square_center_2_ - half_vertical;
        dzeta_1_ds = speed;
        dzeta_2_ds = 0.0;
      } else if (s < edge1_end) {
        const double edge_s = s - edge0_end;
        zeta_1 = square_center_1_ + half_parallel;
        zeta_2 = square_center_2_ - half_vertical + edge_s;
        dzeta_1_ds = 0.0;
        dzeta_2_ds = speed;
      } else if (s < edge2_end) {
        const double edge_s = s - edge1_end;
        zeta_1 = square_center_1_ + half_parallel - edge_s;
        zeta_2 = square_center_2_ + half_vertical;
        dzeta_1_ds = -speed;
        dzeta_2_ds = 0.0;
      } else {
        const double edge_s = s - edge2_end;
        zeta_1 = square_center_1_ - half_parallel;
        zeta_2 = square_center_2_ + half_vertical - edge_s;
        dzeta_1_ds = 0.0;
        dzeta_2_ds = -speed;
      }
      return;
    }

    if (pattern_type_ == "constant") {
      zeta_1 = constant_speed_1_ * path_parameter;
      zeta_2 = constant_speed_2_ * path_parameter;
      dzeta_1_ds = constant_speed_1_;
      dzeta_2_ds = constant_speed_2_;
      return;
    }

    const double w1 = 2.0 * M_PI / positivePeriod(lissajous_period_1_);
    const double w2 = 2.0 * M_PI / positivePeriod(lissajous_period_2_);
    zeta_1 = lissajous_amp_1_ * std::sin(w1 * path_parameter);
    zeta_2 = lissajous_amp_2_ * std::sin(w2 * path_parameter);
    dzeta_1_ds = lissajous_amp_1_ * w1 * std::cos(w1 * path_parameter);
    dzeta_2_ds = lissajous_amp_2_ * w2 * std::cos(w2 * path_parameter);
  }

  ReferenceState getReferenceStateLocked() const
  {
    ReferenceState ref;
    const bool use_ee = (reference_object_ == "end_effector");

    if (!use_ee) {
      if (!(pose_received_ && vel_received_ && acc_received_ && angvel_received_)) {
        return ref;
      }
      ref.pos_w = Eigen::Vector3d(pose_w_[0], pose_w_[1], pose_w_[2]);
      ref.vel_w = Eigen::Vector3d(vel_w_[0], vel_w_[1], vel_w_[2]);
      ref.acc_w = Eigen::Vector3d(acc_w_[0], acc_w_[1], acc_w_[2]);
      ref.angvel_b = Eigen::Vector3d(angvel_b_[0], angvel_b_[1], angvel_b_[2]);
      ref.q_wb = q_WB_;
      ref.yaw_w = yaw_w_;
      ref.valid = true;
      return ref;
    }

    if (!(pose_received_ && ee_pose_received_ && ee_vel_received_ && ee_acc_received_ && angvel_received_)) {
      return ref;
    }

    ref.pos_w = Eigen::Vector3d(ee_pose_w_[0], ee_pose_w_[1], ee_pose_w_[2]);
    ref.vel_w = Eigen::Vector3d(ee_vel_w_[0], ee_vel_w_[1], ee_vel_w_[2]);
    ref.acc_w = Eigen::Vector3d(ee_acc_w_[0], ee_acc_w_[1], ee_acc_w_[2]);
    ref.angvel_b = Eigen::Vector3d(angvel_b_[0], angvel_b_[1], angvel_b_[2]);
    ref.q_wb = q_WB_;
    ref.yaw_w = yaw_w_;
    ref.valid = true;
    return ref;
  }

  void publishDebugCmdPoses(
    const Eigen::Vector3d & p_drone_des_w,
    const Eigen::Vector3d & p_ee_des_w,
    double yaw_des,
    const rclcpp::Time & stamp)
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
    const Eigen::Vector3d & v_cmd_c,
    const Eigen::Vector3d & v_act_c,
    const rclcpp::Time & stamp)
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

  void publishNaNContactVelocityDebug(const rclcpp::Time & stamp)
  {
    geometry_msgs::msg::Vector3Stamped nan_msg;
    nan_msg.header.stamp = stamp;
    nan_msg.header.frame_id = "contact";
    nan_msg.vector.x = std::numeric_limits<double>::quiet_NaN();
    nan_msg.vector.y = std::numeric_limits<double>::quiet_NaN();
    nan_msg.vector.z = std::numeric_limits<double>::quiet_NaN();
    pub_contact_vel_cmd_->publish(nan_msg);
    pub_contact_vel_actual_->publish(nan_msg);
  }

  void syncIntegratedReferenceToMeasured(const ReferenceState & ref)
  {
    if (ref.valid) {
      active_ref_pos_w_ = ref.pos_w;
      active_ref_yaw_ = ref.yaw_w;
    } else {
      active_ref_pos_w_.setZero();
      active_ref_yaw_ = 0.0;
    }
  }

  void keyboardCb(const std_msgs::msg::Float64MultiArray::SharedPtr msg)
  {
    if (msg->data.size() < 4) {
      return;
    }

    std::lock_guard<std::mutex> lk(cmd_mtx_);
    sp_in_[0] = msg->data[0];
    sp_in_[1] = msg->data[1];
    sp_in_[2] = msg->data[2];
    sp_in_yaw_ = msg->data[3];
    trajectory_cmd_enabled_ = (msg->data.size() >= 5 && msg->data[4] > 0.5);
    sp_received_ = true;
  }

  void useVelModeCb(const std_msgs::msg::Float32::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lk(contact_mtx_);
    use_vel_mode_ = (msg->data > 0.5f);
  }

  void cmdForceCb(const std_msgs::msg::Float32::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lk(contact_mtx_);
    cmd_force_desired_ = static_cast<double>(msg->data);
  }

  void contactFrameQuatCb(const geometry_msgs::msg::QuaternionStamped::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lk(contact_mtx_);
    const Eigen::Quaterniond q_wc = quatMsgToEigen(msg->quaternion).normalized();
    contact_R_C_ = q_wc.toRotationMatrix();
    contact_n_w_ = contact_R_C_.col(0);
    contact_t1_w_ = contact_R_C_.col(1);
    contact_t2_w_ = contact_R_C_.col(2);
    contact_frame_received_ = true;
  }

  void contactForceXCb(const std_msgs::msg::Float32::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lk(contact_mtx_);
    contact_force_x_ = static_cast<double>(msg->data);
    contact_force_x_received_ = true;
  }

  void normalDebugMetricsCb(const std_msgs::msg::Float64MultiArray::SharedPtr msg)
  {
    if (msg->data.size() < 37) {
      return;
    }

    std::lock_guard<std::mutex> lk(contact_mtx_);
    force_direction_n_f_ = Eigen::Vector3d(msg->data[31], msg->data[32], msg->data[33]);
    have_force_direction_n_f_ = std::isfinite(force_direction_n_f_.norm());
  }

  void poseCb(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lk(state_mtx_);
    pose_w_[0] = msg->pose.position.x;
    pose_w_[1] = msg->pose.position.y;
    pose_w_[2] = msg->pose.position.z;
    q_WB_ = quatMsgToEigen(msg->pose.orientation).normalized();

    const auto & q = msg->pose.orientation;
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

  void angVelCb(const geometry_msgs::msg::Vector3Stamped::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lk(state_mtx_);
    angvel_b_[0] = msg->vector.x;
    angvel_b_[1] = msg->vector.y;
    angvel_b_[2] = msg->vector.z;
    angvel_received_ = true;
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

  void publishPositionCommand(const Eigen::Vector3d & p_drone_des_w, double yaw_des)
  {
    std_msgs::msg::Float64MultiArray out;
    out.data.resize(4);
    out.data[0] = p_drone_des_w.x();
    out.data[1] = p_drone_des_w.y();
    out.data[2] = p_drone_des_w.z();
    out.data[3] = yaw_des;
    pub_pos_cmd_->publish(out);
  }

  void publishVelocityCommand(const Eigen::Vector3d & v_drone_des_w, double yaw_des)
  {
    std_msgs::msg::Float64MultiArray out;
    out.data.resize(4);
    out.data[0] = v_drone_des_w.x();
    out.data[1] = v_drone_des_w.y();
    out.data[2] = v_drone_des_w.z();
    out.data[3] = yaw_des;
    pub_vel_cmd_->publish(out);
  }

  void update()
  {
    {
      std::lock_guard<std::mutex> lk(cmd_mtx_);
      if (!sp_received_) {
        return;
      }
    }

    const rclcpp::Time now = this->now();
    double dt = 0.0;
    if (last_update_time_.nanoseconds() != 0) {
      dt = (now - last_update_time_).seconds();
    }
    last_update_time_ = now;

    if (dt <= 1e-5 || dt > 0.1) {
      return;
    }

    ReferenceState ref;
    {
      std::lock_guard<std::mutex> lk(state_mtx_);
      ref = getReferenceStateLocked();
    }

    std::array<double, 3> sp_in_local{};
    double sp_in_yaw_local = 0.0;
    bool trajectory_enabled_local = false;
    {
      std::lock_guard<std::mutex> lk(cmd_mtx_);
      sp_in_local = sp_in_;
      sp_in_yaw_local = sp_in_yaw_;
      trajectory_enabled_local = trajectory_cmd_enabled_;
    }

    bool use_vel_mode_local = false;
    Eigen::Matrix3d contact_R_C = Eigen::Matrix3d::Identity();
    Eigen::Vector3d contact_n_w = Eigen::Vector3d::UnitX();
    Eigen::Vector3d contact_t1_w = Eigen::Vector3d::UnitY();
    Eigen::Vector3d contact_t2_w = Eigen::Vector3d::UnitZ();
    Eigen::Vector3d force_direction_n_f = Eigen::Vector3d::Zero();
    double contact_force_x_local = 0.0;
    double cmd_force_desired_local = 0.0;
    bool contact_frame_ok = false;
    bool contact_force_ok = false;
    bool have_force_direction_n_f = false;
    {
      std::lock_guard<std::mutex> lk(contact_mtx_);
      use_vel_mode_local = use_vel_mode_;
      contact_R_C = contact_R_C_;
      contact_n_w = contact_n_w_;
      contact_t1_w = contact_t1_w_;
      contact_t2_w = contact_t2_w_;
      force_direction_n_f = force_direction_n_f_;
      contact_force_x_local = contact_force_x_;
      cmd_force_desired_local = cmd_force_desired_;
      contact_frame_ok = contact_frame_received_;
      contact_force_ok = contact_force_x_received_;
      have_force_direction_n_f = have_force_direction_n_f_;
    }

    if (!integrated_ref_initialized_) {
      syncIntegratedReferenceToMeasured(ref);
      integrated_ref_initialized_ = true;
      prev_use_vel_mode_ = use_vel_mode_local;
    }

    if (use_vel_mode_local && !prev_use_vel_mode_) {
      syncIntegratedReferenceToMeasured(ref);
      pattern_ramp_state_sec_ = 0.0;
      path_parameter_s_ = 0.0;
      nu_n_state_ = 0.0;
      legacy_force_error_initialized_ = false;
      legacy_force_error_dot_filt_ = 0.0;
    }
    if (!use_vel_mode_local && prev_use_vel_mode_) {
      nu_n_state_ = 0.0;
      legacy_force_error_initialized_ = false;
      legacy_force_error_dot_filt_ = 0.0;
    }
    prev_use_vel_mode_ = use_vel_mode_local;

    Eigen::Vector3d p_drone_des_w = Eigen::Vector3d::Zero();
    Eigen::Vector3d p_ee_des_w = Eigen::Vector3d::Zero();

    if (!use_vel_mode_local) {
      publishNaNContactVelocityDebug(now);

      if (ref.valid) {
        const Eigen::Vector3d pos_delta_world(sp_in_local[0], sp_in_local[1], sp_in_local[2]);
        if (reference_object_ == "end_effector") {
          p_ee_des_w = ref.pos_w + pos_delta_world;
          p_drone_des_w = p_ee_des_w - ref.q_wb.toRotationMatrix() * d_B_;
        } else {
          p_drone_des_w = ref.pos_w + pos_delta_world;
          p_ee_des_w = p_drone_des_w + ref.q_wb.toRotationMatrix() * d_B_;
        }
        active_ref_pos_w_ = (reference_object_ == "end_effector") ? p_ee_des_w : p_drone_des_w;
        active_ref_yaw_ = ref.yaw_w + sp_in_yaw_local;
      }

      publishPositionCommand(p_drone_des_w, active_ref_yaw_);
      publishDebugCmdPoses(p_drone_des_w, p_ee_des_w, active_ref_yaw_, now);
      return;
    }

    const bool contact_basis_ok =
      std::isfinite(contact_n_w.norm()) &&
      std::isfinite(contact_t1_w.norm()) &&
      std::isfinite(contact_t2_w.norm()) &&
      contact_n_w.norm() > 1e-6 &&
      contact_t1_w.norm() > 1e-6 &&
      contact_t2_w.norm() > 1e-6;

    const Eigen::Vector3d manual_cmd_local(sp_in_local[0], sp_in_local[1], sp_in_local[2]);
    Eigen::Vector3d manual_cmd_world = manual_cmd_local;
    if (manual_velocity_frame_ == "contact" && contact_basis_ok) {
      manual_cmd_world = contact_R_C * manual_cmd_local;
    }

    double ramp_target = trajectory_enabled_local ? 1.0 : 0.0;
    if (ramp_target > pattern_gain_) {
      pattern_ramp_state_sec_ = std::min(pattern_ramp_sec_, pattern_ramp_state_sec_ + dt);
    } else {
      pattern_ramp_state_sec_ = std::max(0.0, pattern_ramp_state_sec_ - dt);
    }
    double pattern_gain_dot = 0.0;
    smoothstepQuintic(pattern_ramp_state_sec_, std::max(1e-3, pattern_ramp_sec_), pattern_gain_, pattern_gain_dot);
    if (!trajectory_enabled_local && pattern_ramp_state_sec_ <= 1e-6) {
      pattern_gain_ = 0.0;
      pattern_gain_dot = 0.0;
      path_parameter_s_ = 0.0;
    }

    double directional_consistency_score = 1.0;
    double alpha_frame = 1.0;
    if (
      force_control_mode_ == "paper" &&
      contact_basis_ok &&
      have_force_direction_n_f &&
      ref.valid)
    {
      const Eigen::Vector3d q_f = -force_direction_n_f.normalized();
      const Eigen::Vector3d r_world = ref.q_wb.toRotationMatrix() * d_B_;
      const Eigen::Vector3d tau_q = r_world.cross(q_f);
      const double denom =
        contact_t1_w.squaredNorm() * tau_q.squaredNorm() + epsilon_tau_c_;
      if (denom > 1e-12) {
        const double numer = std::pow(contact_t1_w.dot(tau_q), 2.0);
        directional_consistency_score = std::clamp(numer / denom, 0.0, 1.0);
        alpha_frame = std::exp(
          -(1.0 - directional_consistency_score) / std::max(1e-6, cbar_tau_));
        alpha_frame = clamp01(alpha_frame);
      }
    }

    const double s_dot = pattern_gain_ * alpha_frame * pattern_speed_;
    path_parameter_s_ += s_dot * dt;

    double zeta_1 = 0.0;
    double zeta_2 = 0.0;
    double dzeta_1_ds = 0.0;
    double dzeta_2_ds = 0.0;
    computePatternState(path_parameter_s_, zeta_1, zeta_2, dzeta_1_ds, dzeta_2_ds);

    Eigen::Vector3d v_tan_world = Eigen::Vector3d::Zero();
    if (contact_basis_ok) {
      const Eigen::Vector2d u_tan(dzeta_1_ds * s_dot, dzeta_2_ds * s_dot);
      v_tan_world = contact_t1_w * u_tan.x() + contact_t2_w * u_tan.y();
    }

    double v_n_feedback = 0.0;
    if (contact_force_ok) {
      const double force_error = cmd_force_desired_local - contact_force_x_local;

      if (force_control_mode_ == "legacy") {
        double force_error_dot_raw = 0.0;
        if (legacy_force_error_initialized_) {
          force_error_dot_raw = (force_error - legacy_force_error_prev_) / std::max(1e-6, dt);
          const double alpha = std::clamp(
            dt * legacy_force_derivative_cutoff_rad_s_ /
            (1.0 + dt * legacy_force_derivative_cutoff_rad_s_),
            0.0, 1.0);
          legacy_force_error_dot_filt_ +=
            alpha * (force_error_dot_raw - legacy_force_error_dot_filt_);
        } else {
          legacy_force_error_dot_filt_ = 0.0;
          legacy_force_error_initialized_ = true;
        }
        legacy_force_error_prev_ = force_error;
        v_n_feedback =
          legacy_force_kp_ * force_error +
          legacy_force_kd_ * legacy_force_error_dot_filt_;
        v_n_feedback = std::clamp(
          v_n_feedback,
          -legacy_force_v_n_max_,
          legacy_force_v_n_max_);

        std_msgs::msg::Float64MultiArray dbg;
        dbg.data.resize(2);
        dbg.data[0] = force_error_dot_raw;
        dbg.data[1] = legacy_force_error_dot_filt_;
        pub_force_lpf_->publish(dbg);
      } else {
        const double nu_dot = paper_force_k_f_ * force_error - paper_force_k_l_ * nu_n_state_;
        nu_n_state_ += dt * nu_dot;
        nu_n_state_ = std::clamp(nu_n_state_, -paper_force_v_n_max_, paper_force_v_n_max_);
        v_n_feedback = nu_n_state_;

        std_msgs::msg::Float64MultiArray dbg;
        dbg.data.resize(2);
        dbg.data[0] = force_error;
        dbg.data[1] = nu_n_state_;
        pub_force_lpf_->publish(dbg);
      }
    } else {
      nu_n_state_ -= paper_force_k_l_ * nu_n_state_ * dt;
      nu_n_state_ = std::clamp(nu_n_state_, -paper_force_v_n_max_, paper_force_v_n_max_);
      legacy_force_error_initialized_ = false;
      legacy_force_error_dot_filt_ = 0.0;
    }

    const double v_n_limit =
      (force_control_mode_ == "legacy") ? legacy_force_v_n_max_ : paper_force_v_n_max_;
    const double manual_v_n = std::clamp(
      manual_cmd_local.x(),
      -v_n_limit,
      v_n_limit);
    const double commanded_v_n = std::clamp(
      manual_v_n + v_n_feedback,
      -v_n_limit,
      v_n_limit);

    Eigen::Vector3d v_contact_des_world = manual_cmd_world;
    if (contact_basis_ok) {
      if (manual_velocity_frame_ == "contact") {
        const Eigen::Vector3d manual_tangential_world =
          contact_t1_w * manual_cmd_local.y() + contact_t2_w * manual_cmd_local.z();
        v_contact_des_world =
          manual_tangential_world + commanded_v_n * contact_n_w + v_tan_world;
      } else {
        v_contact_des_world += v_tan_world + commanded_v_n * contact_n_w;
      }

      const Eigen::Vector3d v_act_w = ref.valid ? ref.vel_w : Eigen::Vector3d::Zero();
      publishContactVelocityDebug(
        contact_R_C.transpose() * v_contact_des_world,
        contact_R_C.transpose() * v_act_w,
        now);
    } else {
      publishNaNContactVelocityDebug(now);
    }

    Eigen::Vector3d v_active_des_world = v_contact_des_world;
    Eigen::Vector3d v_drone_des_world = v_contact_des_world;
    if (reference_object_ == "end_effector" && ref.valid) {
      const Eigen::Vector3d tip_offset_rate_world =
        ref.q_wb.toRotationMatrix() * (ref.angvel_b.cross(d_B_));
      v_drone_des_world = v_contact_des_world - tip_offset_rate_world;
      v_active_des_world = v_contact_des_world;
    } else if (reference_object_ == "drone") {
      v_active_des_world = v_drone_des_world;
    }

    active_ref_pos_w_ += v_active_des_world * dt;

    const bool manual_yaw_cmd_active = std::abs(sp_in_yaw_local) > 1e-6;
    if (manual_yaw_cmd_active) {
      active_ref_yaw_ = wrapPi(active_ref_yaw_ + sp_in_yaw_local * dt);
    } else if (yaw_strategy_ == "normal_align" && contact_frame_ok) {
      const double nxy = std::hypot(contact_n_w.x(), contact_n_w.y());
      if (nxy > normal_xy_min_) {
        const double psi_align = std::atan2(contact_n_w.y(), contact_n_w.x());
        const double e_yaw = wrapPi(psi_align - active_ref_yaw_);
        active_ref_yaw_ = wrapPi(active_ref_yaw_ + yaw_align_kp_ * e_yaw * dt);
      }
    } else if (!ref.valid) {
      active_ref_yaw_ = wrapPi(active_ref_yaw_);
    }

    if (reference_object_ == "end_effector") {
      p_ee_des_w = active_ref_pos_w_;
      if (ref.valid) {
        p_drone_des_w = p_ee_des_w - ref.q_wb.toRotationMatrix() * d_B_;
      }
    } else {
      p_drone_des_w = active_ref_pos_w_;
      if (ref.valid) {
        p_ee_des_w = p_drone_des_w + ref.q_wb.toRotationMatrix() * d_B_;
      }
    }

    if (command_embedding_mode_ == "velocity_loop_direct") {
      publishVelocityCommand(v_drone_des_world, active_ref_yaw_);
    } else {
      publishPositionCommand(p_drone_des_w, active_ref_yaw_);
    }
    publishDebugCmdPoses(p_drone_des_w, p_ee_des_w, active_ref_yaw_, now);
  }

  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr sub_keyboard_;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr sub_use_vel_mode_;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr sub_cmd_force_;
  rclcpp::Subscription<geometry_msgs::msg::QuaternionStamped>::SharedPtr sub_contact_frame_quat_;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr sub_contact_force_x_;
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr sub_normal_debug_metrics_;

  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr sub_pose_;
  rclcpp::Subscription<geometry_msgs::msg::Vector3Stamped>::SharedPtr sub_vel_;
  rclcpp::Subscription<geometry_msgs::msg::Vector3Stamped>::SharedPtr sub_acc_;
  rclcpp::Subscription<geometry_msgs::msg::Vector3Stamped>::SharedPtr sub_angvel_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr sub_ee_pose_;
  rclcpp::Subscription<geometry_msgs::msg::Vector3Stamped>::SharedPtr sub_ee_vel_;
  rclcpp::Subscription<geometry_msgs::msg::Vector3Stamped>::SharedPtr sub_ee_acc_;

  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr pub_pos_cmd_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr pub_vel_cmd_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr pub_force_lpf_;
  rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr pub_contact_vel_cmd_;
  rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr pub_contact_vel_actual_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pub_cmd_drone_pose_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pub_cmd_ee_pose_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pub_cmd_active_pose_;

  rclcpp::TimerBase::SharedPtr timer_;

  std::string reference_object_;
  std::string command_embedding_mode_;
  std::string force_control_mode_;
  std::string manual_velocity_frame_;
  std::string yaw_strategy_;
  std::string pattern_type_;

  double yaw_align_kp_{3.0};
  double normal_xy_min_{1e-3};
  double cbar_tau_{0.25};
  double epsilon_tau_c_{1.0e-6};

  double paper_force_k_f_{5.0};
  double paper_force_k_l_{2.0};
  double paper_force_v_n_max_{0.20};
  double legacy_force_kp_{5.0};
  double legacy_force_kd_{0.005};
  double legacy_force_v_n_max_{0.20};
  double legacy_force_derivative_cutoff_rad_s_{3.0};

  double pattern_ramp_sec_{1.0};
  double pattern_speed_{0.25};

  double lissajous_amp_1_{0.20};
  double lissajous_amp_2_{0.00};
  double lissajous_period_1_{15.0};
  double lissajous_period_2_{10.0};

  double circle_radius_{0.20};
  double circle_period_sec_{7.0};
  double circle_center_1_{0.0};
  double circle_center_2_{0.0};
  double circle_phase_rad_{-0.5 * M_PI};

  double square_parallel_length_{0.40};
  double square_vertical_length_{0.40};
  double square_period_sec_{20.0};
  double square_center_1_{0.0};
  double square_center_2_{0.0};
  double square_phase_{0.0};

  double constant_speed_1_{0.10};
  double constant_speed_2_{0.0};

  Eigen::Vector3d d_B_{0.0, 0.0, 0.0};

  std::string pose_topic_;
  std::string vel_topic_;
  std::string acc_topic_;
  std::string angvel_topic_;
  std::string ee_pose_topic_;
  std::string ee_vel_topic_;
  std::string ee_acc_topic_;
  std::string contact_frame_quat_topic_;
  std::string contact_force_x_topic_;
  std::string normal_debug_metrics_topic_;

  std::mutex cmd_mtx_;
  std::array<double, 3> sp_in_{0.0, 0.0, 0.0};
  double sp_in_yaw_{0.0};
  bool trajectory_cmd_enabled_{false};
  bool sp_received_{false};

  std::mutex contact_mtx_;
  bool use_vel_mode_{true};
  double cmd_force_desired_{0.0};
  Eigen::Matrix3d contact_R_C_{Eigen::Matrix3d::Identity()};
  Eigen::Vector3d contact_n_w_{Eigen::Vector3d::UnitX()};
  Eigen::Vector3d contact_t1_w_{Eigen::Vector3d::UnitY()};
  Eigen::Vector3d contact_t2_w_{Eigen::Vector3d::UnitZ()};
  double contact_force_x_{0.0};
  Eigen::Vector3d force_direction_n_f_{Eigen::Vector3d::Zero()};
  bool contact_frame_received_{false};
  bool contact_force_x_received_{false};
  bool have_force_direction_n_f_{false};

  std::mutex state_mtx_;
  std::array<double, 3> pose_w_{0.0, 0.0, 0.0};
  std::array<double, 3> vel_w_{0.0, 0.0, 0.0};
  std::array<double, 3> acc_w_{0.0, 0.0, 0.0};
  std::array<double, 3> angvel_b_{0.0, 0.0, 0.0};
  std::array<double, 3> ee_pose_w_{0.0, 0.0, 0.0};
  std::array<double, 3> ee_vel_w_{0.0, 0.0, 0.0};
  std::array<double, 3> ee_acc_w_{0.0, 0.0, 0.0};
  Eigen::Quaterniond q_WB_{1.0, 0.0, 0.0, 0.0};
  double yaw_w_{0.0};
  bool pose_received_{false};
  bool vel_received_{false};
  bool acc_received_{false};
  bool angvel_received_{false};
  bool ee_pose_received_{false};
  bool ee_vel_received_{false};
  bool ee_acc_received_{false};

  bool integrated_ref_initialized_{false};
  bool prev_use_vel_mode_{true};
  Eigen::Vector3d active_ref_pos_w_{Eigen::Vector3d::Zero()};
  double active_ref_yaw_{0.0};
  double pattern_gain_{0.0};
  double pattern_ramp_state_sec_{0.0};
  double path_parameter_s_{0.0};
  double nu_n_state_{0.0};
  bool legacy_force_error_initialized_{false};
  double legacy_force_error_prev_{0.0};
  double legacy_force_error_dot_filt_{0.0};

  rclcpp::Time last_update_time_{0, 0, RCL_ROS_TIME};
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TrajectoryGeneration>());
  rclcpp::shutdown();
  return 0;
}
