#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/quaternion_stamped.hpp>
#include <geometry_msgs/msg/vector3_stamped.hpp>
#include <geometry_msgs/msg/wrench_stamped.hpp>
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
    const int su_traj1_shape = declare_parameter<int>("su_position.traj1Shape", 3);
    const double su_traj1_size_x = declare_parameter<double>("su_position.traj1SizeX", 0.20);
    const double su_traj1_size_y = declare_parameter<double>("su_position.traj1SizeY", 0.00);
    const double su_traj1_speed = declare_parameter<double>("su_position.traj1Speed", 0.25);
    const int su_alpha_frame_mode = declare_parameter<int>("su_position.alpha_frame_mode", 0);
    const double su_epsilon_tau_n = declare_parameter<double>("su_position.epsilon_tau_n", 1.0e-6);
    const double su_c_bar_tau = declare_parameter<double>("su_position.c_bar_tau", 0.20);
    const double su_omega_bar_n = declare_parameter<double>("su_position.omega_bar_n", 1.0);
    const double su_normal_vel_bar_n = declare_parameter<double>("su_position.normal_vel_bar_n", 0.0);
    const int su_alpha_gate_power_r =
      declare_parameter<int>("su_position.alpha_gate_power_r", 2);
    const double su_omega_n_lpf_cutoff_hz =
      declare_parameter<double>("su_position.omega_n_lpf_cutoff_hz", 0.0);
    const double su_normal_leakage_lpf_cutoff_hz =
      declare_parameter<double>("su_position.normal_leakage_lpf_cutoff_hz", 0.0);
    const double su_alpha_min = declare_parameter<double>("su_position.alpha_min", 0.0);
    const double su_preload_gf = declare_parameter<double>("su_position.preloadGf", 5.0);
    const double su_preload_gv = declare_parameter<double>("su_position.preloadGv", 2.0);
    const double su_preload_nu = declare_parameter<double>("su_position.preloadNu", 0.20);
    const int su_preload_en = declare_parameter<int>("su_position.preloadEn", 1);

    reference_object_ = declare_parameter<std::string>(
      "reference_object", "end_effector");
    command_embedding_mode_ = normalizeModeName(
      declare_parameter<std::string>(
        "command_embedding_mode", "position_loop_embedding"));
    force_control_mode_ = normalizeForceControlMode(
      declare_parameter<std::string>("force_control.mode", "paper"));
    manual_velocity_frame_ = normalizeFrameName(
      declare_parameter<std::string>("manual_velocity_frame", "contact"));
    yaw_align_kp_ = declare_parameter<double>("yaw.normal_align_kp", 3.0);
    normal_xy_min_ = declare_parameter<double>("yaw.normal_xy_min", 1e-3);

    declare_parameter<double>("preload.k_f", su_preload_gf);
    declare_parameter<double>("preload.k_l", su_preload_gv);
    declare_parameter<double>("preload.v_n_max", su_preload_nu);
    preload_enable_ = (su_preload_en != 0);
    paper_force_k_f_ = declare_parameter<double>(
      "force_control.paper.k_f", su_preload_gf);
    paper_force_k_l_ = declare_parameter<double>(
      "force_control.paper.k_l", su_preload_gv);
    paper_force_v_n_max_ = declare_parameter<double>(
      "force_control.paper.v_n_max", su_preload_nu);
    legacy_force_kp_ = declare_parameter<double>("force_control.legacy.kp", 5.0);
    legacy_force_kd_ = declare_parameter<double>("force_control.legacy.kd", 0.005);
    legacy_force_v_n_max_ = declare_parameter<double>("force_control.legacy.v_n_max", 0.20);
    legacy_force_derivative_cutoff_rad_s_ = declare_parameter<double>(
      "force_control.legacy.derivative_cutoff_rad_s", 3.0);

    const auto requested_pattern_type = declare_parameter<std::string>(
      "pattern.type", shapeIdToPatternType(su_traj1_shape));
    pattern_type_ = normalizePatternType(requested_pattern_type);
    pattern_ramp_sec_ = declare_parameter<double>("pattern.ramp_sec", 1.0);
    pattern_speed_ = declare_parameter<double>("pattern.v_s", su_traj1_speed);
    alpha_frame_mode_ = std::max(0, su_alpha_frame_mode);
    epsilon_tau_n_ = su_epsilon_tau_n;
    c_bar_tau_ = su_c_bar_tau;
    omega_bar_n_ = su_omega_bar_n;
    normal_vel_bar_n_ = su_normal_vel_bar_n;
    alpha_gate_power_r_ = (su_alpha_gate_power_r == 1) ? 1 : 2;
    if (su_alpha_gate_power_r != alpha_gate_power_r_) {
      RCLCPP_WARN(
        get_logger(),
        "su_position.alpha_gate_power_r=%d is unsupported. Falling back to %d.",
        su_alpha_gate_power_r,
        alpha_gate_power_r_);
    }
    omega_n_lpf_cutoff_hz_ = su_omega_n_lpf_cutoff_hz;
    normal_leakage_lpf_cutoff_hz_ = su_normal_leakage_lpf_cutoff_hz;
    alpha_min_ = clamp01(su_alpha_min);

    lissajous_amp_1_ = declare_parameter<double>("lissajous.amp_1", su_traj1_size_x);
    lissajous_amp_2_ = declare_parameter<double>("lissajous.amp_2", su_traj1_size_y);
    lissajous_ratio_ = declare_parameter<double>("lissajous.ratio", 2.0);

    circle_radius_1_ = declare_parameter<double>("circle.radius_1", su_traj1_size_x);
    circle_radius_2_ = declare_parameter<double>("circle.radius_2", su_traj1_size_y);

    square_parallel_length_ = declare_parameter<double>("square.parallel_length", su_traj1_size_x);
    square_vertical_length_ = declare_parameter<double>("square.vertical_length", su_traj1_size_y);

    constant_speed_1_ = declare_parameter<double>("constant.speed_1", su_traj1_speed);
    constant_speed_2_ = declare_parameter<double>("constant.speed_2", 0.0);

    single_sided_line_length_1_ = declare_parameter<double>(
      "single_sided_line.length_1", su_traj1_size_x);
    single_sided_line_length_2_ = declare_parameter<double>(
      "single_sided_line.length_2", su_traj1_size_y);

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
    ee_applied_wrench_consistency_topic_ = declare_parameter<std::string>(
      "ee_applied_wrench_consistency_topic", "/crazyflie/out/ee_applied_mob_2nd_tau");
    contact_signal_timeout_sec_ = declare_parameter<double>(
      "contact_signal_timeout_sec", 0.15);

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
    sub_ee_applied_wrench_consistency_ =
      create_subscription<geometry_msgs::msg::WrenchStamped>(
      ee_applied_wrench_consistency_topic_, 10,
      std::bind(
        &TrajectoryGeneration::eeAppliedWrenchConsistencyCb,
        this,
        std::placeholders::_1));

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
    pub_control_metrics_ = create_publisher<std_msgs::msg::Float64MultiArray>(
      "/su/debug/control_metrics", 10);
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

  static double clampPositive(double value)
  {
    return std::max(0.0, value);
  }

  static double inversePolynomialGate(double value, double threshold, int power_r = 2)
  {
    if (!(std::isfinite(threshold) && threshold > 0.0)) {
      return 1.0;
    }
    const double safe_threshold = threshold;
    const double ratio = clampPositive(value) / safe_threshold;
    if (power_r == 1) {
      return 1.0 / (1.0 + ratio);
    }
    return 1.0 / (1.0 + ratio * ratio);
  }

  bool isSignalFresh(const rclcpp::Time & stamp, const rclcpp::Time & now) const
  {
    if (stamp.nanoseconds() <= 0) {
      return false;
    }
    const double timeout = std::max(0.0, contact_signal_timeout_sec_);
    if (timeout <= 0.0) {
      return true;
    }
    return (now - stamp).seconds() <= timeout;
  }

  static double lpfAlphaFromCutoffHz(double dt, double cutoff_hz)
  {
    if (!(std::isfinite(dt) && dt > 0.0 && std::isfinite(cutoff_hz) && cutoff_hz > 0.0)) {
      return 1.0;
    }
    const double tau = 1.0 / (2.0 * M_PI * cutoff_hz);
    return std::clamp(dt / (tau + dt), 0.0, 1.0);
  }

  double lowPassScalar(
    double current,
    double dt,
    double cutoff_hz,
    double & state,
    bool & initialized) const
  {
    if (!(std::isfinite(current))) {
      initialized = false;
      state = std::numeric_limits<double>::quiet_NaN();
      return current;
    }
    if (!(std::isfinite(cutoff_hz) && cutoff_hz > 0.0)) {
      state = current;
      initialized = true;
      return current;
    }
    if (!initialized || !std::isfinite(state)) {
      state = current;
      initialized = true;
      return state;
    }
    const double alpha = lpfAlphaFromCutoffHz(dt, cutoff_hz);
    state += alpha * (current - state);
    return state;
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
    if (
      value == "single_sided_line" ||
      value == "line" ||
      value == "singleline")
    {
      return "single_sided_line";
    }
    return "lissajous";
  }

  static std::string shapeIdToPatternType(int shape_id)
  {
    switch (shape_id) {
      case 1:
        return "circle";
      case 2:
        return "square";
      case 4:
        return "single_sided_line";
      case 0:
      default:
        return "lissajous";
    }
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

  static std::string normalizeForceControlMode(std::string value)
  {
    value = normalizeLower(std::move(value));
    if (value == "legacy" || value == "previous" || value == "old") {
      return "legacy";
    }
    return "paper";
  }

  static double wrapPositive(double value, double period)
  {
    if (period <= 1.0e-9) {
      return value;
    }

    value = std::fmod(value, period);
    if (value < 0.0) {
      value += period;
    }
    return value;
  }

  void resetPatternProgress()
  {
    path_parameter_s_ = 0.0;
  }

  void evaluatePatternDerivative(
    double path_parameter,
    Eigen::Vector2d & derivative,
    double & period) const
  {
    if (pattern_type_ == "circle") {
      const double a = std::max(0.0, circle_radius_1_);
      const double b = std::max(0.0, circle_radius_2_);
      period = 2.0 * M_PI;
      derivative.x() = -a * std::sin(path_parameter);
      derivative.y() = b * std::cos(path_parameter);
      return;
    }

    if (pattern_type_ == "square") {
      const double parallel_length = std::max(0.0, square_parallel_length_);
      const double vertical_length = std::max(0.0, square_vertical_length_);
      period = 4.0;
      const double u = wrapPositive(path_parameter, period);
      if (u < 1.0) {
        derivative.x() = parallel_length;
        derivative.y() = 0.0;
      } else if (u < 2.0) {
        derivative.x() = 0.0;
        derivative.y() = vertical_length;
      } else if (u < 3.0) {
        derivative.x() = -parallel_length;
        derivative.y() = 0.0;
      } else {
        derivative.x() = 0.0;
        derivative.y() = -vertical_length;
      }
      return;
    }

    if (pattern_type_ == "constant") {
      period = 0.0;
      derivative.x() = constant_speed_1_;
      derivative.y() = constant_speed_2_;
      return;
    }

    if (pattern_type_ == "single_sided_line") {
      const double delta_1 = single_sided_line_length_1_;
      const double delta_2 = single_sided_line_length_2_;
      period = 2.0;
      const double u = wrapPositive(path_parameter, period);
      if (u < 1.0) {
        derivative.x() = delta_1;
        derivative.y() = delta_2;
      } else {
        derivative.x() = -delta_1;
        derivative.y() = -delta_2;
      }
      return;
    }

    const double a = std::max(0.0, lissajous_amp_1_);
    const double b = std::max(0.0, lissajous_amp_2_);
    const double ratio = std::max(1.0e-6, lissajous_ratio_);
    period = 2.0 * M_PI;
    derivative.x() = a * std::cos(path_parameter);
    derivative.y() = b * ratio * std::cos(ratio * path_parameter);
  }

  Eigen::Vector2d computePatternTangentialVelocity(double speed_cmd) const
  {
    const double speed_abs = std::abs(speed_cmd);
    if (speed_abs <= 1e-9) {
      return Eigen::Vector2d::Zero();
    }

    Eigen::Vector2d derivative = Eigen::Vector2d::Zero();
    double period = 0.0;
    evaluatePatternDerivative(path_parameter_s_, derivative, period);
    const double derivative_norm = derivative.norm();
    if (derivative_norm <= 1e-9) {
      return Eigen::Vector2d::Zero();
    }

    return speed_cmd * derivative / derivative_norm;
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
    position_reset_requested_ = (msg->data.size() >= 6 && msg->data[5] > 0.5);
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
    last_contact_frame_stamp_ = this->now();
  }

  void contactForceXCb(const std_msgs::msg::Float32::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lk(contact_mtx_);
    contact_force_x_ = static_cast<double>(msg->data);
    contact_force_x_received_ = true;
    last_contact_force_x_stamp_ = this->now();
  }

  void normalDebugMetricsCb(const std_msgs::msg::Float64MultiArray::SharedPtr msg)
  {
    if (msg->data.size() < 34) {
      return;
    }

    std::lock_guard<std::mutex> lk(contact_mtx_);
    estimated_normal_n_geo_ = Eigen::Vector3d(msg->data[28], msg->data[29], msg->data[30]);
    force_direction_n_f_ = Eigen::Vector3d(msg->data[31], msg->data[32], msg->data[33]);
    have_estimated_normal_n_geo_ =
      std::isfinite(estimated_normal_n_geo_.norm()) && estimated_normal_n_geo_.norm() > 1.0e-9;
    have_force_direction_n_f_ =
      std::isfinite(force_direction_n_f_.norm()) && force_direction_n_f_.norm() > 1.0e-9;
    last_normal_debug_metrics_stamp_ = this->now();
  }

  void eeAppliedWrenchConsistencyCb(const geometry_msgs::msg::WrenchStamped::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lk(contact_mtx_);
    applied_torque_tau_l_w_ = Eigen::Vector3d(
      msg->wrench.torque.x,
      msg->wrench.torque.y,
      msg->wrench.torque.z);
    have_applied_torque_tau_l_w_ = std::isfinite(applied_torque_tau_l_w_.norm());
    last_applied_torque_stamp_ = this->now();
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

  void publishControlMetrics(
    double alpha_frame,
    double omega_n,
    double normal_leakage,
    double alpha_u1,
    double alpha_u2,
    double preload_feedback,
    double c_tau,
    double pattern_progress,
    double pattern_speed_cmd)
  {
    std_msgs::msg::Float64MultiArray msg;
    msg.data = {
      alpha_frame,
      omega_n,
      normal_leakage,
      alpha_u1,
      alpha_u2,
      preload_feedback,
      c_tau,
      pattern_progress,
      pattern_speed_cmd
    };
    pub_control_metrics_->publish(msg);
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
    bool position_reset_requested_local = false;
    {
      std::lock_guard<std::mutex> lk(cmd_mtx_);
      sp_in_local = sp_in_;
      sp_in_yaw_local = sp_in_yaw_;
      trajectory_enabled_local = trajectory_cmd_enabled_;
      position_reset_requested_local = position_reset_requested_;
    }

    bool use_vel_mode_local = false;
    Eigen::Matrix3d contact_R_C = Eigen::Matrix3d::Identity();
    Eigen::Vector3d contact_n_w = Eigen::Vector3d::UnitX();
    Eigen::Vector3d contact_t1_w = Eigen::Vector3d::UnitY();
    Eigen::Vector3d contact_t2_w = Eigen::Vector3d::UnitZ();
    Eigen::Vector3d force_direction_n_f = Eigen::Vector3d::Zero();
    Eigen::Vector3d estimated_normal_n_geo = Eigen::Vector3d::Zero();
    Eigen::Vector3d applied_torque_tau_l_w = Eigen::Vector3d::Zero();
    double contact_force_x_local = 0.0;
    double cmd_force_desired_local = 0.0;
    bool contact_frame_ok = false;
    bool contact_force_ok = false;
    bool force_direction_ok = false;
    bool estimated_normal_ok = false;
    bool applied_torque_ok = false;
    {
      std::lock_guard<std::mutex> lk(contact_mtx_);
      use_vel_mode_local = use_vel_mode_;
      contact_R_C = contact_R_C_;
      contact_n_w = contact_n_w_;
      contact_t1_w = contact_t1_w_;
      contact_t2_w = contact_t2_w_;
      force_direction_n_f = force_direction_n_f_;
      estimated_normal_n_geo = estimated_normal_n_geo_;
      applied_torque_tau_l_w = applied_torque_tau_l_w_;
      contact_force_x_local = contact_force_x_;
      cmd_force_desired_local = cmd_force_desired_;
      contact_frame_ok = contact_frame_received_;
      contact_force_ok = contact_force_x_received_;
      force_direction_ok = have_force_direction_n_f_;
      estimated_normal_ok = have_estimated_normal_n_geo_;
      applied_torque_ok = have_applied_torque_tau_l_w_;
    }

    contact_frame_ok =
      contact_frame_ok && isSignalFresh(last_contact_frame_stamp_, now);
    contact_force_ok =
      contact_force_ok && isSignalFresh(last_contact_force_x_stamp_, now);
    force_direction_ok =
      force_direction_ok && isSignalFresh(last_normal_debug_metrics_stamp_, now);
    estimated_normal_ok =
      estimated_normal_ok && isSignalFresh(last_normal_debug_metrics_stamp_, now);
    applied_torque_ok =
      applied_torque_ok && isSignalFresh(last_applied_torque_stamp_, now);

    if (!integrated_ref_initialized_) {
      syncIntegratedReferenceToMeasured(ref);
      integrated_ref_initialized_ = true;
      prev_use_vel_mode_ = use_vel_mode_local;
    }

    if (use_vel_mode_local && !prev_use_vel_mode_) {
      syncIntegratedReferenceToMeasured(ref);
      pattern_ramp_state_sec_ = 0.0;
      resetPatternProgress();
      nu_n_state_ = 0.0;
      legacy_force_error_initialized_ = false;
      legacy_force_error_dot_filt_ = 0.0;
    }
    if (!use_vel_mode_local && prev_use_vel_mode_) {
      syncIntegratedReferenceToMeasured(ref);
      prev_position_sp_in_ = sp_in_local;
      prev_position_sp_in_yaw_ = sp_in_yaw_local;
      nu_n_state_ = 0.0;
      legacy_force_error_initialized_ = false;
      legacy_force_error_dot_filt_ = 0.0;
    }
    prev_use_vel_mode_ = use_vel_mode_local;

    Eigen::Vector3d p_drone_des_w = Eigen::Vector3d::Zero();
    Eigen::Vector3d p_ee_des_w = Eigen::Vector3d::Zero();

    if (!use_vel_mode_local) {
      publishNaNContactVelocityDebug(now);
      publishControlMetrics(
        1.0,
        std::numeric_limits<double>::quiet_NaN(),
        std::numeric_limits<double>::quiet_NaN(),
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
        0.0);

      if (ref.valid) {
        if (position_reset_requested_local) {
          syncIntegratedReferenceToMeasured(ref);
          prev_position_sp_in_.fill(0.0);
          prev_position_sp_in_yaw_ = 0.0;
        }

        const Eigen::Vector3d pos_delta_world(
          sp_in_local[0] - prev_position_sp_in_[0],
          sp_in_local[1] - prev_position_sp_in_[1],
          sp_in_local[2] - prev_position_sp_in_[2]);
        const double yaw_delta = sp_in_yaw_local - prev_position_sp_in_yaw_;

        active_ref_pos_w_ += pos_delta_world;
        active_ref_yaw_ = wrapPi(active_ref_yaw_ + yaw_delta);

        if (reference_object_ == "end_effector") {
          p_ee_des_w = active_ref_pos_w_;
          p_drone_des_w = p_ee_des_w - ref.q_wb.toRotationMatrix() * d_B_;
        } else {
          p_drone_des_w = active_ref_pos_w_;
          p_ee_des_w = p_drone_des_w + ref.q_wb.toRotationMatrix() * d_B_;
        }

        prev_position_sp_in_ = sp_in_local;
        prev_position_sp_in_yaw_ = sp_in_yaw_local;
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
      resetPatternProgress();
    }

    double directional_consistency_score = std::numeric_limits<double>::quiet_NaN();
    double alpha_frame = 1.0;
    double omega_n = std::numeric_limits<double>::quiet_NaN();
    double normal_leakage = std::numeric_limits<double>::quiet_NaN();
    if (ref.valid && estimated_normal_ok && applied_torque_ok)
    {
      const Eigen::Vector3d r_world = ref.q_wb.toRotationMatrix() * d_B_;
      const Eigen::Vector3d tau_n_w = r_world.cross(estimated_normal_n_geo);
      const double tau_l_norm_sq = applied_torque_tau_l_w.squaredNorm();
      const double tau_n_norm_sq = tau_n_w.squaredNorm();
      if (tau_l_norm_sq > 1.0e-12 && tau_n_norm_sq > 1.0e-12) {
        const double tau_alignment = applied_torque_tau_l_w.dot(tau_n_w);
        directional_consistency_score = clamp01(
          (tau_alignment * tau_alignment) /
          (tau_l_norm_sq * tau_n_norm_sq + clampPositive(epsilon_tau_n_)));
      }
    }

    if (trajectory_enabled_local && alpha_frame_mode_ == 1 &&
      std::isfinite(directional_consistency_score))
    {
        alpha_frame = std::exp(
          -(1.0 - directional_consistency_score) /
          std::max(clampPositive(c_bar_tau_), 1.0e-6));
    } else if (trajectory_enabled_local && alpha_frame_mode_ == 2 &&
      estimated_normal_ok && force_direction_ok)
    {
      Eigen::Vector3d n_geo_unit = estimated_normal_n_geo.normalized();

      omega_n = 0.0;
      if (prev_alpha_frame_mode2_normal_valid_) {
        Eigen::Vector3d aligned_n_geo_unit = n_geo_unit;
        if (prev_alpha_frame_mode2_n_geo_.dot(aligned_n_geo_unit) < 0.0) {
          aligned_n_geo_unit = -aligned_n_geo_unit;
        }
        omega_n = (aligned_n_geo_unit - prev_alpha_frame_mode2_n_geo_).norm() /
          std::max(dt, 1.0e-6);
        prev_alpha_frame_mode2_n_geo_ = aligned_n_geo_unit;
      } else {
        prev_alpha_frame_mode2_n_geo_ = n_geo_unit;
        prev_alpha_frame_mode2_normal_valid_ = true;
      }

      omega_n = lowPassScalar(
        omega_n, dt, omega_n_lpf_cutoff_hz_, omega_n_lpf_state_, omega_n_lpf_initialized_);
      double alpha_gate_product = 1.0;
      if (omega_bar_n_ > 0.0) {
        alpha_gate_product *= inversePolynomialGate(omega_n, omega_bar_n_, alpha_gate_power_r_);
      }
      if (normal_vel_bar_n_ > 0.0 && ref.valid) {
        const double vel_norm = ref.vel_w.norm();
        normal_leakage =
          std::abs(n_geo_unit.dot(ref.vel_w)) / (vel_norm + 1.0e-6);
        normal_leakage = lowPassScalar(
          normal_leakage,
          dt,
          normal_leakage_lpf_cutoff_hz_,
          normal_leakage_lpf_state_,
          normal_leakage_lpf_initialized_);
        alpha_gate_product *= inversePolynomialGate(
          normal_leakage, normal_vel_bar_n_, alpha_gate_power_r_);
      }
      alpha_frame = alpha_min_ + (1.0 - alpha_min_) * alpha_gate_product;
    } else if (alpha_frame_mode_ == 2) {
      prev_alpha_frame_mode2_normal_valid_ = false;
      omega_n_lpf_initialized_ = false;
      normal_leakage_lpf_initialized_ = false;
    }
    const double s_dot = pattern_gain_ * alpha_frame * pattern_speed_;
    {
      Eigen::Vector2d derivative = Eigen::Vector2d::Zero();
      double period = 0.0;
      evaluatePatternDerivative(path_parameter_s_, derivative, period);
      const double derivative_norm = derivative.norm();
      if (derivative_norm > 1e-9) {
        path_parameter_s_ += (s_dot / derivative_norm) * dt;
        path_parameter_s_ = wrapPositive(path_parameter_s_, period);
      }
    }

    Eigen::Vector3d v_tan_world = Eigen::Vector3d::Zero();
    Eigen::Vector2d u_tan_scaled = Eigen::Vector2d::Zero();
    if (contact_basis_ok) {
      u_tan_scaled = computePatternTangentialVelocity(s_dot);
      v_tan_world = contact_t1_w * u_tan_scaled.x() + contact_t2_w * u_tan_scaled.y();
    }

    double v_n_feedback = 0.0;
    double r_v_contact = 0.0;
    if (contact_basis_ok && ref.valid) {
      r_v_contact = contact_n_w.dot(ref.vel_w);
    }
    const bool preload_active =
      preload_enable_ &&
      std::abs(cmd_force_desired_local) > 1e-6;
    if (contact_force_ok && preload_active) {
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
        nu_n_state_ =
          paper_force_k_f_ * force_error +
          paper_force_k_l_ * r_v_contact;
        nu_n_state_ = std::clamp(nu_n_state_, -paper_force_v_n_max_, paper_force_v_n_max_);
        v_n_feedback = nu_n_state_;

        std_msgs::msg::Float64MultiArray dbg;
        dbg.data.resize(2);
        dbg.data[0] = force_error;
        dbg.data[1] = nu_n_state_;
        pub_force_lpf_->publish(dbg);
      }
    } else {
      nu_n_state_ = 0.0;
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
          manual_tangential_world - commanded_v_n * contact_n_w + v_tan_world;
      } else {
        v_contact_des_world += v_tan_world - commanded_v_n * contact_n_w;
      }

      const Eigen::Vector3d v_act_w = ref.valid ? ref.vel_w : Eigen::Vector3d::Zero();
      publishContactVelocityDebug(
        contact_R_C.transpose() * v_contact_des_world,
        contact_R_C.transpose() * v_act_w,
        now);
    } else {
      publishNaNContactVelocityDebug(now);
    }

    publishControlMetrics(
      alpha_frame,
      omega_n,
      normal_leakage,
      u_tan_scaled.x(),
      u_tan_scaled.y(),
      contact_force_ok ? contact_force_x_local : 0.0,
      directional_consistency_score,
      path_parameter_s_,
      s_dot);

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
    } else if (contact_frame_ok) {
      const double nxy = std::hypot(contact_n_w.x(), contact_n_w.y());
      if (nxy > normal_xy_min_) {
        const double psi_align = std::atan2(-contact_n_w.y(), -contact_n_w.x());
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
  rclcpp::Subscription<geometry_msgs::msg::WrenchStamped>::SharedPtr
    sub_ee_applied_wrench_consistency_;

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
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr pub_control_metrics_;
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
  std::string pattern_type_;

  double yaw_align_kp_{3.0};
  double normal_xy_min_{1e-3};
  double paper_force_k_f_{5.0};
  double paper_force_k_l_{2.0};
  double paper_force_v_n_max_{0.20};
  bool preload_enable_{true};
  double legacy_force_kp_{5.0};
  double legacy_force_kd_{0.005};
  double legacy_force_v_n_max_{0.20};
  double legacy_force_derivative_cutoff_rad_s_{3.0};

  double pattern_ramp_sec_{1.0};
  double pattern_speed_{0.25};
  int alpha_frame_mode_{0};
  double epsilon_tau_n_{1.0e-6};
  double c_bar_tau_{0.20};
  double omega_bar_n_{1.0};
  double normal_vel_bar_n_{0.0};
  int alpha_gate_power_r_{2};
  double omega_n_lpf_cutoff_hz_{0.0};
  double normal_leakage_lpf_cutoff_hz_{0.0};
  double alpha_min_{0.0};

  double lissajous_amp_1_{0.20};
  double lissajous_amp_2_{0.00};
  double lissajous_ratio_{2.0};

  double circle_radius_1_{0.20};
  double circle_radius_2_{0.20};

  double square_parallel_length_{0.40};
  double square_vertical_length_{0.40};

  double constant_speed_1_{0.10};
  double constant_speed_2_{0.0};
  double single_sided_line_length_1_{0.20};
  double single_sided_line_length_2_{0.0};

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
  std::string ee_applied_wrench_consistency_topic_;
  double contact_signal_timeout_sec_{0.15};

  std::mutex cmd_mtx_;
  std::array<double, 3> sp_in_{0.0, 0.0, 0.0};
  double sp_in_yaw_{0.0};
  bool trajectory_cmd_enabled_{false};
  bool sp_received_{false};

  std::mutex contact_mtx_;
  bool use_vel_mode_{false};
  double cmd_force_desired_{0.0};
  Eigen::Matrix3d contact_R_C_{(Eigen::Matrix3d() <<
    -1.0, 0.0, 0.0,
     0.0, -1.0, 0.0,
     0.0, 0.0, 1.0).finished()};
  Eigen::Vector3d contact_n_w_{-Eigen::Vector3d::UnitX()};
  Eigen::Vector3d contact_t1_w_{-Eigen::Vector3d::UnitY()};
  Eigen::Vector3d contact_t2_w_{Eigen::Vector3d::UnitZ()};
  double contact_force_x_{0.0};
  Eigen::Vector3d force_direction_n_f_{Eigen::Vector3d::Zero()};
  Eigen::Vector3d estimated_normal_n_geo_{Eigen::Vector3d::Zero()};
  Eigen::Vector3d applied_torque_tau_l_w_{Eigen::Vector3d::Zero()};
  Eigen::Vector3d prev_alpha_frame_mode2_n_geo_{Eigen::Vector3d::Zero()};
  double omega_n_lpf_state_{std::numeric_limits<double>::quiet_NaN()};
  double normal_leakage_lpf_state_{std::numeric_limits<double>::quiet_NaN()};
  bool contact_frame_received_{true};
  bool contact_force_x_received_{false};
  bool have_force_direction_n_f_{false};
  bool have_estimated_normal_n_geo_{false};
  bool have_applied_torque_tau_l_w_{false};
  rclcpp::Time last_contact_frame_stamp_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_contact_force_x_stamp_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_normal_debug_metrics_stamp_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_applied_torque_stamp_{0, 0, RCL_ROS_TIME};
  bool prev_alpha_frame_mode2_normal_valid_{false};
  bool omega_n_lpf_initialized_{false};
  bool normal_leakage_lpf_initialized_{false};

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
  bool prev_use_vel_mode_{false};
  Eigen::Vector3d active_ref_pos_w_{Eigen::Vector3d::Zero()};
  double active_ref_yaw_{0.0};
  double pattern_gain_{0.0};
  double pattern_ramp_state_sec_{0.0};
  double path_parameter_s_{0.0};
  double nu_n_state_{0.0};
  bool legacy_force_error_initialized_{false};
  double legacy_force_error_prev_{0.0};
  double legacy_force_error_dot_filt_{0.0};
  bool position_reset_requested_{false};
  std::array<double, 3> prev_position_sp_in_{0.0, 0.0, 0.0};
  double prev_position_sp_in_yaw_{0.0};

  rclcpp::Time last_update_time_{0, 0, RCL_ROS_TIME};
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TrajectoryGeneration>());
  rclcpp::shutdown();
  return 0;
}
