#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/vector3_stamped.hpp>
#include <geometry_msgs/msg/wrench_stamped.hpp>
#include <rcl_interfaces/msg/set_parameters_result.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <Eigen/Dense>

#include <algorithm>
#include <array>
#include <cmath>
#include <mutex>
#include <sstream>
#include <string>
#include <vector>

using namespace std::chrono_literals;

class WrenchObserver : public rclcpp::Node
{
public:
  using Vec3 = Eigen::Matrix<double, 3, 1>;

  struct ObserverState
  {
    Vec3 p_lin_hat_world{Vec3::Zero()};
    Vec3 p_ang_hat_body{Vec3::Zero()};
    Vec3 force_hat_world{Vec3::Zero()};
    Vec3 torque_hat_body{Vec3::Zero()};
    std::array<double, 3> world_force_hat_ext{{0.0, 0.0, 0.0}};
    std::array<double, 3> body_torque_hat_ext{{0.0, 0.0, 0.0}};
  };

  struct ConsistencyObserverState
  {
    Vec3 p_lin_hat_base_world{Vec3::Zero()};
    Vec3 p_lin_hat_con_world{Vec3::Zero()};
    Vec3 p_ang_hat_body{Vec3::Zero()};
    Vec3 force_hat_base_world{Vec3::Zero()};
    Vec3 force_hat_con_world{Vec3::Zero()};
    Vec3 force_hat_dot_base_world{Vec3::Zero()};
    Vec3 force_hat_dot_con_world{Vec3::Zero()};
    Vec3 force_update_residual_world{Vec3::Zero()};
    Vec3 force_update_integral_world{Vec3::Zero()};
    Vec3 force_update_consistency_world{Vec3::Zero()};
    Vec3 consistency_integral_world{Vec3::Zero()};
    Vec3 force_hat_world{Vec3::Zero()};
    Vec3 torque_hat_body{Vec3::Zero()};
    Vec3 torque_hat_dot_body{Vec3::Zero()};
    Vec3 tau_hat_ext_world{Vec3::Zero()};
    Vec3 r_cross_f_hat_ext_world{Vec3::Zero()};
    Vec3 e_tau_world{Vec3::Zero()};
    double rho_tau{0.0};
    std::array<double, 3> world_force_hat_ext{{0.0, 0.0, 0.0}};
    std::array<double, 3> body_torque_hat_ext{{0.0, 0.0, 0.0}};
  };

  WrenchObserver()
  : Node("wrench_observer")
  {
    const double configured_mass = declare_parameter<double>("mass", 0.04338);
    const auto configured_com_bias = declare_parameter<std::vector<double>>(
      "com_bias", std::vector<double>{0.0, 0.0, 0.0});
    const auto configured_inertia_diag = declare_parameter<std::vector<double>>(
      "inertia_diag", std::vector<double>{2.3951e-5, 2.3951e-5, 3.2347e-5});
    const auto configured_ee_offset = declare_parameter<std::vector<double>>(
      "end_effector_offset", std::vector<double>{0.09, 0.0, 0.085});

    input_topic_ = declare_parameter<std::string>("input_topic", "/crazyflie/in/input");
    cmd_force_topic_ = declare_parameter<std::string>("cmd_force_topic", "su/cmd_force");
    pose_topic_ = declare_parameter<std::string>("pose_topic", "/crazyflie/out/pose");
    vel_topic_ = declare_parameter<std::string>("vel_topic", "/crazyflie/out/vel");
    angvel_topic_ = declare_parameter<std::string>("angvel_topic", "/crazyflie/out/ang_vel");
    drone_wrench_topic_2nd_order_ = declare_parameter<std::string>(
      "drone_wrench_topic_2nd_order", "/crazyflie/out/mob_2nd");
    drone_wrench_topic_consistency_ = declare_parameter<std::string>(
      "drone_wrench_topic_consistency", "/crazyflie/out/mob_2nd_tau");
    drone_wrench_topic_consistency_integral_ = declare_parameter<std::string>(
      "drone_wrench_topic_consistency_integral", "/crazyflie/out/mob_2nd_tau_i");
    drone_wrench_topic_consistency_terms_ = declare_parameter<std::string>(
      "drone_wrench_topic_consistency_terms", "/crazyflie/out/mob_2nd_tau_terms");
    drone_wrench_topic_consistency_base_ = declare_parameter<std::string>(
      "drone_wrench_topic_consistency_base", "/crazyflie/out/mob_2nd_tau_base");
    drone_wrench_topic_consistency_match_ = declare_parameter<std::string>(
      "drone_wrench_topic_consistency_match", "/crazyflie/out/mob_2nd_tau_consistency");
    drone_wrench_topic_consistency_residual_ = declare_parameter<std::string>(
      "drone_wrench_topic_consistency_residual", "/crazyflie/out/mob_2nd_tau_residual");
    drone_wrench_topic_kalman_ = declare_parameter<std::string>(
      "drone_wrench_topic_kalman", "/crazyflie/out/mob_kalman");
    drone_wrench_topic_adaptive_ = declare_parameter<std::string>(
      "drone_wrench_topic_adaptive", "/crazyflie/out/mob_adaptive");
    ee_applied_wrench_topic_2nd_order_ = declare_parameter<std::string>(
      "ee_applied_wrench_topic_2nd_order", "/crazyflie/out/ee_applied_mob_2nd");
    ee_applied_wrench_topic_consistency_ = declare_parameter<std::string>(
      "ee_applied_wrench_topic_consistency", "/crazyflie/out/ee_applied_mob_2nd_tau");
    ee_applied_wrench_topic_consistency_integral_ = declare_parameter<std::string>(
      "ee_applied_wrench_topic_consistency_integral", "/crazyflie/out/ee_applied_mob_2nd_tau_i");
    ee_applied_wrench_topic_consistency_base_ = declare_parameter<std::string>(
      "ee_applied_wrench_topic_consistency_base", "/crazyflie/out/ee_applied_mob_2nd_tau_base");
    ee_applied_wrench_topic_kalman_ = declare_parameter<std::string>(
      "ee_applied_wrench_topic_kalman", "/crazyflie/out/ee_applied_mob_kalman");
    ee_applied_wrench_topic_adaptive_ = declare_parameter<std::string>(
      "ee_applied_wrench_topic_adaptive", "/crazyflie/out/ee_applied_mob_adaptive");

    observer_hz_ = declare_parameter<double>("observer_hz", 250.0);
    dt_fixed_enable_ = declare_parameter<bool>("mob.dt_fixed_enable", true);
    dt_fixed_value_ = declare_parameter<double>("mob.dt_fixed_value", 0.004);
    dt_alpha_ = declare_parameter<double>("mob.dt_alpha", 0.2);
    vel_lpf_enable_ = declare_parameter<bool>("mob.vel_lpf_enable", false);
    vel_lpf_cutoff_hz_ = declare_parameter<double>("mob.vel_lpf_cutoff_hz", 0.5);

    mass_ = configured_mass;
    gravity_ = declare_parameter<double>("g", 9.81);
    jxx_ = declare_parameter<double>(
      "mob.Jxx", configured_inertia_diag.size() > 0 ? configured_inertia_diag[0] : 2.3951e-5);
    jyy_ = declare_parameter<double>(
      "mob.Jyy", configured_inertia_diag.size() > 1 ? configured_inertia_diag[1] : 2.3951e-5);
    jzz_ = declare_parameter<double>(
      "mob.Jzz", configured_inertia_diag.size() > 2 ? configured_inertia_diag[2] : 3.2347e-5);

    // 2nd-order MOB tuning
    kf_2nd_order_ = declare_parameter<double>("mob.Kf_2nd_order", 5.0);
    ktau_2nd_order_ = declare_parameter<double>("mob.Ktau_2nd_order", 20.0);
    mob_alpha_2nd_order_ = declare_parameter<double>("mob.mob_alpha_2nd_order", 0.2);
    kp_ = declare_parameter<double>("mob.Kp", 2.0);
    kptau_ = declare_parameter<double>("mob.KpTau", 10.0);
    ke_ = declare_parameter<double>("mob.Ke", 10.0);
    kei_ = declare_parameter<double>("mob.KeI", 0.0);
    ke_ep_ = declare_parameter<double>("mob.Ke_ep", ke_);
    ke_epi_ = declare_parameter<double>("mob.Ke_epi", ke_);
    kei_epi_ = declare_parameter<double>("mob.KeI_epi", kei_);
    k_ep_sweep_gains_ = declare_parameter<std::vector<double>>(
      "mob.k_ep_sweep_gains", std::vector<double>{100.0, 300.0, 500.0, 700.0, 900.0});
    epsilon_tau_ = declare_parameter<double>("mob.epsilon_tau", 1.0e-6);
    kalman_sigma_force_perp_ = declare_parameter<double>("mob.kalman.sigma_force_perp", 0.10);
    kalman_sigma_force_thrust_ = declare_parameter<double>("mob.kalman.sigma_force_thrust", 0.50);
    kalman_sigma_torque_ = declare_parameter<double>("mob.kalman.sigma_torque", 0.01);
    adaptive_gamma_ = declare_parameter<double>("mob.adaptive.gamma", 0.2);
    adaptive_lambda_b_ = declare_parameter<double>("mob.adaptive.lambda_b", 0.01);
    adaptive_bias_limit_ = declare_parameter<double>("mob.adaptive.bias_limit", 2.0);
    adaptive_sigma_force_perp_ = declare_parameter<double>("mob.adaptive.sigma_force_perp", 0.10);
    adaptive_sigma_force_thrust_ = declare_parameter<double>("mob.adaptive.sigma_force_thrust", 0.50);
    adaptive_sigma_torque_ = declare_parameter<double>("mob.adaptive.sigma_torque", 0.01);

    arm_xy_ = declare_parameter<double>("arm_xy", 0.035355);
    k_tau_motor_ = declare_parameter<double>("k_tau", 0.00569278844371417);
    thrust_min_ = declare_parameter<double>("thrust_min", 0.0);
    thrust_max_ = declare_parameter<double>("thrust_max", 0.20);

    auto ee_offset_param = configured_ee_offset;
    if (ee_offset_param.size() != 3) {
      RCLCPP_WARN(get_logger(), "end_effector_offset must have size 3. Falling back to [0.09, 0.0, 0.085].");
      ee_offset_param = {0.09, 0.0, 0.085};
    }
    ee_offset_body_ = Eigen::Vector3d(ee_offset_param[0], ee_offset_param[1], ee_offset_param[2]);
    if (configured_com_bias.size() != 3) {
      RCLCPP_WARN(get_logger(), "com_bias must have size 3. Falling back to [0.0, 0.0, 0.0].");
      com_offset_body_ = Eigen::Vector3d::Zero();
    } else {
      com_offset_body_ = Eigen::Vector3d(
        configured_com_bias[0], configured_com_bias[1], configured_com_bias[2]);
    }

    auto motor_dir_param = declare_parameter<std::vector<double>>(
      "motor_dir", std::vector<double>{1.0, -1.0, 1.0, -1.0});
    if (motor_dir_param.size() != 4) {
      RCLCPP_WARN(get_logger(), "motor_dir must have size 4. Falling back to [1, -1, 1, -1].");
      motor_dir_param = {1.0, -1.0, 1.0, -1.0};
    }
    for (size_t i = 0; i < 4; ++i) {
      motor_dir_[i] = motor_dir_param[i];
    }

    buildAllocationMatrices();

    auto qos = rclcpp::SensorDataQoS();
    sub_input_ = create_subscription<std_msgs::msg::Float32MultiArray>(
      input_topic_, qos, std::bind(&WrenchObserver::inputCb, this, std::placeholders::_1));
    sub_cmd_force_ = create_subscription<std_msgs::msg::Float32>(
      cmd_force_topic_, qos, std::bind(&WrenchObserver::cmdForceCb, this, std::placeholders::_1));
    sub_pose_ = create_subscription<geometry_msgs::msg::PoseStamped>(
      pose_topic_, qos, std::bind(&WrenchObserver::poseCb, this, std::placeholders::_1));
    sub_vel_ = create_subscription<geometry_msgs::msg::Vector3Stamped>(
      vel_topic_, qos, std::bind(&WrenchObserver::velCb, this, std::placeholders::_1));
    sub_angvel_ = create_subscription<geometry_msgs::msg::Vector3Stamped>(
      angvel_topic_, qos, std::bind(&WrenchObserver::angVelCb, this, std::placeholders::_1));

    pub_drone_wrench_2nd_order_ = create_publisher<geometry_msgs::msg::WrenchStamped>(
      drone_wrench_topic_2nd_order_, 10);
    pub_drone_wrench_consistency_ = create_publisher<geometry_msgs::msg::WrenchStamped>(
      drone_wrench_topic_consistency_, 10);
    pub_drone_wrench_consistency_integral_ = create_publisher<geometry_msgs::msg::WrenchStamped>(
      drone_wrench_topic_consistency_integral_, 10);
    pub_drone_wrench_consistency_terms_ = create_publisher<geometry_msgs::msg::WrenchStamped>(
      drone_wrench_topic_consistency_terms_, 10);
    pub_drone_wrench_consistency_base_ = create_publisher<geometry_msgs::msg::WrenchStamped>(
      drone_wrench_topic_consistency_base_, 10);
    pub_drone_wrench_consistency_match_ = create_publisher<geometry_msgs::msg::WrenchStamped>(
      drone_wrench_topic_consistency_match_, 10);
    pub_drone_wrench_consistency_residual_ = create_publisher<geometry_msgs::msg::WrenchStamped>(
      drone_wrench_topic_consistency_residual_, 10);
    pub_drone_wrench_kalman_ = create_publisher<geometry_msgs::msg::WrenchStamped>(
      drone_wrench_topic_kalman_, 10);
    pub_drone_wrench_adaptive_ = create_publisher<geometry_msgs::msg::WrenchStamped>(
      drone_wrench_topic_adaptive_, 10);
    pub_ee_applied_wrench_2nd_order_ = create_publisher<geometry_msgs::msg::WrenchStamped>(
      ee_applied_wrench_topic_2nd_order_, 10);
    pub_ee_applied_wrench_consistency_ = create_publisher<geometry_msgs::msg::WrenchStamped>(
      ee_applied_wrench_topic_consistency_, 10);
    pub_ee_applied_wrench_consistency_integral_ = create_publisher<geometry_msgs::msg::WrenchStamped>(
      ee_applied_wrench_topic_consistency_integral_, 10);
    pub_ee_applied_wrench_consistency_base_ = create_publisher<geometry_msgs::msg::WrenchStamped>(
      ee_applied_wrench_topic_consistency_base_, 10);
    pub_ee_applied_wrench_kalman_ = create_publisher<geometry_msgs::msg::WrenchStamped>(
      ee_applied_wrench_topic_kalman_, 10);
    pub_ee_applied_wrench_adaptive_ = create_publisher<geometry_msgs::msg::WrenchStamped>(
      ee_applied_wrench_topic_adaptive_, 10);
    for (const double gain : k_ep_sweep_gains_) {
      if (!std::isfinite(gain)) {
        RCLCPP_WARN(get_logger(), "Skipping non-finite mob.k_ep_sweep_gains entry.");
        continue;
      }
      const std::string tag = gainTopicTag(gain);
      sweep_ke_gains_.push_back(gain);
      observer_consistency_sweep_.emplace_back();
      pub_drone_wrench_consistency_sweep_.push_back(
        create_publisher<geometry_msgs::msg::WrenchStamped>(
          "/crazyflie/out/mob_2nd_tau_ke_" + tag, 10));
      pub_ee_applied_wrench_consistency_sweep_.push_back(
        create_publisher<geometry_msgs::msg::WrenchStamped>(
          "/crazyflie/out/ee_applied_mob_2nd_tau_ke_" + tag, 10));
    }

    const double safe_hz = std::max(1.0, observer_hz_);
    timer_est_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::duration<double>(1.0 / safe_hz)),
      std::bind(&WrenchObserver::loopEst, this));

    param_callback_handle_ = add_on_set_parameters_callback(
      std::bind(&WrenchObserver::onParametersSet, this, std::placeholders::_1));

    last_time_ = now();

    RCLCPP_INFO(get_logger(), "wrench_observer started");
    RCLCPP_INFO(get_logger(), "sub: %s | %s | %s | %s | %s",
      input_topic_.c_str(), cmd_force_topic_.c_str(), pose_topic_.c_str(), vel_topic_.c_str(),
      angvel_topic_.c_str());
    RCLCPP_INFO(get_logger(), "pub: %s | %s",
      drone_wrench_topic_2nd_order_.c_str(),
      drone_wrench_topic_consistency_.c_str());
  }

private:
  static double lpf1(double y_prev, double x, double alpha)
  {
    return y_prev + alpha * (x - y_prev);
  }

  static double lpfAlphaFromCutoff(double dt, double cutoff_hz)
  {
    if (!(std::isfinite(dt) && dt > 0.0 && std::isfinite(cutoff_hz) && cutoff_hz > 0.0)) {
      return 1.0;
    }
    const double tau = 1.0 / (2.0 * M_PI * cutoff_hz);
    return std::clamp(dt / (tau + dt), 0.0, 1.0);
  }

  geometry_msgs::msg::WrenchStamped makeWrenchMsg(
    const rclcpp::Time & stamp,
    const Vec3 & force_world,
    const Vec3 & torque_world) const
  {
    geometry_msgs::msg::WrenchStamped msg;
    msg.header.stamp = stamp;
    msg.header.frame_id = "world";
    msg.wrench.force.x = force_world.x();
    msg.wrench.force.y = force_world.y();
    msg.wrench.force.z = force_world.z();
    msg.wrench.torque.x = torque_world.x();
    msg.wrench.torque.y = torque_world.y();
    msg.wrench.torque.z = torque_world.z();
    return msg;
  }

  Vec3 gainMul(double gain, const Vec3 & vec) const
  {
    return gain * vec;
  }

  static Eigen::Matrix3d skew(const Vec3 & vec)
  {
    Eigen::Matrix3d mat;
    mat << 0.0, -vec.z(), vec.y(),
      vec.z(), 0.0, -vec.x(),
      -vec.y(), vec.x(), 0.0;
    return mat;
  }

  static double sanitizePositive(double value, double fallback)
  {
    if (!std::isfinite(value) || value <= 1.0e-9) {
      return fallback;
    }
    return value;
  }

  static Vec3 projectTorqueToLeverPlane(const Vec3 & torque_world, const Vec3 & lever_arm_world)
  {
    const double lever_norm_sq = lever_arm_world.squaredNorm();
    if (!(std::isfinite(lever_norm_sq) && lever_norm_sq > 1.0e-12)) {
      return torque_world;
    }

    const Vec3 lever_hat = lever_arm_world / std::sqrt(lever_norm_sq);
    return torque_world - lever_hat * lever_hat.dot(torque_world);
  }

  static Eigen::Matrix3d buildForceUncertainty(
    const Vec3 & thrust_axis_world,
    double sigma_force_perp,
    double sigma_force_thrust)
  {
    const Vec3 unit_thrust =
      thrust_axis_world.norm() > 1.0e-9 ? thrust_axis_world.normalized() : Vec3(0.0, 0.0, 1.0);
    const double sigma_perp_sq = std::pow(sanitizePositive(sigma_force_perp, 0.10), 2);
    const double sigma_thrust_sq = std::pow(sanitizePositive(sigma_force_thrust, 0.50), 2);
    const Eigen::Matrix3d thrust_projector = unit_thrust * unit_thrust.transpose();
    return sigma_perp_sq * (Eigen::Matrix3d::Identity() - thrust_projector) +
           sigma_thrust_sq * thrust_projector;
  }

  static Eigen::Matrix3d buildTorqueUncertainty(double sigma_torque)
  {
    const double sigma_tau_sq = std::pow(sanitizePositive(sigma_torque, 0.01), 2);
    return sigma_tau_sq * Eigen::Matrix3d::Identity();
  }

  Vec3 runKalmanLikeFusion(
    const Vec3 & force_world_raw,
    const Vec3 & torque_world_raw,
    const Vec3 & ee_offset_world,
    const Vec3 & thrust_axis_world,
    double sigma_force_perp,
    double sigma_force_thrust,
    double sigma_torque) const
  {
    if (ee_offset_world.squaredNorm() <= 1.0e-12) {
      return force_world_raw;
    }

    const Eigen::Matrix3d a_r = skew(ee_offset_world);
    const Vec3 torque_world_proj = projectTorqueToLeverPlane(torque_world_raw, ee_offset_world);
    const Eigen::Matrix3d r_force =
      buildForceUncertainty(thrust_axis_world, sigma_force_perp, sigma_force_thrust);
    const Eigen::Matrix3d r_tau = buildTorqueUncertainty(sigma_torque);
    const Eigen::Matrix3d innovation_cov = a_r * r_force * a_r.transpose() + r_tau;
    Eigen::LDLT<Eigen::Matrix3d> ldlt(innovation_cov);
    if (ldlt.info() != Eigen::Success) {
      return force_world_raw;
    }

    const Eigen::Matrix3d consistency_gain =
      r_force * a_r.transpose() * ldlt.solve(Eigen::Matrix3d::Identity());
    const Vec3 innovation = torque_world_proj - a_r * force_world_raw;
    return force_world_raw + consistency_gain * innovation;
  }

  Vec3 runAdaptiveBiasFusion(
    const Vec3 & force_world_raw,
    const Vec3 & torque_world_raw,
    const Vec3 & ee_offset_world,
    const Vec3 & thrust_axis_world)
  {
    if (ee_offset_world.squaredNorm() <= 1.0e-12 || thrust_axis_world.squaredNorm() <= 1.0e-12) {
      return force_world_raw;
    }

    const Vec3 unit_thrust = thrust_axis_world.normalized();
    const Eigen::Matrix3d a_r = skew(ee_offset_world);
    const Vec3 torque_world_proj = projectTorqueToLeverPlane(torque_world_raw, ee_offset_world);
    const Vec3 h_k = a_r * unit_thrust;
    const Vec3 y_k = a_r * force_world_raw - torque_world_proj;
    const Eigen::Matrix3d residual_cov =
      a_r * buildForceUncertainty(
        unit_thrust, adaptive_sigma_force_perp_, adaptive_sigma_force_thrust_) * a_r.transpose() +
      buildTorqueUncertainty(adaptive_sigma_torque_);
    Eigen::LDLT<Eigen::Matrix3d> ldlt(residual_cov);
    if (ldlt.info() == Eigen::Success) {
      const Vec3 prediction_error = y_k - h_k * adaptive_bias_hat_;
      const Vec3 weighted_h = ldlt.solve(h_k);
      const Vec3 weighted_error = ldlt.solve(prediction_error);
      const double information_scalar = h_k.dot(weighted_h);
      const double denominator =
        sanitizePositive(adaptive_lambda_b_, 0.01) + std::max(0.0, information_scalar);
      const double gamma = std::clamp(adaptive_gamma_, 0.0, 2.0);
      adaptive_bias_hat_ += gamma * h_k.dot(weighted_error) / denominator;
    }

    const double bias_limit = std::abs(adaptive_bias_limit_);
    if (std::isfinite(bias_limit) && bias_limit > 0.0) {
      adaptive_bias_hat_ = std::clamp(adaptive_bias_hat_, -bias_limit, bias_limit);
    }

    return force_world_raw - adaptive_bias_hat_ * unit_thrust;
  }

  static std::string gainTopicTag(double gain)
  {
    const double rounded = std::round(gain);
    if (std::abs(gain - rounded) < 1.0e-9) {
      return std::to_string(static_cast<int>(rounded));
    }

    std::ostringstream oss;
    oss << gain;
    std::string tag = oss.str();
    std::replace(tag.begin(), tag.end(), '.', 'p');
    return tag;
  }

  void runObserverVariant(
    ObserverState & state,
    const Vec3 & p_lin_world,
    const Vec3 & p_ang_body,
    const Vec3 & u_lin_world,
    const Vec3 & u_tau_body,
    const Vec3 & grav_world,
    const Vec3 & cori_body,
    double dt,
    bool use_correction,
    bool use_force_integration,
    double kf,
    double ktau,
    double mob_alpha)
  {
    const Vec3 p_lin_residual_world = p_lin_world - state.p_lin_hat_world;
    const Vec3 p_ang_residual_body = p_ang_body - state.p_ang_hat_body;

    if (use_force_integration) {
      const Vec3 force_hat_dot_world = gainMul(kf, p_lin_residual_world);
      const Vec3 torque_hat_dot_body = gainMul(ktau, p_ang_residual_body);
      state.force_hat_world += dt * force_hat_dot_world;
      state.torque_hat_body += dt * torque_hat_dot_body;
    } else {
      state.force_hat_world = gainMul(kf, p_lin_residual_world);
      state.torque_hat_body = gainMul(ktau, p_ang_residual_body);
    }

    Vec3 p_lin_hat_dot_world = u_lin_world - grav_world + state.force_hat_world;
    if (use_correction) {
      p_lin_hat_dot_world += gainMul(kp_, p_lin_residual_world);
    }
    state.p_lin_hat_world += dt * p_lin_hat_dot_world;

    Vec3 p_ang_hat_dot_body = u_tau_body - cori_body + state.torque_hat_body;
    if (use_correction) {
      p_ang_hat_dot_body += gainMul(kptau_, p_ang_residual_body);
    }
    state.p_ang_hat_body += dt * p_ang_hat_dot_body;

    for (int i = 0; i < 3; ++i) {
      state.world_force_hat_ext[i] =
        lpf1(state.world_force_hat_ext[i], state.force_hat_world[i], mob_alpha);
      state.body_torque_hat_ext[i] =
        lpf1(state.body_torque_hat_ext[i], state.torque_hat_body[i], mob_alpha);
    }
  }

  void runConsistencyResidualObserver(
    ConsistencyObserverState & state,
    const Vec3 & p_lin_world,
    const Vec3 & p_ang_body,
    const Vec3 & u_lin_world,
    const Vec3 & u_tau_body,
    const Vec3 & grav_world,
    const Vec3 & cori_body,
    const Vec3 & ee_offset_world,
    const Eigen::Matrix3d & r_bw,
    double dt,
    double kf,
    double ktau,
    double mob_alpha,
    double ke_gain,
    double kei_gain)
  {
    const Vec3 p_ang_residual_body = p_ang_body - state.p_ang_hat_body;
    const Vec3 torque_hat_dot_body = gainMul(ktau, p_ang_residual_body);
    state.torque_hat_dot_body = torque_hat_dot_body;
    state.torque_hat_body += dt * torque_hat_dot_body;
    const Vec3 tau_hat_ext_world = r_bw * state.torque_hat_body;

    const Vec3 p_lin_residual_base_world = p_lin_world - state.p_lin_hat_base_world;
    const Vec3 force_hat_dot_base_world = gainMul(kf, p_lin_residual_base_world);
    state.force_hat_dot_base_world = force_hat_dot_base_world;
    state.force_hat_base_world += dt * force_hat_dot_base_world;

    const Vec3 p_lin_residual_con_world = p_lin_world - state.p_lin_hat_con_world;
    const Vec3 r_cross_f_before_world = ee_offset_world.cross(state.force_hat_con_world);
    const Vec3 e_tau_world = tau_hat_ext_world - r_cross_f_before_world;
    const Vec3 consistency_residual_world =
      skew(ee_offset_world).transpose() * e_tau_world;
    const double rho_tau =
      e_tau_world.norm() /
      (ee_offset_world.norm() * state.force_hat_con_world.norm() + epsilon_tau_);
    const Vec3 force_update_residual_world = ke_gain * consistency_residual_world;
    if (std::isfinite(kei_gain) && std::abs(kei_gain) > 1.0e-12) {
      state.consistency_integral_world +=
        dt * consistency_residual_world;
      state.force_update_integral_world = kei_gain * state.consistency_integral_world;
    } else {
      state.consistency_integral_world.setZero();
      state.force_update_integral_world.setZero();
    }
    const Vec3 force_update_consistency_world =
      force_update_residual_world + state.force_update_integral_world;
    // k_ep:  f_hat_dot = legacy_term + ke  * consistency_residual
    // k_epi: f_hat_dot = legacy_term + ke  * consistency_residual
    //                              + kei * integral(consistency_residual)
    const Vec3 force_hat_dot_con_world =
      gainMul(kf, p_lin_residual_con_world) + force_update_consistency_world;
    state.force_hat_dot_con_world = force_hat_dot_con_world;
    state.force_hat_con_world += dt * force_hat_dot_con_world;

    state.force_hat_world = state.force_hat_con_world;
    state.force_update_residual_world = force_update_residual_world;
    state.force_update_consistency_world = force_update_consistency_world;
    state.tau_hat_ext_world = tau_hat_ext_world;
    state.r_cross_f_hat_ext_world = r_cross_f_before_world;
    state.e_tau_world = e_tau_world;
    state.rho_tau = rho_tau;

    Vec3 p_ang_hat_dot_body = u_tau_body - cori_body + state.torque_hat_body;
    p_ang_hat_dot_body += gainMul(kptau_, p_ang_residual_body);
    state.p_ang_hat_body += dt * p_ang_hat_dot_body;

    Vec3 p_lin_hat_dot_base_world = u_lin_world - grav_world + state.force_hat_base_world;
    p_lin_hat_dot_base_world += gainMul(kp_, p_lin_residual_base_world);
    state.p_lin_hat_base_world += dt * p_lin_hat_dot_base_world;

    Vec3 p_lin_hat_dot_con_world = u_lin_world - grav_world + state.force_hat_con_world;
    p_lin_hat_dot_con_world += gainMul(kp_, p_lin_residual_con_world);
    state.p_lin_hat_con_world += dt * p_lin_hat_dot_con_world;

    for (int i = 0; i < 3; ++i) {
      state.world_force_hat_ext[i] =
        lpf1(state.world_force_hat_ext[i], state.force_hat_con_world[i], mob_alpha);
      state.body_torque_hat_ext[i] =
        lpf1(state.body_torque_hat_ext[i], state.torque_hat_body[i], mob_alpha);
    }
  }

  void buildAllocationMatrices()
  {
    const double a = arm_xy_;
    const std::array<double, 4> x{{+a, -a, -a, +a}};
    const std::array<double, 4> y{{-a, -a, +a, +a}};

    for (int c = 0; c < 4; ++c) {
      b_(0, c) = y[c];
      b_(1, c) = -x[c];
      b_(2, c) = motor_dir_[c] * k_tau_motor_;
      b_(3, c) = 1.0;
    }
    b_inv_ = b_.inverse();
  }

  void inputCb(const std_msgs::msg::Float32MultiArray::SharedPtr msg)
  {
    if (msg->data.size() < 4) {
      return;
    }

    std::lock_guard<std::mutex> lk(mtx_);
    input_tau_fz_[0] = msg->data[0];
    input_tau_fz_[1] = msg->data[1];
    input_tau_fz_[2] = msg->data[2];
    input_tau_fz_[3] = msg->data[3];
    have_input_ = true;
  }

  void poseCb(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lk(mtx_);
    pose_ = *msg;
    have_pose_ = true;
  }

  void cmdForceCb(const std_msgs::msg::Float32::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lk(mtx_);
    cmd_force_desired_ = static_cast<double>(msg->data);
  }

  void velCb(const geometry_msgs::msg::Vector3Stamped::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lk(mtx_);
    vel_ = *msg;
    have_vel_ = true;
  }

  void angVelCb(const geometry_msgs::msg::Vector3Stamped::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lk(mtx_);
    angvel_ = *msg;
    have_angvel_ = true;
  }

  void loopEst()
  {
    std::array<double, 4> input_tau_fz{};
    geometry_msgs::msg::PoseStamped pose;
    geometry_msgs::msg::Vector3Stamped vel;
    geometry_msgs::msg::Vector3Stamped angvel;

    {
      std::lock_guard<std::mutex> lk(mtx_);
      if (!(have_input_ && have_pose_ && have_vel_ && have_angvel_)) {
        return;
      }
      input_tau_fz = input_tau_fz_;
      pose = pose_;
      vel = vel_;
      angvel = angvel_;
    }

    const rclcpp::Time t_now = now();
    const double dt_meas = (t_now - last_time_).seconds();
    last_time_ = t_now;

    if (std::isfinite(dt_meas) && dt_meas > 0.0 && dt_meas < 0.05) {
      dt_filt_ = lpf1(dt_filt_, dt_meas, dt_alpha_);
      dt_filt_ = std::clamp(dt_filt_, 1e-4, 5e-2);
    }

    const double dt = dt_fixed_enable_ ? dt_fixed_value_ : dt_filt_;
    if (!(std::isfinite(dt) && dt > 0.0)) {
      return;
    }

    const double vel_lpf_alpha = lpfAlphaFromCutoff(dt, vel_lpf_cutoff_hz_);

    const Eigen::Vector4d wrench_cmd(
      input_tau_fz[0], input_tau_fz[1], input_tau_fz[2], input_tau_fz[3]);
    Eigen::Vector4d motor_thrust = b_inv_ * wrench_cmd;
    for (int i = 0; i < 4; ++i) {
      motor_thrust[i] = std::clamp(motor_thrust[i], thrust_min_, thrust_max_);
    }

    const double f1 = motor_thrust[0];
    const double f2 = motor_thrust[1];
    const double f3 = motor_thrust[2];
    const double f4 = motor_thrust[3];

    const double tx = arm_xy_ * ((f3 + f4) - (f1 + f2));
    const double ty = arm_xy_ * ((f2 + f3) - (f1 + f4));
    const double tz = k_tau_motor_ * (
      motor_dir_[0] * f1 + motor_dir_[1] * f2 + motor_dir_[2] * f3 + motor_dir_[3] * f4);
    const double fz = f1 + f2 + f3 + f4;

    const auto &q = pose.pose.orientation;
    double qx = q.x;
    double qy = q.y;
    double qz = q.z;
    double qw = q.w;
    const double qnorm = std::sqrt(qx * qx + qy * qy + qz * qz + qw * qw);
    if (qnorm < 1e-9) {
      return;
    }
    qx /= qnorm;
    qy /= qnorm;
    qz /= qnorm;
    qw /= qnorm;

    const Eigen::Quaterniond q_wb(qw, qx, qy, qz);
    const Eigen::Matrix3d r_bw = q_wb.toRotationMatrix();

    double vx = vel.vector.x;
    double vy = vel.vector.y;
    double vz = vel.vector.z;
    double wx = angvel.vector.x;
    double wy = angvel.vector.y;
    double wz = angvel.vector.z;

    if (!std::isfinite(vx)) vx = 0.0;
    if (!std::isfinite(vy)) vy = 0.0;
    if (!std::isfinite(vz)) vz = 0.0;
    if (!std::isfinite(wx)) wx = 0.0;
    if (!std::isfinite(wy)) wy = 0.0;
    if (!std::isfinite(wz)) wz = 0.0;

    if (vel_lpf_enable_) {
      if (!vel_lpf_initialized_) {
        vel_filt_ = Eigen::Vector3d(vx, vy, vz);
        angvel_filt_ = Eigen::Vector3d(wx, wy, wz);
        vel_lpf_initialized_ = true;
      } else {
        vel_filt_.x() = lpf1(vel_filt_.x(), vx, vel_lpf_alpha);
        vel_filt_.y() = lpf1(vel_filt_.y(), vy, vel_lpf_alpha);
        vel_filt_.z() = lpf1(vel_filt_.z(), vz, vel_lpf_alpha);
        angvel_filt_.x() = lpf1(angvel_filt_.x(), wx, vel_lpf_alpha);
        angvel_filt_.y() = lpf1(angvel_filt_.y(), wy, vel_lpf_alpha);
        angvel_filt_.z() = lpf1(angvel_filt_.z(), wz, vel_lpf_alpha);
      }

      vx = vel_filt_.x();
      vy = vel_filt_.y();
      vz = vel_filt_.z();
      wx = angvel_filt_.x();
      wy = angvel_filt_.y();
      wz = angvel_filt_.z();
    } else {
      vel_lpf_initialized_ = false;
    }

    const Vec3 body_force_input(0.0, 0.0, fz);
    const Vec3 world_force_input = r_bw * body_force_input;
    Vec3 body_torque_input(tx, ty, tz);
    body_torque_input += com_offset_body_.cross(body_force_input);
    const double mass_obs = mass_;
    const double jxx_obs = jxx_;
    const double jyy_obs = jyy_;
    const double jzz_obs = jzz_;

    const Vec3 p_lin_world(mass_obs * vx, mass_obs * vy, mass_obs * vz);
    const Vec3 p_ang_body(jxx_obs * wx, jyy_obs * wy, jzz_obs * wz);

    const double iw_x = jxx_obs * wx;
    const double iw_y = jyy_obs * wy;
    const double iw_z = jzz_obs * wz;

    const Vec3 cori_body(
      (wy * iw_z - wz * iw_y),
      (wz * iw_x - wx * iw_z),
      (wx * iw_y - wy * iw_x));

    const Vec3 grav_world(0.0, 0.0, mass_obs * gravity_);
    const Vec3 thrust_axis_world = r_bw * Vec3(0.0, 0.0, 1.0);

    runObserverVariant(
      observer_v4_, p_lin_world, p_ang_body, world_force_input, body_torque_input,
      grav_world, cori_body, dt, true, true,
      kf_2nd_order_, ktau_2nd_order_, mob_alpha_2nd_order_);
    const Vec3 ee_offset_world = r_bw * ee_offset_body_;
    runConsistencyResidualObserver(
      observer_consistency_, p_lin_world, p_ang_body, world_force_input, body_torque_input,
      grav_world, cori_body, ee_offset_world, r_bw, dt,
      kf_2nd_order_, ktau_2nd_order_, mob_alpha_2nd_order_, ke_ep_, 0.0);
    runConsistencyResidualObserver(
      observer_consistency_integral_, p_lin_world, p_ang_body, world_force_input, body_torque_input,
      grav_world, cori_body, ee_offset_world, r_bw, dt,
      kf_2nd_order_, ktau_2nd_order_, mob_alpha_2nd_order_, ke_epi_,
      std::abs(cmd_force_desired_) > 1.0e-6 ? kei_epi_ : 0.0);
    for (size_t i = 0; i < observer_consistency_sweep_.size(); ++i) {
      runConsistencyResidualObserver(
        observer_consistency_sweep_[i], p_lin_world, p_ang_body, world_force_input, body_torque_input,
        grav_world, cori_body, ee_offset_world, r_bw, dt,
        kf_2nd_order_, ktau_2nd_order_, mob_alpha_2nd_order_, sweep_ke_gains_[i], 0.0);
    }
    const Vec3 drone_force_world_2nd_order(
      observer_v4_.world_force_hat_ext[0],
      observer_v4_.world_force_hat_ext[1],
      observer_v4_.world_force_hat_ext[2]);
    const Vec3 drone_torque_body_2nd_order(
      observer_v4_.body_torque_hat_ext[0],
      observer_v4_.body_torque_hat_ext[1],
      observer_v4_.body_torque_hat_ext[2]);
    const Vec3 drone_torque_world_2nd_order = r_bw * drone_torque_body_2nd_order;
    const Vec3 drone_force_world_consistency(
      observer_consistency_.world_force_hat_ext[0],
      observer_consistency_.world_force_hat_ext[1],
      observer_consistency_.world_force_hat_ext[2]);
    const Vec3 drone_torque_body_consistency(
      observer_consistency_.body_torque_hat_ext[0],
      observer_consistency_.body_torque_hat_ext[1],
      observer_consistency_.body_torque_hat_ext[2]);
    const Vec3 drone_torque_world_consistency = r_bw * drone_torque_body_consistency;
    const Vec3 drone_force_world_consistency_integral(
      observer_consistency_integral_.world_force_hat_ext[0],
      observer_consistency_integral_.world_force_hat_ext[1],
      observer_consistency_integral_.world_force_hat_ext[2]);
    const Vec3 drone_torque_body_consistency_integral(
      observer_consistency_integral_.body_torque_hat_ext[0],
      observer_consistency_integral_.body_torque_hat_ext[1],
      observer_consistency_integral_.body_torque_hat_ext[2]);
    const Vec3 drone_torque_world_consistency_integral =
      r_bw * drone_torque_body_consistency_integral;
    const Vec3 drone_force_world_consistency_base = observer_consistency_.force_hat_base_world;
    const Vec3 drone_force_world_kalman = runKalmanLikeFusion(
      drone_force_world_2nd_order,
      drone_torque_world_2nd_order,
      ee_offset_world,
      thrust_axis_world,
      kalman_sigma_force_perp_,
      kalman_sigma_force_thrust_,
      kalman_sigma_torque_);
    const Vec3 drone_torque_world_kalman = ee_offset_world.cross(drone_force_world_kalman);
    const Vec3 drone_force_world_adaptive = runAdaptiveBiasFusion(
      drone_force_world_2nd_order,
      drone_torque_world_2nd_order,
      ee_offset_world,
      thrust_axis_world);
    const Vec3 drone_torque_world_adaptive = ee_offset_world.cross(drone_force_world_adaptive);
    pub_drone_wrench_2nd_order_->publish(
      makeWrenchMsg(t_now, drone_force_world_2nd_order, drone_torque_world_2nd_order));
    pub_drone_wrench_consistency_->publish(
      makeWrenchMsg(t_now, drone_force_world_consistency, drone_torque_world_consistency));
    pub_drone_wrench_consistency_integral_->publish(
      makeWrenchMsg(
        t_now,
        drone_force_world_consistency_integral,
        drone_torque_world_consistency_integral));
    pub_drone_wrench_consistency_base_->publish(
      makeWrenchMsg(t_now, drone_force_world_consistency_base, drone_torque_world_consistency));
    pub_drone_wrench_kalman_->publish(
      makeWrenchMsg(t_now, drone_force_world_kalman, drone_torque_world_kalman));
    pub_drone_wrench_adaptive_->publish(
      makeWrenchMsg(t_now, drone_force_world_adaptive, drone_torque_world_adaptive));
    pub_ee_applied_wrench_2nd_order_->publish(
      makeWrenchMsg(t_now, -drone_force_world_2nd_order, -drone_torque_world_2nd_order));
    pub_ee_applied_wrench_consistency_->publish(
      makeWrenchMsg(t_now, -drone_force_world_consistency, -drone_torque_world_consistency));
    pub_ee_applied_wrench_consistency_integral_->publish(
      makeWrenchMsg(
        t_now,
        -drone_force_world_consistency_integral,
        -drone_torque_world_consistency_integral));
    pub_ee_applied_wrench_consistency_base_->publish(
      makeWrenchMsg(t_now, -drone_force_world_consistency_base, -drone_torque_world_consistency));
    pub_ee_applied_wrench_kalman_->publish(
      makeWrenchMsg(t_now, -drone_force_world_kalman, -drone_torque_world_kalman));
    pub_ee_applied_wrench_adaptive_->publish(
      makeWrenchMsg(t_now, -drone_force_world_adaptive, -drone_torque_world_adaptive));
    for (size_t i = 0; i < observer_consistency_sweep_.size(); ++i) {
      const auto & state = observer_consistency_sweep_[i];
      const Vec3 drone_force_world_sweep(
        state.world_force_hat_ext[0],
        state.world_force_hat_ext[1],
        state.world_force_hat_ext[2]);
      const Vec3 drone_torque_body_sweep(
        state.body_torque_hat_ext[0],
        state.body_torque_hat_ext[1],
        state.body_torque_hat_ext[2]);
      const Vec3 drone_torque_world_sweep = r_bw * drone_torque_body_sweep;
      pub_drone_wrench_consistency_sweep_[i]->publish(
        makeWrenchMsg(t_now, drone_force_world_sweep, drone_torque_world_sweep));
      pub_ee_applied_wrench_consistency_sweep_[i]->publish(
        makeWrenchMsg(t_now, -drone_force_world_sweep, -drone_torque_world_sweep));
    }
    pub_drone_wrench_consistency_terms_->publish(
      makeWrenchMsg(
        t_now,
        observer_consistency_.force_hat_dot_base_world,
        observer_consistency_.force_update_consistency_world));
    pub_drone_wrench_consistency_match_->publish(
      makeWrenchMsg(
        t_now,
        observer_consistency_.tau_hat_ext_world,
        observer_consistency_.r_cross_f_hat_ext_world));
    pub_drone_wrench_consistency_residual_->publish(
      makeWrenchMsg(
        t_now,
        observer_consistency_.e_tau_world,
        Vec3(observer_consistency_.rho_tau, 0.0, 0.0)));
  }

  std::mutex mtx_;

  rcl_interfaces::msg::SetParametersResult onParametersSet(
    const std::vector<rclcpp::Parameter> & params)
  {
    std::lock_guard<std::mutex> lk(mtx_);

    for (const auto & param : params) {
      const auto & name = param.get_name();

      if (name == "mass") {
        mass_ = param.as_double();
      } else if (name == "g") {
        gravity_ = param.as_double();
      } else if (name == "com_bias") {
        const auto values = param.as_double_array();
        if (values.size() != 3) {
          rcl_interfaces::msg::SetParametersResult result;
          result.successful = false;
          result.reason = "com_bias must have exactly 3 values";
          return result;
        }
        com_offset_body_ = Eigen::Vector3d(values[0], values[1], values[2]);
      } else if (name == "inertia_diag") {
        const auto values = param.as_double_array();
        if (values.size() != 3) {
          rcl_interfaces::msg::SetParametersResult result;
          result.successful = false;
          result.reason = "inertia_diag must have exactly 3 values";
          return result;
        }
        jxx_ = values[0];
        jyy_ = values[1];
        jzz_ = values[2];
      } else if (name == "end_effector_offset") {
        const auto values = param.as_double_array();
        if (values.size() != 3) {
          rcl_interfaces::msg::SetParametersResult result;
          result.successful = false;
          result.reason = "end_effector_offset must have exactly 3 values";
          return result;
        }
        ee_offset_body_ = Eigen::Vector3d(values[0], values[1], values[2]);
      } else if (name == "mob.Ke") {
        ke_ = param.as_double();
      } else if (name == "mob.KeI") {
        kei_ = param.as_double();
      } else if (name == "mob.Ke_ep") {
        ke_ep_ = param.as_double();
      } else if (name == "mob.Ke_epi") {
        ke_epi_ = param.as_double();
      } else if (name == "mob.KeI_epi") {
        kei_epi_ = param.as_double();
      } else if (name == "mob.kalman.sigma_force_perp") {
        kalman_sigma_force_perp_ = param.as_double();
      } else if (name == "mob.kalman.sigma_force_thrust") {
        kalman_sigma_force_thrust_ = param.as_double();
      } else if (name == "mob.kalman.sigma_torque") {
        kalman_sigma_torque_ = param.as_double();
      } else if (name == "mob.adaptive.gamma") {
        adaptive_gamma_ = param.as_double();
      } else if (name == "mob.adaptive.lambda_b") {
        adaptive_lambda_b_ = param.as_double();
      } else if (name == "mob.adaptive.bias_limit") {
        adaptive_bias_limit_ = param.as_double();
      } else if (name == "mob.adaptive.sigma_force_perp") {
        adaptive_sigma_force_perp_ = param.as_double();
      } else if (name == "mob.adaptive.sigma_force_thrust") {
        adaptive_sigma_force_thrust_ = param.as_double();
      } else if (name == "mob.adaptive.sigma_torque") {
        adaptive_sigma_torque_ = param.as_double();
      }
    }

    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;
    return result;
  }

  rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr sub_input_;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr sub_cmd_force_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr sub_pose_;
  rclcpp::Subscription<geometry_msgs::msg::Vector3Stamped>::SharedPtr sub_vel_;
  rclcpp::Subscription<geometry_msgs::msg::Vector3Stamped>::SharedPtr sub_angvel_;

  rclcpp::Publisher<geometry_msgs::msg::WrenchStamped>::SharedPtr pub_drone_wrench_2nd_order_;
  rclcpp::Publisher<geometry_msgs::msg::WrenchStamped>::SharedPtr pub_drone_wrench_consistency_;
  rclcpp::Publisher<geometry_msgs::msg::WrenchStamped>::SharedPtr pub_drone_wrench_consistency_integral_;
  rclcpp::Publisher<geometry_msgs::msg::WrenchStamped>::SharedPtr pub_drone_wrench_consistency_terms_;
  rclcpp::Publisher<geometry_msgs::msg::WrenchStamped>::SharedPtr pub_drone_wrench_consistency_base_;
  rclcpp::Publisher<geometry_msgs::msg::WrenchStamped>::SharedPtr pub_drone_wrench_consistency_match_;
  rclcpp::Publisher<geometry_msgs::msg::WrenchStamped>::SharedPtr pub_drone_wrench_consistency_residual_;
  rclcpp::Publisher<geometry_msgs::msg::WrenchStamped>::SharedPtr pub_drone_wrench_kalman_;
  rclcpp::Publisher<geometry_msgs::msg::WrenchStamped>::SharedPtr pub_drone_wrench_adaptive_;
  rclcpp::Publisher<geometry_msgs::msg::WrenchStamped>::SharedPtr pub_ee_applied_wrench_2nd_order_;
  rclcpp::Publisher<geometry_msgs::msg::WrenchStamped>::SharedPtr pub_ee_applied_wrench_consistency_;
  rclcpp::Publisher<geometry_msgs::msg::WrenchStamped>::SharedPtr pub_ee_applied_wrench_consistency_integral_;
  rclcpp::Publisher<geometry_msgs::msg::WrenchStamped>::SharedPtr pub_ee_applied_wrench_consistency_base_;
  rclcpp::Publisher<geometry_msgs::msg::WrenchStamped>::SharedPtr pub_ee_applied_wrench_kalman_;
  rclcpp::Publisher<geometry_msgs::msg::WrenchStamped>::SharedPtr pub_ee_applied_wrench_adaptive_;
  std::vector<rclcpp::Publisher<geometry_msgs::msg::WrenchStamped>::SharedPtr>
    pub_drone_wrench_consistency_sweep_;
  std::vector<rclcpp::Publisher<geometry_msgs::msg::WrenchStamped>::SharedPtr>
    pub_ee_applied_wrench_consistency_sweep_;
  rclcpp::TimerBase::SharedPtr timer_est_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_callback_handle_;

  std::string input_topic_;
  std::string cmd_force_topic_;
  std::string pose_topic_;
  std::string vel_topic_;
  std::string angvel_topic_;
  std::string drone_wrench_topic_2nd_order_;
  std::string drone_wrench_topic_consistency_;
  std::string drone_wrench_topic_consistency_integral_;
  std::string drone_wrench_topic_consistency_terms_;
  std::string drone_wrench_topic_consistency_base_;
  std::string drone_wrench_topic_consistency_match_;
  std::string drone_wrench_topic_consistency_residual_;
  std::string drone_wrench_topic_kalman_;
  std::string drone_wrench_topic_adaptive_;
  std::string ee_applied_wrench_topic_2nd_order_;
  std::string ee_applied_wrench_topic_consistency_;
  std::string ee_applied_wrench_topic_consistency_integral_;
  std::string ee_applied_wrench_topic_consistency_base_;
  std::string ee_applied_wrench_topic_kalman_;
  std::string ee_applied_wrench_topic_adaptive_;

  double observer_hz_{250.0};
  bool dt_fixed_enable_{true};
  double dt_fixed_value_{0.004};
  double dt_alpha_{0.2};
  bool vel_lpf_enable_{true};
  double vel_lpf_cutoff_hz_{0.5};

  double mass_{0.04338};
  double gravity_{9.81};
  double jxx_{2.3951e-5};
  double jyy_{2.3951e-5};
  double jzz_{3.2347e-5};

  // 2nd-order MOB tuning
  double kf_2nd_order_{1.0};
  double ktau_2nd_order_{1.0};
  double mob_alpha_2nd_order_{0.2};
  double kp_{2.0};
  double kptau_{2.0};
  double ke_{1.0};
  double kei_{0.0};
  double ke_ep_{1.0};
  double ke_epi_{1.0};
  double kei_epi_{0.0};
  std::vector<double> k_ep_sweep_gains_;
  std::vector<double> sweep_ke_gains_;
  double epsilon_tau_{1.0e-6};
  double kalman_sigma_force_perp_{0.10};
  double kalman_sigma_force_thrust_{0.50};
  double kalman_sigma_torque_{0.01};
  double adaptive_gamma_{0.2};
  double adaptive_lambda_b_{0.01};
  double adaptive_bias_limit_{2.0};
  double adaptive_sigma_force_perp_{0.10};
  double adaptive_sigma_force_thrust_{0.50};
  double adaptive_sigma_torque_{0.01};

  double arm_xy_{0.035355};
  double k_tau_motor_{0.00569278844371417};
  double thrust_min_{0.0};
  double thrust_max_{0.20};
  std::array<double, 4> motor_dir_{{1.0, -1.0, 1.0, -1.0}};
  Eigen::Vector3d ee_offset_body_{0.09, 0.0, 0.085};
  Eigen::Vector3d com_offset_body_{0.0, 0.0, 0.0};

  Eigen::Matrix4d b_{Eigen::Matrix4d::Identity()};
  Eigen::Matrix4d b_inv_{Eigen::Matrix4d::Identity()};

  std::array<double, 4> input_tau_fz_{{0.0, 0.0, 0.0, 0.0}};
  double cmd_force_desired_{0.0};
  geometry_msgs::msg::PoseStamped pose_;
  geometry_msgs::msg::Vector3Stamped vel_;
  geometry_msgs::msg::Vector3Stamped angvel_;
  bool have_input_{false};
  bool have_pose_{false};
  bool have_vel_{false};
  bool have_angvel_{false};

  rclcpp::Time last_time_{0, 0, RCL_ROS_TIME};
  double dt_filt_{0.004};
  bool vel_lpf_initialized_{false};
  Eigen::Vector3d vel_filt_{0.0, 0.0, 0.0};
  Eigen::Vector3d angvel_filt_{0.0, 0.0, 0.0};

  ObserverState observer_v4_;
  ConsistencyObserverState observer_consistency_;
  ConsistencyObserverState observer_consistency_integral_;
  std::vector<ConsistencyObserverState> observer_consistency_sweep_;
  double adaptive_bias_hat_{0.0};
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<WrenchObserver>());
  rclcpp::shutdown();
  return 0;
}
