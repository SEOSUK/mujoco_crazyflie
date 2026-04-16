#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/vector3_stamped.hpp>
#include <geometry_msgs/msg/wrench_stamped.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <Eigen/Dense>

#include <algorithm>
#include <array>
#include <cmath>
#include <mutex>
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
    Vec3 p_lin_hat_world{Vec3::Zero()};
    Vec3 p_ang_hat_body{Vec3::Zero()};
    Vec3 force_hat_world{Vec3::Zero()};
    Vec3 torque_hat_body{Vec3::Zero()};
    Vec3 force_update_base_world{Vec3::Zero()};
    Vec3 force_update_consistency_world{Vec3::Zero()};
    Vec3 torque_hat_world{Vec3::Zero()};
    Vec3 moment_from_force_world{Vec3::Zero()};
    std::array<double, 3> world_force_hat_ext{{0.0, 0.0, 0.0}};
    std::array<double, 3> body_torque_hat_ext{{0.0, 0.0, 0.0}};
  };

  WrenchObserver()
  : Node("wrench_observer")
  {
    input_topic_ = declare_parameter<std::string>("input_topic", "/crazyflie/in/input");
    pose_topic_ = declare_parameter<std::string>("pose_topic", "/crazyflie/out/pose");
    vel_topic_ = declare_parameter<std::string>("vel_topic", "/crazyflie/out/vel");
    angvel_topic_ = declare_parameter<std::string>("angvel_topic", "/crazyflie/out/ang_vel");
    drone_wrench_topic_ = declare_parameter<std::string>(
      "drone_wrench_topic", "/crazyflie/out/mob");
    drone_wrench_topic_2nd_order_ = declare_parameter<std::string>(
      "drone_wrench_topic_2nd_order", "/crazyflie/out/mob_2nd");
    drone_wrench_topic_consistency_ = declare_parameter<std::string>(
      "drone_wrench_topic_consistency", "/crazyflie/out/mob_2nd_tau");
    drone_wrench_topic_consistency_terms_ = declare_parameter<std::string>(
      "drone_wrench_topic_consistency_terms", "/crazyflie/out/mob_2nd_tau_terms");
    drone_wrench_topic_consistency_match_ = declare_parameter<std::string>(
      "drone_wrench_topic_consistency_match", "/crazyflie/out/mob_2nd_tau_consistency");

    observer_hz_ = declare_parameter<double>("observer_hz", 250.0);
    dt_fixed_enable_ = declare_parameter<bool>("mob.dt_fixed_enable", true);
    dt_fixed_value_ = declare_parameter<double>("mob.dt_fixed_value", 0.004);
    dt_alpha_ = declare_parameter<double>("mob.dt_alpha", 0.2);
    vel_lpf_enable_ = declare_parameter<bool>("mob.vel_lpf_enable", false);
    vel_lpf_cutoff_hz_ = declare_parameter<double>("mob.vel_lpf_cutoff_hz", 0.5);

    mass_ = declare_parameter<double>("mass", 0.04338);
    gravity_ = declare_parameter<double>("g", 9.81);
    jxx_ = declare_parameter<double>("mob.Jxx", 2.3951e-5);
    jyy_ = declare_parameter<double>("mob.Jyy", 2.3951e-5);
    jzz_ = declare_parameter<double>("mob.Jzz", 3.2347e-5);

    // 1st-order MOB tuning
    kf_1st_order_ = declare_parameter<double>("mob.Kf_1st_order", 5.0);
    ktau_1st_order_ = declare_parameter<double>("mob.Ktau_1st_order", 20.0);
    mob_alpha_1st_order_ = declare_parameter<double>("mob.mob_alpha_1st_order", 0.2);

    // 2nd-order MOB tuning
    kf_2nd_order_ = declare_parameter<double>("mob.Kf_2nd_order", 5.0);
    ktau_2nd_order_ = declare_parameter<double>("mob.Ktau_2nd_order", 20.0);
    mob_alpha_2nd_order_ = declare_parameter<double>("mob.mob_alpha_2nd_order", 0.2);
    kp_ = declare_parameter<double>("mob.Kp", 2.0);
    kptau_ = declare_parameter<double>("mob.KpTau", 10.0);
    ke_ = declare_parameter<double>("mob.Ke", 10.0);
    sigma_tau_ = declare_parameter<double>("mob.sigma_tau", 10.0);

    arm_xy_ = declare_parameter<double>("arm_xy", 0.035355);
    k_tau_motor_ = declare_parameter<double>("k_tau", 0.00569278844371417);
    thrust_min_ = declare_parameter<double>("thrust_min", 0.0);
    thrust_max_ = declare_parameter<double>("thrust_max", 0.20);

    auto ee_offset_param = declare_parameter<std::vector<double>>(
      "end_effector_offset", std::vector<double>{0.09, 0.0, 0.085});
    if (ee_offset_param.size() != 3) {
      RCLCPP_WARN(get_logger(), "end_effector_offset must have size 3. Falling back to [0.09, 0.0, 0.085].");
      ee_offset_param = {0.09, 0.0, 0.085};
    }
    ee_offset_body_ = Eigen::Vector3d(ee_offset_param[0], ee_offset_param[1], ee_offset_param[2]);

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
    sub_pose_ = create_subscription<geometry_msgs::msg::PoseStamped>(
      pose_topic_, qos, std::bind(&WrenchObserver::poseCb, this, std::placeholders::_1));
    sub_vel_ = create_subscription<geometry_msgs::msg::Vector3Stamped>(
      vel_topic_, qos, std::bind(&WrenchObserver::velCb, this, std::placeholders::_1));
    sub_angvel_ = create_subscription<geometry_msgs::msg::Vector3Stamped>(
      angvel_topic_, qos, std::bind(&WrenchObserver::angVelCb, this, std::placeholders::_1));

    pub_drone_wrench_ = create_publisher<geometry_msgs::msg::WrenchStamped>(drone_wrench_topic_, 10);
    pub_drone_wrench_2nd_order_ = create_publisher<geometry_msgs::msg::WrenchStamped>(
      drone_wrench_topic_2nd_order_, 10);
    pub_drone_wrench_consistency_ = create_publisher<geometry_msgs::msg::WrenchStamped>(
      drone_wrench_topic_consistency_, 10);
    pub_drone_wrench_consistency_terms_ = create_publisher<geometry_msgs::msg::WrenchStamped>(
      drone_wrench_topic_consistency_terms_, 10);
    pub_drone_wrench_consistency_match_ = create_publisher<geometry_msgs::msg::WrenchStamped>(
      drone_wrench_topic_consistency_match_, 10);

    const double safe_hz = std::max(1.0, observer_hz_);
    timer_est_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::duration<double>(1.0 / safe_hz)),
      std::bind(&WrenchObserver::loopEst, this));

    last_time_ = now();

    RCLCPP_INFO(get_logger(), "wrench_observer started");
    RCLCPP_INFO(get_logger(), "sub: %s | %s | %s | %s",
      input_topic_.c_str(), pose_topic_.c_str(), vel_topic_.c_str(), angvel_topic_.c_str());
    RCLCPP_INFO(get_logger(), "pub: %s | %s | %s",
      drone_wrench_topic_.c_str(), drone_wrench_topic_2nd_order_.c_str(),
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
        lpf1(state.world_force_hat_ext[i], -state.force_hat_world[i], mob_alpha);
      state.body_torque_hat_ext[i] =
        lpf1(state.body_torque_hat_ext[i], -state.torque_hat_body[i], mob_alpha);
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
    double mob_alpha)
  {
    const Vec3 p_lin_residual_world = p_lin_world - state.p_lin_hat_world;
    const Vec3 p_ang_residual_body = p_ang_body - state.p_ang_hat_body;

    const Vec3 torque_ext_hat_body = gainMul(ktau, p_ang_residual_body);
    state.torque_hat_body = torque_ext_hat_body;

    Vec3 p_ang_hat_dot_body = u_tau_body - cori_body + torque_ext_hat_body;
    p_ang_hat_dot_body += gainMul(kptau_, p_ang_residual_body);
    state.p_ang_hat_body += dt * p_ang_hat_dot_body;

    const Vec3 torque_ext_hat_world = r_bw * torque_ext_hat_body;
    const Vec3 e_tau_world =
      torque_ext_hat_world - ee_offset_world.cross(state.force_hat_world);
    const Vec3 force_update_base_world = gainMul(kf, p_lin_residual_world);
    const Vec3 force_update_consistency_world =
      sigma_tau_ * ke_ * (skew(ee_offset_world).transpose() * e_tau_world);
    const Vec3 force_hat_dot_world =
      force_update_base_world + force_update_consistency_world;
    state.force_hat_world += dt * force_hat_dot_world;
    state.force_update_base_world = force_update_base_world;
    state.force_update_consistency_world = force_update_consistency_world;
    state.torque_hat_world = torque_ext_hat_world;
    state.moment_from_force_world = ee_offset_world.cross(state.force_hat_world);

    Vec3 p_lin_hat_dot_world = u_lin_world - grav_world + state.force_hat_world;
    p_lin_hat_dot_world += gainMul(kp_, p_lin_residual_world);
    state.p_lin_hat_world += dt * p_lin_hat_dot_world;

    for (int i = 0; i < 3; ++i) {
      state.world_force_hat_ext[i] =
        lpf1(state.world_force_hat_ext[i], -state.force_hat_world[i], mob_alpha);
      state.body_torque_hat_ext[i] =
        lpf1(state.body_torque_hat_ext[i], -torque_ext_hat_body[i], mob_alpha);
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
    const double tz = k_tau_motor_ * (-f1 + f2 - f3 + f4);
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

    const Vec3 world_force_input = r_bw * Vec3(0.0, 0.0, fz);
    const Vec3 body_torque_input(tx, ty, tz);
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

    runObserverVariant(
      observer_v1_, p_lin_world, p_ang_body, world_force_input, body_torque_input,
      grav_world, cori_body, dt, false, false,
      kf_1st_order_, ktau_1st_order_, mob_alpha_1st_order_);
    runObserverVariant(
      observer_v4_, p_lin_world, p_ang_body, world_force_input, body_torque_input,
      grav_world, cori_body, dt, true, true,
      kf_2nd_order_, ktau_2nd_order_, mob_alpha_2nd_order_);
    const Vec3 ee_offset_world = r_bw * ee_offset_body_;
    runConsistencyResidualObserver(
      observer_consistency_, p_lin_world, p_ang_body, world_force_input, body_torque_input,
      grav_world, cori_body, ee_offset_world, r_bw, dt,
      kf_2nd_order_, ktau_2nd_order_, mob_alpha_2nd_order_);

    const Vec3 drone_force_world(
      observer_v1_.world_force_hat_ext[0],
      observer_v1_.world_force_hat_ext[1],
      observer_v1_.world_force_hat_ext[2]);
    const Vec3 drone_torque_body(
      observer_v1_.body_torque_hat_ext[0],
      observer_v1_.body_torque_hat_ext[1],
      observer_v1_.body_torque_hat_ext[2]);
    const Vec3 drone_torque_world = r_bw * drone_torque_body;
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

    pub_drone_wrench_->publish(makeWrenchMsg(t_now, drone_force_world, drone_torque_world));
    pub_drone_wrench_2nd_order_->publish(
      makeWrenchMsg(t_now, drone_force_world_2nd_order, drone_torque_world_2nd_order));
    pub_drone_wrench_consistency_->publish(
      makeWrenchMsg(t_now, drone_force_world_consistency, drone_torque_world_consistency));
    pub_drone_wrench_consistency_terms_->publish(
      makeWrenchMsg(
        t_now,
        observer_consistency_.force_update_base_world,
        observer_consistency_.force_update_consistency_world));
    pub_drone_wrench_consistency_match_->publish(
      makeWrenchMsg(
        t_now,
        observer_consistency_.torque_hat_world,
        observer_consistency_.moment_from_force_world));
  }

  std::mutex mtx_;

  rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr sub_input_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr sub_pose_;
  rclcpp::Subscription<geometry_msgs::msg::Vector3Stamped>::SharedPtr sub_vel_;
  rclcpp::Subscription<geometry_msgs::msg::Vector3Stamped>::SharedPtr sub_angvel_;

  rclcpp::Publisher<geometry_msgs::msg::WrenchStamped>::SharedPtr pub_drone_wrench_;
  rclcpp::Publisher<geometry_msgs::msg::WrenchStamped>::SharedPtr pub_drone_wrench_2nd_order_;
  rclcpp::Publisher<geometry_msgs::msg::WrenchStamped>::SharedPtr pub_drone_wrench_consistency_;
  rclcpp::Publisher<geometry_msgs::msg::WrenchStamped>::SharedPtr pub_drone_wrench_consistency_terms_;
  rclcpp::Publisher<geometry_msgs::msg::WrenchStamped>::SharedPtr pub_drone_wrench_consistency_match_;
  rclcpp::TimerBase::SharedPtr timer_est_;

  std::string input_topic_;
  std::string pose_topic_;
  std::string vel_topic_;
  std::string angvel_topic_;
  std::string drone_wrench_topic_;
  std::string drone_wrench_topic_2nd_order_;
  std::string drone_wrench_topic_consistency_;
  std::string drone_wrench_topic_consistency_terms_;
  std::string drone_wrench_topic_consistency_match_;

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

  // 1st-order MOB tuning
  double kf_1st_order_{1.0};
  double ktau_1st_order_{1.0};
  double mob_alpha_1st_order_{0.2};

  // 2nd-order MOB tuning
  double kf_2nd_order_{1.0};
  double ktau_2nd_order_{1.0};
  double mob_alpha_2nd_order_{0.2};
  double kp_{2.0};
  double kptau_{2.0};
  double ke_{1.0};
  double sigma_tau_{1.0};

  double arm_xy_{0.035355};
  double k_tau_motor_{0.00569278844371417};
  double thrust_min_{0.0};
  double thrust_max_{0.20};
  std::array<double, 4> motor_dir_{{1.0, -1.0, 1.0, -1.0}};
  Eigen::Vector3d ee_offset_body_{0.09, 0.0, 0.085};

  Eigen::Matrix4d b_{Eigen::Matrix4d::Identity()};
  Eigen::Matrix4d b_inv_{Eigen::Matrix4d::Identity()};

  std::array<double, 4> input_tau_fz_{{0.0, 0.0, 0.0, 0.0}};
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

  ObserverState observer_v1_;
  ObserverState observer_v4_;
  ConsistencyObserverState observer_consistency_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<WrenchObserver>());
  rclcpp::shutdown();
  return 0;
}
