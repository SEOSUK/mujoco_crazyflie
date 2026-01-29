/**
 * @file trajectory_generation.cpp
 *
 * @brief
 * Trajectory generation node that fuses user commands and (future) force feedback
 * to generate final position/yaw commands for the Crazyflie low-level controller.
 *
 * Publishes:
 *  - /crazyflie/in/pos_cmd                 [std_msgs/Float64MultiArray: x,y,z,yaw(rad)]
 *  - /su/force_lpf                        [std_msgs/Float64MultiArray: raw, filt]
 *  - /estimated_contact_frame_quat        [geometry_msgs/QuaternionStamped]  (R_C as quat)
 */

#include <rclcpp/rclcpp.hpp>

#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>

#include <geometry_msgs/msg/wrench_stamped.hpp>
#include <geometry_msgs/msg/quaternion_stamped.hpp>

#include <array>
#include <algorithm>
#include <cmath>
#include <mutex>

#include <Eigen/Dense>

using namespace std::chrono_literals;

class TrajectoryGeneration : public rclcpp::Node
{
public:
  TrajectoryGeneration()
  : Node("trajectory_generation")
  {
    // -------------------------
    // Subscribers (inputs)
    // -------------------------
    sub_keyboard_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
      "/su/keyboard_input", 10,
      std::bind(&TrajectoryGeneration::keyboardCb, this, std::placeholders::_1));

    sub_use_vel_mode_ = this->create_subscription<std_msgs::msg::Float32>(
      "su/use_vel_mode", 10,
      std::bind(&TrajectoryGeneration::useVelModeCb, this, std::placeholders::_1));

    sub_cmd_force_ = this->create_subscription<std_msgs::msg::Float32>(
      "su/cmd_force", 10,
      std::bind(&TrajectoryGeneration::cmdForceCb, this, std::placeholders::_1));

    sub_contact_force_ = this->create_subscription<geometry_msgs::msg::WrenchStamped>(
      "/crazyflie/out/EE_contact_force_filt", 10,
      std::bind(&TrajectoryGeneration::contactForceCb, this, std::placeholders::_1));

    // -------------------------
    // Publishers
    // -------------------------
    pub_pos_cmd_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(
      "/crazyflie/in/pos_cmd", 10);

    pub_force_lpf_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(
      "/su/force_lpf", 10);

    pub_contact_quat_ = this->create_publisher<geometry_msgs::msg::QuaternionStamped>(
      "/estimated_contact_frame_quat", 10);

    // -------------------------
    // Update loops
    // -------------------------
    timer_ = this->create_wall_timer(
      10ms, std::bind(&TrajectoryGeneration::update, this)); // 100 Hz

    force_timer_ = this->create_wall_timer(
      10ms, std::bind(&TrajectoryGeneration::forceUpdate, this));  // 100 Hz

    RCLCPP_INFO(this->get_logger(), "trajectory_generation started");
  }

private:
  bool flip_measured_force_ = false;   // 필요하면 true로
  // -------------------------
  // Utils
  // -------------------------
  static inline double wrap_pi(double a)
  {
    while (a > M_PI) a -= 2.0 * M_PI;
    while (a < -M_PI) a += 2.0 * M_PI;
    return a;
  }

  // yaw align parameters (velocity mode)
  double yaw_align_kp_   = 3.0;   // [rad/s] per rad (1st-order yaw tracking)
  double normal_xy_min_  = 1e-3;  // avoid atan2 on near-vertical normal

  // =========================
  // Contact frame estimation
  // =========================
  struct ContactFrame
  {
    Eigen::Vector3d n_w = Eigen::Vector3d::UnitX();     // contact normal in world
    Eigen::Matrix3d R_C = Eigen::Matrix3d::Identity();  // contact->world
    bool valid = false;
  };

  ContactFrame cf_;

  // Params (rough defaults)
  double normal_force_threshold_ = 0.005; // [N] below this, consider "no contact"
  double normal_lpf_alpha_       = 0.2;   // 0~1 (bigger = faster)

  bool normal_vector_estimation(ContactFrame &cf_out)
  {
    // 1) get latest measured force (world frame assumed)
    std::array<double,3> Fw_arr{0.0,0.0,0.0};
    {
      std::lock_guard<std::mutex> lk(force_mtx_);
      Fw_arr = contact_F_;
    }
    Eigen::Vector3d Fw(Fw_arr[0], Fw_arr[1], Fw_arr[2]);

    const double Fnorm = Fw.norm();
    if (Fnorm < normal_force_threshold_) {
      cf_out.valid = false;
      return false;
    }

    // 2) estimate normal direction (flip sign here if needed)
    Eigen::Vector3d n_new = Fw / (Fnorm + 1e-12);

    // 3) LPF the normal (optional)
    if (!cf_out.valid) {
      cf_out.n_w = n_new;
    } else {
      cf_out.n_w = (1.0 - normal_lpf_alpha_) * cf_out.n_w + normal_lpf_alpha_ * n_new;
      const double nn = cf_out.n_w.norm();
      if (nn > 1e-9) cf_out.n_w /= nn;
      else cf_out.n_w = n_new;
    }

    // 4) build contact frame axes
    const Eigen::Vector3d x_c = cf_out.n_w.normalized();

    // world gravity axis reference (z-up). If NED, use (0,0,-1)
    const Eigen::Vector3d g_axis = Eigen::Vector3d::UnitZ();

    // y_c = x_c × g_axis
    Eigen::Vector3d y_c = x_c.cross(g_axis);
    double y_norm = y_c.norm();

    // fallback if parallel
    if (y_norm < 1e-6) {
      Eigen::Vector3d a = Eigen::Vector3d::UnitY();
      if (std::abs(x_c.dot(a)) > 0.9) a = Eigen::Vector3d::UnitX();
      y_c = x_c.cross(a);
      y_norm = y_c.norm();
      if (y_norm < 1e-9) {
        cf_out.valid = false;
        return false;
      }
    }
    y_c /= (y_norm + 1e-12);

    // ✅ flip y-axis direction (as requested)
    y_c = -y_c;

    // z_c = x_c × y_c
    Eigen::Vector3d z_c = x_c.cross(y_c);
    const double z_norm = z_c.norm();
    if (z_norm < 1e-9) {
      cf_out.valid = false;
      return false;
    }
    z_c /= z_norm;

    // 5) R_C = [x_c y_c z_c]
    cf_out.R_C.col(0) = x_c;
    cf_out.R_C.col(1) = y_c;
    cf_out.R_C.col(2) = z_c;

    cf_out.valid = true;
    return true;
  }

  void publishContactQuat(const Eigen::Matrix3d &R_C, const rclcpp::Time &stamp)
  {
    // Eigen quaternion from rotation matrix
    Eigen::Quaterniond q(R_C);
    q.normalize();

    geometry_msgs::msg::QuaternionStamped msg;
    msg.header.stamp = stamp;
    msg.header.frame_id = "world";  // adjust if you use different world frame id

    msg.quaternion.w = q.w();
    msg.quaternion.x = q.x();
    msg.quaternion.y = q.y();
    msg.quaternion.z = q.z();

    pub_contact_quat_->publish(msg);
  }

  // =========================
  // Callbacks
  // =========================
  void keyboardCb(const std_msgs::msg::Float64MultiArray::SharedPtr msg)
  {
    if (msg->data.size() < 4) return;

    sp_in_[0] = msg->data[0];
    sp_in_[1] = msg->data[1];
    sp_in_[2] = msg->data[2];
    sp_in_yaw_ = msg->data[3]; // NOTE: yaw and yaw_rate are assumed rad / rad/s

    sp_received_ = true;
  }

  void useVelModeCb(const std_msgs::msg::Float32::SharedPtr msg)
  {
    su_cmd_use_vel_mode_ = msg->data;
  }

  void cmdForceCb(const std_msgs::msg::Float32::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lk(force_mtx_);
    su_cmd_fx_ = msg->data; // F_des_x (store)
  }

  void contactForceCb(const geometry_msgs::msg::WrenchStamped::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lk(force_mtx_);
    contact_F_[0] = msg->wrench.force.x;
    contact_F_[1] = msg->wrench.force.y;
    contact_F_[2] = msg->wrench.force.z;

    f_ext_received_ = true;
  }

  // =========================
  // Force update (admittance helper)
  // =========================
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

  // snapshot
  float su_cmd_fx_local = 0.0f;
  std::array<double,3> contact_F_local{0.0,0.0,0.0};
  bool f_ok = false;

  {
    std::lock_guard<std::mutex> lk(force_mtx_);
    su_cmd_fx_local = su_cmd_fx_;
    contact_F_local = contact_F_;
    f_ok = f_ext_received_;
  }
  if (!f_ok) return;

  // dt
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

  // -----------------------------
  // 1) world force -> contact x
  // -----------------------------
  Eigen::Vector3d Fw(contact_F_local[0], contact_F_local[1], contact_F_local[2]);
  if (flip_measured_force_) {
    Fw = -Fw;
  }

  // Update contact frame estimate using the same normal estimator
  ContactFrame cf_local = cf_;
  const bool cf_ok = normal_vector_estimation(cf_local);
  if (!(cf_ok && cf_local.valid)) {
    // frame not reliable -> don't update force controller
    eF_state_initialized_ = false;
    return;
  }
  cf_ = cf_local;

  // world -> contact
  const Eigen::Vector3d Fc = cf_local.R_C.transpose() * Fw;
  const double Fcx = Fc.x();   // ✅ normal force measurement

  // -----------------------------
  // 2) force error uses ONLY normal axis
  // -----------------------------
  std::array<double,3> eF{0.0,0.0,0.0};
  eF[0] = static_cast<double>(su_cmd_fx_local) - Fcx;  // ✅ contact-frame x
  eF[1] = 0.0;
  eF[2] = 0.0;

  // init
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

  // numeric diff (x only도 충분)
  std::array<double,3> eF_dot_raw{0.0,0.0,0.0};
  eF_dot_raw[0] = (eF[0] - eF_prev_[0]) / dt;

  // LPF on dot(e_F)
  const double wc = 3.0;
  const double alpha = wc * dt;
  const double a = std::clamp(alpha, 0.0, 1.0);

  std::array<double,3> eF_dot_filt{0.0,0.0,0.0};
  eF_dot_filt[0] = (1.0 - a) * eF_dot_filt_prev_[0] + a * eF_dot_raw[0];

  // state update
  eF_prev_ = eF;
  eF_dot_filt_prev_ = eF_dot_filt;

  {
    std::lock_guard<std::mutex> lk(force_mtx_);
    eF_ = eF;
    eF_dot_filt_ = eF_dot_filt;
  }

  // debug publish: raw and filtered derivative (x-axis)
  std_msgs::msg::Float64MultiArray lpf;
  lpf.data.resize(2);
  lpf.data[0] = eF_dot_raw[0];
  lpf.data[1] = eF_dot_filt[0];
  pub_force_lpf_->publish(lpf);
}


  // =========================
  // Main update loop
  // =========================
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

    const bool vel_mode_on = (su_cmd_use_vel_mode_ > 0.5f);

    // init integrator
    if (!su_int_initialized_) {
      if (!vel_mode_on) {
        su_int_pos_ = sp_in_;
        su_int_yaw_ = sp_in_yaw_;
      } else {
        su_int_pos_ = {0.0, 0.0, 0.0};
        su_int_yaw_ = 0.0;
      }
      su_int_initialized_ = true;
      su_vel_mode_prev_ = vel_mode_on;
      su_pos_base_valid_ = false;
    }

    // mode switching
    if (vel_mode_on && !su_vel_mode_prev_) {
      su_pos_base_valid_ = false;
    } else if (!vel_mode_on && su_vel_mode_prev_) {
      su_pos_base_ = su_int_pos_;
      su_yaw_base_ = su_int_yaw_;
      su_pos_base_valid_ = true;
    }
    su_vel_mode_prev_ = vel_mode_on;

    // mode behavior
    if (!vel_mode_on) {
      // Position mode
      if (su_pos_base_valid_) {
        su_int_pos_[0] = su_pos_base_[0] + sp_in_[0];
        su_int_pos_[1] = su_pos_base_[1] + sp_in_[1];
        su_int_pos_[2] = su_pos_base_[2] + sp_in_[2];
        su_int_yaw_    = su_yaw_base_    + sp_in_yaw_;
      } else {
        su_int_pos_ = sp_in_;
        su_int_yaw_ = sp_in_yaw_;
      }

    } else {
      // Velocity mode
      double vx_cmd   = sp_in_[0];
      double vy_cmd   = sp_in_[1];
      double vz_cmd   = sp_in_[2];
      double vyaw_cmd = sp_in_yaw_; // [rad/s]

      // read admittance states
      std::array<double,3> eF{0.0, 0.0, 0.0};
      std::array<double,3> eF_dot_filt{0.0, 0.0, 0.0};
      {
        std::lock_guard<std::mutex> lk(force_mtx_);
        eF = eF_;
        eF_dot_filt = eF_dot_filt_;
      }

      // admittance (x only)
      const double K_p = 3.0;
      const double K_d = 0.005;
      const double vel_adm_x = K_p * eF[0] + K_d * eF_dot_filt[0];
      vx_cmd += vel_adm_x;

      // contact frame conversion
      Eigen::Vector3d v_c(vx_cmd, vy_cmd, vz_cmd);
      Eigen::Vector3d v_w = v_c; // fallback: treat cmd as world vel

      ContactFrame cf_local = cf_;
      const bool cf_ok = normal_vector_estimation(cf_local);
      if (cf_ok && cf_local.valid) {
        v_w = cf_local.R_C * v_c;
        cf_ = cf_local;

        // publish quaternion of R_C
        publishContactQuat(cf_local.R_C, now);
      }

      // integrate world position
      su_int_pos_[0] += v_w.x() * dt;
      su_int_pos_[1] += v_w.y() * dt;
      su_int_pos_[2] += v_w.z() * dt;

      // yaw align to normal (if available), else use user yaw rate
      double yaw_next = su_int_yaw_; // fallback
      if (cf_ok && cf_local.valid) {
        const Eigen::Vector3d n_w = cf_local.n_w;
        const double nxy = std::hypot(n_w.x(), n_w.y());

        if (nxy > normal_xy_min_) {
          const double psi_align = std::atan2(n_w.y(), n_w.x()); // [rad]
          const double e_yaw = wrap_pi(psi_align - su_int_yaw_);
          double psi_dot = yaw_align_kp_ * e_yaw; // [rad/s]
          // psi_dot = std::clamp(psi_dot, -2.0, 2.0); // optional
          yaw_next = su_int_yaw_ + psi_dot * dt;
        } else {
          yaw_next = su_int_yaw_ + vyaw_cmd * dt;
        }
      } else {
        yaw_next = su_int_yaw_ + vyaw_cmd * dt;
      }
      su_int_yaw_ = yaw_next;
    }

    publishOut();
  }

  void publishOut()
  {
    std_msgs::msg::Float64MultiArray out;
    out.data.resize(4);
    out.data[0] = su_int_pos_[0];
    out.data[1] = su_int_pos_[1];
    out.data[2] = su_int_pos_[2];
    out.data[3] = su_int_yaw_; // yaw [rad]
    pub_pos_cmd_->publish(out);
  }

  // =========================
  // ROS interfaces
  // =========================
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr  sub_keyboard_;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr            sub_use_vel_mode_;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr            sub_cmd_force_;
  rclcpp::Subscription<geometry_msgs::msg::WrenchStamped>::SharedPtr sub_contact_force_;

  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr     pub_pos_cmd_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr     pub_force_lpf_;
  rclcpp::Publisher<geometry_msgs::msg::QuaternionStamped>::SharedPtr pub_contact_quat_;

  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::TimerBase::SharedPtr force_timer_;

  // =========================
  // Internal state
  // =========================
  // inputs (sp_in)
  std::array<double, 3> sp_in_{0.0, 0.0, 0.0};
  double sp_in_yaw_{0.0};
  bool sp_received_{false};

  // params/commands
  float su_cmd_use_vel_mode_{0.0f}; // default: position mode
  float su_cmd_fx_{0.0f};           // F_des_x

  // integrator state
  std::array<double, 3> su_int_pos_{0.0, 0.0, 0.0};
  double su_int_yaw_{0.0};
  bool su_int_initialized_{false};

  // mode switching bookkeeping
  bool su_vel_mode_prev_{false};

  // vel->pos base
  std::array<double, 3> su_pos_base_{0.0, 0.0, 0.0};
  double su_yaw_base_{0.0};
  bool su_pos_base_valid_{false};

  // force/admittance shared state
  std::mutex force_mtx_;

  std::array<double,3> eF_{0.0, 0.0, 0.0};
  std::array<double,3> eF_dot_filt_{0.0, 0.0, 0.0};

  std::array<double,3> eF_prev_{0.0, 0.0, 0.0};
  std::array<double,3> eF_dot_filt_prev_{0.0, 0.0, 0.0};

  std::array<double,3> contact_F_{0.0, 0.0, 0.0};
  bool eF_state_initialized_{false};
  bool f_ext_received_{false};

  rclcpp::Time eF_last_time_;
  rclcpp::Time last_time_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TrajectoryGeneration>());
  rclcpp::shutdown();
  return 0;
}
