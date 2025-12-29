#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/vector3_stamped.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <builtin_interfaces/msg/time.hpp>

#include <Eigen/Dense>
#include <mutex>
#include <cmath>
#include <algorithm>
#include <string>

// ---------------- utils ----------------
static inline double clamp(double v, double lo, double hi)
{ return std::max(lo, std::min(hi, v)); }

static inline double wrap_pi(double a)
{
  while (a > M_PI) a -= 2*M_PI;
  while (a < -M_PI) a += 2*M_PI;
  return a;
}

// ROS quat: (x,y,z,w)
static inline double roll_from_quat(double x, double y, double z, double w)
{
  const double sinr_cosp = 2.0 * (w*x + y*z);
  const double cosr_cosp = 1.0 - 2.0 * (x*x + y*y);
  return std::atan2(sinr_cosp, cosr_cosp);
}

static inline double pitch_from_quat(double x, double y, double z, double w)
{
  const double sinp = 2.0 * (w*y - z*x);
  if (std::abs(sinp) >= 1.0) {
    return std::copysign(M_PI / 2.0, sinp);  // 90 deg
  }
  return std::asin(sinp);
}

static inline double yaw_from_quat(double x, double y, double z, double w)
{
  return std::atan2(2.0*(w*z + x*y), 1.0 - 2.0*(y*y + z*z));
}

// ---------------- PID ----------------
struct PID {
  double kp{0}, kd{0};
  double prev{0};
  bool first{true};

  // 1st-order LPF for derivative term
  double d_filt{0.0};
  bool d_first{true};

  // Fixed cutoff frequency [Hz]
  static constexpr double DERIV_CUTOFF_HZ = 3.0;

  static inline double lpf_alpha(double dt, double fc_hz)
  {
    // alpha = dt / (RC + dt), RC = 1/(2*pi*fc)
    const double rc = 1.0 / (2.0 * M_PI * fc_hz);
    return dt / (rc + dt);
  }

  double step(double err, double dt) {
    if (dt <= 0) return 0;

    // raw numerical derivative of error
    double raw_d = 0.0;
    if (!first) {
      raw_d = (err - prev) / dt;
    }

    // update prev/first for next step
    first = false;
    prev = err;

    // low-pass filter the derivative
    const double a = lpf_alpha(dt, DERIV_CUTOFF_HZ);
    if (d_first) {
      d_filt = raw_d;   // initialize to avoid startup spike
      d_first = false;
    } else {
      d_filt = d_filt + a * (raw_d - d_filt);
    }

    return kp*err + kd*d_filt;
  }

  void reset() {
    prev = 0;
    first = true;
    d_filt = 0.0;
    d_first = true;
  }

  void declare(rclcpp::Node* node, const std::string& prefix, double kp_default, double kd_default)
  {
    kp = node->declare_parameter(prefix + ".kp", kp_default);
    kd = node->declare_parameter(prefix + ".kd", kd_default);
  }

  void refresh(rclcpp::Node* node, const std::string& prefix)
  {
    kp = node->get_parameter(prefix + ".kp").as_double();
    kd = node->get_parameter(prefix + ".kd").as_double();
  }
};

// ---------------- Node ----------------
class PIDCascade : public rclcpp::Node
{
public:
  PIDCascade() : Node("low_level_controller")
  {
    // ---------- params (system) ----------
    mass_ = declare_parameter("mass", 0.04338);
    g_    = declare_parameter("g", 9.81);

    max_tilt_ = declare_parameter("max_tilt_rad", 35.0 * M_PI/180.0);
    max_rate_ = declare_parameter("max_rate", 8.0);
    max_tau_  = declare_parameter("max_tau", 0.02);
    max_Fz_   = declare_parameter("max_thrust", 1.0);

    // ---------- params (loop dt) ----------
    dt_pos_  = declare_parameter("dt.pos", 0.01);     // 100 Hz
    dt_vel_  = declare_parameter("dt.vel", 0.01);     // 100 Hz
    dt_att_  = declare_parameter("dt.att", 0.004);    // 250 Hz
    dt_rate_ = declare_parameter("dt.rate", 0.0025);  // 400 Hz

    // ---------- params (PID gains) ----------
    pos_x_.declare(this, "pos.x", 1.5, 0.0);
    pos_y_.declare(this, "pos.y", 1.5, 0.0);
    pos_z_.declare(this, "pos.z", 2.0, 0.0);

    vel_x_.declare(this, "vel.x", 3.0, 0.0);
    vel_y_.declare(this, "vel.y", 3.0, 0.0);
    vel_z_.declare(this, "vel.z", 6.0, 0.0);

    att_r_.declare(this, "att.roll", 6.0, 0.0);
    att_p_.declare(this, "att.pitch", 6.0, 0.0);
    att_y_.declare(this, "att.yaw", 3.0, 0.0);

    rate_r_.declare(this, "rate.roll", 0.002, 0.0);
    rate_p_.declare(this, "rate.pitch", 0.002, 0.0);
    rate_y_.declare(this, "rate.yaw", 0.001, 0.0);

    // ---------- subs ----------
    sub_cmd_ = create_subscription<std_msgs::msg::Float64MultiArray>(
      "/crazyflie/in/pos_cmd", 10,
      std::bind(&PIDCascade::cb_cmd, this, std::placeholders::_1));

    sub_pose_ = create_subscription<geometry_msgs::msg::PoseStamped>(
      "/crazyflie/out/pose", 10,
      std::bind(&PIDCascade::cb_pose, this, std::placeholders::_1));

    sub_vel_ = create_subscription<geometry_msgs::msg::Vector3Stamped>(
      "/crazyflie/out/vel", 10,
      std::bind(&PIDCascade::cb_vel, this, std::placeholders::_1));

    sub_w_ = create_subscription<geometry_msgs::msg::Vector3Stamped>(
      "/crazyflie/out/ang_vel", 10,
      std::bind(&PIDCascade::cb_angvel, this, std::placeholders::_1));

    // ---------- pub ----------
    pub_out_ = create_publisher<std_msgs::msg::Float32MultiArray>(
      "/crazyflie/in/input", 10);

    pub_vdes_ = create_publisher<geometry_msgs::msg::Vector3Stamped>(
      "/crazyflie/debug/v_des", 10);

    pub_rpydes_ = create_publisher<geometry_msgs::msg::Vector3Stamped>(
      "/crazyflie/debug/rpy_des", 10);

    pub_wdes_ = create_publisher<geometry_msgs::msg::Vector3Stamped>(
      "/crazyflie/debug/w_des", 10);

    // ---------- timers ----------
    t_pos_  = create_wall_timer(std::chrono::milliseconds(10),    std::bind(&PIDCascade::loop_pos,  this));
    t_vel_  = create_wall_timer(std::chrono::milliseconds(10),    std::bind(&PIDCascade::loop_vel,  this));
    t_att_  = create_wall_timer(std::chrono::milliseconds(4),     std::bind(&PIDCascade::loop_att,  this));
    t_rate_ = create_wall_timer(std::chrono::microseconds(2500),  std::bind(&PIDCascade::loop_rate, this));

    // initial param refresh
    refresh_params();

    RCLCPP_INFO(get_logger(), "PID cascade controller ready (debug publish enabled).");
    RCLCPP_INFO(get_logger(), "All numerical derivatives inside PIDs are 1st-order low-pass filtered (fc=5Hz).");
  }

private:
  // ---------- callbacks ----------
  void cb_cmd(const std_msgs::msg::Float64MultiArray::SharedPtr m)
  {
    if (m->data.size() < 4) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                           "pos_cmd needs 4 doubles: [x, y, z, yaw]");
      return;
    }
    std::lock_guard<std::mutex> lk(mtx_);
    cmd_pos_ << m->data[0], m->data[1], m->data[2];
    cmd_yaw_ = m->data[3];
    have_cmd_ = true;
  }

  void cb_pose(const geometry_msgs::msg::PoseStamped::SharedPtr m)
  {
    std::lock_guard<std::mutex> lk(mtx_);
    pos_ << m->pose.position.x, m->pose.position.y, m->pose.position.z;

    const double qx = m->pose.orientation.x;
    const double qy = m->pose.orientation.y;
    const double qz = m->pose.orientation.z;
    const double qw = m->pose.orientation.w;

    roll_  = roll_from_quat(qx, qy, qz, qw);
    pitch_ = pitch_from_quat(qx, qy, qz, qw);
    yaw_   = yaw_from_quat(qx, qy, qz, qw);
  }

  void cb_vel(const geometry_msgs::msg::Vector3Stamped::SharedPtr m)
  {
    std::lock_guard<std::mutex> lk(mtx_);
    vel_ << m->vector.x, m->vector.y, m->vector.z;
  }

  void cb_angvel(const geometry_msgs::msg::Vector3Stamped::SharedPtr m)
  {
    std::lock_guard<std::mutex> lk(mtx_);
    w_ << m->vector.x, m->vector.y, m->vector.z;
  }

  // ---------- parameter refresh ----------
  void refresh_params()
  {
    mass_     = get_parameter("mass").as_double();
    g_        = get_parameter("g").as_double();
    max_tilt_ = get_parameter("max_tilt_rad").as_double();
    max_rate_ = get_parameter("max_rate").as_double();
    max_tau_  = get_parameter("max_tau").as_double();
    max_Fz_   = get_parameter("max_thrust").as_double();

    dt_pos_  = get_parameter("dt.pos").as_double();
    dt_vel_  = get_parameter("dt.vel").as_double();
    dt_att_  = get_parameter("dt.att").as_double();
    dt_rate_ = get_parameter("dt.rate").as_double();

    pos_x_.refresh(this, "pos.x");
    pos_y_.refresh(this, "pos.y");
    pos_z_.refresh(this, "pos.z");

    vel_x_.refresh(this, "vel.x");
    vel_y_.refresh(this, "vel.y");
    vel_z_.refresh(this, "vel.z");

    att_r_.refresh(this, "att.roll");
    att_p_.refresh(this, "att.pitch");
    att_y_.refresh(this, "att.yaw");

    rate_r_.refresh(this, "rate.roll");
    rate_p_.refresh(this, "rate.pitch");
    rate_y_.refresh(this, "rate.yaw");
  }

  // ---------- loops ----------
  void loop_pos()
  {
    if (!have_cmd_) return;
    std::lock_guard<std::mutex> lk(mtx_);

    const double dt = std::max(1e-6, dt_pos_);
    v_des_.x() = pos_x_.step(cmd_pos_.x() - pos_.x(), dt);
    v_des_.y() = pos_y_.step(cmd_pos_.y() - pos_.y(), dt);
    v_des_.z() = pos_z_.step(cmd_pos_.z() - pos_.z(), dt);
  }

  void loop_vel()
  {
    if (!have_cmd_) return;
    std::lock_guard<std::mutex> lk(mtx_);

    const double dt = std::max(1e-6, dt_vel_);
    a_des_.x() = vel_x_.step(v_des_.x() - vel_.x(), dt);
    a_des_.y() = vel_y_.step(v_des_.y() - vel_.y(), dt);
    a_des_.z() = vel_z_.step(v_des_.z() - vel_.z(), dt);
  }

  void loop_att()
  {
    if (!have_cmd_) return;
    std::lock_guard<std::mutex> lk(mtx_);

    const double roll_d  = clamp(-a_des_.y() / std::max(1e-9, g_), -max_tilt_, max_tilt_);
    const double pitch_d = clamp( a_des_.x() / std::max(1e-9, g_), -max_tilt_, max_tilt_);
    const double yaw_d   = cmd_yaw_;

    roll_d_  = roll_d;
    pitch_d_ = pitch_d;
    yaw_d_   = yaw_d;

    const double dt = std::max(1e-6, dt_att_);

    w_des_.x() = att_r_.step(wrap_pi(roll_d  - roll_),  dt);
    w_des_.y() = att_p_.step(wrap_pi(pitch_d - pitch_), dt);
    w_des_.z() = att_y_.step(wrap_pi(yaw_d   - yaw_),   dt);

    w_des_.x() = clamp(w_des_.x(), -max_rate_, max_rate_);
    w_des_.y() = clamp(w_des_.y(), -max_rate_, max_rate_);
    w_des_.z() = clamp(w_des_.z(), -max_rate_, max_rate_);
  }

  void loop_rate()
  {
    if (!have_cmd_) return;

    std::lock_guard<std::mutex> lk(mtx_);

    refresh_params();
    const rclcpp::Time now = this->get_clock()->now();
    builtin_interfaces::msg::Time stamp;
    stamp.sec = static_cast<int32_t>(now.seconds());
    stamp.nanosec = static_cast<uint32_t>(now.nanoseconds() % 1000000000LL);

    const double dt = std::max(1e-6, dt_rate_);

    const double tau_x = clamp(rate_r_.step(w_des_.x() - w_.x(), dt), -max_tau_, max_tau_);
    const double tau_y = clamp(rate_p_.step(w_des_.y() - w_.y(), dt), -max_tau_, max_tau_);
    const double tau_z = clamp(rate_y_.step(w_des_.z() - w_.z(), dt), -max_tau_, max_tau_);

    const double Fz = clamp(mass_ * (g_ + a_des_.z()), 0.0, max_Fz_);

    std_msgs::msg::Float32MultiArray out;
    out.data = { (float)tau_x, (float)tau_y, (float)tau_z, (float)Fz };
    pub_out_->publish(out);

    // -------- debug publish (same stamp) --------
    geometry_msgs::msg::Vector3Stamped vmsg;
    vmsg.header.stamp = stamp;
    vmsg.header.frame_id = "world";
    vmsg.vector.x = v_des_.x();
    vmsg.vector.y = v_des_.y();
    vmsg.vector.z = v_des_.z();
    pub_vdes_->publish(vmsg);

    geometry_msgs::msg::Vector3Stamped rpymsg;
    rpymsg.header.stamp = stamp;
    rpymsg.header.frame_id = "world";
    rpymsg.vector.x = roll_d_;
    rpymsg.vector.y = pitch_d_;
    rpymsg.vector.z = yaw_d_;
    pub_rpydes_->publish(rpymsg);

    geometry_msgs::msg::Vector3Stamped wmsg;
    wmsg.header.stamp = stamp;
    wmsg.header.frame_id = "body";
    wmsg.vector.x = w_des_.x();
    wmsg.vector.y = w_des_.y();
    wmsg.vector.z = w_des_.z();
    pub_wdes_->publish(wmsg);
  }

  // ---------- state ----------
  std::mutex mtx_;
  bool have_cmd_{false};

  Eigen::Vector3d cmd_pos_{0,0,0}, pos_{0,0,0}, vel_{0,0,0};
  Eigen::Vector3d v_des_{0,0,0}, a_des_{0,0,0};
  Eigen::Vector3d w_{0,0,0}, w_des_{0,0,0};

  double roll_{0}, pitch_{0}, yaw_{0}, cmd_yaw_{0};

  // PIDs
  PID pos_x_, pos_y_, pos_z_;
  PID vel_x_, vel_y_, vel_z_;
  PID att_r_, att_p_, att_y_;
  PID rate_r_, rate_p_, rate_y_;

  // params
  double mass_{0.04338}, g_{9.81};
  double max_tilt_{35.0*M_PI/180.0}, max_rate_{8.0}, max_tau_{0.02}, max_Fz_{1.0};
  double dt_pos_{0.01}, dt_vel_{0.01}, dt_att_{0.004}, dt_rate_{0.0025};

  // desired rpy (computed in loop_att, published in loop_rate)
  double roll_d_{0.0}, pitch_d_{0.0}, yaw_d_{0.0};

  // debug pubs
  rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr pub_vdes_;
  rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr pub_rpydes_;
  rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr pub_wdes_;

  // ROS
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr sub_cmd_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr sub_pose_;
  rclcpp::Subscription<geometry_msgs::msg::Vector3Stamped>::SharedPtr sub_vel_;
  rclcpp::Subscription<geometry_msgs::msg::Vector3Stamped>::SharedPtr sub_w_;
  rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr pub_out_;

  rclcpp::TimerBase::SharedPtr t_pos_, t_vel_, t_att_, t_rate_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PIDCascade>());
  rclcpp::shutdown();
  return 0;
}

