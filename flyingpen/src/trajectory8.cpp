// trajectory8.cpp
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>

#include <cmath>
#include <chrono>
#include <algorithm>

using namespace std::chrono_literals;

static inline double clamp(double v, double lo, double hi)
{ return std::max(lo, std::min(hi, v)); }

static inline double wrap_pi(double a)
{
  while (a > M_PI) a -= 2*M_PI;
  while (a < -M_PI) a += 2*M_PI;
  return a;
}

class Trajectory8 : public rclcpp::Node
{
public:
  Trajectory8() : Node("trajectory8")
  {
    // ---- parameters ----
    // publish
    rate_hz_ = declare_parameter("rate_hz", 50.0);

    // initial hover pose
    x0_ = declare_parameter("x0", 0.0);
    y0_ = declare_parameter("y0", 0.0);
    z0_ = declare_parameter("z0", 0.5);
    yaw0_ = declare_parameter("yaw0", 0.0);

    hover_time_ = declare_parameter("hover_time", 3.0);  // seconds

    // 8-trajectory settings (Lemniscate of Gerono)
    A_ = declare_parameter("A", 0.4);       // x amplitude [m]
    B_ = declare_parameter("B", 0.4);       // y amplitude [m]
    freq_ = declare_parameter("freq", 0.1); // Hz (cycle speed)
    phase_ = declare_parameter("phase", 0.0); // rad

    // optional z motion
    z_sine_enable_ = declare_parameter("z_sine_enable", false);
    z_amp_ = declare_parameter("z_amp", 0.05);     // [m]
    z_freq_ = declare_parameter("z_freq", 0.2);    // [Hz]

    // yaw options
    yaw_follow_velocity_ = declare_parameter("yaw_follow_velocity", false);
    yaw_rate_limit_ = declare_parameter("yaw_rate_limit", 2.0); // rad/s
    yaw_smooth_alpha_ = declare_parameter("yaw_smooth_alpha", 0.2); // 0~1

    // topic
    topic_ = declare_parameter("topic", std::string("/crazyflie/in/pos_cmd"));

    pub_ = create_publisher<std_msgs::msg::Float64MultiArray>(topic_, 10);

    start_time_ = now();
    last_time_ = start_time_;
    last_yaw_cmd_ = yaw0_;

    // init dt LPF states
    dt_filt_ = 0.0;
    dt_first_ = true;

    const auto period_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::duration<double>(1.0 / std::max(1e-6, rate_hz_))
    );
    timer_ = create_wall_timer(period_ns, std::bind(&Trajectory8::tick, this));

    RCLCPP_INFO(get_logger(),
      "trajectory8 started. Publishing to %s (hover %.2fs then figure-8).",
      topic_.c_str(), hover_time_);
    RCLCPP_INFO(get_logger(),
      "dt is low-pass filtered (1st order) at fc=5Hz for yaw rate limiting.");
  }

private:
  void tick()
  {
    // refresh params lightly (optional)
    rate_hz_ = get_parameter("rate_hz").as_double();

    const rclcpp::Time t_now = now();
    const double t = (t_now - start_time_).seconds();

    double x = x0_, y = y0_, z = z0_;
    double yaw = yaw0_;

    // ---------- dt (raw) ----------
    const double dt_raw = std::max(1e-6, (t_now - last_time_).seconds());
    last_time_ = t_now;

    // ---------- dt LPF (1st order, fc=5Hz) ----------
    constexpr double fc = 5.0;  // Hz
    const double rc = 1.0 / (2.0 * M_PI * fc);
    const double alpha = dt_raw / (rc + dt_raw);

    if (dt_first_) {
      dt_filt_ = dt_raw;   // init to avoid startup spike
      dt_first_ = false;
    } else {
      dt_filt_ = dt_filt_ + alpha * (dt_raw - dt_filt_);
    }

    // (optional) safety clamp on filtered dt (prevents huge jump after pause)
    const double dt = clamp(dt_filt_, 0.0, 0.05);  // up to 50ms

    if (t < hover_time_) {
      // hover phase: hold initial pose
      x = x0_; y = y0_; z = z0_; yaw = yaw0_;
      last_yaw_cmd_ = yaw0_;
    } else {
      const double tau = t - hover_time_;
      const double w = 2.0 * M_PI * freq_;
      const double th = w * tau + phase_;

      // figure-8 (Gerono)
      const double s = std::sin(th);
      const double c = std::cos(th);

      x = x0_ + A_ * s;
      y = y0_ + B_ * s * c; // (B/2)*sin(2th)

      if (z_sine_enable_) {
        const double wz = 2.0 * M_PI * z_freq_;
        z = z0_ + z_amp_ * std::sin(wz * tau);
      } else {
        z = z0_;
      }

      if (yaw_follow_velocity_) {
        // desired yaw = direction of motion in XY
        // analytic derivatives:
        // xdot = A*w*cos(th)
        // ydot = B*w*cos(2th)
        const double xdot = A_ * w * c;
        const double ydot = B_ * w * std::cos(2.0 * th);

        double yaw_des = yaw0_;
        if (std::hypot(xdot, ydot) > 1e-6) {
          yaw_des = std::atan2(ydot, xdot);
        }

        // limit yaw rate + smooth
        double yaw_err  = wrap_pi(yaw_des - last_yaw_cmd_);
        double yaw_step = clamp(yaw_err, -yaw_rate_limit_ * dt, yaw_rate_limit_ * dt);
        double yaw_next = wrap_pi(last_yaw_cmd_ + yaw_step);

        // low-pass smoothing (alpha 0~1; higher=more responsive)
        yaw = wrap_pi((1.0 - yaw_smooth_alpha_) * last_yaw_cmd_ + yaw_smooth_alpha_ * yaw_next);
        last_yaw_cmd_ = yaw;
      } else {
        yaw = yaw0_;
        last_yaw_cmd_ = yaw0_;
      }
    }

    std_msgs::msg::Float64MultiArray msg;
    msg.data = {x, y, z, yaw};
    pub_->publish(msg);
  }

private:
  // params
  double rate_hz_{50.0};

  double x0_{0.0}, y0_{0.0}, z0_{0.5}, yaw0_{0.0};
  double hover_time_{3.0};

  double A_{0.4}, B_{0.4};
  double freq_{0.1};
  double phase_{0.0};

  bool z_sine_enable_{false};
  double z_amp_{0.05};
  double z_freq_{0.2};

  bool yaw_follow_velocity_{false};
  double yaw_rate_limit_{2.0};
  double yaw_smooth_alpha_{0.2};

  std::string topic_{"/crazyflie/in/pos_cmd"};

  // dt LPF state
  double dt_filt_{0.0};
  bool dt_first_{true};

  // ROS
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Time start_time_;
  rclcpp::Time last_time_;
  double last_yaw_cmd_{0.0};
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Trajectory8>());
  rclcpp::shutdown();
  return 0;
}
