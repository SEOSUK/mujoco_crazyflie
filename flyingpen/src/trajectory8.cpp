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

    // user requested: hover 10 sec
    hover_time_ = declare_parameter("hover_time", 10.0);

    // user requested: x amp 0.5, y amp 0.5, freq 0.1
    A_ = declare_parameter("A", 0.4);          // x amplitude [m]
    B_ = declare_parameter("B", 0.4);          // y amplitude [m]
    freq_ = declare_parameter("freq", 0.1);    // Hz
    phase_ = declare_parameter("phase", 0.0);  // rad

    // optional z motion
    z_sine_enable_ = declare_parameter("z_sine_enable", true);
    z_amp_ = declare_parameter("z_amp", 0.1);     
    z_freq_ = declare_parameter("z_freq", 0.2);

    // yaw sine motion
    yaw_sine_enable_ = declare_parameter("yaw_sine_enable", true);
    yaw_amp_deg_ = declare_parameter("yaw_amp_deg", 45.0);   // +-45 deg
    yaw_freq_ = declare_parameter("yaw_freq", 0.1);          // Hz
    yaw_phase_ = declare_parameter("yaw_phase", 0.0);        // rad

    // topic
    topic_ = declare_parameter("topic", std::string("/crazyflie/in/pos_cmd"));

    pub_ = create_publisher<std_msgs::msg::Float64MultiArray>(topic_, 10);

    start_time_ = now();

    const auto period_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::duration<double>(1.0 / std::max(1e-6, rate_hz_))
    );
    timer_ = create_wall_timer(period_ns, std::bind(&Trajectory8::tick, this));

    RCLCPP_INFO(get_logger(),
      "trajectory8 started. Publishing to %s (hover %.2fs then figure-8 with yaw sine).",
      topic_.c_str(), hover_time_);
  }

private:
  void tick()
  {
    rate_hz_ = get_parameter("rate_hz").as_double();

    const rclcpp::Time t_now = now();
    const double t = (t_now - start_time_).seconds();

    double x = x0_, y = y0_, z = z0_;
    double yaw = yaw0_;

    if (t < hover_time_) {
      // hover phase
      x = x0_;
      y = y0_;
      z = z0_;
      yaw = yaw0_;
    } else {
      const double tau = t - hover_time_;

      // XY figure-8
      const double w = 2.0 * M_PI * freq_;
      const double th = w * tau + phase_;

      const double s = std::sin(th);
      const double c = std::cos(th);

      x = x0_ + A_ * s;
      y = y0_ + B_ * s * c;   // Gerono figure-8

      // Z motion
      if (z_sine_enable_) {
        const double wz = 2.0 * M_PI * z_freq_;
        z = z0_ + z_amp_ * std::sin(wz * tau);
      } else {
        z = z0_;
      }

      // Yaw sine motion: yaw = yaw0 + 45deg * sin(2*pi*0.2*t)
      if (yaw_sine_enable_) {
        const double yaw_amp_rad = yaw_amp_deg_ * M_PI / 180.0;
        const double wyaw = 2.0 * M_PI * yaw_freq_;
        yaw = wrap_pi(yaw0_ + yaw_amp_rad * std::sin(wyaw * tau + yaw_phase_));
      } else {
        yaw = yaw0_;
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
  double hover_time_{10.0};

  double A_{0.5}, B_{0.5};
  double freq_{0.1};
  double phase_{0.0};

  bool z_sine_enable_{false};
  double z_amp_{0.1};
  double z_freq_{0.2};

  bool yaw_sine_enable_{true};
  double yaw_amp_deg_{45.0};
  double yaw_freq_{0.2};
  double yaw_phase_{0.0};

  std::string topic_{"/crazyflie/in/pos_cmd"};

  // ROS
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Time start_time_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Trajectory8>());
  rclcpp::shutdown();
  return 0;
}
