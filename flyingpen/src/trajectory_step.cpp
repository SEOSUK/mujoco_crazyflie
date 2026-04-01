// trajectory_step.cpp
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>

#include <cmath>
#include <chrono>
#include <algorithm>
#include <string>

using namespace std::chrono_literals;

static inline double clamp(double v, double lo, double hi)
{
  return std::max(lo, std::min(hi, v));
}

static inline double wrap_pi(double a)
{
  while (a > M_PI) a -= 2 * M_PI;
  while (a < -M_PI) a += 2 * M_PI;
  return a;
}

class TrajectoryStep : public rclcpp::Node
{
public:
  TrajectoryStep() : Node("trajectory_step")
  {
    rate_hz_ = declare_parameter("rate_hz", 50.0);

    x0_ = declare_parameter("x0", 0.0);
    y0_ = declare_parameter("y0", 0.0);
    z0_ = declare_parameter("z0", 0.5);
    yaw0_ = declare_parameter("yaw0", 0.0);

    hover_time_ = declare_parameter("hover_time", 10.0);

    step_amp_ = declare_parameter("step_amp", 0.3);       // +/- 0.3 m
    step_period_ = declare_parameter("step_period", 5.0); // 5 sec each side

    topic_ = declare_parameter("topic", std::string("/crazyflie/in/pos_cmd"));

    pub_ = create_publisher<std_msgs::msg::Float64MultiArray>(topic_, 10);

    start_time_ = now();

    const auto period_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::duration<double>(1.0 / std::max(1e-6, rate_hz_))
    );
    timer_ = create_wall_timer(period_ns, std::bind(&TrajectoryStep::tick, this));

    RCLCPP_INFO(
      get_logger(),
      "trajectory_step started. Publishing to %s (hover %.2fs then x step +/- %.2f every %.2fs).",
      topic_.c_str(), hover_time_, step_amp_, step_period_);
  }

private:
  void tick()
  {
    const rclcpp::Time t_now = now();
    const double t = (t_now - start_time_).seconds();

    double x = x0_;
    double y = y0_;
    double z = z0_;
    double yaw = yaw0_;

    if (t < hover_time_) {
      x = x0_;
      y = y0_;
      z = z0_;
      yaw = yaw0_;
    } else {
      const double tau = t - hover_time_;

      // 0,1,2,3,... every step_period_ seconds
      const long long k = static_cast<long long>(std::floor(tau / std::max(1e-6, step_period_)));

      // even -> +amp, odd -> -amp
      const double sign = (k % 2 == 0) ? 1.0 : -1.0;

      x = x0_ + sign * step_amp_;
      y = y0_;
      z = z0_;
      yaw = wrap_pi(yaw0_);
    }

    std_msgs::msg::Float64MultiArray msg;
    msg.data = {x, y, z, yaw};
    pub_->publish(msg);
  }

private:
  double rate_hz_{50.0};

  double x0_{0.0}, y0_{0.0}, z0_{0.5}, yaw0_{0.0};
  double hover_time_{10.0};

  double step_amp_{0.3};
  double step_period_{5.0};

  std::string topic_{"/crazyflie/in/pos_cmd"};

  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Time start_time_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TrajectoryStep>());
  rclcpp::shutdown();
  return 0;
}
