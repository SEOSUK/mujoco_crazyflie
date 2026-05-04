#include <algorithm>
#include <array>
#include <cmath>
#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>

namespace {

constexpr double kPi = 3.14159265358979323846;

class Trajectory8Node : public rclcpp::Node {
public:
  Trajectory8Node()
  : Node("trajectory8") {
    declare_parameter("publish_hz", 50.0);
    declare_parameter("rise_time", 5.0);
    declare_parameter("target_z", 2.0);
    declare_parameter("xy_amplitude", 1.0);
    declare_parameter("figure_period", 12.0);
    declare_parameter("yaw", 0.0);

    const double publish_hz = get_parameter("publish_hz").as_double();
    rise_time_ = get_parameter("rise_time").as_double();
    target_z_ = get_parameter("target_z").as_double();
    xy_amplitude_ = get_parameter("xy_amplitude").as_double();
    figure_period_ = get_parameter("figure_period").as_double();
    yaw_ = get_parameter("yaw").as_double();

    pub_pos_cmd_ = create_publisher<std_msgs::msg::Float64MultiArray>(
      "/crazyflie/in/pos_cmd", 10);

    start_time_ = now();
    const auto period = std::chrono::duration<double>(1.0 / std::max(1.0, publish_hz));
    timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::milliseconds>(period),
      std::bind(&Trajectory8Node::publishCommand, this));

    RCLCPP_INFO(
      get_logger(),
      "Publishing /crazyflie/in/pos_cmd: rise to %.2fm in %.1fs, then figure-eight in XY plane.",
      target_z_, rise_time_);
  }

private:
  void publishCommand() {
    const double elapsed = (now() - start_time_).seconds();

    double x = 0.0;
    double y = 0.0;
    double z = target_z_;

    if (elapsed < rise_time_) {
      const double alpha = std::clamp(elapsed / std::max(1e-6, rise_time_), 0.0, 1.0);
      z = target_z_ * alpha;
    } else {
      const double t = elapsed - rise_time_;
      const double omega = 2.0 * kPi / std::max(1e-6, figure_period_);
      const double s = std::sin(omega * t);
      const double c = std::cos(omega * t);
      x = xy_amplitude_ * s;
      y = xy_amplitude_ * s * c;
    }

    std_msgs::msg::Float64MultiArray msg;
    msg.data = {x, y, z, yaw_};
    pub_pos_cmd_->publish(msg);
  }

  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr pub_pos_cmd_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Time start_time_;

  double rise_time_ {5.0};
  double target_z_ {2.0};
  double xy_amplitude_ {1.0};
  double figure_period_ {12.0};
  double yaw_ {0.0};
};

}  // namespace

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<Trajectory8Node>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
