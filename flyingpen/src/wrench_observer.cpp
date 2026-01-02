// wrench_observer.cpp
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>

#include <mutex>
#include <chrono>

using namespace std::chrono_literals;

class WrenchObserver : public rclcpp::Node
{
public:
  WrenchObserver()
  : Node("wrench_observer")
  {
    // ============================================================
    // TODO
    //   - (sub) /crazyflie/in/input     : Float32MultiArray  [tau_x, tau_y, tau_z, Fz] 커맨드임
    //   - (sub) /crazyflie/out/pose     : PoseStamped                                  자세피드백임
    //   - (sub) /crazyflie/out/vel      : Vector3Stamped                               속도피드백임
    //
    //    너가 만들어야할 출력.
    //   - (pub) /crazyflie/out/mob_wrench : Float32MultiArray [Fx,Fy,Fz]
    //   - (pub) /crazyflie/out/dob_wrench : Float32MultiArray [Fx,Fy,Fz]
    //     ※ 외력추정 결과는 world frame에서 정의할 것!
    // ============================================================

    // -------------------------
    // Publishers (TEMP)
    // -------------------------
    pub_mob_ = this->create_publisher<std_msgs::msg::Float32MultiArray>(
      "/crazyflie/out/mob_wrench", 10);

    // -------------------------
    // Timers (observer update loop)
    // -------------------------
    // 권장: plant pub_hz(예: 400Hz)와 맞추거나, 200~500Hz 범위
    timer_est_ = this->create_wall_timer(
      5ms, std::bind(&WrenchObserver::loop_est, this));  // 200 Hz

    RCLCPP_INFO(this->get_logger(), "wrench_observer started (TEMP mob output).");
  }

private:
  // =========================
  // Timer callback (main update)
  // =========================
  void loop_est()
  {
    // dt 계산 (wall time 기반)
    const rclcpp::Time now = this->now();
    double dt = 0.0;
    if (last_time_.nanoseconds() != 0) {
      dt = (now - last_time_).seconds();
      // SU: dt 필요하면 쓰고, 아니면 무시해도 됨
    }
    last_time_ = now;

    // dt 튀는 구간 보호
    if (dt <= 1e-6 || dt > 0.1) {
      return;
    }

    // -------------------------
    std_msgs::msg::Float32MultiArray mob_msg;
    mob_msg.data.resize(3);
    mob_msg.data[0] = 0.01f;  // Fx
    mob_msg.data[1] = 0.0f;  // Fy
    mob_msg.data[2] = 0.0f;  // Fz

    pub_mob_->publish(mob_msg);
  }

  // =========================
  // Internal state
  // =========================
  std::mutex mtx_;

  // Publisher
  rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr pub_mob_;

  // Timer
  rclcpp::TimerBase::SharedPtr timer_est_;
  rclcpp::Time last_time_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<WrenchObserver>());
  rclcpp::shutdown();
  return 0;
}
