#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <array>
#include <algorithm>

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

    // -------------------------
    // Publisher (final output)
    // -------------------------
    pub_pos_cmd_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(
      "/crazyflie/in/pos_cmd", 10);

    // -------------------------
    // Update loop
    // -------------------------
    timer_ = this->create_wall_timer(
      10ms, std::bind(&TrajectoryGeneration::update, this)); // 100 Hz

    RCLCPP_INFO(this->get_logger(), "trajectory_generation started");
  }

private:
  // =========================
  // Callbacks
  // =========================
  void keyboardCb(const std_msgs::msg::Float64MultiArray::SharedPtr msg)
  {
    if (msg->data.size() < 4) return;

    sp_in_[0] = msg->data[0];
    sp_in_[1] = msg->data[1];
    sp_in_[2] = msg->data[2];
    sp_in_yaw_ = msg->data[3]; // deg or deg/s depending on mode

    sp_received_ = true;
  }

  void useVelModeCb(const std_msgs::msg::Float32::SharedPtr msg)
  {
    su_cmd_use_vel_mode_ = msg->data;
  }

  void cmdForceCb(const std_msgs::msg::Float32::SharedPtr msg)
  {
    su_cmd_fx_ = msg->data; // F_des_x (저장만)
  }

  // =========================
  // Main update loop
  // =========================
  void update()
  {
    if (!sp_received_) {
      return; // 입력 아직 없음
    }

    const rclcpp::Time now = this->now();
    double dt = 0.0;
    if (last_time_.nanoseconds() != 0) {
      dt = (now - last_time_).seconds();
    }
    last_time_ = now;

    // dt 튀는 구간은 리셋 (원본 코드 스타일)
    if (dt <= 1e-5 || dt > 0.1) {
      // 첫 호출/이상 dt: 내부 초기화는 다음 정상 tick에서 하도록
      publishOut(); // 그래도 일단 현재 상태(내부값) 혹은 초기값 송신
      return;
    }

    const bool vel_mode_on = (su_cmd_use_vel_mode_ > 0.5f);

    // 0) 첫 정상 tick에서 내부 적분 상태 초기화
    if (!su_int_initialized_) {
      // 펌웨어는 state 기반 초기화인데,
      // 여기서는 "현재 입력 setpoint"를 기준으로 초기화하는 게 자연스러움.
      if (!vel_mode_on) {
        su_int_pos_ = sp_in_;
        su_int_yaw_ = sp_in_yaw_;
      } else {
        // vel 모드로 시작하면 현재 위치를 모르니 0에서 시작 (필요시 외부 state 토픽 추가 권장)
        su_int_pos_ = {0.0, 0.0, 0.0};
        su_int_yaw_ = 0.0;
      }
      su_int_initialized_ = true;
      su_vel_mode_prev_ = vel_mode_on;
      su_pos_base_valid_ = false;
    }

    // 1) 모드 전환 감지 (원본 로직 동일)
    if (vel_mode_on && !su_vel_mode_prev_) {
      // pos -> vel 전환: 내부 적분 상태 유지, base invalidate
      su_pos_base_valid_ = false;

      // (Admittance 상태가 있다면 여기서 리셋하면 됨)
      // adm_reset();

    } else if (!vel_mode_on && su_vel_mode_prev_) {
      // vel -> pos 전환: 기준점 저장 후, pos 입력을 "변위"로 해석
      su_pos_base_ = su_int_pos_;
      su_yaw_base_ = su_int_yaw_;
      su_pos_base_valid_ = true;

      // (Admittance 상태가 있다면 여기서도 리셋)
      // adm_reset();
    }
    su_vel_mode_prev_ = vel_mode_on;

    // 2) 모드별 처리 (원본 핵심)
    if (!vel_mode_on) {
      // --------------------------
      // Position mode
      // --------------------------
      if (su_pos_base_valid_) {
        // vel -> pos 이후: position/yaw = base + delta
        su_int_pos_[0] = su_pos_base_[0] + sp_in_[0];
        su_int_pos_[1] = su_pos_base_[1] + sp_in_[1];
        su_int_pos_[2] = su_pos_base_[2] + sp_in_[2];
        su_int_yaw_    = su_yaw_base_    + sp_in_yaw_;
      } else {
        // 초기 pos 모드: absolute
        su_int_pos_ = sp_in_;
        su_int_yaw_ = sp_in_yaw_;
      }

    } else {
      // --------------------------
      // Velocity mode
      // --------------------------
      double vx_cmd   = sp_in_[0];
      double vy_cmd   = sp_in_[1];
      double vz_cmd   = sp_in_[2];
      double vyaw_cmd = sp_in_yaw_; // deg/s

      // ----- Admittance 훅 -----
      // 원본은 (F_des - F_meas) 및 F_dot 필요.
      // 현재 노드 입력에는 F_meas(외력 추정)가 없어서 적용 불가.
      // 추후 예: "/su/f_ext_world" 같은 토픽(Float64MultiArray[3])을 추가하면 여기서 반영하면 됨.
      //
      // const double F_des_x = su_cmd_fx_;
      // const double F_meas_x = f_ext_world_[0];
      // vx_cmd += v_adm_x;

      // ----- 적분 -----
      su_int_pos_[0] += vx_cmd   * dt;
      su_int_pos_[1] += vy_cmd   * dt;
      su_int_pos_[2] += vz_cmd   * dt;
      su_int_yaw_    += vyaw_cmd * dt;
    }

    // 3) 최종 출력: 항상 absolute pos/yaw
    publishOut();
  }

  void publishOut()
  {
    std_msgs::msg::Float64MultiArray out;
    out.data.resize(4);
    out.data[0] = su_int_pos_[0];
    out.data[1] = su_int_pos_[1];
    out.data[2] = su_int_pos_[2];
    out.data[3] = su_int_yaw_;
    pub_pos_cmd_->publish(out);
  }

  // =========================
  // ROS interfaces
  // =========================
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr sub_keyboard_;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr          sub_use_vel_mode_;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr          sub_cmd_force_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr   pub_pos_cmd_;
  rclcpp::TimerBase::SharedPtr                                     timer_;

  // =========================
  // Internal state (su_cmd_integrator 대응)
  // =========================
  // inputs (sp_in)
  std::array<double, 3> sp_in_{0.0, 0.0, 0.0};
  double sp_in_yaw_{0.0};
  bool sp_received_{false};

  // params/commands
  float su_cmd_use_vel_mode_{1.0f}; // default: vel mode
  float su_cmd_fx_{0.0f};           // F_des_x (저장만)

  // integrator state
  std::array<double, 3> su_int_pos_{0.0, 0.0, 0.0};
  double su_int_yaw_{0.0};
  bool su_int_initialized_{false};

  // mode switching bookkeeping
  bool su_vel_mode_prev_{false};

  // vel->pos 이후 변위 명령을 위한 기준점
  std::array<double, 3> su_pos_base_{0.0, 0.0, 0.0};
  double su_yaw_base_{0.0};
  bool su_pos_base_valid_{false};

  rclcpp::Time last_time_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TrajectoryGeneration>());
  rclcpp::shutdown();
  return 0;
}

