/**
 * @file trajectory_generation.cpp
 *
 * @brief
 * Trajectory generation node that fuses user commands and (future) force feedback
 * to generate final position/yaw commands for the Crazyflie low-level controller.
 *
 * ------------------------------------------------------------
 * Overview
 * ------------------------------------------------------------
 * This node acts as a high-level command integrator between:
 *
 *   - User inputs (keyboard, teleoperation)
 *   - Mode switching (position mode / velocity mode)
 *   - (Future) force feedback for admittance-based interaction control
 *
 * The output of this node is a **continuous absolute position + yaw command**
 * published to:
 *
 *   /crazyflie/in/pos_cmd  [Float64MultiArray: x, y, z, yaw]
 *
 * which is consumed by the low-level PID cascade controller.
 *
 * ------------------------------------------------------------
 * Input Interfaces
 * ------------------------------------------------------------
 * 1) /su/keyboard_input  (Float64MultiArray)
 *    - In position mode:
 *        [x, y, z, yaw] are interpreted as absolute position/yaw commands.
 *    - In velocity mode:
 *        [vx, vy, vz, yaw_rate] are interpreted as velocity commands.
 *
 * 2) /su/use_vel_mode    (Float32)
 *    - 0.0 : Position mode
 *    - 1.0 : Velocity mode
 *
 * 3) /su/cmd_force       (Float32)
 *    - Desired interaction force (currently stored only).
 *    - Intended for future admittance control integration.
 *
 * ------------------------------------------------------------
 * Internal Logic
 * ------------------------------------------------------------
 * - The node maintains an internal integrator state (su_int_pos_, su_int_yaw_)
 *   that always represents the **absolute commanded pose**.
 *
 * - Position mode:
 *     The incoming command is treated as an absolute pose, or as a delta
 *     relative to a stored base pose when switching from velocity mode.
 *
 * - Velocity mode:
 *     The incoming command is integrated over time to update the internal
 *     position and yaw state.
 *
 * - Mode switching:
 *     - pos -> vel : internal integrator is preserved.
 *     - vel -> pos : current internal pose is stored as a base reference,
 *                    and subsequent position commands are interpreted as offsets.
 *
 * ------------------------------------------------------------
 * Future Extension: Admittance Control
 * ------------------------------------------------------------
 * This node is designed to be extended with admittance control by incorporating
 * measured external force feedback, e.g.:
 *
 *   /su/f_ext_world  (external force estimate in world frame)
 *
 * The admittance logic can be injected in the velocity-mode section by modifying
 * the commanded velocity based on force error:
 *
 *   v_cmd += v_admittance(F_des - F_meas)
 *
 * This design keeps the admittance logic decoupled from the low-level controller,
 * allowing safe and modular experimentation with interaction control strategies.
 *
 * ------------------------------------------------------------
 * Design Philosophy
 * ------------------------------------------------------------
 * - High-level command shaping (integration, mode logic, admittance)
 *   is handled here.
 * - Low-level stability and tracking are handled entirely by the PID cascade.
 * - All outputs are absolute pose commands, simplifying downstream control.
 */

 
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <geometry_msgs/msg/wrench_stamped.hpp>
#include <array>
#include <algorithm>
#include <cmath>
#include <mutex>


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
    // Publisher (final output)
    // -------------------------
    pub_pos_cmd_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(
      "/crazyflie/in/pos_cmd", 10);

    pub_force_lpf_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(
      "/su/force_lpf", 10);

    

    // -------------------------
    // Update loop
    // -------------------------
    timer_ = this->create_wall_timer(
      10ms, std::bind(&TrajectoryGeneration::update, this)); // 100 Hz
    
    // force/admittance update (분리)
    force_timer_ = this->create_wall_timer(
      10ms, std::bind(&TrajectoryGeneration::forceUpdate, this));  // 100 Hz 

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
    std::lock_guard<std::mutex> lk(force_mtx_);
    su_cmd_fx_ = msg->data; // F_des_x (저장만)
  }

  void contactForceCb(const geometry_msgs::msg::WrenchStamped::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lk(force_mtx_);
    contact_F_[0] = msg->wrench.force.x;
    contact_F_[1] = msg->wrench.force.y;
    contact_F_[2] = msg->wrench.force.z;

    f_ext_received_ = true;
  }


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


    // --- shared snapshot ---
    float su_cmd_fx_local = 0.0f;
    std::array<double,3> contact_F_local{0.0,0.0,0.0};
    bool  f_ok = false;

    {
      std::lock_guard<std::mutex> lk(force_mtx_);
      su_cmd_fx_local = su_cmd_fx_;
      contact_F_local = contact_F_;   
      f_ok = f_ext_received_;
    }
    if (!f_ok) return;

    // --- dt ---
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

    // --- e_F 정의 ---
    std::array<double,3> eF{0.0,0.0,0.0};                              
    eF[0] = static_cast<double>(su_cmd_fx_local) - contact_F_local[0];   // x축
    eF[1] = 0.0 - contact_F_local[1];                                    // y축
    eF[2] = 0.0 - contact_F_local[2];                                    // z축

    // --- init ---
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

    // --- numeric diff ---
    std::array<double,3> eF_dot_raw{0.0,0.0,0.0};
    eF_dot_raw[0] = (eF[0] - eF_prev_[0]) / dt;        
    eF_dot_raw[1] = (eF[1] - eF_prev_[1]) / dt;
    eF_dot_raw[2] = (eF[2] - eF_prev_[2]) / dt; 

    // --- LPF on dot(e_F) ---
    const double wc = 3.0;                 // Hz
    const double alpha = wc * dt;           // Euler
    const double a = std::clamp(alpha, 0.0, 1.0);

    std::array<double,3> eF_dot_filt{0.0,0.0,0.0}; 
    eF_dot_filt[0] = (1.0 - a) * eF_dot_filt_prev_[0] + a * eF_dot_raw[0];
    eF_dot_filt[1] = (1.0 - a) * eF_dot_filt_prev_[1] + a * eF_dot_raw[1];
    eF_dot_filt[2] = (1.0 - a) * eF_dot_filt_prev_[2] + a * eF_dot_raw[2];

    // --- state update ---
    eF_prev_ = eF;
    eF_dot_filt_prev_ = eF_dot_filt;

    {
      std::lock_guard<std::mutex> lk(force_mtx_);
      eF_ = eF;
      eF_dot_filt_ = eF_dot_filt;
    }

    // --- debug publish ---
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



      std::array<double,3> eF{0.0, 0.0, 0.0};
      std::array<double,3> eF_dot_filt{0.0, 0.0, 0.0};

      {
        std::lock_guard<std::mutex> lk(force_mtx_);
        eF = eF_;
        eF_dot_filt = eF_dot_filt_;
      }

      // x축만 adm 구현
      const double K_p = 3.0;
      const double K_d = 0.005;
      const double vel_adm_x = K_p * eF[0] + K_d * eF_dot_filt[0];

      // 실제 속도 명령에 반영
      vx_cmd += vel_adm_x;




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
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr  sub_keyboard_;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr            sub_use_vel_mode_;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr            sub_cmd_force_;
  rclcpp::Subscription<geometry_msgs::msg::WrenchStamped>::SharedPtr sub_contact_force_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr     pub_pos_cmd_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr     pub_force_lpf_;
  rclcpp::TimerBase::SharedPtr                                       timer_;
  rclcpp::TimerBase::SharedPtr                                       force_timer_;


  // =========================
  // Internal state (su_cmd_integrator 대응)
  // =========================
  // inputs (sp_in)
  std::array<double, 3> sp_in_{0.0, 0.0, 0.0};
  double sp_in_yaw_{0.0};
  bool sp_received_{false};

  // params/commands
  float su_cmd_use_vel_mode_{0.0f}; // default: position mode
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

  // contact force (scalar, world-x)
  std::mutex force_mtx_;

  // eF 및 eF_dot(LPF 결과) 공유 상태
  std::array<double,3> eF_{0.0, 0.0, 0.0};
  std::array<double,3> eF_dot_filt_{0.0, 0.0, 0.0};

  // 내부 상태(미분/필터용)
  std::array<double,3> eF_prev_{0.0, 0.0, 0.0};
  std::array<double,3> eF_dot_filt_prev_{0.0, 0.0, 0.0};
  std::array<double,3> contact_F_{0.0, 0.0, 0.0};   // from WrenchStamped.force
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

