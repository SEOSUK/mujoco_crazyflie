#include "mujoco_bridge/mujoco_contact.hpp"
#include "mujoco_bridge/viewer_controls.hpp"

#include <mujoco/mujoco.h>
#include <GLFW/glfw3.h>

#include <rclcpp/rclcpp.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/vector3_stamped.hpp>
#include <geometry_msgs/msg/wrench_stamped.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>

#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <deque>
#include <memory>
#include <mutex>
#include <optional>
#include <random>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>
#include <algorithm>

namespace mujoco_bridge
{

namespace
{

constexpr double PHYSICS_HZ = 1000.0;
constexpr double PUB_HZ = 400.0;
constexpr double VIEWER_HZ = 60.0;

inline std::array<double, 4> quat_normalize_wxyz(const std::array<double, 4>& q)
{
  const double n = std::sqrt(q[0]*q[0] + q[1]*q[1] + q[2]*q[2] + q[3]*q[3]);
  if (n < 1e-12) {
    return {1.0, 0.0, 0.0, 0.0};
  }
  return {q[0]/n, q[1]/n, q[2]/n, q[3]/n};
}

inline std::array<double, 4> quat_mul_wxyz(
  const std::array<double, 4>& q1,
  const std::array<double, 4>& q2)
{
  const auto& [w1, x1, y1, z1] = q1;
  const auto& [w2, x2, y2, z2] = q2;
  return {
    w1*w2 - x1*x2 - y1*y2 - z1*z2,
    w1*x2 + x1*w2 + y1*z2 - z1*y2,
    w1*y2 - x1*z2 + y1*w2 + z1*x2,
    w1*z2 + x1*y2 - y1*x2 + z1*w2
  };
}

inline std::array<double, 4> rotvec_to_quat_wxyz(const std::array<double, 3>& r)
{
  const double angle = std::sqrt(r[0]*r[0] + r[1]*r[1] + r[2]*r[2]);
  if (angle < 1e-12) {
    return {1.0, 0.0, 0.0, 0.0};
  }

  const double ax = r[0] / angle;
  const double ay = r[1] / angle;
  const double az = r[2] / angle;
  const double half = 0.5 * angle;
  const double s = std::sin(half);

  return {std::cos(half), ax*s, ay*s, az*s};
}

inline void rotmat_from_quat_wxyz(const std::array<double, 4>& q, double R[9])
{
  const double w = q[0], x = q[1], y = q[2], z = q[3];
  R[0] = 1.0 - 2.0*(y*y + z*z);
  R[1] = 2.0*(x*y - w*z);
  R[2] = 2.0*(x*z + w*y);

  R[3] = 2.0*(x*y + w*z);
  R[4] = 1.0 - 2.0*(x*x + z*z);
  R[5] = 2.0*(y*z - w*x);

  R[6] = 2.0*(x*z - w*y);
  R[7] = 2.0*(y*z + w*x);
  R[8] = 1.0 - 2.0*(x*x + y*y);
}

inline std::array<double, 4> quat_wxyz_to_xyzw(const std::array<double, 4>& q)
{
  return {q[1], q[2], q[3], q[0]};
}

inline std::array<double, 3> mat3_mul_vec(const double R[9], const std::array<double, 3>& v)
{
  return {
    R[0]*v[0] + R[1]*v[1] + R[2]*v[2],
    R[3]*v[0] + R[4]*v[1] + R[5]*v[2],
    R[6]*v[0] + R[7]*v[1] + R[8]*v[2]
  };
}

inline std::array<double, 3> mat3_t_mul_vec(const double R[9], const std::array<double, 3>& v)
{
  return {
    R[0]*v[0] + R[3]*v[1] + R[6]*v[2],
    R[1]*v[0] + R[4]*v[1] + R[7]*v[2],
    R[2]*v[0] + R[5]*v[1] + R[8]*v[2]
  };
}

inline std::array<double, 3> add3(const std::array<double, 3>& a, const std::array<double, 3>& b)
{
  return {a[0] + b[0], a[1] + b[1], a[2] + b[2]};
}

inline std::array<double, 3> lpf3(
  const std::array<double, 3>& x,
  const std::array<double, 3>& prev,
  const double dt,
  const double cutoff_hz)
{
  if (cutoff_hz <= 1e-9) {
    return x;
  }
  const double wc = 2.0 * M_PI * cutoff_hz;
  const double alpha = 1.0 - std::exp(-wc * dt);
  return {
    (1.0 - alpha) * prev[0] + alpha * x[0],
    (1.0 - alpha) * prev[1] + alpha * x[1],
    (1.0 - alpha) * prev[2] + alpha * x[2]
  };
}

inline double lpf_alpha(const double dt, const double cutoff_hz)
{
  if (cutoff_hz <= 1e-9) {
    return 1.0;
  }
  const double wc = 2.0 * M_PI * cutoff_hz;
  return 1.0 - std::exp(-wc * dt);
}

inline double wrap_to_pi(double angle)
{
  while (angle > M_PI) {
    angle -= 2.0 * M_PI;
  }
  while (angle < -M_PI) {
    angle += 2.0 * M_PI;
  }
  return angle;
}

inline std::array<double, 4> quat_nlerp_wxyz(
  const std::array<double, 4>& q_prev,
  const std::array<double, 4>& q_curr,
  const double alpha)
{
  std::array<double, 4> q_target = q_curr;
  const double dot = q_prev[0] * q_curr[0] + q_prev[1] * q_curr[1] +
    q_prev[2] * q_curr[2] + q_prev[3] * q_curr[3];
  if (dot < 0.0) {
    for (double &v : q_target) {
      v = -v;
    }
  }

  return quat_normalize_wxyz({
    (1.0 - alpha) * q_prev[0] + alpha * q_target[0],
    (1.0 - alpha) * q_prev[1] + alpha * q_target[1],
    (1.0 - alpha) * q_prev[2] + alpha * q_target[2],
    (1.0 - alpha) * q_prev[3] + alpha * q_target[3]
  });
}

inline std::array<double, 3> sine3(
  const double t,
  const double hz,
  const std::array<double, 3>& amp,
  const std::array<double, 3>& phase)
{
  const double w = 2.0 * M_PI * hz;
  return {
    amp[0] * std::sin(w*t + phase[0]),
    amp[1] * std::sin(w*t + phase[1]),
    amp[2] * std::sin(w*t + phase[2])
  };
}

inline std::array<double, 3> to_amp3(const std::vector<double>& v)
{
  if (v.empty()) {
    return {0.0, 0.0, 0.0};
  }
  if (v.size() == 1) {
    const double a = std::max(0.0, v[0]);
    return {a, a, a};
  }
  return {
    std::max(0.0, v[0]),
    std::max(0.0, v[1]),
    std::max(0.0, v[2])
  };
}

struct ViewerContext
{
  GLFWwindow* window{nullptr};
  mjvCamera cam{};
  mjvOption opt{};
  mjvScene scn{};
  mjrContext con{};
  mjvPerturb pert{};

  double orbit_sensitivity{1.0};
  double pan_sensitivity{1.0};
  double zoom_sensitivity{1.0};

  double min_distance{0.08};
  double max_distance{20.0};
};

}  // namespace

class MujocoBridge : public rclcpp::Node
{
public:
  MujocoBridge()
  : Node("mujoco_bridge")
  {
    declare_parameters();
    load_parameters();
    init_noise_phase();

    load_model();
    bind_handles();
    init_allocation();

    setup_ros_interfaces();

    contact_manager_ = std::make_unique<MujocoContact>(
      this, model_, data_, pub_contact_force_, pub_contact_force_filt_);

    sim_thread_ = std::thread(&MujocoBridge::sim_loop, this);
    viewer_thread_ = std::thread(&MujocoBridge::viewer_loop, this);

    RCLCPP_INFO(get_logger(), "mujoco_bridge started");
  }

  ~MujocoBridge() override
  {
    stop_.store(true);

    if (viewer_thread_.joinable()) {
      viewer_thread_.join();
    }
    if (sim_thread_.joinable()) {
      sim_thread_.join();
    }

    contact_manager_.reset();

    if (data_) {
      mj_deleteData(data_);
      data_ = nullptr;
    }
    if (model_) {
      mj_deleteModel(model_);
      model_ = nullptr;
    }
  }

private:
  void declare_parameters()
  {
    declare_parameter("physics_hz", PHYSICS_HZ);
    declare_parameter("pub_hz", PUB_HZ);
    declare_parameter("viewer_hz", VIEWER_HZ);

    declare_parameter("arm_xy", 0.035355);
    declare_parameter("k_tau", 0.00594);
    declare_parameter("motor_dir", std::vector<double>{1.0, -1.0, 1.0, -1.0});
    declare_parameter("thrust_min", 0.0);
    declare_parameter("thrust_max", 0.20);

    declare_parameter("noise.enable", false);
    declare_parameter("noise.seed", 0);
    declare_parameter("noise.hz", 30.0);
    declare_parameter("noise.pos_amp", std::vector<double>{0.0, 0.0, 0.0});
    declare_parameter("noise.vel_amp", std::vector<double>{0.0, 0.0, 0.0});
    declare_parameter("noise.att_amp", std::vector<double>{0.0, 0.0, 0.0});
    declare_parameter("noise.ang_vel_amp", std::vector<double>{0.0, 0.0, 0.0});
    declare_parameter("noise.ang_acc_amp", std::vector<double>{0.0, 0.0, 0.0});
    declare_parameter("noise.lpf.enable", false);
    declare_parameter("noise.lpf.cutoff_hz", 30.0);

    declare_parameter("state_filter.pos.enable", false);
    declare_parameter("state_filter.pos.cutoff_hz", 10.0);
    declare_parameter("state_filter.att.enable", false);
    declare_parameter("state_filter.att.cutoff_hz", 10.0);
    declare_parameter("state_filter.vel.enable", false);
    declare_parameter("state_filter.vel.cutoff_hz", 10.0);
    declare_parameter("state_filter.ang_vel.enable", false);
    declare_parameter("state_filter.ang_vel.cutoff_hz", 10.0);
    declare_parameter("state_filter.acc.enable", false);
    declare_parameter("state_filter.acc.cutoff_hz", 10.0);

    declare_parameter("ang_acc_lpf.enable", true);
    declare_parameter("ang_acc_lpf.cutoff_hz", 10.0);

    declare_parameter("actuator.delay.enable", true);
    declare_parameter("actuator.delay.sec", 0.0);
    declare_parameter("actuator.lpf.enable", true);
    declare_parameter("actuator.lpf.cutoff_hz", 0.0);

    declare_parameter("wind.enable", false);
    declare_parameter("wind.force", std::vector<double>{0.0, 0.0, 0.0});
    declare_parameter("wind.torque", std::vector<double>{0.0, 0.0, 0.0});
    declare_parameter("wind.indicator_force_gain", 1.0);
    declare_parameter("wind.indicator_flutter_gain", 0.15);
    declare_parameter("wind.indicator_flutter_hz", 6.0);

    declare_parameter("viewer.orbit_sensitivity", 1.0);
    declare_parameter("viewer.pan_sensitivity", 1.2);
    declare_parameter("viewer.zoom_sensitivity", 1.0);
    declare_parameter("viewer.prop_visual_spin.enable", true);
    declare_parameter("viewer.prop_visual_spin_gain", 1800.0);
    declare_parameter("viewer.min_distance", 0.05);
    declare_parameter("viewer.max_distance", 30.0);
    declare_parameter("viewer.znear", 0.0002);
    declare_parameter("viewer.zfar", 200.0);
    declare_parameter("viewer.window_x", 900);
    declare_parameter("viewer.window_y", 40);
    declare_parameter("viewer.window_width", 1000);
    declare_parameter("viewer.window_height", 820);
  }

  void load_parameters()
  {
    physics_hz_ = get_parameter("physics_hz").as_double();
    pub_hz_ = get_parameter("pub_hz").as_double();
    viewer_hz_ = get_parameter("viewer_hz").as_double();

    a_ = get_parameter("arm_xy").as_double();
    k_tau_ = get_parameter("k_tau").as_double();
    thrust_min_ = get_parameter("thrust_min").as_double();
    thrust_max_ = get_parameter("thrust_max").as_double();

    {
      const auto v = get_parameter("motor_dir").as_double_array();
      for (size_t i = 0; i < std::min<size_t>(4, v.size()); ++i) {
        motor_dir_[i] = v[i];
      }
    }

    noise_enable_ = get_parameter("noise.enable").as_bool();
    rng_seed_ = get_parameter("noise.seed").as_int();
    noise_hz_ = get_parameter("noise.hz").as_double();
    noise_lpf_enable_ = get_parameter("noise.lpf.enable").as_bool();
    noise_lpf_cutoff_hz_ = get_parameter("noise.lpf.cutoff_hz").as_double();

    state_pos_lpf_enable_ = get_parameter("state_filter.pos.enable").as_bool();
    state_pos_lpf_cutoff_hz_ = get_parameter("state_filter.pos.cutoff_hz").as_double();
    state_att_lpf_enable_ = get_parameter("state_filter.att.enable").as_bool();
    state_att_lpf_cutoff_hz_ = get_parameter("state_filter.att.cutoff_hz").as_double();
    state_vel_lpf_enable_ = get_parameter("state_filter.vel.enable").as_bool();
    state_vel_lpf_cutoff_hz_ = get_parameter("state_filter.vel.cutoff_hz").as_double();
    state_ang_vel_lpf_enable_ = get_parameter("state_filter.ang_vel.enable").as_bool();
    state_ang_vel_lpf_cutoff_hz_ = get_parameter("state_filter.ang_vel.cutoff_hz").as_double();
    state_acc_lpf_enable_ = get_parameter("state_filter.acc.enable").as_bool();
    state_acc_lpf_cutoff_hz_ = get_parameter("state_filter.acc.cutoff_hz").as_double();

    ang_acc_lpf_enable_ = get_parameter("ang_acc_lpf.enable").as_bool();
    ang_acc_lpf_cutoff_hz_ = get_parameter("ang_acc_lpf.cutoff_hz").as_double();

    pos_amp_ = to_amp3(get_parameter("noise.pos_amp").as_double_array());
    vel_amp_ = to_amp3(get_parameter("noise.vel_amp").as_double_array());
    att_amp_ = to_amp3(get_parameter("noise.att_amp").as_double_array());
    ang_vel_amp_ = to_amp3(get_parameter("noise.ang_vel_amp").as_double_array());
    ang_acc_amp_ = to_amp3(get_parameter("noise.ang_acc_amp").as_double_array());

    act_delay_enable_ = get_parameter("actuator.delay.enable").as_bool();
    act_delay_sec_ = get_parameter("actuator.delay.sec").as_double();
    act_lpf_enable_ = get_parameter("actuator.lpf.enable").as_bool();
    act_lpf_cutoff_hz_ = get_parameter("actuator.lpf.cutoff_hz").as_double();

    wind_enable_ = get_parameter("wind.enable").as_bool();
    wind_force_ = vec3_from_parameter("wind.force");
    wind_torque_ = vec3_from_parameter("wind.torque");
    wind_indicator_force_gain_ = get_parameter("wind.indicator_force_gain").as_double();
    wind_indicator_flutter_gain_ = get_parameter("wind.indicator_flutter_gain").as_double();
    wind_indicator_flutter_hz_ = get_parameter("wind.indicator_flutter_hz").as_double();

    viewer_orbit_sensitivity_ = get_parameter("viewer.orbit_sensitivity").as_double();
    viewer_pan_sensitivity_ = get_parameter("viewer.pan_sensitivity").as_double();
    viewer_zoom_sensitivity_ = get_parameter("viewer.zoom_sensitivity").as_double();
    prop_visual_spin_enable_ = get_parameter("viewer.prop_visual_spin.enable").as_bool();
    prop_visual_spin_gain_ = get_parameter("viewer.prop_visual_spin_gain").as_double();
    viewer_min_distance_ = get_parameter("viewer.min_distance").as_double();
    viewer_max_distance_ = get_parameter("viewer.max_distance").as_double();
    viewer_znear_ = get_parameter("viewer.znear").as_double();
    viewer_zfar_ = get_parameter("viewer.zfar").as_double();
    viewer_window_x_ = static_cast<int>(get_parameter("viewer.window_x").as_int());
    viewer_window_y_ = static_cast<int>(get_parameter("viewer.window_y").as_int());
    viewer_window_width_ = static_cast<int>(get_parameter("viewer.window_width").as_int());
    viewer_window_height_ = static_cast<int>(get_parameter("viewer.window_height").as_int());
  }

  void load_model()
  {
    const auto share_dir = ament_index_cpp::get_package_share_directory("mujoco_bridge");
    const std::string xml_path = share_dir + "/data/scene.xml";

    char error[1024] = {0};
    model_ = mj_loadXML(xml_path.c_str(), nullptr, error, sizeof(error));
    if (!model_) {
      throw std::runtime_error(std::string("mj_loadXML failed: ") + error);
    }

    data_ = mj_makeData(model_);
    if (!data_) {
      throw std::runtime_error("mj_makeData failed");
    }

    model_->opt.timestep = 1.0 / std::max(1e-9, physics_hz_);
    dt_ = model_->opt.timestep;
  }

  void bind_handles()
  {
    bid_drone_ = mj_name2id(model_, mjOBJ_BODY, "drone");
    if (bid_drone_ < 0) {
      throw std::runtime_error("Body 'drone' not found");
    }

    const int jnt_adr = model_->body_jntadr[bid_drone_];
    const int jnt_num = model_->body_jntnum[bid_drone_];
    if (jnt_num < 1) {
      throw std::runtime_error("Body 'drone' has no joint");
    }

    jid_drone_ = jnt_adr;
    qpos_adr_drone_ = model_->jnt_qposadr[jid_drone_];
    qvel_adr_drone_ = model_->jnt_dofadr[jid_drone_];

    if (model_->jnt_type[jid_drone_] != mjJNT_FREE) {
      throw std::runtime_error("Body 'drone' first joint is not FREE");
    }

    bid_wind_indicator_tip_ = mj_name2id(model_, mjOBJ_BODY, "wind_indicator_tip");
    if (bid_wind_indicator_tip_ < 0) {
      RCLCPP_WARN(
        get_logger(), "Body 'wind_indicator_tip' not found. Wind indicator physics disabled.");
    }

    imu_acc_sid_ = sensor_id("imu_acc");
    imu_gyro_sid_ = sensor_id("imu_gyro");

    if (imu_acc_sid_ < 0) {
      throw std::runtime_error("Sensor 'imu_acc' not found");
    }
    if (imu_gyro_sid_ < 0) {
      throw std::runtime_error("Sensor 'imu_gyro' not found");
    }

    for (int i = 0; i < 4; ++i) {
      const std::string name = "motor" + std::to_string(i) + "_force";
      actuator_ids_[i] = mj_name2id(model_, mjOBJ_ACTUATOR, name.c_str());
      if (actuator_ids_[i] < 0) {
        throw std::runtime_error("Actuator not found: " + name);
      }

      const std::string prop_name = "prop" + std::to_string(i);
      prop_body_ids_[i] = mj_name2id(model_, mjOBJ_BODY, prop_name.c_str());
      if (prop_body_ids_[i] < 0) {
        throw std::runtime_error("Propeller body not found: " + prop_name);
      }
      const int qadr = 4 * prop_body_ids_[i];
      prop_base_quat_[i] = {
        static_cast<double>(model_->body_quat[qadr + 0]),
        static_cast<double>(model_->body_quat[qadr + 1]),
        static_cast<double>(model_->body_quat[qadr + 2]),
        static_cast<double>(model_->body_quat[qadr + 3])
      };
    }

    delay_steps_ = 0;
    if (act_delay_enable_ && act_delay_sec_ > 0.0) {
      delay_steps_ = std::max(0, static_cast<int>(std::llround(act_delay_sec_ / dt_)));
    }

    delay_buffer_.clear();
    for (int i = 0; i < delay_steps_ + 1; ++i) {
      delay_buffer_.push_back({0.0, 0.0, 0.0, 0.0});
    }

  }

  int sensor_id(const std::string& name) const
  {
    return mj_name2id(model_, mjOBJ_SENSOR, name.c_str());
  }

  std::array<double, 3> vec3_from_parameter(const std::string& name) const
  {
    const auto v = get_parameter(name).as_double_array();
    std::array<double, 3> out{0.0, 0.0, 0.0};
    for (size_t i = 0; i < std::min<size_t>(3, v.size()); ++i) {
      out[i] = v[i];
    }
    return out;
  }


  void init_allocation()
  {
    const double x[4] = {+a_, -a_, -a_, +a_};
    const double y[4] = {-a_, -a_, +a_, +a_};

    for (int c = 0; c < 4; ++c) {
      B_[0][c] = y[c];
      B_[1][c] = -x[c];
      B_[2][c] = motor_dir_[c] * k_tau_;
      B_[3][c] = 1.0;
    }

    compute_inverse_4x4(B_, B_inv_);
  }

  void setup_ros_interfaces()
  {
    sub_input_ = create_subscription<std_msgs::msg::Float32MultiArray>(
      "/crazyflie/in/input", 10,
      std::bind(&MujocoBridge::input_callback, this, std::placeholders::_1));
    sub_wind_ = create_subscription<geometry_msgs::msg::Vector3Stamped>(
      "/crazyflie/in/wind", 10,
      std::bind(&MujocoBridge::wind_callback, this, std::placeholders::_1));

    pub_pose_ = create_publisher<geometry_msgs::msg::PoseStamped>("/crazyflie/out/pose", 10);
    pub_vel_ = create_publisher<geometry_msgs::msg::Vector3Stamped>("/crazyflie/out/vel", 10);
    pub_angvel_ = create_publisher<geometry_msgs::msg::Vector3Stamped>("/crazyflie/out/ang_vel", 10);
    pub_acc_ = create_publisher<geometry_msgs::msg::Vector3Stamped>("/crazyflie/out/acc", 10);
    pub_angacc_ = create_publisher<geometry_msgs::msg::Vector3Stamped>("/crazyflie/out/ang_acc", 10);
    pub_angvel_gt_ = create_publisher<geometry_msgs::msg::Vector3Stamped>("/crazyflie/out/ang_vel_gt", 10);
    pub_motor_thrust_ = create_publisher<std_msgs::msg::Float32MultiArray>("/crazyflie/out/motor_thrust", 10);
    pub_contact_force_ = create_publisher<geometry_msgs::msg::WrenchStamped>("/crazyflie/out/EE_contact_force", 10);
    pub_contact_force_filt_ = create_publisher<geometry_msgs::msg::WrenchStamped>("/crazyflie/out/EE_contact_force_filt", 10);
  }

  void input_callback(const std_msgs::msg::Float32MultiArray::SharedPtr msg)
  {
    if (!msg || msg->data.size() < 4) {
      return;
    }

    std::lock_guard<std::mutex> lock(cmd_mtx_);
    u_cmd_[0] = msg->data[0];
    u_cmd_[1] = msg->data[1];
    u_cmd_[2] = msg->data[2];
    u_cmd_[3] = msg->data[3];
  }

  void wind_callback(const geometry_msgs::msg::Vector3Stamped::SharedPtr msg)
  {
    if (!msg) {
      return;
    }

    std::lock_guard<std::mutex> lock(wind_mtx_);
    wind_force_ = {msg->vector.x, msg->vector.y, msg->vector.z};
    wind_topic_received_ = true;
  }

  std::array<double, 4> apply_actuator_dynamics(const std::array<double, 4>& f_cmd)
  {
    std::array<double, 4> f_delayed = f_cmd;

    if (act_delay_enable_ && delay_steps_ > 0) {
      delay_buffer_.push_back(f_cmd);
      f_delayed = delay_buffer_.front();
      delay_buffer_.pop_front();
    }

    if (act_lpf_enable_ && act_lpf_cutoff_hz_ > 1e-9) {
      const double wc = 2.0 * M_PI * act_lpf_cutoff_hz_;
      const double alpha = 1.0 - std::exp(-wc * dt_);
      for (int i = 0; i < 4; ++i) {
        f_act_[i] = (1.0 - alpha) * f_act_[i] + alpha * f_delayed[i];
      }
    } else {
      f_act_ = f_delayed;
    }

    return f_act_;
  }

  void apply_control_locked()
  {
    std::array<double, 4> w{};
    {
      std::lock_guard<std::mutex> lock(cmd_mtx_);
      w = u_cmd_;
    }

    std::array<double, 4> f_cmd{};
    for (int r = 0; r < 4; ++r) {
      f_cmd[r] =
        B_inv_[r][0] * w[0] +
        B_inv_[r][1] * w[1] +
        B_inv_[r][2] * w[2] +
        B_inv_[r][3] * w[3];
      f_cmd[r] = std::clamp(f_cmd[r], thrust_min_, thrust_max_);
    }

    const auto f_applied = apply_actuator_dynamics(f_cmd);

    for (int i = 0; i < 4; ++i) {
      const double u = std::clamp(f_applied[i], thrust_min_, thrust_max_);
      last_motor_thrust_[i] = u;
      data_->ctrl[actuator_ids_[i]] = static_cast<mjtNum>(u);
    }
  }

  void apply_wind_disturbance_locked()
  {
    std::array<double, 3> wind_force{};
    std::array<double, 3> wind_torque{};
    bool wind_active{false};
    {
      std::lock_guard<std::mutex> lock(wind_mtx_);
      wind_force = wind_force_;
      wind_torque = wind_torque_;
      wind_active = wind_topic_received_;
    }

    const int drone_adr = 6 * bid_drone_;
    for (int i = 0; i < 6; ++i) {
      data_->xfrc_applied[drone_adr + i] = 0.0;
    }
    if (bid_wind_indicator_tip_ >= 0) {
      const int indicator_adr = 6 * bid_wind_indicator_tip_;
      for (int i = 0; i < 6; ++i) {
        data_->xfrc_applied[indicator_adr + i] = 0.0;
      }
    }

    if (!wind_active) {
      return;
    }

    for (int i = 0; i < 3; ++i) {
      data_->xfrc_applied[drone_adr + i] = static_cast<mjtNum>(wind_force[i]);
      data_->xfrc_applied[drone_adr + 3 + i] = static_cast<mjtNum>(wind_torque[i]);
    }

    if (bid_wind_indicator_tip_ < 0) {
      return;
    }

    const double wind_xy_norm = std::hypot(wind_force[0], wind_force[1]);
    if (wind_xy_norm < 1e-6) {
      return;
    }

    const double wind_x = wind_force[0] / wind_xy_norm;
    const double wind_y = wind_force[1] / wind_xy_norm;
    const double flutter =
      wind_indicator_flutter_gain_ * wind_xy_norm *
      std::sin(2.0 * M_PI * wind_indicator_flutter_hz_ * data_->time);

    const int indicator_adr = 6 * bid_wind_indicator_tip_;
    data_->xfrc_applied[indicator_adr + 0] =
      static_cast<mjtNum>(wind_indicator_force_gain_ * wind_force[0] - flutter * wind_y);
    data_->xfrc_applied[indicator_adr + 1] =
      static_cast<mjtNum>(wind_indicator_force_gain_ * wind_force[1] + flutter * wind_x);
  }

  void update_propeller_visuals_locked()
  {
    if (!prop_visual_spin_enable_) {
      for (int i = 0; i < 4; ++i) {
        const int qadr = 4 * prop_body_ids_[i];
        model_->body_quat[qadr + 0] = static_cast<mjtNum>(prop_base_quat_[i][0]);
        model_->body_quat[qadr + 1] = static_cast<mjtNum>(prop_base_quat_[i][1]);
        model_->body_quat[qadr + 2] = static_cast<mjtNum>(prop_base_quat_[i][2]);
        model_->body_quat[qadr + 3] = static_cast<mjtNum>(prop_base_quat_[i][3]);
      }
      return;
    }

    for (int i = 0; i < 4; ++i) {
      prop_spin_angle_[i] = wrap_to_pi(
        prop_spin_angle_[i] + motor_dir_[i] * prop_visual_spin_gain_ * last_motor_thrust_[i] * dt_);
      const std::array<double, 4> spin_q = {
        std::cos(0.5 * prop_spin_angle_[i]), 0.0, 0.0, std::sin(0.5 * prop_spin_angle_[i])
      };
      const std::array<double, 4> q_prop = quat_mul_wxyz(prop_base_quat_[i], spin_q);
      const int qadr = 4 * prop_body_ids_[i];
      model_->body_quat[qadr + 0] = static_cast<mjtNum>(q_prop[0]);
      model_->body_quat[qadr + 1] = static_cast<mjtNum>(q_prop[1]);
      model_->body_quat[qadr + 2] = static_cast<mjtNum>(q_prop[2]);
      model_->body_quat[qadr + 3] = static_cast<mjtNum>(q_prop[3]);
    }
  }

  std::array<double, 3> read_sensor_vec3(const int sid) const
  {
    if (sid < 0) {
      return {0.0, 0.0, 0.0};
    }
    const int adr = model_->sensor_adr[sid];
    const int dim = model_->sensor_dim[sid];
    std::array<double, 3> out{0.0, 0.0, 0.0};
    for (int i = 0; i < std::min(3, dim); ++i) {
      out[i] = static_cast<double>(data_->sensordata[adr + i]);
    }
    return out;
  }

  void read_state_locked(
    std::array<double, 3>& pos_w,
    std::array<double, 4>& quat_wxyz,
    std::array<double, 3>& linvel_w,
    std::array<double, 3>& angvel_b_gt)
  {
    const int qa = qpos_adr_drone_;
    const int va = qvel_adr_drone_;

    pos_w = {
      static_cast<double>(data_->qpos[qa + 0]),
      static_cast<double>(data_->qpos[qa + 1]),
      static_cast<double>(data_->qpos[qa + 2])
    };

    quat_wxyz = {
      static_cast<double>(data_->qpos[qa + 3]),
      static_cast<double>(data_->qpos[qa + 4]),
      static_cast<double>(data_->qpos[qa + 5]),
      static_cast<double>(data_->qpos[qa + 6])
    };

    linvel_w = {
      static_cast<double>(data_->qvel[va + 0]),
      static_cast<double>(data_->qvel[va + 1]),
      static_cast<double>(data_->qvel[va + 2])
    };

    const std::array<double, 3> angvel_w = {
      static_cast<double>(data_->qvel[va + 3]),
      static_cast<double>(data_->qvel[va + 4]),
      static_cast<double>(data_->qvel[va + 5])
    };

    double R[9];
    rotmat_from_quat_wxyz(quat_wxyz, R);
    angvel_b_gt = mat3_t_mul_vec(R, angvel_w);
  }

  std::array<double, 3> read_imu_acc_world_locked(const std::array<double, 4>& quat_wxyz) const
  {
    const auto acc_b = read_sensor_vec3(imu_acc_sid_);
    double R[9];
    rotmat_from_quat_wxyz(quat_wxyz, R);

    const auto acc_rot = mat3_mul_vec(R, acc_b);
    const std::array<double, 3> g_w = {
      static_cast<double>(model_->opt.gravity[0]),
      static_cast<double>(model_->opt.gravity[1]),
      static_cast<double>(model_->opt.gravity[2])
    };
    return add3(acc_rot, g_w);
  }

  std::array<double, 3> read_imu_gyro_body_locked() const
  {
    return read_sensor_vec3(imu_gyro_sid_);
  }

  void init_noise_phase()
  {
    std::seed_seq seq{static_cast<unsigned int>(rng_seed_ == 0 ? 1 : rng_seed_)};
    std::mt19937 gen(seq);
    std::uniform_real_distribution<double> uni(0.0, 2.0 * M_PI);

    for (int i = 0; i < 3; ++i) {
      pos_phase_[i] = uni(gen);
      vel_phase_[i] = uni(gen);
      att_phase_[i] = uni(gen);
      ang_vel_phase_[i] = uni(gen);
      ang_acc_phase_[i] = uni(gen);
    }
  }

  void update_noise()
  {
    if (!noise_enable_) {
      pos_noise_ = {0.0, 0.0, 0.0};
      vel_noise_ = {0.0, 0.0, 0.0};
      att_noise_ = {0.0, 0.0, 0.0};
      ang_vel_noise_ = {0.0, 0.0, 0.0};
      ang_acc_noise_ = {0.0, 0.0, 0.0};
      return;
    }

    const auto now = get_clock()->now();
    if (noise_t0_.nanoseconds() == 0) {
      noise_t0_ = now;
    }

    const double t = (now - noise_t0_).seconds();
    pos_noise_ = sine3(t, noise_hz_, pos_amp_, pos_phase_);
    vel_noise_ = sine3(t, noise_hz_, vel_amp_, vel_phase_);
    att_noise_ = sine3(t, noise_hz_, att_amp_, att_phase_);
    ang_vel_noise_ = sine3(t, noise_hz_, ang_vel_amp_, ang_vel_phase_);
    ang_acc_noise_ = sine3(t, noise_hz_, ang_acc_amp_, ang_acc_phase_);
  }

  void publish_outputs(const double dt_pub)
  {
    std::array<double, 3> pos_w{}, linvel_w{}, angvel_b_gt{};
    std::array<double, 4> quat_wxyz{};

    std::array<double, 3> linacc_w{}, gyro_b{};
    {
      std::lock_guard<std::mutex> lock(scene_mtx_);
      read_state_locked(pos_w, quat_wxyz, linvel_w, angvel_b_gt);

      quat_wxyz = quat_normalize_wxyz(quat_wxyz);
      linacc_w = read_imu_acc_world_locked(quat_wxyz);
      gyro_b = read_imu_gyro_body_locked();
    }

    update_noise();

    const auto pos_noisy = add3(pos_w, pos_noise_);
    const auto vel_noisy = add3(linvel_w, vel_noise_);
    const auto gyro_noisy = add3(gyro_b, ang_vel_noise_);

    const auto dq = rotvec_to_quat_wxyz(att_noise_);
    const auto quat_noisy = quat_normalize_wxyz(quat_mul_wxyz(quat_wxyz, dq));

    if (!noise_lpf_initialized_) {
      pos_filt_ = pos_noisy;
      vel_filt_ = vel_noisy;
      ang_vel_filt_ = gyro_noisy;
      ang_acc_filt_ = {0.0, 0.0, 0.0};
      noise_lpf_initialized_ = true;
    } else if (noise_lpf_enable_) {
      pos_filt_ = lpf3(pos_noisy, pos_filt_, dt_, noise_lpf_cutoff_hz_);
      vel_filt_ = lpf3(vel_noisy, vel_filt_, dt_, noise_lpf_cutoff_hz_);
      ang_vel_filt_ = lpf3(gyro_noisy, ang_vel_filt_, dt_, noise_lpf_cutoff_hz_);
    } else {
      pos_filt_ = pos_noisy;
      vel_filt_ = vel_noisy;
      ang_vel_filt_ = gyro_noisy;
    }

    if (!state_lpf_initialized_) {
      pos_state_filt_ = pos_filt_;
      att_state_filt_ = quat_noisy;
      vel_state_filt_ = vel_filt_;
      ang_vel_state_filt_ = ang_vel_filt_;
      acc_state_filt_ = linacc_w;
      state_lpf_initialized_ = true;
    } else {
      if (state_pos_lpf_enable_) {
        pos_state_filt_ = lpf3(pos_filt_, pos_state_filt_, dt_, state_pos_lpf_cutoff_hz_);
      } else {
        pos_state_filt_ = pos_filt_;
      }

      if (state_att_lpf_enable_) {
        att_state_filt_ = quat_nlerp_wxyz(
          att_state_filt_, quat_noisy, lpf_alpha(dt_, state_att_lpf_cutoff_hz_));
      } else {
        att_state_filt_ = quat_noisy;
      }

      if (state_vel_lpf_enable_) {
        vel_state_filt_ = lpf3(vel_filt_, vel_state_filt_, dt_, state_vel_lpf_cutoff_hz_);
      } else {
        vel_state_filt_ = vel_filt_;
      }

      if (state_ang_vel_lpf_enable_) {
        ang_vel_state_filt_ = lpf3(
          ang_vel_filt_, ang_vel_state_filt_, dt_, state_ang_vel_lpf_cutoff_hz_);
      } else {
        ang_vel_state_filt_ = ang_vel_filt_;
      }

      if (state_acc_lpf_enable_) {
        acc_state_filt_ = lpf3(linacc_w, acc_state_filt_, dt_, state_acc_lpf_cutoff_hz_);
      } else {
        acc_state_filt_ = linacc_w;
      }
    }

    std::array<double, 3> angacc_b{0.0, 0.0, 0.0};
    const double dt = std::max(1e-6, dt_pub);
    if (prev_gyro_used_b_.has_value()) {
      for (int i = 0; i < 3; ++i) {
        angacc_b[i] = (gyro_noisy[i] - (*prev_gyro_used_b_)[i]) / dt;
      }
    }
    prev_gyro_used_b_ = gyro_noisy;

    const auto angacc_noisy = add3(angacc_b, ang_acc_noise_);
    if (ang_acc_lpf_enable_) {
      ang_acc_filt_ = lpf3(angacc_noisy, ang_acc_filt_, dt_, ang_acc_lpf_cutoff_hz_);
    } else {
      ang_acc_filt_ = angacc_noisy;
    }

    const auto stamp = now();

    geometry_msgs::msg::PoseStamped pose_msg;
    pose_msg.header.stamp = stamp;
    pose_msg.header.frame_id = "world";
    pose_msg.pose.position.x = pos_state_filt_[0];
    pose_msg.pose.position.y = pos_state_filt_[1];
    pose_msg.pose.position.z = pos_state_filt_[2];

    const auto q_xyzw = quat_wxyz_to_xyzw(att_state_filt_);
    pose_msg.pose.orientation.x = q_xyzw[0];
    pose_msg.pose.orientation.y = q_xyzw[1];
    pose_msg.pose.orientation.z = q_xyzw[2];
    pose_msg.pose.orientation.w = q_xyzw[3];
    pub_pose_->publish(pose_msg);

    geometry_msgs::msg::Vector3Stamped vel_msg;
    vel_msg.header.stamp = stamp;
    vel_msg.header.frame_id = "world";
    vel_msg.vector.x = vel_state_filt_[0];
    vel_msg.vector.y = vel_state_filt_[1];
    vel_msg.vector.z = vel_state_filt_[2];
    pub_vel_->publish(vel_msg);

    geometry_msgs::msg::Vector3Stamped angvel_msg;
    angvel_msg.header.stamp = stamp;
    angvel_msg.header.frame_id = "body";
    angvel_msg.vector.x = ang_vel_state_filt_[0];
    angvel_msg.vector.y = ang_vel_state_filt_[1];
    angvel_msg.vector.z = ang_vel_state_filt_[2];
    pub_angvel_->publish(angvel_msg);

    geometry_msgs::msg::Vector3Stamped angvel_gt_msg;
    angvel_gt_msg.header.stamp = stamp;
    angvel_gt_msg.header.frame_id = "body";
    angvel_gt_msg.vector.x = angvel_b_gt[0];
    angvel_gt_msg.vector.y = angvel_b_gt[1];
    angvel_gt_msg.vector.z = angvel_b_gt[2];
    pub_angvel_gt_->publish(angvel_gt_msg);

    geometry_msgs::msg::Vector3Stamped acc_msg;
    acc_msg.header.stamp = stamp;
    acc_msg.header.frame_id = "world";
    acc_msg.vector.x = acc_state_filt_[0];
    acc_msg.vector.y = acc_state_filt_[1];
    acc_msg.vector.z = acc_state_filt_[2];
    pub_acc_->publish(acc_msg);

    geometry_msgs::msg::Vector3Stamped angacc_msg;
    angacc_msg.header.stamp = stamp;
    angacc_msg.header.frame_id = "body";
    angacc_msg.vector.x = ang_acc_filt_[0];
    angacc_msg.vector.y = ang_acc_filt_[1];
    angacc_msg.vector.z = ang_acc_filt_[2];
    pub_angacc_->publish(angacc_msg);

    std_msgs::msg::Float32MultiArray motor_msg;
    motor_msg.data.resize(4);
    for (int i = 0; i < 4; ++i) {
      motor_msg.data[i] = static_cast<float>(last_motor_thrust_[i]);
    }
    pub_motor_thrust_->publish(motor_msg);

    contact_manager_->update_raw_and_publish(get_clock()->now());
  }

  void sim_loop()
  {
    using clock = std::chrono::steady_clock;
    const auto sim_period = std::chrono::duration<double>(1.0 / std::max(1e-9, physics_hz_));
    const auto pub_period = std::chrono::duration<double>(1.0 / std::max(1e-9, pub_hz_));

    auto next_tick = clock::now();
    auto last_pub = clock::now();

    while (rclcpp::ok() && !stop_.load()) {
      {
        std::lock_guard<std::mutex> lock(scene_mtx_);
        apply_control_locked();
        apply_wind_disturbance_locked();
        update_propeller_visuals_locked();
        mj_step(model_, data_);
      }

      const auto now_tp = clock::now();
      if (now_tp - last_pub >= pub_period) {
        const double dt_pub = std::chrono::duration<double>(now_tp - last_pub).count();
        publish_outputs(dt_pub);
        last_pub = now_tp;
      }

      next_tick += std::chrono::duration_cast<clock::duration>(sim_period);
      std::this_thread::sleep_until(next_tick);
    }
  }

  void apply_viewer_clipping()
  {
    if (!model_) {
      return;
    }
    model_->vis.map.znear = std::max(1e-6, viewer_znear_);
    model_->vis.map.zfar = std::max(viewer_znear_ * 10.0, viewer_zfar_);
  }

  static void mouse_button_callback(GLFWwindow* window, int button, int act, int mods)
  {
    (void)button;
    (void)act;
    (void)mods;

    auto* self = static_cast<MujocoBridge*>(glfwGetWindowUserPointer(window));
    if (!self || !self->viewer_ctx_) {
      return;
    }

    std::lock_guard<std::mutex> lock(self->scene_mtx_);
    ViewerContext& v = *self->viewer_ctx_;
    self->viewer_controls_.onMouseButton(window, self->model_, self->data_, &v.opt, &v.scn, &v.cam);
  }

  static void cursor_pos_callback(GLFWwindow* window, double xpos, double ypos)
  {
    auto* self = static_cast<MujocoBridge*>(glfwGetWindowUserPointer(window));
    if (!self || !self->viewer_ctx_) {
      return;
    }

    std::lock_guard<std::mutex> lock(self->scene_mtx_);
    ViewerContext& v = *self->viewer_ctx_;
    self->viewer_controls_.onCursorPos(
      window, self->model_, self->data_, &v.opt, &v.scn, &v.cam, xpos, ypos);
  }

  static void scroll_callback(GLFWwindow* window, double xoffset, double yoffset)
  {
    (void)xoffset;

    auto* self = static_cast<MujocoBridge*>(glfwGetWindowUserPointer(window));
    if (!self || !self->viewer_ctx_) {
      return;
    }

    std::lock_guard<std::mutex> lock(self->scene_mtx_);
    ViewerContext& v = *self->viewer_ctx_;
    self->viewer_controls_.onScroll(window, self->model_, self->data_, &v.cam, yoffset);
  }

  static void key_callback(GLFWwindow* window, int key, int scancode, int act, int mods)
  {
    (void)scancode;
    (void)mods;

    auto* self = static_cast<MujocoBridge*>(glfwGetWindowUserPointer(window));
    if (!self || !self->viewer_ctx_) {
      return;
    }

    if (act == GLFW_PRESS && key == GLFW_KEY_BACKSPACE) {
      std::lock_guard<std::mutex> lock(self->scene_mtx_);
      mj_resetData(self->model_, self->data_);
      mj_forward(self->model_, self->data_);
    }
  }

  void viewer_loop()
  {
    if (viewer_hz_ <= 0.0) {
      return;
    }

    if (!glfwInit()) {
      RCLCPP_WARN(get_logger(), "glfwInit failed. Viewer disabled.");
      return;
    }

    ViewerContext v;
    viewer_ctx_ = &v;

    v.orbit_sensitivity = viewer_orbit_sensitivity_;
    v.pan_sensitivity = viewer_pan_sensitivity_;
    v.zoom_sensitivity = viewer_zoom_sensitivity_;
    v.min_distance = viewer_min_distance_;
    v.max_distance = viewer_max_distance_;

    ViewerControlsConfig viewer_controls_config;
    viewer_controls_config.orbit_sensitivity = viewer_orbit_sensitivity_;
    viewer_controls_config.pan_sensitivity = viewer_pan_sensitivity_;
    viewer_controls_config.zoom_sensitivity = viewer_zoom_sensitivity_;
    viewer_controls_config.min_distance = viewer_min_distance_;
    viewer_controls_config.max_distance = viewer_max_distance_;
    viewer_controls_.configure(viewer_controls_config);

    mjv_defaultCamera(&v.cam);
    mjv_defaultOption(&v.opt);
    mjv_defaultScene(&v.scn);
    mjr_defaultContext(&v.con);
    mjv_defaultPerturb(&v.pert);

    v.window = glfwCreateWindow(
      std::max(320, viewer_window_width_),
      std::max(240, viewer_window_height_),
      "mujoco_bridge", nullptr, nullptr);
    if (!v.window) {
      viewer_ctx_ = nullptr;
      RCLCPP_WARN(get_logger(), "glfwCreateWindow failed. Viewer disabled.");
      glfwTerminate();
      return;
    }
    glfwSetWindowPos(v.window, viewer_window_x_, viewer_window_y_);

    glfwMakeContextCurrent(v.window);
    glfwSwapInterval(1);

    glfwSetWindowUserPointer(v.window, this);
    glfwSetMouseButtonCallback(v.window, mouse_button_callback);
    glfwSetCursorPosCallback(v.window, cursor_pos_callback);
    glfwSetScrollCallback(v.window, scroll_callback);
    glfwSetKeyCallback(v.window, key_callback);

    apply_viewer_clipping();

    mjv_makeScene(model_, &v.scn, 2000);
    mjr_makeContext(model_, &v.con, mjFONTSCALE_150);

    v.cam.type = mjCAMERA_FREE;
    v.cam.azimuth = 90.0;
    v.cam.elevation = -20.0;
    v.cam.distance = 2.0;
    v.cam.lookat[0] = 0.0;
    v.cam.lookat[1] = 0.0;
    v.cam.lookat[2] = 0.5;

    const auto viewer_period =
      std::chrono::duration<double>(1.0 / std::max(1e-9, viewer_hz_));

    while (rclcpp::ok() && !stop_.load() && !glfwWindowShouldClose(v.window)) {
      {
        std::lock_guard<std::mutex> lock(scene_mtx_);
        mjv_updateScene(model_, data_, &v.opt, &v.pert, &v.cam, mjCAT_ALL, &v.scn);
      }

      contact_manager_->update_contact_resultant_arrow_in_viewer(&v.scn);

      mjrRect viewport{0, 0, 0, 0};
      glfwGetFramebufferSize(v.window, &viewport.width, &viewport.height);
      mjr_render(viewport, &v.scn, &v.con);

      glfwSwapBuffers(v.window);
      glfwPollEvents();
      std::this_thread::sleep_for(viewer_period);
    }

    viewer_ctx_ = nullptr;
    mjr_freeContext(&v.con);
    mjv_freeScene(&v.scn);
    glfwDestroyWindow(v.window);
    glfwTerminate();
  }

  static void compute_inverse_4x4(const double A[4][4], double invA[4][4])
  {
    double aug[4][8]{};

    for (int i = 0; i < 4; ++i) {
      for (int j = 0; j < 4; ++j) {
        aug[i][j] = A[i][j];
      }
      aug[i][4 + i] = 1.0;
    }

    for (int col = 0; col < 4; ++col) {
      int pivot = col;
      for (int r = col + 1; r < 4; ++r) {
        if (std::fabs(aug[r][col]) > std::fabs(aug[pivot][col])) {
          pivot = r;
        }
      }

      if (std::fabs(aug[pivot][col]) < 1e-12) {
        throw std::runtime_error("Allocation matrix is singular");
      }

      if (pivot != col) {
        for (int j = 0; j < 8; ++j) {
          std::swap(aug[pivot][j], aug[col][j]);
        }
      }

      const double diag = aug[col][col];
      for (int j = 0; j < 8; ++j) {
        aug[col][j] /= diag;
      }

      for (int r = 0; r < 4; ++r) {
        if (r == col) {
          continue;
        }
        const double f = aug[r][col];
        for (int j = 0; j < 8; ++j) {
          aug[r][j] -= f * aug[col][j];
        }
      }
    }

    for (int i = 0; i < 4; ++i) {
      for (int j = 0; j < 4; ++j) {
        invA[i][j] = aug[i][j + 4];
      }
    }
  }

private:
  mjModel* model_{nullptr};
  mjData* data_{nullptr};

  int bid_drone_{-1};
  int bid_wind_indicator_tip_{-1};
  int jid_drone_{-1};
  int qpos_adr_drone_{-1};
  int qvel_adr_drone_{-1};

  int imu_acc_sid_{-1};
  int imu_gyro_sid_{-1};
  std::array<int, 4> actuator_ids_{{-1, -1, -1, -1}};

  double physics_hz_{PHYSICS_HZ};
  double pub_hz_{PUB_HZ};
  double viewer_hz_{VIEWER_HZ};
  double dt_{1.0 / PHYSICS_HZ};

  double a_{0.035355};
  double k_tau_{0.00594};
  std::array<double, 4> motor_dir_{{1.0, -1.0, 1.0, -1.0}};
  double thrust_min_{0.0};
  double thrust_max_{0.20};

  double B_[4][4]{};
  double B_inv_[4][4]{};

  std::mutex scene_mtx_;
  std::mutex cmd_mtx_;
  std::mutex wind_mtx_;
  std::array<double, 4> u_cmd_{{0.0, 0.0, 0.0, 0.0}};

  bool act_delay_enable_{true};
  double act_delay_sec_{0.0};
  bool act_lpf_enable_{true};
  double act_lpf_cutoff_hz_{0.0};
  int delay_steps_{0};
  std::deque<std::array<double, 4>> delay_buffer_;
  std::array<double, 4> f_act_{{0.0, 0.0, 0.0, 0.0}};
  std::array<double, 4> last_motor_thrust_{{0.0, 0.0, 0.0, 0.0}};

  bool wind_enable_{false};
  bool wind_topic_received_{false};
  std::array<double, 3> wind_force_{{0.0, 0.0, 0.0}};
  std::array<double, 3> wind_torque_{{0.0, 0.0, 0.0}};
  double wind_indicator_force_gain_{1.0};
  double wind_indicator_flutter_gain_{0.15};
  double wind_indicator_flutter_hz_{6.0};

  bool noise_enable_{false};
  int64_t rng_seed_{0};
  double noise_hz_{30.0};
  bool noise_lpf_enable_{false};
  double noise_lpf_cutoff_hz_{30.0};
  bool state_pos_lpf_enable_{false};
  double state_pos_lpf_cutoff_hz_{10.0};
  bool state_att_lpf_enable_{false};
  double state_att_lpf_cutoff_hz_{10.0};
  bool state_vel_lpf_enable_{false};
  double state_vel_lpf_cutoff_hz_{10.0};
  bool state_ang_vel_lpf_enable_{false};
  double state_ang_vel_lpf_cutoff_hz_{10.0};
  bool state_acc_lpf_enable_{false};
  double state_acc_lpf_cutoff_hz_{10.0};

  bool ang_acc_lpf_enable_{true};
  double ang_acc_lpf_cutoff_hz_{10.0};

  double viewer_orbit_sensitivity_{1.0};
  double viewer_pan_sensitivity_{1.2};
  double viewer_zoom_sensitivity_{1.0};
  bool prop_visual_spin_enable_{true};
  double prop_visual_spin_gain_{1800.0};
  double viewer_min_distance_{0.05};
  double viewer_max_distance_{30.0};
  double viewer_znear_{0.0002};
  double viewer_zfar_{200.0};
  int viewer_window_x_{900};
  int viewer_window_y_{40};
  int viewer_window_width_{1000};
  int viewer_window_height_{820};

  std::array<int, 4> prop_body_ids_{{-1, -1, -1, -1}};
  std::array<std::array<double, 4>, 4> prop_base_quat_{{
    {{1.0, 0.0, 0.0, 0.0}},
    {{1.0, 0.0, 0.0, 0.0}},
    {{1.0, 0.0, 0.0, 0.0}},
    {{1.0, 0.0, 0.0, 0.0}}
  }};
  std::array<double, 4> prop_spin_angle_{{0.0, 0.0, 0.0, 0.0}};

  std::array<double, 3> pos_amp_{{0.0, 0.0, 0.0}};
  std::array<double, 3> vel_amp_{{0.0, 0.0, 0.0}};
  std::array<double, 3> att_amp_{{0.0, 0.0, 0.0}};
  std::array<double, 3> ang_vel_amp_{{0.0, 0.0, 0.0}};
  std::array<double, 3> ang_acc_amp_{{0.0, 0.0, 0.0}};

  std::array<double, 3> pos_phase_{{0.0, 0.0, 0.0}};
  std::array<double, 3> vel_phase_{{0.0, 0.0, 0.0}};
  std::array<double, 3> att_phase_{{0.0, 0.0, 0.0}};
  std::array<double, 3> ang_vel_phase_{{0.0, 0.0, 0.0}};
  std::array<double, 3> ang_acc_phase_{{0.0, 0.0, 0.0}};

  rclcpp::Time noise_t0_{0, 0, RCL_ROS_TIME};
  bool noise_lpf_initialized_{false};
  bool state_lpf_initialized_{false};

  std::array<double, 3> pos_noise_{{0.0, 0.0, 0.0}};
  std::array<double, 3> vel_noise_{{0.0, 0.0, 0.0}};
  std::array<double, 3> att_noise_{{0.0, 0.0, 0.0}};
  std::array<double, 3> ang_vel_noise_{{0.0, 0.0, 0.0}};
  std::array<double, 3> ang_acc_noise_{{0.0, 0.0, 0.0}};

  std::array<double, 3> pos_filt_{{0.0, 0.0, 0.0}};
  std::array<double, 3> vel_filt_{{0.0, 0.0, 0.0}};
  std::array<double, 3> pos_state_filt_{{0.0, 0.0, 0.0}};
  std::array<double, 4> att_state_filt_{{1.0, 0.0, 0.0, 0.0}};
  std::array<double, 3> vel_state_filt_{{0.0, 0.0, 0.0}};
  std::array<double, 3> ang_vel_state_filt_{{0.0, 0.0, 0.0}};
  std::array<double, 3> acc_state_filt_{{0.0, 0.0, 0.0}};
  std::array<double, 3> ang_vel_filt_{{0.0, 0.0, 0.0}};
  std::array<double, 3> ang_acc_filt_{{0.0, 0.0, 0.0}};
  std::optional<std::array<double, 3>> prev_gyro_used_b_;

  rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr sub_input_;
  rclcpp::Subscription<geometry_msgs::msg::Vector3Stamped>::SharedPtr sub_wind_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pub_pose_;
  rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr pub_vel_;
  rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr pub_angvel_;
  rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr pub_acc_;
  rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr pub_angacc_;
  rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr pub_angvel_gt_;
  rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr pub_motor_thrust_;
  rclcpp::Publisher<geometry_msgs::msg::WrenchStamped>::SharedPtr pub_contact_force_;
  rclcpp::Publisher<geometry_msgs::msg::WrenchStamped>::SharedPtr pub_contact_force_filt_;

  std::unique_ptr<MujocoContact> contact_manager_;

  ViewerContext* viewer_ctx_{nullptr};
  ViewerControls viewer_controls_{};

  std::thread sim_thread_;
  std::thread viewer_thread_;
  std::atomic<bool> stop_{false};
};

}  // namespace mujoco_bridge

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<mujoco_bridge::MujocoBridge>();
  rclcpp::spin(node);

  node.reset();
  rclcpp::shutdown();
  return 0;
}
