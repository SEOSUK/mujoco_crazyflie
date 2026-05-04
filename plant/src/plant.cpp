#include <array>
#include <atomic>
#include <algorithm>
#include <chrono>
#include <cmath>
#include <csignal>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include <GLFW/glfw3.h>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/vector3_stamped.hpp>
#include <mujoco/mujoco.h>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>

namespace {

constexpr double kPhysicsHz = 500.0;
constexpr double kPubHz = 200.0;
constexpr double kViewerHz = 60.0;
constexpr double kTwoPi = 6.28318530717958647692;

std::array<double, 4> quatWxyzToXyzw(const double* q_wxyz) {
  return {q_wxyz[1], q_wxyz[2], q_wxyz[3], q_wxyz[0]};
}

std::array<double, 4> quatNormalizeWxyz(const double* q_wxyz) {
  double norm = std::sqrt(
    q_wxyz[0] * q_wxyz[0] + q_wxyz[1] * q_wxyz[1] +
    q_wxyz[2] * q_wxyz[2] + q_wxyz[3] * q_wxyz[3]);
  if (norm < 1e-12) {
    return {1.0, 0.0, 0.0, 0.0};
  }
  return {q_wxyz[0] / norm, q_wxyz[1] / norm, q_wxyz[2] / norm, q_wxyz[3] / norm};
}

std::array<double, 9> rotmatFromQuatWxyz(const std::array<double, 4>& q) {
  const double w = q[0];
  const double x = q[1];
  const double y = q[2];
  const double z = q[3];
  return {
    1.0 - 2.0 * (y * y + z * z), 2.0 * (x * y - w * z), 2.0 * (x * z + w * y),
    2.0 * (x * y + w * z), 1.0 - 2.0 * (x * x + z * z), 2.0 * (y * z - w * x),
    2.0 * (x * z - w * y), 2.0 * (y * z + w * x), 1.0 - 2.0 * (x * x + y * y)
  };
}

std::array<double, 3> matVecMul(const std::array<double, 9>& r, const std::array<double, 3>& v) {
  return {
    r[0] * v[0] + r[1] * v[1] + r[2] * v[2],
    r[3] * v[0] + r[4] * v[1] + r[5] * v[2],
    r[6] * v[0] + r[7] * v[1] + r[8] * v[2]
  };
}

std::array<double, 3> matTransposeVecMul(const std::array<double, 9>& r, const std::array<double, 3>& v) {
  return {
    r[0] * v[0] + r[3] * v[1] + r[6] * v[2],
    r[1] * v[0] + r[4] * v[1] + r[7] * v[2],
    r[2] * v[0] + r[5] * v[1] + r[8] * v[2]
  };
}

double wrapAngle(double angle) {
  angle = std::fmod(angle, kTwoPi);
  if (angle < 0.0) {
    angle += kTwoPi;
  }
  return angle;
}

}  // namespace

class CrazyfliePlant : public rclcpp::Node {
public:
  CrazyfliePlant()
  : Node("mujoco_crazyflie_plant") {
    declare_parameter("physics_hz", kPhysicsHz);
    declare_parameter("pub_hz", kPubHz);
    declare_parameter("viewer_hz", kViewerHz);
    declare_parameter("arm_xy", 0.035355);
    declare_parameter("k_tau", 0.00594);
    declare_parameter("motor_dir", std::vector<double>{1.0, -1.0, 1.0, -1.0});
    declare_parameter("thrust_min", 0.0);
    declare_parameter("thrust_max", 0.20);
    declare_parameter("prop_visual_speed", 180.0);

    physics_hz_ = get_parameter("physics_hz").as_double();
    pub_hz_ = get_parameter("pub_hz").as_double();
    viewer_hz_ = get_parameter("viewer_hz").as_double();
    arm_xy_ = get_parameter("arm_xy").as_double();
    k_tau_ = get_parameter("k_tau").as_double();
    thrust_min_ = get_parameter("thrust_min").as_double();
    thrust_max_ = get_parameter("thrust_max").as_double();
    prop_visual_speed_ = get_parameter("prop_visual_speed").as_double();

    auto motor_dir = get_parameter("motor_dir").as_double_array();
    for (size_t i = 0; i < 4; ++i) {
      motor_dir_[i] = i < motor_dir.size() ? motor_dir[i] : (i % 2 == 0 ? 1.0 : -1.0);
    }

    const auto pkg_share = ament_index_cpp::get_package_share_directory("plant");
    const std::string xml_path = pkg_share + "/data/cf21B_500.xml";
    RCLCPP_INFO(get_logger(), "Loading MuJoCo model: %s", xml_path.c_str());

    char error[1024] = "";
    model_ = mj_loadXML(xml_path.c_str(), nullptr, error, sizeof(error));
    if (!model_) {
      throw std::runtime_error(std::string("mj_loadXML failed: ") + error);
    }
    data_ = mj_makeData(model_);
    if (!data_) {
      throw std::runtime_error("mj_makeData failed");
    }
    model_->opt.timestep = 1.0 / std::max(1e-9, physics_hz_);

    imu_acc_sid_ = findSensor("imu_acc");
    imu_gyro_sid_ = findSensor("imu_gyro");
    for (int i = 0; i < 4; ++i) {
      act_force_ids_[i] = findActuator("motor" + std::to_string(i) + "_force");
      act_torque_ids_[i] = findActuator("motor" + std::to_string(i) + "_torque");
      prop_joint_ids_[i] = findJoint("prop" + std::to_string(i) + "_hinge");
      if (prop_joint_ids_[i] >= 0) {
        prop_qpos_adr_[i] = model_->jnt_qposadr[prop_joint_ids_[i]];
        prop_qvel_adr_[i] = model_->jnt_dofadr[prop_joint_ids_[i]];
      }
    }

    buildAllocationMatrix();

    sub_input_ = create_subscription<std_msgs::msg::Float32MultiArray>(
      "/crazyflie/in/input", 10,
      std::bind(&CrazyfliePlant::cbInput, this, std::placeholders::_1));

    pub_pose_ = create_publisher<geometry_msgs::msg::PoseStamped>("/crazyflie/out/pose", 10);
    pub_vel_ = create_publisher<geometry_msgs::msg::Vector3Stamped>("/crazyflie/out/vel", 10);
    pub_angvel_ = create_publisher<geometry_msgs::msg::Vector3Stamped>("/crazyflie/out/ang_vel", 10);
    pub_acc_ = create_publisher<geometry_msgs::msg::Vector3Stamped>("/crazyflie/out/acc", 10);
    pub_angacc_ = create_publisher<geometry_msgs::msg::Vector3Stamped>("/crazyflie/out/ang_acc", 10);
    pub_angvel_gt_ = create_publisher<geometry_msgs::msg::Vector3Stamped>("/crazyflie/out/ang_vel_gt", 10);

    sim_thread_ = std::thread(&CrazyfliePlant::simLoop, this);
    viewer_thread_ = std::thread(&CrazyfliePlant::viewerLoop, this);

    RCLCPP_INFO(
      get_logger(), "Running: physics=%.1fHz, pub=%.1fHz, viewer=%.1fHz",
      physics_hz_, pub_hz_, viewer_hz_);
  }

  ~CrazyfliePlant() override {
    close();
    if (viewer_thread_.joinable()) {
      viewer_thread_.join();
    }
    if (sim_thread_.joinable()) {
      sim_thread_.join();
    }
    if (data_) {
      mj_deleteData(data_);
      data_ = nullptr;
    }
    if (model_) {
      mj_deleteModel(model_);
      model_ = nullptr;
    }
  }

  void close() {
    stop_.store(true);
  }

private:
  struct State {
    std::array<double, 3> pos {};
    std::array<double, 4> quat_wxyz {};
    std::array<double, 3> linvel_w {};
    std::array<double, 3> angvel_b_gt {};
  };

  int findSensor(const std::string& name) {
    const int sid = mj_name2id(model_, mjOBJ_SENSOR, name.c_str());
    if (sid < 0) {
      RCLCPP_WARN(get_logger(), "Sensor NOT found: %s", name.c_str());
    }
    return sid;
  }

  int findActuator(const std::string& name) {
    return mj_name2id(model_, mjOBJ_ACTUATOR, name.c_str());
  }

  int findJoint(const std::string& name) {
    return mj_name2id(model_, mjOBJ_JOINT, name.c_str());
  }

  std::array<double, 3> readSensorVec3(int sid) const {
    if (sid < 0) {
      return {0.0, 0.0, 0.0};
    }
    const int adr = model_->sensor_adr[sid];
    return {
      data_->sensordata[adr],
      data_->sensordata[adr + 1],
      data_->sensordata[adr + 2]
    };
  }

  void buildAllocationMatrix() {
    const std::array<double, 4> x {+arm_xy_, -arm_xy_, -arm_xy_, +arm_xy_};
    const std::array<double, 4> y {-arm_xy_, -arm_xy_, +arm_xy_, +arm_xy_};

    for (int col = 0; col < 4; ++col) {
      B_[0][col] = y[col];
      B_[1][col] = -x[col];
      B_[2][col] = motor_dir_[col] * k_tau_;
      B_[3][col] = 1.0;
    }
  }

  std::array<double, 4> solveMotorThrust(const std::array<double, 4>& wrench) const {
    const double tx = wrench[0];
    const double ty = wrench[1];
    const double tz = wrench[2];
    const double fz = wrench[3];

    const double inv_4a = 1.0 / (4.0 * arm_xy_);
    const double inv_4k = 1.0 / (4.0 * k_tau_);

    std::array<double, 4> f {};
    f[0] = 0.25 * fz - inv_4a * tx - inv_4a * ty + inv_4k * tz;
    f[1] = 0.25 * fz - inv_4a * tx + inv_4a * ty - inv_4k * tz;
    f[2] = 0.25 * fz + inv_4a * tx + inv_4a * ty + inv_4k * tz;
    f[3] = 0.25 * fz + inv_4a * tx - inv_4a * ty - inv_4k * tz;

    for (double& value : f) {
      value = std::clamp(value, thrust_min_, thrust_max_);
    }
    return f;
  }

  void cbInput(const std_msgs::msg::Float32MultiArray::SharedPtr msg) {
    if (msg->data.size() < 4) {
      RCLCPP_WARN(get_logger(), "'/crazyflie/in/input' needs 4 floats: [tau_x, tau_y, tau_z, Fz]");
      return;
    }
    std::scoped_lock lock(mutex_);
    for (size_t i = 0; i < 4; ++i) {
      u_[i] = static_cast<double>(msg->data[i]);
    }
  }

  void applyControl() {
    current_motor_thrust_ = solveMotorThrust(u_);

    for (int i = 0; i < 4; ++i) {
      if (act_force_ids_[i] >= 0) {
        data_->ctrl[act_force_ids_[i]] = current_motor_thrust_[i];
      } else if (i < model_->nu) {
        data_->ctrl[i] = current_motor_thrust_[i];
      }
    }

    for (int i = 0; i < 4; ++i) {
      const double tau = motor_dir_[i] * k_tau_ * current_motor_thrust_[i];
      if (act_torque_ids_[i] >= 0) {
        data_->ctrl[act_torque_ids_[i]] = tau;
      }
    }
  }

  void updatePropellerVisuals(double dt) {
    const double thrust_scale = thrust_max_ > 1e-9 ? thrust_max_ : 1.0;
    for (int i = 0; i < 4; ++i) {
      if (prop_joint_ids_[i] < 0) {
        continue;
      }
      const double normalized = std::clamp(current_motor_thrust_[i] / thrust_scale, 0.0, 1.0);
      const double omega = motor_dir_[i] * prop_visual_speed_ * normalized;
      prop_angle_[i] = wrapAngle(prop_angle_[i] + omega * dt);
      data_->qpos[prop_qpos_adr_[i]] = prop_angle_[i];
      data_->qvel[prop_qvel_adr_[i]] = omega;
    }
  }

  State readState() const {
    State state;
    for (int i = 0; i < 3; ++i) {
      state.pos[i] = data_->qpos[i];
      state.linvel_w[i] = data_->qvel[i];
    }
    const auto quat = quatNormalizeWxyz(&data_->qpos[3]);
    state.quat_wxyz = quat;

    const std::array<double, 3> angvel_w {data_->qvel[3], data_->qvel[4], data_->qvel[5]};
    const auto r_bw = rotmatFromQuatWxyz(quat);
    state.angvel_b_gt = matTransposeVecMul(r_bw, angvel_w);
    return state;
  }

  std::array<double, 3> readImuAccWorld(const std::array<double, 4>& quat_wxyz) const {
    const auto acc_b = readSensorVec3(imu_acc_sid_);
    const auto r_bw = rotmatFromQuatWxyz(quat_wxyz);
    auto acc_w = matVecMul(r_bw, acc_b);
    for (int i = 0; i < 3; ++i) {
      acc_w[i] += model_->opt.gravity[i];
    }
    return acc_w;
  }

  void publishOutputs(double dt_sim) {
    const auto state = readState();
    const auto linacc_w = readImuAccWorld(state.quat_wxyz);
    const auto gyro_b = readSensorVec3(imu_gyro_sid_);

    std::array<double, 3> angacc_b {};
    if (!prev_gyro_valid_) {
      prev_gyro_valid_ = true;
    } else {
      for (int i = 0; i < 3; ++i) {
        angacc_b[i] = (gyro_b[i] - prev_gyro_raw_b_[i]) / std::max(1e-6, dt_sim);
      }
    }
    prev_gyro_raw_b_ = gyro_b;

    const auto stamp = now();

    geometry_msgs::msg::PoseStamped pose_msg;
    pose_msg.header.stamp = stamp;
    pose_msg.header.frame_id = "world";
    pose_msg.pose.position.x = state.pos[0];
    pose_msg.pose.position.y = state.pos[1];
    pose_msg.pose.position.z = state.pos[2];
    const auto quat_xyzw = quatWxyzToXyzw(state.quat_wxyz.data());
    pose_msg.pose.orientation.x = quat_xyzw[0];
    pose_msg.pose.orientation.y = quat_xyzw[1];
    pose_msg.pose.orientation.z = quat_xyzw[2];
    pose_msg.pose.orientation.w = quat_xyzw[3];
    pub_pose_->publish(pose_msg);

    geometry_msgs::msg::Vector3Stamped vel_msg;
    vel_msg.header = pose_msg.header;
    vel_msg.vector.x = state.linvel_w[0];
    vel_msg.vector.y = state.linvel_w[1];
    vel_msg.vector.z = state.linvel_w[2];
    pub_vel_->publish(vel_msg);

    geometry_msgs::msg::Vector3Stamped angvel_msg;
    angvel_msg.header = pose_msg.header;
    angvel_msg.header.frame_id = "body";
    angvel_msg.vector.x = gyro_b[0];
    angvel_msg.vector.y = gyro_b[1];
    angvel_msg.vector.z = gyro_b[2];
    pub_angvel_->publish(angvel_msg);

    geometry_msgs::msg::Vector3Stamped angvel_gt_msg;
    angvel_gt_msg.header = angvel_msg.header;
    angvel_gt_msg.vector.x = state.angvel_b_gt[0];
    angvel_gt_msg.vector.y = state.angvel_b_gt[1];
    angvel_gt_msg.vector.z = state.angvel_b_gt[2];
    pub_angvel_gt_->publish(angvel_gt_msg);

    geometry_msgs::msg::Vector3Stamped acc_msg;
    acc_msg.header = pose_msg.header;
    acc_msg.vector.x = linacc_w[0];
    acc_msg.vector.y = linacc_w[1];
    acc_msg.vector.z = linacc_w[2];
    pub_acc_->publish(acc_msg);

    geometry_msgs::msg::Vector3Stamped angacc_msg;
    angacc_msg.header = angvel_msg.header;
    angacc_msg.vector.x = angacc_b[0];
    angacc_msg.vector.y = angacc_b[1];
    angacc_msg.vector.z = angacc_b[2];
    pub_angacc_->publish(angacc_msg);
  }

  void simLoop() {
    const double dt = 1.0 / std::max(1e-9, physics_hz_);
    const int pub_decim = std::max(1, static_cast<int>(std::llround(physics_hz_ / std::max(1e-9, pub_hz_))));
    int step_count = 0;
    auto next_step = std::chrono::steady_clock::now();

    while (rclcpp::ok() && !stop_.load()) {
      const auto now = std::chrono::steady_clock::now();
      if (now < next_step) {
        std::this_thread::sleep_for(next_step - now);
        continue;
      }

      {
        std::scoped_lock lock(mutex_);
        applyControl();
        updatePropellerVisuals(dt);
        mj_step(model_, data_);
        ++step_count;
        if ((step_count % pub_decim) == 0) {
          publishOutputs(dt);
        }
      }

      next_step += std::chrono::duration_cast<std::chrono::steady_clock::duration>(
        std::chrono::duration<double>(dt));
    }
  }

  void viewerLoop() {
    if (viewer_hz_ <= 0.0) {
      return;
    }
    if (!glfwInit()) {
      RCLCPP_WARN(get_logger(), "GLFW init failed");
      return;
    }

    GLFWwindow* window = glfwCreateWindow(1280, 720, "MuJoCo Crazyflie Plant", nullptr, nullptr);
    if (!window) {
      glfwTerminate();
      RCLCPP_WARN(get_logger(), "GLFW window creation failed");
      return;
    }

    glfwMakeContextCurrent(window);
    glfwSwapInterval(1);
    glfwSetWindowUserPointer(window, this);
    glfwSetCursorPosCallback(window, &CrazyfliePlant::mouseMoveCallback);
    glfwSetMouseButtonCallback(window, &CrazyfliePlant::mouseButtonCallback);
    glfwSetScrollCallback(window, &CrazyfliePlant::scrollCallback);

    mjrContext context;
    mjv_defaultCamera(&cam_);
    mjv_defaultOption(&opt_);
    mjv_defaultPerturb(&pert_);
    mjv_defaultScene(&scene_);
    mjr_defaultContext(&context);

    mjv_makeScene(model_, &scene_, 2000);
    mjr_makeContext(model_, &context, mjFONTSCALE_150);

    cam_.type = mjCAMERA_FREE;
    cam_.trackbodyid = -1;
    cam_.distance = 2.0;
    cam_.azimuth = 135.0;
    cam_.elevation = -20.0;
    cam_.lookat[0] = 0.0;
    cam_.lookat[1] = 0.0;
    cam_.lookat[2] = 0.8;

    const double dt = 1.0 / std::max(1e-9, viewer_hz_);
    while (!glfwWindowShouldClose(window) && rclcpp::ok() && !stop_.load()) {
      const auto frame_start = std::chrono::steady_clock::now();

      int width = 0;
      int height = 0;
      glfwGetFramebufferSize(window, &width, &height);
      mjrRect viewport {0, 0, width, height};

      {
        std::scoped_lock lock(mutex_);
        mjv_updateScene(model_, data_, &opt_, &pert_, &cam_, mjCAT_ALL, &scene_);
      }
      mjr_render(viewport, &scene_, &context);
      glfwSwapBuffers(window);
      glfwPollEvents();

      const auto elapsed = std::chrono::steady_clock::now() - frame_start;
      const auto sleep_time = std::chrono::duration<double>(dt) - elapsed;
      if (sleep_time.count() > 0.0) {
        std::this_thread::sleep_for(
          std::chrono::duration_cast<std::chrono::steady_clock::duration>(sleep_time));
      }
    }

    mjr_freeContext(&context);
    mjv_freeScene(&scene_);
    glfwDestroyWindow(window);
    glfwTerminate();
  }

  static void mouseButtonCallback(GLFWwindow* window, int button, int act, int mods) {
    (void)mods;
    auto* self = static_cast<CrazyfliePlant*>(glfwGetWindowUserPointer(window));
    if (self == nullptr) {
      return;
    }

    self->button_left_ = (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_LEFT) == GLFW_PRESS);
    self->button_middle_ = (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_MIDDLE) == GLFW_PRESS);
    self->button_right_ = (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_RIGHT) == GLFW_PRESS);
    glfwGetCursorPos(window, &self->last_x_, &self->last_y_);

    if (button == GLFW_MOUSE_BUTTON_LEFT && act == GLFW_PRESS) {
      self->shift_pressed_ =
        (glfwGetKey(window, GLFW_KEY_LEFT_SHIFT) == GLFW_PRESS) ||
        (glfwGetKey(window, GLFW_KEY_RIGHT_SHIFT) == GLFW_PRESS);
    }
  }

  static void mouseMoveCallback(GLFWwindow* window, double xpos, double ypos) {
    auto* self = static_cast<CrazyfliePlant*>(glfwGetWindowUserPointer(window));
    if (self == nullptr) {
      return;
    }

    if (!self->button_left_ && !self->button_middle_ && !self->button_right_) {
      return;
    }

    double dx = xpos - self->last_x_;
    double dy = ypos - self->last_y_;
    self->last_x_ = xpos;
    self->last_y_ = ypos;

    int width = 0;
    int height = 0;
    glfwGetWindowSize(window, &width, &height);
    if (height <= 0) {
      return;
    }

    mjtMouse action;
    if (self->button_right_) {
      action = self->shift_pressed_ ? mjMOUSE_MOVE_H : mjMOUSE_MOVE_V;
    } else if (self->button_left_) {
      action = self->shift_pressed_ ? mjMOUSE_ROTATE_H : mjMOUSE_ROTATE_V;
    } else {
      action = mjMOUSE_ZOOM;
    }

    std::scoped_lock lock(self->mutex_);
    mjv_moveCamera(
      self->model_, action,
      dx / static_cast<double>(height),
      dy / static_cast<double>(height),
      &self->scene_, &self->cam_);
  }

  static void scrollCallback(GLFWwindow* window, double xoffset, double yoffset) {
    (void)xoffset;
    auto* self = static_cast<CrazyfliePlant*>(glfwGetWindowUserPointer(window));
    if (self == nullptr) {
      return;
    }

    std::scoped_lock lock(self->mutex_);
    mjv_moveCamera(
      self->model_, mjMOUSE_ZOOM, 0.0, -0.05 * yoffset,
      &self->scene_, &self->cam_);
  }

  mjModel* model_ {nullptr};
  mjData* data_ {nullptr};

  double physics_hz_ {kPhysicsHz};
  double pub_hz_ {kPubHz};
  double viewer_hz_ {kViewerHz};
  double arm_xy_ {0.035355};
  double k_tau_ {0.00594};
  double thrust_min_ {0.0};
  double thrust_max_ {0.20};
  double prop_visual_speed_ {180.0};

  std::array<double, 4> motor_dir_ {1.0, -1.0, 1.0, -1.0};
  std::array<double, 4> u_ {};
  std::array<double, 4> current_motor_thrust_ {};
  std::array<double, 4> prop_angle_ {};
  std::array<std::array<double, 4>, 4> B_ {};

  std::array<int, 4> act_force_ids_ {-1, -1, -1, -1};
  std::array<int, 4> act_torque_ids_ {-1, -1, -1, -1};
  std::array<int, 4> prop_joint_ids_ {-1, -1, -1, -1};
  std::array<int, 4> prop_qpos_adr_ {-1, -1, -1, -1};
  std::array<int, 4> prop_qvel_adr_ {-1, -1, -1, -1};
  int imu_acc_sid_ {-1};
  int imu_gyro_sid_ {-1};

  std::array<double, 3> prev_gyro_raw_b_ {};
  bool prev_gyro_valid_ {false};

  std::mutex mutex_;
  std::atomic<bool> stop_ {false};
  bool button_left_ {false};
  bool button_middle_ {false};
  bool button_right_ {false};
  bool shift_pressed_ {false};
  double last_x_ {0.0};
  double last_y_ {0.0};
  std::thread sim_thread_;
  std::thread viewer_thread_;
  mjvCamera cam_ {};
  mjvOption opt_ {};
  mjvScene scene_ {};
  mjvPerturb pert_ {};

  rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr sub_input_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pub_pose_;
  rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr pub_vel_;
  rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr pub_angvel_;
  rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr pub_acc_;
  rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr pub_angacc_;
  rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr pub_angvel_gt_;
};

CrazyfliePlant* g_node = nullptr;

void signalHandler(int) {
  if (g_node != nullptr) {
    g_node->close();
  }
}

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<CrazyfliePlant>();
  g_node = node.get();
  std::signal(SIGINT, signalHandler);
  rclcpp::spin(node);
  g_node = nullptr;
  node->close();
  rclcpp::shutdown();
  return 0;
}
