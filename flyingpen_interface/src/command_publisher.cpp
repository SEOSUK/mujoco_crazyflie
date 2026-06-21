#include <rclcpp/rclcpp.hpp>
#include <rclcpp/parameter_client.hpp>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>

#include <ncurses.h>

#include <array>
#include <chrono>
#include <cctype>
#include <cmath>
#include <deque>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <sstream>
#include <string>
#include <vector>

using namespace std::chrono_literals;

class CommandPublisher : public rclcpp::Node
{
public:
  CommandPublisher()
  : Node("command_publisher")
  {
    pos_cmd_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(
      "/su/keyboard_input", 10);
    use_vel_mode_pub_ = this->create_publisher<std_msgs::msg::Float32>(
      "su/use_vel_mode", 10);
    force_pub_ = this->create_publisher<std_msgs::msg::Float32>(
      "su/cmd_force", 10);

    pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
      "/crazyflie/out/pose", 10,
      std::bind(&CommandPublisher::poseCallback, this, std::placeholders::_1));
    input_sub_ = this->create_subscription<std_msgs::msg::Float32MultiArray>(
      "/crazyflie/in/input", 10,
      std::bind(&CommandPublisher::inputCallback, this, std::placeholders::_1));

    observer_param_client_ = std::make_shared<rclcpp::AsyncParametersClient>(
      this, "wrench_observer");

    position_tick_ = declareVector3Parameter("position_tick", {0.1, 0.1, 0.1});
    velocity_tick_ = declareVector3Parameter("velocity_tick", {0.055, 0.055, 0.055});
    yaw_tick_deg_ = this->declare_parameter<double>("yaw_tick_deg", 2.0);
    force_delta_ = this->declare_parameter<double>("force_tick", 0.02);
    gravity_ = this->declare_parameter<double>("gravity", 9.81);
    calibration_window_sec_ = this->declare_parameter<double>("calibration_window_sec", 1.0);

    cmd_xyz_yaw_.fill(0.0);
    latest_pose_xyz_yaw_.fill(0.0);
    latest_input_tau_fz_.fill(0.0);
    force_des_ = 0.0;
    use_vel_mode_ = false;
    trajectory_enabled_ = false;
    position_reset_requested_ = true;
    status_msg_ = "ready";

    for (size_t i = 0; i < history_len_; ++i) {
      input_history_.push_back("-");
    }

    initscr();
    cbreak();
    noecho();
    nodelay(stdscr, TRUE);
    keypad(stdscr, TRUE);

    drawLayout();

    timer_ = this->create_wall_timer(
      50ms, std::bind(&CommandPublisher::timerCallback, this));
  }

  ~CommandPublisher() override
  {
    endwin();
  }

private:
  static constexpr size_t history_len_ = 5;

  std::array<double, 3> declareVector3Parameter(
    const std::string & name,
    const std::vector<double> & default_value)
  {
    const auto values = this->declare_parameter<std::vector<double>>(name, default_value);
    std::array<double, 3> result{{0.0, 0.0, 0.0}};
    if (values.size() != 3) {
      RCLCPP_WARN(
        this->get_logger(),
        "Parameter '%s' must have exactly 3 values. Using zeros.",
        name.c_str());
      return result;
    }

    for (size_t i = 0; i < 3; ++i) {
      result[i] = values[i];
    }
    return result;
  }

  static std::string trimLeft(const std::string & text)
  {
    const auto pos = text.find_first_not_of(" \t");
    return (pos == std::string::npos) ? std::string() : text.substr(pos);
  }

  static bool startsWithKey(const std::string & trimmed, const std::string & key)
  {
    return trimmed.rfind(key + ":", 0) == 0;
  }

  static std::string formatDouble(double value)
  {
    std::ostringstream oss;
    oss << std::fixed << std::setprecision(6) << value;
    return oss.str();
  }

  static std::filesystem::path wrenchObserverYamlPath()
  {
    return std::filesystem::path(__FILE__).parent_path().parent_path() / "config" / "wrench_observer.yaml";
  }

  void timerCallback()
  {
    int ch = 0;
    while ((ch = getch()) != ERR) {
      handleKey(static_cast<char>(ch));
    }

    updateCalibration();
    publishPositionCmd();
    publishUseVelMode();
    drawStatusBlock();
    drawCommandBlock();
  }

  void handleKey(char c)
  {
    const char key = static_cast<char>(std::tolower(static_cast<unsigned char>(c)));

    if (calibration_active_ && key != 't') {
      status_msg_ = "hover calibration running";
      return;
    }

    if (key == 'w') {
      onPositiveX();
    } else if (key == 's') {
      onNegativeX();
    } else if (key == 'a') {
      onNegativeY();
    } else if (key == 'd') {
      onPositiveY();
    } else if (key == 'e') {
      onPositiveZ();
    } else if (key == 'q') {
      onNegativeZ();
    } else if (key == 'z') {
      cmd_xyz_yaw_[3] += yawTickRad();
      pushInputHistory(use_vel_mode_ ? "z : yaw rate += tick" : "z : yaw += tick");
    } else if (key == 'c') {
      cmd_xyz_yaw_[3] -= yawTickRad();
      pushInputHistory(use_vel_mode_ ? "c : yaw rate -= tick" : "c : yaw -= tick");
    } else if (key == 'x') {
      resetActiveCommand();
    } else if (key == 'i') {
      setVelocityMode();
    } else if (key == 'u') {
      setPositionMode();
    } else if (key == 'm') {
      trajectory_enabled_ = true;
      status_msg_ = "trajectory running";
      pushInputHistory("m : trajectory run");
    } else if (key == 'n') {
      trajectory_enabled_ = false;
      status_msg_ = "trajectory stopped";
      pushInputHistory("n : trajectory stop");
    } else if (key == 'j') {
      force_des_ += force_delta_;
      publishForce();
      status_msg_ = "force desired increased";
      pushInputHistory("j : force += tick");
    } else if (key == 'k') {
      force_des_ -= force_delta_;
      publishForce();
      status_msg_ = "force desired decreased";
      pushInputHistory("k : force -= tick");
    } else if (key == 'l') {
      force_des_ = 0.0;
      publishForce();
      status_msg_ = "force desired reset";
      pushInputHistory("l : force reset");
    } else if (key == 'f') {
      beginHoverCalibration();
    } else if (key == 't') {
      status_msg_ = "exit key pressed";
      pushInputHistory("t : quit");
      rclcpp::shutdown();
    }
  }

  void onPositiveX()
  {
    cmd_xyz_yaw_[0] += use_vel_mode_ ? velocity_tick_[0] : position_tick_[0];
    pushInputHistory(use_vel_mode_ ? "w : normal += tick" : "w : x += tick");
  }

  void onNegativeX()
  {
    cmd_xyz_yaw_[0] -= use_vel_mode_ ? velocity_tick_[0] : position_tick_[0];
    pushInputHistory(use_vel_mode_ ? "s : normal -= tick" : "s : x -= tick");
  }

  void onPositiveY()
  {
    cmd_xyz_yaw_[1] += use_vel_mode_ ? velocity_tick_[1] : position_tick_[1];
    pushInputHistory(use_vel_mode_ ? "d : t1 += tick" : "d : y += tick");
  }

  void onNegativeY()
  {
    cmd_xyz_yaw_[1] -= use_vel_mode_ ? velocity_tick_[1] : position_tick_[1];
    pushInputHistory(use_vel_mode_ ? "a : t1 -= tick" : "a : y -= tick");
  }

  void onPositiveZ()
  {
    cmd_xyz_yaw_[2] += use_vel_mode_ ? velocity_tick_[2] : position_tick_[2];
    pushInputHistory(use_vel_mode_ ? "e : t2 += tick" : "e : z += tick");
  }

  void onNegativeZ()
  {
    cmd_xyz_yaw_[2] -= use_vel_mode_ ? velocity_tick_[2] : position_tick_[2];
    pushInputHistory(use_vel_mode_ ? "q : t2 -= tick" : "q : z -= tick");
  }

  void setVelocityMode()
  {
    if (use_vel_mode_) {
      status_msg_ = "already in VELOCITY mode";
      return;
    }

    use_vel_mode_ = true;
    trajectory_enabled_ = false;
    cmd_xyz_yaw_[0] = 0.0;
    cmd_xyz_yaw_[1] = 0.0;
    cmd_xyz_yaw_[2] = 0.0;
    cmd_xyz_yaw_[3] = 0.0;
    status_msg_ = "entered VELOCITY mode, normal estimator ON, yaw aligns to -normal";
    pushInputHistory("i : enter velocity mode");
  }

  void setPositionMode()
  {
    if (!use_vel_mode_) {
      status_msg_ = "already in POSITION mode";
      return;
    }

    use_vel_mode_ = false;
    trajectory_enabled_ = false;
    zeroPositionCommandDelta();
    position_reset_requested_ = true;
    status_msg_ = "entered POSITION mode, command delta reset to zero";
    pushInputHistory("u : enter position mode");
  }

  void resetActiveCommand()
  {
    if (use_vel_mode_) {
      cmd_xyz_yaw_[0] = 0.0;
      cmd_xyz_yaw_[1] = 0.0;
      cmd_xyz_yaw_[2] = 0.0;
      status_msg_ = "velocity command reset to zero";
      pushInputHistory("x : zero velocity cmd");
      return;
    }

    zeroPositionCommandDelta();
    position_reset_requested_ = true;
    status_msg_ = "position command delta reset to zero";
    pushInputHistory("x : hold current pose");
  }

  void zeroPositionCommandDelta()
  {
    cmd_xyz_yaw_[0] = 0.0;
    cmd_xyz_yaw_[1] = 0.0;
    cmd_xyz_yaw_[2] = 0.0;
    cmd_xyz_yaw_[3] = 0.0;
  }

  double yawTickRad() const
  {
    return yaw_tick_deg_ * M_PI / 180.0;
  }

  void beginHoverCalibration()
  {
    if (!has_latest_pose_ || !has_latest_input_) {
      status_msg_ = "hover calibration needs pose and input topics";
      return;
    }

    use_vel_mode_ = false;
    trajectory_enabled_ = false;
    zeroPositionCommandDelta();
    position_reset_requested_ = true;

    calibration_active_ = true;
    calibration_start_time_ = this->now();
    calibration_integral_world_fz_ = 0.0;
    calibration_integral_body_fz_ = 0.0;
    calibration_integral_tau_x_ = 0.0;
    calibration_integral_tau_y_ = 0.0;
    calibration_last_sample_time_ = rclcpp::Time(0, 0, this->get_clock()->get_clock_type());

    status_msg_ = "hover calibration started";
    pushInputHistory("f : hover mass/com calibration");
  }

  void updateCalibration()
  {
    if (!calibration_active_ || !has_latest_input_ || !has_latest_pose_) {
      return;
    }

    const rclcpp::Time now = this->now();
    if (calibration_last_sample_time_.nanoseconds() == 0) {
      calibration_last_sample_time_ = now;
      return;
    }

    const double dt = (now - calibration_last_sample_time_).seconds();
    calibration_last_sample_time_ = now;
    if (!(std::isfinite(dt) && dt > 0.0 && dt < 0.1)) {
      return;
    }

    const double yaw = latest_pose_xyz_yaw_[3] * M_PI / 180.0;
    const double pitch = latest_pose_pitch_rad_;
    const double roll = latest_pose_roll_rad_;
    const double cr = std::cos(roll);
    const double sr = std::sin(roll);
    const double cp = std::cos(pitch);
    const double sp = std::sin(pitch);
    const double cy = std::cos(yaw);
    const double sy = std::sin(yaw);
    const double r20 = cy * sp * cr + sy * sr;
    const double r21 = sy * sp * cr - cy * sr;
    const double r22 = cp * cr;

    const double body_fz = latest_input_tau_fz_[3];
    const double world_fz = r22 * body_fz;
    (void)r20;
    (void)r21;

    calibration_integral_world_fz_ += world_fz * dt;
    calibration_integral_body_fz_ += body_fz * dt;
    calibration_integral_tau_x_ += latest_input_tau_fz_[0] * dt;
    calibration_integral_tau_y_ += latest_input_tau_fz_[1] * dt;

    const double elapsed = (now - calibration_start_time_).seconds();
    if (elapsed < calibration_window_sec_) {
      std::ostringstream oss;
      oss << "hover calibration running (" << std::fixed << std::setprecision(2)
          << elapsed << " / " << calibration_window_sec_ << " s)";
      status_msg_ = oss.str();
      return;
    }

    finishHoverCalibration(elapsed);
  }

  void finishHoverCalibration(double elapsed_sec)
  {
    calibration_active_ = false;
    const double safe_elapsed = std::max(elapsed_sec, 1.0e-6);
    const double avg_world_fz = calibration_integral_world_fz_ / safe_elapsed;
    const double avg_body_fz = calibration_integral_body_fz_ / safe_elapsed;
    const double avg_tau_x = calibration_integral_tau_x_ / safe_elapsed;
    const double avg_tau_y = calibration_integral_tau_y_ / safe_elapsed;

    double calibrated_mass = 0.0;
    if (avg_world_fz > 1.0e-6) {
      calibrated_mass = avg_world_fz / std::max(1.0e-6, gravity_);
    }

    double calibrated_com_x = 0.0;
    double calibrated_com_y = 0.0;
    if (std::fabs(avg_body_fz) > 1.0e-6) {
      calibrated_com_x = avg_tau_y / avg_body_fz;
      calibrated_com_y = -avg_tau_x / avg_body_fz;
    }

    const bool runtime_ok = applyCalibrationToObserver(calibrated_mass, calibrated_com_x, calibrated_com_y);
    const bool yaml_ok = writeCalibrationToYaml(calibrated_mass, calibrated_com_x, calibrated_com_y);

    std::ostringstream oss;
    oss << "hover calibration complete: mass=" << std::fixed << std::setprecision(6)
        << calibrated_mass
        << " comOffX=" << calibrated_com_x
        << " comOffY=" << calibrated_com_y
        << " | runtime=" << (runtime_ok ? "ok" : "fail")
        << " yaml=" << (yaml_ok ? "ok" : "fail");
    status_msg_ = oss.str();

    RCLCPP_INFO(
      this->get_logger(),
      "Hover calibration complete: mass=%.6f comOffX=%.6f comOffY=%.6f avgWorldFz=%.6f avgTauXY=(%.6f, %.6f)",
      calibrated_mass, calibrated_com_x, calibrated_com_y, avg_world_fz, avg_tau_x, avg_tau_y);
  }

  bool applyCalibrationToObserver(double mass, double com_x, double com_y)
  {
    if (!observer_param_client_) {
      return false;
    }

    if (!observer_param_client_->wait_for_service(200ms)) {
      RCLCPP_WARN(this->get_logger(), "wrench_observer parameter service not ready");
      return false;
    }

    auto future = observer_param_client_->set_parameters(
      {
        rclcpp::Parameter("mass", mass),
        rclcpp::Parameter("su_wrench.mass", mass),
        rclcpp::Parameter("su_wrench.comOffX", com_x),
        rclcpp::Parameter("su_wrench.comOffY", com_y)
      });

    if (future.wait_for(300ms) != std::future_status::ready) {
      RCLCPP_WARN(this->get_logger(), "Timed out waiting for wrench_observer parameter update");
      return false;
    }

    bool ok = true;
    for (const auto & result : future.get()) {
      ok = ok && result.successful;
      if (!result.successful && !result.reason.empty()) {
        RCLCPP_WARN(this->get_logger(), "Parameter update failed: %s", result.reason.c_str());
      }
    }
    return ok;
  }

  bool writeCalibrationToYaml(double mass, double com_x, double com_y)
  {
    const auto path = wrenchObserverYamlPath();
    std::ifstream in(path);
    if (!in.is_open()) {
      RCLCPP_WARN(this->get_logger(), "Failed to open %s", path.c_str());
      return false;
    }

    std::vector<std::string> lines;
    std::string line;
    while (std::getline(in, line)) {
      lines.push_back(line);
    }
    in.close();

    bool in_ros_parameters = false;
    bool in_su_wrench = false;
    bool mass_updated = false;
    bool su_mass_updated = false;
    bool com_x_updated = false;
    bool com_y_updated = false;

    for (auto & current_line : lines) {
      const std::string trimmed = trimLeft(current_line);
      const size_t indent = current_line.size() - trimmed.size();

      if (indent == 2 && trimmed == "ros__parameters:") {
        in_ros_parameters = true;
        in_su_wrench = false;
        continue;
      }

      if (in_ros_parameters && indent <= 2 && !trimmed.empty() && trimmed.back() == ':') {
        in_ros_parameters = false;
        in_su_wrench = false;
      }

      if (in_ros_parameters && indent == 4 && trimmed == "su_wrench:") {
        in_su_wrench = true;
        continue;
      }

      if (in_su_wrench && indent <= 4 && !(indent == 4 && trimmed == "su_wrench:")) {
        in_su_wrench = false;
      }

      if (in_su_wrench && indent == 6 && startsWithKey(trimmed, "mass")) {
        current_line = "      mass: " + formatDouble(mass);
        su_mass_updated = true;
      } else if (in_su_wrench && indent == 6 && startsWithKey(trimmed, "comOffX")) {
        current_line = "      comOffX: " + formatDouble(com_x);
        com_x_updated = true;
      } else if (in_su_wrench && indent == 6 && startsWithKey(trimmed, "comOffY")) {
        current_line = "      comOffY: " + formatDouble(com_y);
        com_y_updated = true;
      } else if (in_ros_parameters && !in_su_wrench && indent == 4 && startsWithKey(trimmed, "mass")) {
        current_line = "    mass: " + formatDouble(mass);
        mass_updated = true;
      }
    }

    if (!(mass_updated && su_mass_updated && com_x_updated && com_y_updated)) {
      RCLCPP_WARN(this->get_logger(), "Failed to find calibration keys in wrench_observer.yaml");
      return false;
    }

    std::ofstream out(path, std::ios::trunc);
    if (!out.is_open()) {
      RCLCPP_WARN(this->get_logger(), "Failed to write %s", path.c_str());
      return false;
    }

    for (size_t i = 0; i < lines.size(); ++i) {
      out << lines[i];
      if (i + 1 < lines.size()) {
        out << '\n';
      }
    }
    return true;
  }

  void publishPositionCmd()
  {
    std_msgs::msg::Float64MultiArray msg;
    msg.data.resize(6);
    msg.data[0] = cmd_xyz_yaw_[0];
    msg.data[1] = cmd_xyz_yaw_[1];
    msg.data[2] = cmd_xyz_yaw_[2];
    msg.data[3] = cmd_xyz_yaw_[3];
    msg.data[4] = trajectory_enabled_ ? 1.0 : 0.0;
    msg.data[5] = position_reset_requested_ ? 1.0 : 0.0;
    pos_cmd_pub_->publish(msg);
    position_reset_requested_ = false;
  }

  void publishForce()
  {
    std_msgs::msg::Float32 msg;
    msg.data = static_cast<float>(force_des_);
    force_pub_->publish(msg);
  }

  void publishUseVelMode()
  {
    std_msgs::msg::Float32 msg;
    msg.data = use_vel_mode_ ? 1.0f : 0.0f;
    use_vel_mode_pub_->publish(msg);
  }

  void poseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
  {
    if (!msg) {
      return;
    }

    const auto & q = msg->pose.orientation;
    const double sinr_cosp = 2.0 * (q.w * q.x + q.y * q.z);
    const double cosr_cosp = 1.0 - 2.0 * (q.x * q.x + q.y * q.y);
    latest_pose_roll_rad_ = std::atan2(sinr_cosp, cosr_cosp);

    const double sinp = 2.0 * (q.w * q.y - q.z * q.x);
    latest_pose_pitch_rad_ = std::abs(sinp) >= 1.0 ? std::copysign(M_PI / 2.0, sinp) : std::asin(sinp);

    const double siny_cosp = 2.0 * (q.w * q.z + q.x * q.y);
    const double cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
    latest_pose_xyz_yaw_[3] = std::atan2(siny_cosp, cosy_cosp) * 180.0 / M_PI;

    latest_pose_xyz_yaw_[0] = msg->pose.position.x;
    latest_pose_xyz_yaw_[1] = msg->pose.position.y;
    latest_pose_xyz_yaw_[2] = msg->pose.position.z;
    has_latest_pose_ = true;
  }

  void inputCallback(const std_msgs::msg::Float32MultiArray::SharedPtr msg)
  {
    if (!msg || msg->data.size() < 4) {
      return;
    }

    latest_input_tau_fz_[0] = msg->data[0];
    latest_input_tau_fz_[1] = msg->data[1];
    latest_input_tau_fz_[2] = msg->data[2];
    latest_input_tau_fz_[3] = msg->data[3];
    has_latest_input_ = true;
  }

  void pushInputHistory(const std::string & entry)
  {
    input_history_.push_front(entry);
    while (input_history_.size() > history_len_) {
      input_history_.pop_back();
    }
    while (input_history_.size() < history_len_) {
      input_history_.push_back("-");
    }
  }

  void drawLayout()
  {
    clear();
    mvprintw(0, 0, "========================usage========================");
    mvprintw(2, 0, "position: w/s(x), a/d(y), e/q(z), z/c(yaw offset), x(hold), i->velocity");
    mvprintw(3, 0, "velocity: w/s/a/d/e/q(v), z/c(yaw rate), x(zero vel), u->position");
    mvprintw(4, 0, "force/cal: j/k/l(cmd_fx), f(hover mass/com + yaml sync), t quit");
    mvprintw(6, 0, "========================status========================");
    mvprintw(16, 0, "========================command========================");
    refresh();
  }

  void drawStatusBlock()
  {
    move(8, 0);
    clrtoeol();
    printw(
      "mode: %s | traj: %s | pos_tick=(%.3f %.3f %.3f) | vel_tick=(%.3f %.3f %.3f)",
      use_vel_mode_ ? "VELOCITY" : "POSITION",
      trajectory_enabled_ ? "ON" : "OFF",
      position_tick_[0], position_tick_[1], position_tick_[2],
      velocity_tick_[0], velocity_tick_[1], velocity_tick_[2]);

    move(9, 0);
    clrtoeol();
    printw(
      "force_des=%.3f | latest input tau/fz=(%.4f %.4f %.4f %.4f)",
      force_des_,
      latest_input_tau_fz_[0], latest_input_tau_fz_[1], latest_input_tau_fz_[2], latest_input_tau_fz_[3]);

    move(10, 0);
    clrtoeol();
    if (has_latest_pose_) {
      printw(
        "latest pose=(%.3f %.3f %.3f) yaw=%.1f deg",
        latest_pose_xyz_yaw_[0], latest_pose_xyz_yaw_[1], latest_pose_xyz_yaw_[2], latest_pose_xyz_yaw_[3]);
    } else {
      printw("latest pose: waiting for /crazyflie/out/pose");
    }

    move(11, 0);
    clrtoeol();
    if (calibration_active_) {
      const double elapsed = (this->now() - calibration_start_time_).seconds();
      printw("calibration: active %.2f / %.2f s", elapsed, calibration_window_sec_);
    } else {
      printw("calibration: idle");
    }

    move(12, 0);
    clrtoeol();
    printw("status: %s", status_msg_.c_str());

    refresh();
  }

  void drawCommandBlock()
  {
    move(18, 0);
    clrtoeol();
    if (use_vel_mode_) {
      printw(
        "velocity cmd [normal,t1,t2] = %.3f , %.3f , %.3f",
        cmd_xyz_yaw_[0], cmd_xyz_yaw_[1], cmd_xyz_yaw_[2]);
    } else {
      printw(
        "position cmd xyz = %.3f , %.3f , %.3f",
        cmd_xyz_yaw_[0], cmd_xyz_yaw_[1], cmd_xyz_yaw_[2]);
    }

    move(19, 0);
    clrtoeol();
    if (use_vel_mode_) {
      printw("yaw rate cmd = %.1f deg/s", cmd_xyz_yaw_[3] * 180.0 / M_PI);
    } else {
      printw("yaw offset cmd = %.1f deg", cmd_xyz_yaw_[3] * 180.0 / M_PI);
    }

    mvprintw(21, 0, "last inputs (recent 5):");
    for (size_t i = 0; i < history_len_; ++i) {
      move(22 + static_cast<int>(i), 0);
      clrtoeol();
      printw("  %zu) %s", i + 1, input_history_[i].c_str());
    }
    refresh();
  }

  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr pos_cmd_pub_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr use_vel_mode_pub_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr force_pub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_;
  rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr input_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::shared_ptr<rclcpp::AsyncParametersClient> observer_param_client_;

  std::array<double, 4> cmd_xyz_yaw_;
  std::array<double, 4> latest_pose_xyz_yaw_;
  std::array<double, 4> latest_input_tau_fz_;
  std::array<double, 3> position_tick_;
  std::array<double, 3> velocity_tick_;
  std::deque<std::string> input_history_;

  double yaw_tick_deg_{2.0};
  double force_delta_{0.035};
  double force_des_{0.0};
  double gravity_{9.81};
  double calibration_window_sec_{1.0};
  double latest_pose_roll_rad_{0.0};
  double latest_pose_pitch_rad_{0.0};

  bool use_vel_mode_{false};
  bool trajectory_enabled_{false};
  bool position_reset_requested_{true};
  bool has_latest_pose_{false};
  bool has_latest_input_{false};
  bool calibration_active_{false};

  rclcpp::Time calibration_start_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time calibration_last_sample_time_{0, 0, RCL_ROS_TIME};
  double calibration_integral_world_fz_{0.0};
  double calibration_integral_body_fz_{0.0};
  double calibration_integral_tau_x_{0.0};
  double calibration_integral_tau_y_{0.0};

  std::string status_msg_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CommandPublisher>());
  rclcpp::shutdown();
  return 0;
}
