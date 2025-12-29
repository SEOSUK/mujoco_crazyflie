// data_logging.cpp
//
// Publishes /data_logging_msgs (Float64MultiArray) and also logs the same data to CSV.
//
// CSV:
//   dir  : ~/mujoco_crazyflie/src/flyingpen_interface/bag
//   name : strftime("%m%d%H%M").csv
//
// Note:
// - logs "latest" values (not time-synchronized).
// - writes one row per publish().

#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/vector3_stamped.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>

#include <Eigen/Dense>
#include <mutex>
#include <cmath>
#include <cstdint>

#include <fstream>
#include <iomanip>
#include <string>
#include <ctime>
#include <filesystem>

static inline double roll_from_quat(double x, double y, double z, double w)
{
  const double sinr_cosp = 2.0 * (w*x + y*z);
  const double cosr_cosp = 1.0 - 2.0 * (x*x + y*y);
  return std::atan2(sinr_cosp, cosr_cosp);
}

static inline double pitch_from_quat(double x, double y, double z, double w)
{
  const double sinp = 2.0 * (w*y - z*x);
  if (std::abs(sinp) >= 1.0) {
    return std::copysign(M_PI / 2.0, sinp);
  }
  return std::asin(sinp);
}

static inline double yaw_from_quat(double x, double y, double z, double w)
{
  return std::atan2(2.0*(w*z + x*y), 1.0 - 2.0*(y*y + z*z));
}

static std::string expand_user(const std::string& path)
{
  // very small "~" expansion
  if (!path.empty() && path[0] == '~') {
    const char* home = std::getenv("HOME");
    if (home) {
      return std::string(home) + path.substr(1);
    }
  }
  return path;
}

static std::string now_mmddhhmm()
{
  std::time_t t = std::time(nullptr);
  std::tm tm{};
#if defined(_WIN32)
  localtime_s(&tm, &t);
#else
  localtime_r(&t, &tm);
#endif
  char buf[64];
  std::strftime(buf, sizeof(buf), "%m%d%H%M", &tm);
  return std::string(buf);
}

class DataLogger : public rclcpp::Node
{
public:
  DataLogger() : Node("data_logger")
  {
    publish_hz_ = declare_parameter("publish_hz", 400.0);
    if (publish_hz_ <= 0.0) publish_hz_ = 100.0;

    // ---- CSV setup ----
    csv_dir_ = declare_parameter<std::string>(
      "csv_dir", "~/mujoco_crazyflie/src/flyingpen_interface/bag"
    );
    csv_dir_ = expand_user(csv_dir_);

    try {
      std::filesystem::create_directories(csv_dir_);
    } catch (const std::exception& e) {
      RCLCPP_ERROR(get_logger(), "Failed to create csv_dir '%s': %s", csv_dir_.c_str(), e.what());
    }

    csv_path_ = (std::filesystem::path(csv_dir_) / (now_mmddhhmm() + ".csv")).string();
    csv_.open(csv_path_, std::ios::out | std::ios::trunc);
    if (!csv_.is_open()) {
      RCLCPP_ERROR(get_logger(), "Failed to open CSV file: %s", csv_path_.c_str());
    } else {
      write_csv_header();
      RCLCPP_INFO(get_logger(), "CSV logging enabled: %s", csv_path_.c_str());
    }

    // Pub
    pub_ = create_publisher<std_msgs::msg::Float64MultiArray>("/data_logging_msgs", 10);

    // Subs
    sub_cmd_ = create_subscription<std_msgs::msg::Float64MultiArray>(
      "/crazyflie/in/pos_cmd", 10, std::bind(&DataLogger::cb_cmd, this, std::placeholders::_1));

    sub_input_ = create_subscription<std_msgs::msg::Float32MultiArray>(
      "/crazyflie/in/input", 10, std::bind(&DataLogger::cb_input, this, std::placeholders::_1));

    sub_pose_ = create_subscription<geometry_msgs::msg::PoseStamped>(
      "/crazyflie/out/pose", 10, std::bind(&DataLogger::cb_pose, this, std::placeholders::_1));

    sub_vel_ = create_subscription<geometry_msgs::msg::Vector3Stamped>(
      "/crazyflie/out/vel", 10, std::bind(&DataLogger::cb_vel, this, std::placeholders::_1));

    sub_w_ = create_subscription<geometry_msgs::msg::Vector3Stamped>(
      "/crazyflie/out/ang_vel", 10, std::bind(&DataLogger::cb_w, this, std::placeholders::_1));

    sub_acc_ = create_subscription<geometry_msgs::msg::Vector3Stamped>(
      "/crazyflie/out/acc", 10, std::bind(&DataLogger::cb_acc, this, std::placeholders::_1));

    sub_angacc_ = create_subscription<geometry_msgs::msg::Vector3Stamped>(
      "/crazyflie/out/ang_acc", 10, std::bind(&DataLogger::cb_angacc, this, std::placeholders::_1));

    sub_vdes_ = create_subscription<geometry_msgs::msg::Vector3Stamped>(
      "/crazyflie/debug/v_des", 10, std::bind(&DataLogger::cb_vdes, this, std::placeholders::_1));

    sub_rpydes_ = create_subscription<geometry_msgs::msg::Vector3Stamped>(
      "/crazyflie/debug/rpy_des", 10, std::bind(&DataLogger::cb_rpydes, this, std::placeholders::_1));

    sub_wdes_ = create_subscription<geometry_msgs::msg::Vector3Stamped>(
      "/crazyflie/debug/w_des", 10, std::bind(&DataLogger::cb_wdes, this, std::placeholders::_1));

    // Timer
    const auto period = std::chrono::duration<double>(1.0 / publish_hz_);
    timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(period),
      std::bind(&DataLogger::publish, this));

    RCLCPP_INFO(get_logger(), "data_logger started. Publishing /data_logging_msgs at %.1f Hz", publish_hz_);
  }

  ~DataLogger() override
  {
    if (csv_.is_open()) {
      csv_.flush();
      csv_.close();
    }
  }

private:
  void write_csv_header()
  {
    // Columns match msg.data order
    csv_ <<
      "t_sec,"
      "cmd_x,cmd_y,cmd_z,cmd_yaw,"
      "pos_x,pos_y,pos_z,"
      "roll,pitch,yaw,"
      "vel_x,vel_y,vel_z,"
      "w_x,w_y,w_z,"
      "acc_x,acc_y,acc_z,"
      "angacc_x,angacc_y,angacc_z,"
      "vdes_x,vdes_y,vdes_z,"
      "rolld,pitchd,yawd,"
      "wdes_x,wdes_y,wdes_z,"
      "tau_x,tau_y,tau_z,Fz,"
      "validity_bitmask\n";
    csv_.flush();
  }

  // ----- callbacks -----
  void cb_cmd(const std_msgs::msg::Float64MultiArray::SharedPtr m)
  {
    if (m->data.size() < 4) return;
    std::lock_guard<std::mutex> lk(mtx_);
    cmd_pos_ << m->data[0], m->data[1], m->data[2];
    cmd_yaw_ = m->data[3];
    have_cmd_ = true;
  }

  void cb_input(const std_msgs::msg::Float32MultiArray::SharedPtr m)
  {
    if (m->data.size() < 4) return;
    std::lock_guard<std::mutex> lk(mtx_);
    tau_ << (double)m->data[0], (double)m->data[1], (double)m->data[2];
    Fz_ = (double)m->data[3];
    have_input_ = true;
  }

  void cb_pose(const geometry_msgs::msg::PoseStamped::SharedPtr m)
  {
    std::lock_guard<std::mutex> lk(mtx_);
    pos_ << m->pose.position.x, m->pose.position.y, m->pose.position.z;

    const double qx = m->pose.orientation.x;
    const double qy = m->pose.orientation.y;
    const double qz = m->pose.orientation.z;
    const double qw = m->pose.orientation.w;

    roll_  = roll_from_quat(qx, qy, qz, qw);
    pitch_ = pitch_from_quat(qx, qy, qz, qw);
    yaw_   = yaw_from_quat(qx, qy, qz, qw);

    have_pose_ = true;
  }

  void cb_vel(const geometry_msgs::msg::Vector3Stamped::SharedPtr m)
  {
    std::lock_guard<std::mutex> lk(mtx_);
    vel_ << m->vector.x, m->vector.y, m->vector.z;
    have_vel_ = true;
  }

  void cb_w(const geometry_msgs::msg::Vector3Stamped::SharedPtr m)
  {
    std::lock_guard<std::mutex> lk(mtx_);
    w_ << m->vector.x, m->vector.y, m->vector.z;
    have_w_ = true;
  }

  void cb_acc(const geometry_msgs::msg::Vector3Stamped::SharedPtr m)
  {
    std::lock_guard<std::mutex> lk(mtx_);
    acc_ << m->vector.x, m->vector.y, m->vector.z;
    have_acc_ = true;
  }

  void cb_angacc(const geometry_msgs::msg::Vector3Stamped::SharedPtr m)
  {
    std::lock_guard<std::mutex> lk(mtx_);
    angacc_ << m->vector.x, m->vector.y, m->vector.z;
    have_angacc_ = true;
  }

  void cb_vdes(const geometry_msgs::msg::Vector3Stamped::SharedPtr m)
  {
    std::lock_guard<std::mutex> lk(mtx_);
    vdes_ << m->vector.x, m->vector.y, m->vector.z;
    have_vdes_ = true;
  }

  void cb_rpydes(const geometry_msgs::msg::Vector3Stamped::SharedPtr m)
  {
    std::lock_guard<std::mutex> lk(mtx_);
    rolld_  = m->vector.x;
    pitchd_ = m->vector.y;
    yawd_   = m->vector.z;
    have_rpydes_ = true;
  }

  void cb_wdes(const geometry_msgs::msg::Vector3Stamped::SharedPtr m)
  {
    std::lock_guard<std::mutex> lk(mtx_);
    wdes_ << m->vector.x, m->vector.y, m->vector.z;
    have_wdes_ = true;
  }

  // ----- publisher -----
  void publish()
  {
    // snapshot
    Eigen::Vector3d cmd_pos, pos, vel, w, acc, angacc, vdes, wdes, tau;
    double cmd_yaw, roll, pitch, yaw;
    double rolld, pitchd, yawd;
    double Fz;
    uint32_t mask;

    {
      std::lock_guard<std::mutex> lk(mtx_);
      cmd_pos = cmd_pos_;
      cmd_yaw = cmd_yaw_;
      pos = pos_;
      roll = roll_;
      pitch = pitch_;
      yaw = yaw_;
      vel = vel_;
      w = w_;
      acc = acc_;
      angacc = angacc_;
      vdes = vdes_;
      rolld = rolld_;
      pitchd = pitchd_;
      yawd = yawd_;
      wdes = wdes_;
      tau = tau_;
      Fz = Fz_;

      mask = 0u;
      mask |= (have_cmd_    ? (1u<<0) : 0u);
      mask |= (have_pose_   ? (1u<<1) : 0u);
      mask |= (have_vel_    ? (1u<<2) : 0u);
      mask |= (have_w_      ? (1u<<3) : 0u);
      mask |= (have_acc_    ? (1u<<4) : 0u);
      mask |= (have_angacc_ ? (1u<<5) : 0u);
      mask |= (have_vdes_   ? (1u<<6) : 0u);
      mask |= (have_rpydes_ ? (1u<<7) : 0u);
      mask |= (have_wdes_   ? (1u<<8) : 0u);
      mask |= (have_input_  ? (1u<<9) : 0u);
    }

    const double t = this->get_clock()->now().seconds();

    std_msgs::msg::Float64MultiArray msg;
    msg.data.resize(37);

    msg.data[0]  = t;

    msg.data[1]  = cmd_pos.x();
    msg.data[2]  = cmd_pos.y();
    msg.data[3]  = cmd_pos.z();
    msg.data[4]  = cmd_yaw;

    msg.data[5]  = pos.x();
    msg.data[6]  = pos.y();
    msg.data[7]  = pos.z();

    msg.data[8]  = roll;
    msg.data[9]  = pitch;
    msg.data[10] = yaw;

    msg.data[11] = vel.x();
    msg.data[12] = vel.y();
    msg.data[13] = vel.z();

    msg.data[14] = w.x();
    msg.data[15] = w.y();
    msg.data[16] = w.z();

    msg.data[17] = acc.x();
    msg.data[18] = acc.y();
    msg.data[19] = acc.z();

    msg.data[20] = angacc.x();
    msg.data[21] = angacc.y();
    msg.data[22] = angacc.z();

    msg.data[23] = vdes.x();
    msg.data[24] = vdes.y();
    msg.data[25] = vdes.z();

    msg.data[26] = rolld;
    msg.data[27] = pitchd;
    msg.data[28] = yawd;

    msg.data[29] = wdes.x();
    msg.data[30] = wdes.y();
    msg.data[31] = wdes.z();

    msg.data[32] = tau.x();
    msg.data[33] = tau.y();
    msg.data[34] = tau.z();
    msg.data[35] = Fz;

    msg.data[36] = static_cast<double>(mask);

    // publish topic
    pub_->publish(msg);

    // append CSV
    if (csv_.is_open()) {
      // fixed precision for readability
      csv_ << std::setprecision(10) << std::fixed
           << t << ","
           << msg.data[1] << "," << msg.data[2] << "," << msg.data[3] << "," << msg.data[4] << ","
           << msg.data[5] << "," << msg.data[6] << "," << msg.data[7] << ","
           << msg.data[8] << "," << msg.data[9] << "," << msg.data[10] << ","
           << msg.data[11] << "," << msg.data[12] << "," << msg.data[13] << ","
           << msg.data[14] << "," << msg.data[15] << "," << msg.data[16] << ","
           << msg.data[17] << "," << msg.data[18] << "," << msg.data[19] << ","
           << msg.data[20] << "," << msg.data[21] << "," << msg.data[22] << ","
           << msg.data[23] << "," << msg.data[24] << "," << msg.data[25] << ","
           << msg.data[26] << "," << msg.data[27] << "," << msg.data[28] << ","
           << msg.data[29] << "," << msg.data[30] << "," << msg.data[31] << ","
           << msg.data[32] << "," << msg.data[33] << "," << msg.data[34] << "," << msg.data[35] << ","
           << static_cast<uint64_t>(mask)
           << "\n";

      // flush occasionally (parameterize if you want). Here: every line flush is heavy at 400Hz.
      // We'll flush every 200 lines.
      if (++csv_line_count_ % 200 == 0) {
        csv_.flush();
      }
    }
  }

private:
  std::mutex mtx_;

  // latest values
  Eigen::Vector3d cmd_pos_{0,0,0};
  double cmd_yaw_{0.0};

  Eigen::Vector3d pos_{0,0,0};
  double roll_{0.0}, pitch_{0.0}, yaw_{0.0};

  Eigen::Vector3d vel_{0,0,0};
  Eigen::Vector3d w_{0,0,0};
  Eigen::Vector3d acc_{0,0,0};
  Eigen::Vector3d angacc_{0,0,0};

  Eigen::Vector3d vdes_{0,0,0};
  double rolld_{0.0}, pitchd_{0.0}, yawd_{0.0};
  Eigen::Vector3d wdes_{0,0,0};

  Eigen::Vector3d tau_{0,0,0};
  double Fz_{0.0};

  // validity flags
  bool have_cmd_{false};
  bool have_pose_{false};
  bool have_vel_{false};
  bool have_w_{false};
  bool have_acc_{false};
  bool have_angacc_{false};
  bool have_vdes_{false};
  bool have_rpydes_{false};
  bool have_wdes_{false};
  bool have_input_{false};

  // ros
  double publish_hz_{400.0};
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr sub_cmd_;
  rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr sub_input_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr sub_pose_;
  rclcpp::Subscription<geometry_msgs::msg::Vector3Stamped>::SharedPtr sub_vel_;
  rclcpp::Subscription<geometry_msgs::msg::Vector3Stamped>::SharedPtr sub_w_;
  rclcpp::Subscription<geometry_msgs::msg::Vector3Stamped>::SharedPtr sub_acc_;
  rclcpp::Subscription<geometry_msgs::msg::Vector3Stamped>::SharedPtr sub_angacc_;
  rclcpp::Subscription<geometry_msgs::msg::Vector3Stamped>::SharedPtr sub_vdes_;
  rclcpp::Subscription<geometry_msgs::msg::Vector3Stamped>::SharedPtr sub_rpydes_;
  rclcpp::Subscription<geometry_msgs::msg::Vector3Stamped>::SharedPtr sub_wdes_;

  // csv
  std::string csv_dir_;
  std::string csv_path_;
  std::ofstream csv_;
  uint64_t csv_line_count_{0};
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<DataLogger>());
  rclcpp::shutdown();
  return 0;
}
