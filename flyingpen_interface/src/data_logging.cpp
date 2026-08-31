// data_logging.cpp
//
// Publishes /data_logging_msgs (Float64MultiArray) and also logs the same data to CSV.
//
// CSV:
//   dir  : <share>/flyingpen_interface/bag   (default; overridable by parameter csv_dir)
//   name : MMDDHHMM.csv  (local time)
//
// Note:
// - logs "latest" values (not time-synchronized).
// - writes one row per publish().
// - appends MOB compare and consistency-observer debug wrench signals at the end of each row.

#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/vector3_stamped.hpp>
#include <geometry_msgs/msg/quaternion_stamped.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <geometry_msgs/msg/wrench_stamped.hpp>
#include <std_msgs/msg/float32.hpp>



#include <ament_index_cpp/get_package_share_directory.hpp>

#include <Eigen/Dense>
#include <array>
#include <mutex>
#include <cmath>
#include <cstdint>

#include <fstream>
#include <iomanip>
#include <string>
#include <ctime>
#include <filesystem>
#include <limits>
#include <cstdlib>
#include <cerrno>
#include <cstring>

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

static inline double quiet_nan()
{
  return std::numeric_limits<double>::quiet_NaN();
}

static std::string expand_user(const std::string& path)
{
  // small "~" expansion (useful when user overrides via param)
  if (!path.empty() && path[0] == '~') {
    const char* home = std::getenv("HOME");
    if (home) {
      return std::string(home) + path.substr(1);
    }
  }
  return path;
}

// Returns "MMDDHHMM" (local time)
static std::string now_mmddhhmm()
{
  std::time_t t = std::time(nullptr);
  std::tm tm{};
#if defined(_WIN32)
  localtime_s(&tm, &t);
#else
  localtime_r(&t, &tm);
#endif
  char buf[16];
  // MMDDHHMM -> 8 chars, plus null
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
    compare_velocity_epsilon_ = declare_parameter("compare_velocity_epsilon", 0.01);
    compare_gamma_epsilon_ = declare_parameter("compare_gamma_epsilon", 3.0e-3);

    // ---- CSV setup (package-share-based default) ----
    std::string pkg_share;
    try {
      pkg_share = ament_index_cpp::get_package_share_directory("flyingpen_interface");
    } catch (const std::exception& e) {
      RCLCPP_ERROR(get_logger(),
                   "Failed to get package share directory for flyingpen_interface: %s",
                   e.what());
      pkg_share = ".";
    }

    const std::string default_csv_dir =
      (std::filesystem::path(pkg_share) / "bag").string();

    // yaml에서 data_logger.ros__parameters.csv_dir 로 override 가능
    csv_dir_ = declare_parameter<std::string>("csv_dir", default_csv_dir);
    csv_dir_ = expand_user(csv_dir_);

    RCLCPP_INFO(get_logger(), "csv_dir resolved: %s", csv_dir_.c_str());

    // ensure directory exists
    try {
      std::filesystem::create_directories(csv_dir_);
    } catch (const std::exception& e) {
      RCLCPP_ERROR(get_logger(), "Failed to create csv_dir '%s': %s",
                   csv_dir_.c_str(), e.what());
    }

    // build file path: MMDDHHMM.csv
    const std::string fname = now_mmddhhmm() + ".csv";
    csv_path_ = (std::filesystem::path(csv_dir_) / fname).string();

    // open CSV
    errno = 0;
    csv_.open(csv_path_, std::ios::out | std::ios::trunc);
    if (!csv_.is_open() || csv_.fail()) {
      const int err = errno;
      RCLCPP_ERROR(get_logger(),
                   "Failed to open CSV file: %s (errno=%d: %s)",
                   csv_path_.c_str(), err, std::strerror(err));
    } else {
      write_csv_header();
      RCLCPP_INFO(get_logger(), "CSV logging enabled: %s", csv_path_.c_str());
    }

    // Pub
    pub_ = create_publisher<std_msgs::msg::Float64MultiArray>("/data_logging_msgs", 10);

    // Subs
    sub_cmd_ = create_subscription<std_msgs::msg::Float64MultiArray>(
      "/crazyflie/in/pos_cmd", 10,
      std::bind(&DataLogger::cb_cmd, this, std::placeholders::_1));

    sub_input_ = create_subscription<std_msgs::msg::Float32MultiArray>(
      "/crazyflie/in/input", 10,
      std::bind(&DataLogger::cb_input, this, std::placeholders::_1));

    sub_pose_ = create_subscription<geometry_msgs::msg::PoseStamped>(
      "/crazyflie/out/pose", 10,
      std::bind(&DataLogger::cb_pose, this, std::placeholders::_1));
    sub_ee_pose_ = create_subscription<geometry_msgs::msg::PoseStamped>(
      "/crazyflie/out/EE_pose", 10,
      std::bind(&DataLogger::cb_ee_pose, this, std::placeholders::_1));
    sub_ee_vel_ = create_subscription<geometry_msgs::msg::Vector3Stamped>(
      "/crazyflie/out/EE_velocity", 10,
      std::bind(&DataLogger::cb_ee_vel, this, std::placeholders::_1));

    sub_vel_ = create_subscription<geometry_msgs::msg::Vector3Stamped>(
      "/crazyflie/out/vel", 10,
      std::bind(&DataLogger::cb_vel, this, std::placeholders::_1));

    sub_w_ = create_subscription<geometry_msgs::msg::Vector3Stamped>(
      "/crazyflie/out/ang_vel", 10,
      std::bind(&DataLogger::cb_w, this, std::placeholders::_1));

    sub_acc_ = create_subscription<geometry_msgs::msg::Vector3Stamped>(
      "/crazyflie/out/acc", 10,
      std::bind(&DataLogger::cb_acc, this, std::placeholders::_1));

    sub_angacc_ = create_subscription<geometry_msgs::msg::Vector3Stamped>(
      "/crazyflie/out/ang_acc", 10,
      std::bind(&DataLogger::cb_angacc, this, std::placeholders::_1));

    sub_vdes_ = create_subscription<geometry_msgs::msg::Vector3Stamped>(
      "/crazyflie/debug/v_des", 10,
      std::bind(&DataLogger::cb_vdes, this, std::placeholders::_1));

    sub_rpydes_ = create_subscription<geometry_msgs::msg::Vector3Stamped>(
      "/crazyflie/debug/rpy_des", 10,
      std::bind(&DataLogger::cb_rpydes, this, std::placeholders::_1));

    sub_wdes_ = create_subscription<geometry_msgs::msg::Vector3Stamped>(
      "/crazyflie/debug/w_des", 10,
      std::bind(&DataLogger::cb_wdes, this, std::placeholders::_1));

    sub_contact_raw_ = create_subscription<geometry_msgs::msg::WrenchStamped>(
      "/crazyflie/out/EE_contact_force", 10,
      std::bind(&DataLogger::cb_contact_raw, this, std::placeholders::_1));

    sub_contact_filt_ = create_subscription<geometry_msgs::msg::WrenchStamped>(
      "/crazyflie/out/EE_contact_force_filt", 10,
      std::bind(&DataLogger::cb_contact_filt, this, std::placeholders::_1));

    sub_cmd_force_ = create_subscription<std_msgs::msg::Float32>(
      "su/cmd_force", 10,
      std::bind(&DataLogger::cb_cmd_force, this, std::placeholders::_1));
    sub_contact_vel_cmd_ = create_subscription<geometry_msgs::msg::Vector3Stamped>(
      "/su/debug/contact_vel_cmd", 10,
      std::bind(&DataLogger::cb_contact_vel_cmd, this, std::placeholders::_1));
    sub_contact_vel_actual_ = create_subscription<geometry_msgs::msg::Vector3Stamped>(
      "/su/debug/contact_vel_actual", 10,
      std::bind(&DataLogger::cb_contact_vel_actual, this, std::placeholders::_1));
    sub_force_actual_ = create_subscription<std_msgs::msg::Float32>(
      "/su/contact_force_x", 10,
      std::bind(&DataLogger::cb_force_actual, this, std::placeholders::_1));
    sub_normal_debug_metrics_ = create_subscription<std_msgs::msg::Float64MultiArray>(
      "/normal_vector/debug_metrics", 10,
      std::bind(&DataLogger::cb_normal_debug_metrics, this, std::placeholders::_1));
    sub_normal_quat_pure_ = create_subscription<geometry_msgs::msg::QuaternionStamped>(
      "/estimated_contact_frame_quat_pure", 10,
      std::bind(&DataLogger::cb_normal_quat_pure, this, std::placeholders::_1));
    sub_normal_quat_ke_ = create_subscription<geometry_msgs::msg::QuaternionStamped>(
      "/estimated_contact_frame_quat", 10,
      std::bind(&DataLogger::cb_normal_quat_ke, this, std::placeholders::_1));

    sub_force_lpf_ = create_subscription<std_msgs::msg::Float64MultiArray>(
      "/su/force_lpf", 10,
      std::bind(&DataLogger::cb_force_lpf, this, std::placeholders::_1));
    sub_control_metrics_ = create_subscription<std_msgs::msg::Float64MultiArray>(
      "/su/debug/control_metrics", 10,
      std::bind(&DataLogger::cb_control_metrics, this, std::placeholders::_1));

    sub_mob_wrench_2nd_order_ = create_subscription<geometry_msgs::msg::WrenchStamped>(
      "/crazyflie/out/mob_2nd", 10,
      std::bind(&DataLogger::cb_mob_wrench_2nd_order, this, std::placeholders::_1));
    sub_mob_wrench_2nd_tau_ = create_subscription<geometry_msgs::msg::WrenchStamped>(
      "/crazyflie/out/mob_2nd_tau", 10,
      std::bind(&DataLogger::cb_mob_wrench_2nd_tau, this, std::placeholders::_1));
    sub_mob_wrench_eta_t_ = create_subscription<geometry_msgs::msg::WrenchStamped>(
      "/crazyflie/out/mob_eta_t", 10,
      std::bind(&DataLogger::cb_mob_wrench_eta_t, this, std::placeholders::_1));
    sub_mob_wrench_2nd_tau_i_ = create_subscription<geometry_msgs::msg::WrenchStamped>(
      "/crazyflie/out/mob_2nd_tau_i", 10,
      std::bind(&DataLogger::cb_mob_wrench_2nd_tau_i, this, std::placeholders::_1));
    sub_mob_wrench_kalman_ = create_subscription<geometry_msgs::msg::WrenchStamped>(
      "/crazyflie/out/mob_kalman", 10,
      std::bind(&DataLogger::cb_mob_wrench_kalman, this, std::placeholders::_1));
    sub_mob_wrench_adaptive_ = create_subscription<geometry_msgs::msg::WrenchStamped>(
      "/crazyflie/out/mob_adaptive", 10,
      std::bind(&DataLogger::cb_mob_wrench_adaptive, this, std::placeholders::_1));
    sub_mob_2nd_tau_terms_ = create_subscription<geometry_msgs::msg::WrenchStamped>(
      "/crazyflie/out/mob_2nd_tau_terms", 10,
      std::bind(&DataLogger::cb_mob_2nd_tau_terms, this, std::placeholders::_1));
    sub_mob_2nd_tau_consistency_ = create_subscription<geometry_msgs::msg::WrenchStamped>(
      "/crazyflie/out/mob_2nd_tau_consistency", 10,
      std::bind(&DataLogger::cb_mob_2nd_tau_consistency, this, std::placeholders::_1));
    sub_eta_t_hat_ = create_subscription<std_msgs::msg::Float32>(
      "/crazyflie/out/eta_t_hat", 10,
      std::bind(&DataLogger::cb_eta_t_hat, this, std::placeholders::_1));
    sub_thrust_effectiveness_true_ = create_subscription<std_msgs::msg::Float32>(
      "/crazyflie/out/thrust_effectiveness_true", 10,
      std::bind(&DataLogger::cb_thrust_effectiveness_true, this, std::placeholders::_1));
    for (size_t i = 0; i < k_ep_sweep_gains_.size(); ++i) {
      const std::string topic =
        "/crazyflie/out/mob_2nd_tau_ke_" + std::to_string(k_ep_sweep_gains_[i]);
      sub_mob_wrench_k_ep_sweep_[i] = create_subscription<geometry_msgs::msg::WrenchStamped>(
        topic, 10,
        [this, i](const geometry_msgs::msg::WrenchStamped::SharedPtr m) {
          this->cb_mob_wrench_k_ep_sweep(i, m);
        });
    }
    sub_wind_ = create_subscription<geometry_msgs::msg::Vector3Stamped>(
      "/crazyflie/in/wind", 10,
      std::bind(&DataLogger::cb_wind, this, std::placeholders::_1));


    // Timer
    const auto period = std::chrono::duration<double>(1.0 / publish_hz_);
    timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(period),
      std::bind(&DataLogger::publish, this));

    RCLCPP_INFO(get_logger(),
                "data_logger started. Publishing /data_logging_msgs at %.1f Hz",
                publish_hz_);
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
      "contact_Fx,contact_Fy,contact_Fz,"
      "contact_Fx_filt,contact_Fy_filt,contact_Fz_filt,"
      "cmd_force,"
      "F_error_dot_raw,F_error_dot_filt,"
      "validity_bitmask,"
      "mob_2nd_order_Fx,mob_2nd_order_Fy,mob_2nd_order_Fz,mob_2nd_order_Tx,mob_2nd_order_Ty,mob_2nd_order_Tz,"
      "mob_2nd_tau_Fx,mob_2nd_tau_Fy,mob_2nd_tau_Fz,mob_2nd_tau_Tx,mob_2nd_tau_Ty,mob_2nd_tau_Tz,"
      "mob_2nd_tau_i_Fx,mob_2nd_tau_i_Fy,mob_2nd_tau_i_Fz,mob_2nd_tau_i_Tx,mob_2nd_tau_i_Ty,mob_2nd_tau_i_Tz,"
      "mob_kalman_Fx,mob_kalman_Fy,mob_kalman_Fz,mob_kalman_Tx,mob_kalman_Ty,mob_kalman_Tz,"
      "mob_adaptive_Fx,mob_adaptive_Fy,mob_adaptive_Fz,mob_adaptive_Tx,mob_adaptive_Ty,mob_adaptive_Tz,"
      "mob_2nd_tau_ke_100_Fx,mob_2nd_tau_ke_100_Fy,mob_2nd_tau_ke_100_Fz,mob_2nd_tau_ke_100_Tx,mob_2nd_tau_ke_100_Ty,mob_2nd_tau_ke_100_Tz,"
      "mob_2nd_tau_ke_300_Fx,mob_2nd_tau_ke_300_Fy,mob_2nd_tau_ke_300_Fz,mob_2nd_tau_ke_300_Tx,mob_2nd_tau_ke_300_Ty,mob_2nd_tau_ke_300_Tz,"
      "mob_2nd_tau_ke_500_Fx,mob_2nd_tau_ke_500_Fy,mob_2nd_tau_ke_500_Fz,mob_2nd_tau_ke_500_Tx,mob_2nd_tau_ke_500_Ty,mob_2nd_tau_ke_500_Tz,"
      "mob_2nd_tau_ke_700_Fx,mob_2nd_tau_ke_700_Fy,mob_2nd_tau_ke_700_Fz,mob_2nd_tau_ke_700_Tx,mob_2nd_tau_ke_700_Ty,mob_2nd_tau_ke_700_Tz,"
      "mob_2nd_tau_ke_900_Fx,mob_2nd_tau_ke_900_Fy,mob_2nd_tau_ke_900_Fz,mob_2nd_tau_ke_900_Tx,mob_2nd_tau_ke_900_Ty,mob_2nd_tau_ke_900_Tz,"
      "mob_2nd_tau_kfep_x,mob_2nd_tau_kfep_y,mob_2nd_tau_kfep_z,"
      "mob_2nd_tau_consistency_x,mob_2nd_tau_consistency_y,mob_2nd_tau_consistency_z,"
      "mob_2nd_tau_tauhat_x,mob_2nd_tau_tauhat_y,mob_2nd_tau_tauhat_z,"
      "mob_2nd_tau_rxf_x,mob_2nd_tau_rxf_y,mob_2nd_tau_rxf_z,"
      "wind_x,wind_y,wind_z,"
      "ee_pos_x,ee_pos_y,ee_pos_z,"
      "c_hat_vy_cmd,c_hat_vy_act,c_hat_vz_cmd,c_hat_vz_act,"
      "c_hat_fx_act,"
      "alphaFrame,omegaN,normalLeakage,alphaU1,alphaU2,preloadFeedback,cTau,patternProgress,patternSpeed,"
      "n_geo_x,n_geo_y,n_geo_z,"
      "n_f_x,n_f_y,n_f_z,"
      "n_alg_x,n_alg_y,n_alg_z,"
      "f_g_x,f_g_y,f_g_z,"
      "offline_mob2_fx,offline_mob2_fy,offline_mob2_fz,offline_mob2_tx,offline_mob2_ty,offline_mob2_tz,"
      "offline_mobc_fx,offline_mobc_fy,offline_mobc_fz,offline_mobc_tx,offline_mobc_ty,offline_mobc_tz,"
      "offline_tauhat_x,offline_tauhat_y,offline_tauhat_z,"
      "offline_rxf_x,offline_rxf_y,offline_rxf_z,"
      "offline_etau_x,offline_etau_y,offline_etau_z,offline_rho_tau,"
      "offline_normal_pure_nx,offline_normal_pure_ny,offline_normal_pure_nz,"
      "offline_normal_k1_nx,offline_normal_k1_ny,offline_normal_k1_nz,"
      "offline_normal_ke_raw_nx,offline_normal_ke_raw_ny,offline_normal_ke_raw_nz,"
      "offline_normal_ke_gamma_proj_nx,offline_normal_ke_gamma_proj_ny,offline_normal_ke_gamma_proj_nz,"
      "offline_ws_x,offline_ws_y,offline_ws_z,"
      "offline_gamma_v,"
      "offline_normal_vel_eps0_nx,offline_normal_vel_eps0_ny,offline_normal_vel_eps0_nz,"
      "offline_normal_vel_eps_nx,offline_normal_vel_eps_ny,offline_normal_vel_eps_nz,"
      "offline_normal_vel_eps0_gamma_nx,offline_normal_vel_eps0_gamma_ny,offline_normal_vel_eps0_gamma_nz,"
      "offline_contact_force_x,"
      "mob_eta_t_Fx,mob_eta_t_Fy,mob_eta_t_Fz,mob_eta_t_Tx,mob_eta_t_Ty,mob_eta_t_Tz,"
      "eta_t_hat,thrust_effectiveness_true\n";
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

  void cb_ee_pose(const geometry_msgs::msg::PoseStamped::SharedPtr m)
  {
    std::lock_guard<std::mutex> lk(mtx_);
    ee_pos_ << m->pose.position.x, m->pose.position.y, m->pose.position.z;
    have_ee_pose_ = true;
  }

  void cb_vel(const geometry_msgs::msg::Vector3Stamped::SharedPtr m)
  {
    std::lock_guard<std::mutex> lk(mtx_);
    vel_ << m->vector.x, m->vector.y, m->vector.z;
    have_vel_ = true;
  }

  void cb_ee_vel(const geometry_msgs::msg::Vector3Stamped::SharedPtr m)
  {
    std::lock_guard<std::mutex> lk(mtx_);
    ee_vel_ << m->vector.x, m->vector.y, m->vector.z;
    have_ee_vel_ = true;
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

  void cb_contact_raw(const geometry_msgs::msg::WrenchStamped::SharedPtr m)
  {
    std::lock_guard<std::mutex> lk(mtx_);
    contact_F_raw_ << m->wrench.force.x, m->wrench.force.y, m->wrench.force.z;

    // (선택) torque도 로깅하려면:
    // contact_T_ << m->wrench.torque.x, m->wrench.torque.y, m->wrench.torque.z;

    have_contact_raw_ = true;
  }

  void cb_contact_filt(const geometry_msgs::msg::WrenchStamped::SharedPtr m)
  {
    std::lock_guard<std::mutex> lk(mtx_);
    contact_F_filt_ << m->wrench.force.x, m->wrench.force.y, m->wrench.force.z;
    have_contact_filt_ = true;
  }

  void cb_cmd_force(const std_msgs::msg::Float32::SharedPtr m)
  {
    std::lock_guard<std::mutex> lk(mtx_);
    cmd_force_ = static_cast<double>(m->data);
    have_cmd_force_ = true;
  }

  void cb_contact_vel_cmd(const geometry_msgs::msg::Vector3Stamped::SharedPtr m)
  {
    std::lock_guard<std::mutex> lk(mtx_);
    c_hat_v_cmd_ << m->vector.x, m->vector.y, m->vector.z;
    have_contact_vel_cmd_ = true;
  }

  void cb_contact_vel_actual(const geometry_msgs::msg::Vector3Stamped::SharedPtr m)
  {
    std::lock_guard<std::mutex> lk(mtx_);
    c_hat_v_act_ << m->vector.x, m->vector.y, m->vector.z;
    have_contact_vel_actual_ = true;
  }

  void cb_force_actual(const std_msgs::msg::Float32::SharedPtr m)
  {
    std::lock_guard<std::mutex> lk(mtx_);
    c_hat_fx_act_ = static_cast<double>(m->data);
    have_force_actual_ = true;
  }

  void cb_normal_debug_metrics(const std_msgs::msg::Float64MultiArray::SharedPtr m)
  {
    if (m->data.size() < 43) return;
    std::lock_guard<std::mutex> lk(mtx_);
    n_geo_ << m->data[28], m->data[29], m->data[30];
    n_f_ << m->data[31], m->data[32], m->data[33];
    n_alg_ << m->data[34], m->data[35], m->data[36];
    f_g_ << m->data[40], m->data[41], m->data[42];
    if (m->data.size() >= 49) {
      n_ke_gamma_proj_ << m->data[43], m->data[44], m->data[45];
    } else {
      n_ke_gamma_proj_ << quiet_nan(), quiet_nan(), quiet_nan();
    }
    have_normal_debug_metrics_ = true;
  }

  Eigen::Vector3d quat_to_normal_world(const geometry_msgs::msg::QuaternionStamped::SharedPtr & m) const
  {
    const Eigen::Quaterniond q(
      m->quaternion.w, m->quaternion.x, m->quaternion.y, m->quaternion.z);
    if (q.norm() < 1.0e-9) {
      return Eigen::Vector3d(quiet_nan(), quiet_nan(), quiet_nan());
    }
    const Eigen::Matrix3d rot = q.normalized().toRotationMatrix();
    return rot.col(0);
  }

  void cb_normal_quat_pure(const geometry_msgs::msg::QuaternionStamped::SharedPtr m)
  {
    std::lock_guard<std::mutex> lk(mtx_);
    normal_pure_ = quat_to_normal_world(m);
    have_normal_quat_pure_ = true;
  }

  void cb_normal_quat_ke(const geometry_msgs::msg::QuaternionStamped::SharedPtr m)
  {
    std::lock_guard<std::mutex> lk(mtx_);
    normal_ke_ = quat_to_normal_world(m);
    have_normal_quat_ke_ = true;
  }


  void cb_force_lpf(const std_msgs::msg::Float64MultiArray::SharedPtr m)
  {
    if (m->data.size() < 2) return;
    std::lock_guard<std::mutex> lk(mtx_);
    F_error_dot_raw_  = m->data[0];
    F_error_dot_filt_ = m->data[1];
    have_force_lpf_ = true;
  }

  void cb_control_metrics(const std_msgs::msg::Float64MultiArray::SharedPtr m)
  {
    if (m->data.size() < 6) return;
    std::lock_guard<std::mutex> lk(mtx_);
    alpha_frame_ = m->data[0];
    if (m->data.size() >= 9) {
      omega_n_ = m->data[1];
      normal_leakage_ = m->data[2];
      alpha_u1_ = m->data[3];
      alpha_u2_ = m->data[4];
      preload_feedback_ = m->data[5];
      c_tau_ = m->data[6];
      pattern_progress_ = m->data[7];
      pattern_speed_cmd_ = m->data[8];
    } else {
      omega_n_ = quiet_nan();
      normal_leakage_ = quiet_nan();
      alpha_u1_ = m->data[1];
      alpha_u2_ = m->data[2];
      preload_feedback_ = m->data[3];
      c_tau_ = m->data[4];
      pattern_progress_ = m->data[5];
      pattern_speed_cmd_ = m->data[6];
    }
    have_control_metrics_ = true;
  }

  void cb_mob_wrench_2nd_order(const geometry_msgs::msg::WrenchStamped::SharedPtr m)
  {
    std::lock_guard<std::mutex> lk(mtx_);
    mob_force_2nd_order_ << m->wrench.force.x, m->wrench.force.y, m->wrench.force.z;
    mob_torque_2nd_order_ << m->wrench.torque.x, m->wrench.torque.y, m->wrench.torque.z;
    have_mob_wrench_2nd_order_ = true;
  }

  void cb_mob_wrench_2nd_tau(const geometry_msgs::msg::WrenchStamped::SharedPtr m)
  {
    std::lock_guard<std::mutex> lk(mtx_);
    mob_force_2nd_tau_ << m->wrench.force.x, m->wrench.force.y, m->wrench.force.z;
    mob_torque_2nd_tau_ << m->wrench.torque.x, m->wrench.torque.y, m->wrench.torque.z;
    have_mob_wrench_2nd_tau_ = true;
  }

  void cb_mob_wrench_eta_t(const geometry_msgs::msg::WrenchStamped::SharedPtr m)
  {
    std::lock_guard<std::mutex> lk(mtx_);
    mob_force_eta_t_ << m->wrench.force.x, m->wrench.force.y, m->wrench.force.z;
    mob_torque_eta_t_ << m->wrench.torque.x, m->wrench.torque.y, m->wrench.torque.z;
    have_mob_wrench_eta_t_ = true;
  }

  void cb_mob_wrench_2nd_tau_i(const geometry_msgs::msg::WrenchStamped::SharedPtr m)
  {
    std::lock_guard<std::mutex> lk(mtx_);
    mob_force_2nd_tau_i_ << m->wrench.force.x, m->wrench.force.y, m->wrench.force.z;
    mob_torque_2nd_tau_i_ << m->wrench.torque.x, m->wrench.torque.y, m->wrench.torque.z;
    have_mob_wrench_2nd_tau_i_ = true;
  }

  void cb_mob_wrench_kalman(const geometry_msgs::msg::WrenchStamped::SharedPtr m)
  {
    std::lock_guard<std::mutex> lk(mtx_);
    mob_force_kalman_ << m->wrench.force.x, m->wrench.force.y, m->wrench.force.z;
    mob_torque_kalman_ << m->wrench.torque.x, m->wrench.torque.y, m->wrench.torque.z;
    have_mob_wrench_kalman_ = true;
  }

  void cb_mob_wrench_adaptive(const geometry_msgs::msg::WrenchStamped::SharedPtr m)
  {
    std::lock_guard<std::mutex> lk(mtx_);
    mob_force_adaptive_ << m->wrench.force.x, m->wrench.force.y, m->wrench.force.z;
    mob_torque_adaptive_ << m->wrench.torque.x, m->wrench.torque.y, m->wrench.torque.z;
    have_mob_wrench_adaptive_ = true;
  }

  void cb_mob_2nd_tau_terms(const geometry_msgs::msg::WrenchStamped::SharedPtr m)
  {
    std::lock_guard<std::mutex> lk(mtx_);
    mob_2nd_tau_kfep_ << m->wrench.force.x, m->wrench.force.y, m->wrench.force.z;
    mob_2nd_tau_consistency_term_ << m->wrench.torque.x, m->wrench.torque.y, m->wrench.torque.z;
    have_mob_2nd_tau_terms_ = true;
  }

  void cb_mob_2nd_tau_consistency(const geometry_msgs::msg::WrenchStamped::SharedPtr m)
  {
    std::lock_guard<std::mutex> lk(mtx_);
    mob_2nd_tau_tauhat_world_ << m->wrench.force.x, m->wrench.force.y, m->wrench.force.z;
    mob_2nd_tau_rxf_world_ << m->wrench.torque.x, m->wrench.torque.y, m->wrench.torque.z;
    have_mob_2nd_tau_consistency_ = true;
  }

  void cb_mob_wrench_k_ep_sweep(
    size_t index,
    const geometry_msgs::msg::WrenchStamped::SharedPtr m)
  {
    if (index >= k_ep_sweep_gains_.size()) {
      return;
    }
    std::lock_guard<std::mutex> lk(mtx_);
    mob_force_k_ep_sweep_[index] << m->wrench.force.x, m->wrench.force.y, m->wrench.force.z;
    mob_torque_k_ep_sweep_[index] << m->wrench.torque.x, m->wrench.torque.y, m->wrench.torque.z;
    have_mob_wrench_k_ep_sweep_[index] = true;
  }

  void cb_wind(const geometry_msgs::msg::Vector3Stamped::SharedPtr m)
  {
    std::lock_guard<std::mutex> lk(mtx_);
    wind_force_ << m->vector.x, m->vector.y, m->vector.z;
    have_wind_ = true;
  }

  void cb_eta_t_hat(const std_msgs::msg::Float32::SharedPtr m)
  {
    std::lock_guard<std::mutex> lk(mtx_);
    eta_t_hat_ = static_cast<double>(m->data);
    have_eta_t_hat_ = true;
  }

  void cb_thrust_effectiveness_true(const std_msgs::msg::Float32::SharedPtr m)
  {
    std::lock_guard<std::mutex> lk(mtx_);
    thrust_effectiveness_true_ = static_cast<double>(m->data);
    have_thrust_effectiveness_true_ = true;
  }





  // ----- publisher -----
  void publish()
  {
    Eigen::Vector3d cmd_pos, pos, vel, w, acc, angacc, vdes, wdes, tau, contact_F_raw, contact_F_filt;
    Eigen::Vector3d mob_force_2nd_order, mob_torque_2nd_order;
    Eigen::Vector3d mob_force_2nd_tau, mob_torque_2nd_tau;
    Eigen::Vector3d mob_force_eta_t, mob_torque_eta_t;
    Eigen::Vector3d mob_force_2nd_tau_i, mob_torque_2nd_tau_i;
    Eigen::Vector3d mob_force_kalman, mob_torque_kalman;
    Eigen::Vector3d mob_force_adaptive, mob_torque_adaptive;
    std::array<Eigen::Vector3d, 5> mob_force_k_ep_sweep;
    std::array<Eigen::Vector3d, 5> mob_torque_k_ep_sweep;
    Eigen::Vector3d mob_2nd_tau_kfep, mob_2nd_tau_consistency_term;
    Eigen::Vector3d mob_2nd_tau_tauhat_world, mob_2nd_tau_rxf_world;
    Eigen::Vector3d wind_force, ee_vel;
    Eigen::Vector3d ee_pos, c_hat_v_cmd, c_hat_v_act, n_geo, n_f, n_alg, f_g;
    Eigen::Vector3d normal_pure, normal_ke, n_ke_gamma_proj;
    Eigen::Vector3d n_ke_raw_direct(quiet_nan(), quiet_nan(), quiet_nan());
    Eigen::Vector3d n_ke_gamma_proj_direct(quiet_nan(), quiet_nan(), quiet_nan());
    Eigen::Vector3d w_s_direct(quiet_nan(), quiet_nan(), quiet_nan());
    Eigen::Vector3d n_vel_eps0_direct(quiet_nan(), quiet_nan(), quiet_nan());
    Eigen::Vector3d n_vel_eps_direct(quiet_nan(), quiet_nan(), quiet_nan());
    Eigen::Vector3d n_vel_eps0_gamma_direct(quiet_nan(), quiet_nan(), quiet_nan());
    double gamma_v_direct = quiet_nan();
    double cmd_yaw, roll, pitch, yaw;
    double rolld, pitchd, yawd;
    double Fz, cmd_force, F_error_dot_raw, F_error_dot_filt, c_hat_fx_act;
    double eta_t_hat, thrust_effectiveness_true;
    double alpha_frame, omega_n, normal_leakage, alpha_u1, alpha_u2, preload_feedback, c_tau, pattern_progress, pattern_speed_cmd;


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
      contact_F_raw = contact_F_raw_;
      contact_F_filt = contact_F_filt_;
      cmd_force = cmd_force_;
      F_error_dot_raw  = F_error_dot_raw_;
      F_error_dot_filt = F_error_dot_filt_;
      mob_force_2nd_order = mob_force_2nd_order_;
      mob_torque_2nd_order = mob_torque_2nd_order_;
      mob_force_2nd_tau = mob_force_2nd_tau_;
      mob_torque_2nd_tau = mob_torque_2nd_tau_;
      mob_force_eta_t = mob_force_eta_t_;
      mob_torque_eta_t = mob_torque_eta_t_;
      mob_force_2nd_tau_i = mob_force_2nd_tau_i_;
      mob_torque_2nd_tau_i = mob_torque_2nd_tau_i_;
      mob_force_kalman = mob_force_kalman_;
      mob_torque_kalman = mob_torque_kalman_;
      mob_force_adaptive = mob_force_adaptive_;
      mob_torque_adaptive = mob_torque_adaptive_;
      mob_force_k_ep_sweep = mob_force_k_ep_sweep_;
      mob_torque_k_ep_sweep = mob_torque_k_ep_sweep_;
      mob_2nd_tau_kfep = mob_2nd_tau_kfep_;
      mob_2nd_tau_consistency_term = mob_2nd_tau_consistency_term_;
      mob_2nd_tau_tauhat_world = mob_2nd_tau_tauhat_world_;
      mob_2nd_tau_rxf_world = mob_2nd_tau_rxf_world_;
      wind_force = wind_force_;
      ee_vel = ee_vel_;
      ee_pos = ee_pos_;
      c_hat_v_cmd = c_hat_v_cmd_;
      c_hat_v_act = c_hat_v_act_;
      c_hat_fx_act = c_hat_fx_act_;
      n_geo = n_geo_;
      n_f = n_f_;
      n_alg = n_alg_;
      f_g = f_g_;
      n_ke_gamma_proj = n_ke_gamma_proj_;
      normal_pure = normal_pure_;
      normal_ke = normal_ke_;
      eta_t_hat = eta_t_hat_;
      thrust_effectiveness_true = thrust_effectiveness_true_;
      alpha_frame = alpha_frame_;
      omega_n = omega_n_;
      normal_leakage = normal_leakage_;
      alpha_u1 = alpha_u1_;
      alpha_u2 = alpha_u2_;
      preload_feedback = preload_feedback_;
      c_tau = c_tau_;
      pattern_progress = pattern_progress_;
      pattern_speed_cmd = pattern_speed_cmd_;


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
      mask |= (have_contact_raw_ ? (1u<<10) : 0u);
      mask |= (have_contact_filt_ ? (1u<<11) : 0u);
      mask |= (have_cmd_force_ ? (1u<<12) : 0u);
      mask |= (have_force_lpf_ ? (1u<<13) : 0u);
      mask |= (have_mob_wrench_2nd_tau_i_ ? (1u<<14) : 0u);
      mask |= (have_mob_wrench_2nd_order_ ? (1u<<15) : 0u);
      mask |= (have_mob_wrench_2nd_tau_ ? (1u<<16) : 0u);
      mask |= (have_mob_2nd_tau_terms_ ? (1u<<17) : 0u);
      mask |= (have_mob_2nd_tau_consistency_ ? (1u<<18) : 0u);
      mask |= (have_wind_ ? (1u<<19) : 0u);
      mask |= (have_ee_pose_ ? (1u<<20) : 0u);
      mask |= (have_mob_wrench_kalman_ ? (1u<<28) : 0u);
      mask |= (have_ee_vel_ ? (1u<<29) : 0u);
      mask |= (have_mob_wrench_adaptive_ ? (1u<<30) : 0u);
      mask |= (have_contact_vel_cmd_ ? (1u<<21) : 0u);
      mask |= (have_contact_vel_actual_ ? (1u<<22) : 0u);
      mask |= (have_force_actual_ ? (1u<<23) : 0u);
      mask |= (have_normal_debug_metrics_ ? (1u<<24) : 0u);
      mask |= (have_control_metrics_ ? (1u<<25) : 0u);
      mask |= (have_normal_quat_pure_ ? (1u<<26) : 0u);
      mask |= (have_normal_quat_ke_ ? (1u<<27) : 0u);
      mask |= (have_mob_wrench_eta_t_ ? (1u<<31) : 0u);

    }

    const double t = this->get_clock()->now().seconds();

    const Eigen::Vector3d ke_force_ee_applied = -mob_force_2nd_tau;
    const double force_eps = 1.0e-9;
    const double vel_eps = compare_velocity_epsilon_;
    const double gamma_eps = compare_gamma_epsilon_;
    const double mob_force_norm = ke_force_ee_applied.norm();
    const double ee_vel_norm = ee_vel.norm();
    const bool have_force = std::isfinite(mob_force_norm) && mob_force_norm > force_eps;
    const bool have_vel_eps0 = std::isfinite(ee_vel_norm) && ee_vel_norm > 1.0e-12;
    const bool have_vel_eps = std::isfinite(ee_vel_norm) && ee_vel_norm > vel_eps;
    if (have_vel_eps) {
      w_s_direct = ee_vel / ee_vel_norm;
    }
    if (have_vel_eps0) {
      const double vel_norm_sq = ee_vel_norm * ee_vel_norm;
      gamma_v_direct = vel_norm_sq / (vel_norm_sq + gamma_eps);
    } else {
      gamma_v_direct = 0.0;
    }

    if (have_force) {
      n_ke_raw_direct = ke_force_ee_applied / mob_force_norm;

      Eigen::Vector3d w_s_eps0 = Eigen::Vector3d::Zero();
      if (have_vel_eps0) {
        w_s_eps0 = ee_vel / ee_vel_norm;
      }
      Eigen::Vector3d w_s_eps = Eigen::Vector3d::Zero();
      if (have_vel_eps) {
        w_s_eps = ee_vel / ee_vel_norm;
      }

      auto normalized_or_raw = [&](const Eigen::Vector3d & vec) {
        const double n = vec.norm();
        if (std::isfinite(n) && n > force_eps) {
          Eigen::Vector3d out = vec / n;
          if (out.dot(n_ke_raw_direct) < 0.0) {
            out = -out;
          }
          return out;
        }
        return n_ke_raw_direct;
      };

      const Eigen::Vector3d f_proj_eps0 =
        (Eigen::Matrix3d::Identity() - w_s_eps0 * w_s_eps0.transpose()) * ke_force_ee_applied;
      const Eigen::Vector3d f_proj_eps =
        (Eigen::Matrix3d::Identity() - w_s_eps * w_s_eps.transpose()) * ke_force_ee_applied;
      const Eigen::Vector3d f_proj_eps0_gamma =
        (Eigen::Matrix3d::Identity() - gamma_v_direct * (w_s_eps0 * w_s_eps0.transpose())) * ke_force_ee_applied;

      n_vel_eps0_direct = normalized_or_raw(f_proj_eps0);
      n_vel_eps_direct = normalized_or_raw(f_proj_eps);
      n_vel_eps0_gamma_direct = normalized_or_raw(f_proj_eps0_gamma);
      n_ke_gamma_proj_direct = n_vel_eps0_gamma_direct;
    }

    std_msgs::msg::Float64MultiArray msg;
    msg.data.resize(128);

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

    msg.data[36] = contact_F_raw.x();
    msg.data[37] = contact_F_raw.y();
    msg.data[38] = contact_F_raw.z();

    msg.data[39] = contact_F_filt.x();
    msg.data[40] = contact_F_filt.y();
    msg.data[41] = contact_F_filt.z();


    msg.data[42] = cmd_force;

    msg.data[43] = F_error_dot_raw;
    msg.data[44] = F_error_dot_filt;

    msg.data[45] = static_cast<double>(mask);
    msg.data[46] = quiet_nan();
    msg.data[47] = quiet_nan();
    msg.data[48] = quiet_nan();
    msg.data[49] = quiet_nan();
    msg.data[50] = quiet_nan();
    msg.data[51] = quiet_nan();
    msg.data[52] = mob_force_2nd_order.x();
    msg.data[53] = mob_force_2nd_order.y();
    msg.data[54] = mob_force_2nd_order.z();
    msg.data[55] = mob_torque_2nd_order.x();
    msg.data[56] = mob_torque_2nd_order.y();
    msg.data[57] = mob_torque_2nd_order.z();
    msg.data[58] = mob_force_2nd_tau.x();
    msg.data[59] = mob_force_2nd_tau.y();
    msg.data[60] = mob_force_2nd_tau.z();
    msg.data[61] = mob_torque_2nd_tau.x();
    msg.data[62] = mob_torque_2nd_tau.y();
    msg.data[63] = mob_torque_2nd_tau.z();
    msg.data[64] = mob_2nd_tau_kfep.x();
    msg.data[65] = mob_2nd_tau_kfep.y();
    msg.data[66] = mob_2nd_tau_kfep.z();
    msg.data[67] = mob_2nd_tau_consistency_term.x();
    msg.data[68] = mob_2nd_tau_consistency_term.y();
    msg.data[69] = mob_2nd_tau_consistency_term.z();
    msg.data[70] = mob_2nd_tau_tauhat_world.x();
    msg.data[71] = mob_2nd_tau_tauhat_world.y();
    msg.data[72] = mob_2nd_tau_tauhat_world.z();
    msg.data[73] = mob_2nd_tau_rxf_world.x();
    msg.data[74] = mob_2nd_tau_rxf_world.y();
    msg.data[75] = mob_2nd_tau_rxf_world.z();
    msg.data[76] = wind_force.x();
    msg.data[77] = wind_force.y();
    msg.data[78] = wind_force.z();
    msg.data[79] = ee_pos.x();
    msg.data[80] = ee_pos.y();
    msg.data[81] = ee_pos.z();
    msg.data[82] = c_hat_v_cmd.y();
    msg.data[83] = c_hat_v_act.y();
    msg.data[84] = c_hat_v_cmd.z();
    msg.data[85] = c_hat_v_act.z();
    msg.data[86] = c_hat_fx_act;
    msg.data[87] = alpha_frame;
    msg.data[88] = omega_n;
    msg.data[89] = normal_leakage;
    msg.data[90] = alpha_u1;
    msg.data[91] = alpha_u2;
    msg.data[92] = preload_feedback;
    msg.data[93] = c_tau;
    msg.data[94] = pattern_progress;
    msg.data[95] = pattern_speed_cmd;
    msg.data[96] = n_geo.x();
    msg.data[97] = n_geo.y();
    msg.data[98] = n_geo.z();
    msg.data[99] = n_f.x();
    msg.data[100] = n_f.y();
    msg.data[101] = n_f.z();
    msg.data[102] = n_alg.x();
    msg.data[103] = n_alg.y();
    msg.data[104] = n_alg.z();
    msg.data[105] = f_g.x();
    msg.data[106] = f_g.y();
    msg.data[107] = f_g.z();
    msg.data[108] = mob_force_2nd_tau_i.x();
    msg.data[109] = mob_force_2nd_tau_i.y();
    msg.data[110] = mob_force_2nd_tau_i.z();
    msg.data[111] = mob_torque_2nd_tau_i.x();
    msg.data[112] = mob_torque_2nd_tau_i.y();
    msg.data[113] = mob_torque_2nd_tau_i.z();
    msg.data[114] = mob_force_kalman.x();
    msg.data[115] = mob_force_kalman.y();
    msg.data[116] = mob_force_kalman.z();
    msg.data[117] = mob_force_adaptive.x();
    msg.data[118] = mob_force_adaptive.y();
    msg.data[119] = mob_force_adaptive.z();
    msg.data[120] = mob_force_eta_t.x();
    msg.data[121] = mob_force_eta_t.y();
    msg.data[122] = mob_force_eta_t.z();
    msg.data[123] = mob_torque_eta_t.x();
    msg.data[124] = mob_torque_eta_t.y();
    msg.data[125] = mob_torque_eta_t.z();
    msg.data[126] = eta_t_hat;
    msg.data[127] = thrust_effectiveness_true;



    pub_->publish(msg);

    if (csv_.is_open()) {
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
           << msg.data[36] << "," << msg.data[37] << "," << msg.data[38] << ","
           << msg.data[39] << "," << msg.data[40] << "," << msg.data[41] << ","
           << msg.data[42] << ","
           << msg.data[43] << "," << msg.data[44] << ","
           << static_cast<uint64_t>(mask) << ","
           << msg.data[52] << "," << msg.data[53] << "," << msg.data[54] << ","
           << msg.data[55] << "," << msg.data[56] << "," << msg.data[57] << ","
           << msg.data[58] << "," << msg.data[59] << "," << msg.data[60] << ","
           << msg.data[61] << "," << msg.data[62] << "," << msg.data[63] << ","
           << mob_force_2nd_tau_i.x() << "," << mob_force_2nd_tau_i.y() << "," << mob_force_2nd_tau_i.z() << ","
           << mob_torque_2nd_tau_i.x() << "," << mob_torque_2nd_tau_i.y() << "," << mob_torque_2nd_tau_i.z() << ","
           << mob_force_kalman.x() << "," << mob_force_kalman.y() << "," << mob_force_kalman.z() << ","
           << mob_torque_kalman.x() << "," << mob_torque_kalman.y() << "," << mob_torque_kalman.z() << ","
           << mob_force_adaptive.x() << "," << mob_force_adaptive.y() << "," << mob_force_adaptive.z() << ","
           << mob_torque_adaptive.x() << "," << mob_torque_adaptive.y() << "," << mob_torque_adaptive.z() << ","
           << mob_force_k_ep_sweep[0].x() << "," << mob_force_k_ep_sweep[0].y() << "," << mob_force_k_ep_sweep[0].z() << ","
           << mob_torque_k_ep_sweep[0].x() << "," << mob_torque_k_ep_sweep[0].y() << "," << mob_torque_k_ep_sweep[0].z() << ","
           << mob_force_k_ep_sweep[1].x() << "," << mob_force_k_ep_sweep[1].y() << "," << mob_force_k_ep_sweep[1].z() << ","
           << mob_torque_k_ep_sweep[1].x() << "," << mob_torque_k_ep_sweep[1].y() << "," << mob_torque_k_ep_sweep[1].z() << ","
           << mob_force_k_ep_sweep[2].x() << "," << mob_force_k_ep_sweep[2].y() << "," << mob_force_k_ep_sweep[2].z() << ","
           << mob_torque_k_ep_sweep[2].x() << "," << mob_torque_k_ep_sweep[2].y() << "," << mob_torque_k_ep_sweep[2].z() << ","
           << mob_force_k_ep_sweep[3].x() << "," << mob_force_k_ep_sweep[3].y() << "," << mob_force_k_ep_sweep[3].z() << ","
           << mob_torque_k_ep_sweep[3].x() << "," << mob_torque_k_ep_sweep[3].y() << "," << mob_torque_k_ep_sweep[3].z() << ","
           << mob_force_k_ep_sweep[4].x() << "," << mob_force_k_ep_sweep[4].y() << "," << mob_force_k_ep_sweep[4].z() << ","
           << mob_torque_k_ep_sweep[4].x() << "," << mob_torque_k_ep_sweep[4].y() << "," << mob_torque_k_ep_sweep[4].z() << ","
           << msg.data[64] << "," << msg.data[65] << "," << msg.data[66] << ","
           << msg.data[67] << "," << msg.data[68] << "," << msg.data[69] << ","
           << msg.data[70] << "," << msg.data[71] << "," << msg.data[72] << ","
           << msg.data[73] << "," << msg.data[74] << "," << msg.data[75] << ","
           << msg.data[76] << "," << msg.data[77] << "," << msg.data[78] << ","
           << msg.data[79] << "," << msg.data[80] << "," << msg.data[81] << ","
           << msg.data[82] << "," << msg.data[83] << "," << msg.data[84] << "," << msg.data[85] << ","
           << msg.data[86] << ","
           << msg.data[87] << "," << msg.data[88] << "," << msg.data[89] << "," << msg.data[90] << ","
           << msg.data[91] << "," << msg.data[92] << "," << msg.data[93] << "," << msg.data[94] << ","
           << msg.data[95] << ","
           << msg.data[96] << "," << msg.data[97] << "," << msg.data[98] << ","
           << msg.data[99] << "," << msg.data[100] << "," << msg.data[101] << ","
           << msg.data[102] << "," << msg.data[103] << "," << msg.data[104] << ","
           << msg.data[105] << "," << msg.data[106] << "," << msg.data[107] << ","
           << mob_force_2nd_order.x() << "," << mob_force_2nd_order.y() << "," << mob_force_2nd_order.z() << ","
           << mob_torque_2nd_order.x() << "," << mob_torque_2nd_order.y() << "," << mob_torque_2nd_order.z() << ","
           << mob_force_2nd_tau.x() << "," << mob_force_2nd_tau.y() << "," << mob_force_2nd_tau.z() << ","
           << mob_torque_2nd_tau.x() << "," << mob_torque_2nd_tau.y() << "," << mob_torque_2nd_tau.z() << ","
           << mob_2nd_tau_tauhat_world.x() << "," << mob_2nd_tau_tauhat_world.y() << "," << mob_2nd_tau_tauhat_world.z() << ","
           << mob_2nd_tau_rxf_world.x() << "," << mob_2nd_tau_rxf_world.y() << "," << mob_2nd_tau_rxf_world.z() << ","
           << mob_2nd_tau_consistency_term.x() << "," << mob_2nd_tau_consistency_term.y() << "," << mob_2nd_tau_consistency_term.z() << ","
           << quiet_nan() << ","
           << normal_pure.x() << "," << normal_pure.y() << "," << normal_pure.z() << ","
           << normal_ke.x() << "," << normal_ke.y() << "," << normal_ke.z() << ","
           << n_ke_raw_direct.x() << "," << n_ke_raw_direct.y() << "," << n_ke_raw_direct.z() << ","
           << n_ke_gamma_proj_direct.x() << "," << n_ke_gamma_proj_direct.y() << "," << n_ke_gamma_proj_direct.z() << ","
           << w_s_direct.x() << "," << w_s_direct.y() << "," << w_s_direct.z() << ","
           << gamma_v_direct << ","
           << n_vel_eps0_direct.x() << "," << n_vel_eps0_direct.y() << "," << n_vel_eps0_direct.z() << ","
           << n_vel_eps_direct.x() << "," << n_vel_eps_direct.y() << "," << n_vel_eps_direct.z() << ","
           << n_vel_eps0_gamma_direct.x() << "," << n_vel_eps0_gamma_direct.y() << "," << n_vel_eps0_gamma_direct.z() << ","
           << c_hat_fx_act << ","
           << mob_force_eta_t.x() << "," << mob_force_eta_t.y() << "," << mob_force_eta_t.z() << ","
           << mob_torque_eta_t.x() << "," << mob_torque_eta_t.y() << "," << mob_torque_eta_t.z() << ","
           << eta_t_hat << "," << thrust_effectiveness_true
           << "\n";

      if (++csv_line_count_ % 200 == 0) {
        csv_.flush();
      }
    }
  }

private:
  std::mutex mtx_;

  Eigen::Vector3d cmd_pos_{0,0,0};
  double cmd_yaw_{0.0};

  Eigen::Vector3d pos_{0,0,0};
  Eigen::Vector3d ee_pos_{0,0,0};
  Eigen::Vector3d ee_vel_{0,0,0};
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

  Eigen::Vector3d contact_F_raw_{0,0,0};
  Eigen::Vector3d contact_F_filt_{0,0,0};


  double cmd_force_{0.0};
  Eigen::Vector3d c_hat_v_cmd_{0,0,0};
  Eigen::Vector3d c_hat_v_act_{0,0,0};
  double c_hat_fx_act_{0.0};
  Eigen::Vector3d n_geo_{0,0,0};
  Eigen::Vector3d n_f_{0,0,0};
  Eigen::Vector3d n_alg_{0,0,0};
  Eigen::Vector3d f_g_{0,0,0};
  double alpha_frame_{quiet_nan()};
  double omega_n_{quiet_nan()};
  double normal_leakage_{quiet_nan()};
  double alpha_u1_{quiet_nan()};
  double alpha_u2_{quiet_nan()};
  double preload_feedback_{quiet_nan()};
  double c_tau_{quiet_nan()};
  double pattern_progress_{quiet_nan()};
  double pattern_speed_cmd_{quiet_nan()};
  Eigen::Vector3d n_ke_gamma_proj_{quiet_nan(), quiet_nan(), quiet_nan()};
  Eigen::Vector3d normal_pure_{quiet_nan(), quiet_nan(), quiet_nan()};
  Eigen::Vector3d normal_ke_{quiet_nan(), quiet_nan(), quiet_nan()};
  double F_error_dot_raw_{0.0};
  double F_error_dot_filt_{0.0};
  Eigen::Vector3d mob_force_2nd_order_{0,0,0};
  Eigen::Vector3d mob_torque_2nd_order_{0,0,0};
  Eigen::Vector3d mob_force_2nd_tau_{0,0,0};
  Eigen::Vector3d mob_torque_2nd_tau_{0,0,0};
  Eigen::Vector3d mob_force_eta_t_{0,0,0};
  Eigen::Vector3d mob_torque_eta_t_{0,0,0};
  Eigen::Vector3d mob_force_2nd_tau_i_{0,0,0};
  Eigen::Vector3d mob_torque_2nd_tau_i_{0,0,0};
  Eigen::Vector3d mob_force_kalman_{0,0,0};
  Eigen::Vector3d mob_torque_kalman_{0,0,0};
  Eigen::Vector3d mob_force_adaptive_{0,0,0};
  Eigen::Vector3d mob_torque_adaptive_{0,0,0};
  double eta_t_hat_{quiet_nan()};
  double thrust_effectiveness_true_{quiet_nan()};
  const std::array<int, 5> k_ep_sweep_gains_{{100, 300, 500, 700, 900}};
  std::array<Eigen::Vector3d, 5> mob_force_k_ep_sweep_{{
    Eigen::Vector3d{0,0,0}, Eigen::Vector3d{0,0,0}, Eigen::Vector3d{0,0,0},
    Eigen::Vector3d{0,0,0}, Eigen::Vector3d{0,0,0}}};
  std::array<Eigen::Vector3d, 5> mob_torque_k_ep_sweep_{{
    Eigen::Vector3d{0,0,0}, Eigen::Vector3d{0,0,0}, Eigen::Vector3d{0,0,0},
    Eigen::Vector3d{0,0,0}, Eigen::Vector3d{0,0,0}}};
  Eigen::Vector3d mob_2nd_tau_kfep_{0,0,0};
  Eigen::Vector3d mob_2nd_tau_consistency_term_{0,0,0};
  Eigen::Vector3d mob_2nd_tau_tauhat_world_{0,0,0};
  Eigen::Vector3d mob_2nd_tau_rxf_world_{0,0,0};
  Eigen::Vector3d wind_force_{0,0,0};


  bool have_force_lpf_{false};
  bool have_cmd_{false};
  bool have_pose_{false};
  bool have_ee_pose_{false};
  bool have_ee_vel_{false};
  bool have_vel_{false};
  bool have_w_{false};
  bool have_acc_{false};
  bool have_angacc_{false};
  bool have_vdes_{false};
  bool have_rpydes_{false};
  bool have_wdes_{false};
  bool have_input_{false};
  bool have_cmd_force_{false};
  bool have_contact_vel_cmd_{false};
  bool have_contact_vel_actual_{false};
  bool have_force_actual_{false};
  bool have_normal_debug_metrics_{false};
  bool have_control_metrics_{false};
  bool have_contact_raw_{false};
  bool have_contact_filt_{false};
  bool have_mob_wrench_2nd_order_{false};
  bool have_mob_wrench_2nd_tau_{false};
  bool have_mob_wrench_eta_t_{false};
  bool have_mob_wrench_2nd_tau_i_{false};
  bool have_mob_wrench_kalman_{false};
  bool have_mob_wrench_adaptive_{false};
  std::array<bool, 5> have_mob_wrench_k_ep_sweep_{{false, false, false, false, false}};
  bool have_mob_2nd_tau_terms_{false};
  bool have_mob_2nd_tau_consistency_{false};
  bool have_wind_{false};
  bool have_normal_quat_pure_{false};
  bool have_normal_quat_ke_{false};
  bool have_eta_t_hat_{false};
  bool have_thrust_effectiveness_true_{false};

  double publish_hz_{400.0};
  double compare_velocity_epsilon_{0.01};
  double compare_gamma_epsilon_{3.0e-3};
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr sub_cmd_;
  rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr sub_input_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr sub_pose_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr sub_ee_pose_;
  rclcpp::Subscription<geometry_msgs::msg::Vector3Stamped>::SharedPtr sub_ee_vel_;
  rclcpp::Subscription<geometry_msgs::msg::Vector3Stamped>::SharedPtr sub_vel_;
  rclcpp::Subscription<geometry_msgs::msg::Vector3Stamped>::SharedPtr sub_w_;
  rclcpp::Subscription<geometry_msgs::msg::Vector3Stamped>::SharedPtr sub_acc_;
  rclcpp::Subscription<geometry_msgs::msg::Vector3Stamped>::SharedPtr sub_angacc_;
  rclcpp::Subscription<geometry_msgs::msg::Vector3Stamped>::SharedPtr sub_vdes_;
  rclcpp::Subscription<geometry_msgs::msg::Vector3Stamped>::SharedPtr sub_rpydes_;
  rclcpp::Subscription<geometry_msgs::msg::Vector3Stamped>::SharedPtr sub_wdes_;
  rclcpp::Subscription<geometry_msgs::msg::WrenchStamped>::SharedPtr sub_contact_raw_;
  rclcpp::Subscription<geometry_msgs::msg::WrenchStamped>::SharedPtr sub_contact_filt_;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr sub_cmd_force_;
  rclcpp::Subscription<geometry_msgs::msg::Vector3Stamped>::SharedPtr sub_contact_vel_cmd_;
  rclcpp::Subscription<geometry_msgs::msg::Vector3Stamped>::SharedPtr sub_contact_vel_actual_;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr sub_force_actual_;
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr sub_normal_debug_metrics_;
  rclcpp::Subscription<geometry_msgs::msg::QuaternionStamped>::SharedPtr sub_normal_quat_pure_;
  rclcpp::Subscription<geometry_msgs::msg::QuaternionStamped>::SharedPtr sub_normal_quat_ke_;
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr sub_force_lpf_;
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr sub_control_metrics_;
  rclcpp::Subscription<geometry_msgs::msg::WrenchStamped>::SharedPtr sub_mob_wrench_2nd_order_;
  rclcpp::Subscription<geometry_msgs::msg::WrenchStamped>::SharedPtr sub_mob_wrench_2nd_tau_;
  rclcpp::Subscription<geometry_msgs::msg::WrenchStamped>::SharedPtr sub_mob_wrench_eta_t_;
  rclcpp::Subscription<geometry_msgs::msg::WrenchStamped>::SharedPtr sub_mob_wrench_2nd_tau_i_;
  rclcpp::Subscription<geometry_msgs::msg::WrenchStamped>::SharedPtr sub_mob_wrench_kalman_;
  rclcpp::Subscription<geometry_msgs::msg::WrenchStamped>::SharedPtr sub_mob_wrench_adaptive_;
  std::array<rclcpp::Subscription<geometry_msgs::msg::WrenchStamped>::SharedPtr, 5>
    sub_mob_wrench_k_ep_sweep_;
  rclcpp::Subscription<geometry_msgs::msg::WrenchStamped>::SharedPtr sub_mob_2nd_tau_terms_;
  rclcpp::Subscription<geometry_msgs::msg::WrenchStamped>::SharedPtr sub_mob_2nd_tau_consistency_;
  rclcpp::Subscription<geometry_msgs::msg::Vector3Stamped>::SharedPtr sub_wind_;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr sub_eta_t_hat_;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr sub_thrust_effectiveness_true_;


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
