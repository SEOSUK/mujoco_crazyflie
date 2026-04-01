#include <rclcpp/rclcpp.hpp>

#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>

#include <geometry_msgs/msg/wrench_stamped.hpp>
#include <geometry_msgs/msg/quaternion_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/vector3_stamped.hpp>


#include <array>
#include <algorithm>
#include <cmath>
#include <mutex>
#include <limits>
#include <string>
#include <vector>

#include <Eigen/Dense>

using namespace std::chrono_literals;

class TrajectoryGeneration : public rclcpp::Node
{
public:
  TrajectoryGeneration()
  : Node("trajectory_generation")
  {
    // --------------------------------------------------
    // Parameters
    // --------------------------------------------------
    reference_object_ = this->declare_parameter<std::string>(
      "reference_object", "drone");   // "drone" or "end_effector"

    normal_estimator_method_ = this->declare_parameter<std::string>(
      "normal_estimator_method", "direct"); // "direct", "kroc", "action_normal"

    flip_measured_force_ = this->declare_parameter<bool>(
      "flip_measured_force", false);

    yaw_align_kp_ = this->declare_parameter<double>(
      "yaw_align_kp", 3.0);

    normal_xy_min_ = this->declare_parameter<double>(
      "normal_xy_min", 1e-3);

    normal_force_threshold_ = this->declare_parameter<double>(
      "normal_force_threshold", 0.005);

    normal_lpf_alpha_ = this->declare_parameter<double>(
      "normal_lpf_alpha", 0.2);

    force_adm_kp_ = this->declare_parameter<double>(
      "force_adm_kp", 5.0);

    force_adm_kd_ = this->declare_parameter<double>(
      "force_adm_kd", 0.005);

    auto ee_off = this->declare_parameter<std::vector<double>>(
      "end_effector_offset", {0.09, 0.0, 0.085});
    if (ee_off.size() != 3) {
      RCLCPP_WARN(this->get_logger(),
                  "end_effector_offset must be size 3. Fallback to [0,0,0].");
      ee_off = {0.0, 0.0, 0.0};
    }
    d_B_ = Eigen::Vector3d(ee_off[0], ee_off[1], ee_off[2]);

    // topics
    pose_topic_ = this->declare_parameter<std::string>(
      "pose_topic", "/crazyflie/out/pose");
    vel_topic_ = this->declare_parameter<std::string>(
      "vel_topic", "/crazyflie/out/vel");
    acc_topic_ = this->declare_parameter<std::string>(
      "acc_topic", "/crazyflie/out/acc");

    ee_pose_topic_ = this->declare_parameter<std::string>(
      "ee_pose_topic", "/crazyflie/out/EE_pose");
    ee_vel_topic_ = this->declare_parameter<std::string>(
      "ee_vel_topic", "/crazyflie/out/EE_velocity");
    ee_acc_topic_ = this->declare_parameter<std::string>(
      "ee_acc_topic", "/crazyflie/out/EE_acceleration");

    contact_force_topic_ = this->declare_parameter<std::string>(
      "contact_force_topic", "/crazyflie/out/EE_contact_force_filt");

    // --------------------------------------------------
    // Subscribers
    // --------------------------------------------------
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
      contact_force_topic_, 10,
      std::bind(&TrajectoryGeneration::contactForceCb, this, std::placeholders::_1));

    sub_pose_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
      pose_topic_, 10,
      std::bind(&TrajectoryGeneration::poseCb, this, std::placeholders::_1));

    sub_vel_ = this->create_subscription<geometry_msgs::msg::Vector3Stamped>(
      vel_topic_, 10,
      std::bind(&TrajectoryGeneration::velCb, this, std::placeholders::_1));

    sub_acc_ = this->create_subscription<geometry_msgs::msg::Vector3Stamped>(
      acc_topic_, 10,
      std::bind(&TrajectoryGeneration::accCb, this, std::placeholders::_1));

    sub_ee_pose_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
      ee_pose_topic_, 10,
      std::bind(&TrajectoryGeneration::eePoseCb, this, std::placeholders::_1));

    sub_ee_vel_ = this->create_subscription<geometry_msgs::msg::Vector3Stamped>(
      ee_vel_topic_, 10,
      std::bind(&TrajectoryGeneration::eeVelCb, this, std::placeholders::_1));

    sub_ee_acc_ = this->create_subscription<geometry_msgs::msg::Vector3Stamped>(
      ee_acc_topic_, 10,
      std::bind(&TrajectoryGeneration::eeAccCb, this, std::placeholders::_1));

    // --------------------------------------------------
    // Publishers
    // --------------------------------------------------
    pub_pos_cmd_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(
      "/crazyflie/in/pos_cmd", 10);

    pub_force_lpf_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(
      "/su/force_lpf", 10);

    pub_contact_quat_ = this->create_publisher<geometry_msgs::msg::QuaternionStamped>(
      "/estimated_contact_frame_quat", 10);

    pub_contact_force_x_ = this->create_publisher<std_msgs::msg::Float32>(
      "/su/contact_force_x", 10);

    pub_contact_vel_cmd_ = this->create_publisher<geometry_msgs::msg::Vector3Stamped>(
      "/su/debug/contact_vel_cmd", 10);

    pub_contact_vel_actual_ = this->create_publisher<geometry_msgs::msg::Vector3Stamped>(
      "/su/debug/contact_vel_actual", 10);

    // new debug pose publishers
    pub_cmd_drone_pose_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
      "/crazyflie/debug/cmd_drone", 10);

    pub_cmd_ee_pose_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
      "/crazyflie/debug/cmd_ee", 10);

    pub_cmd_active_pose_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
      "/crazyflie/debug/cmd_active", 10);

    // --------------------------------------------------
    // Timers
    // --------------------------------------------------
    timer_ = this->create_wall_timer(
      10ms, std::bind(&TrajectoryGeneration::update, this)); // 100 Hz

    force_timer_ = this->create_wall_timer(
      10ms, std::bind(&TrajectoryGeneration::forceUpdate, this)); // 100 Hz

    RCLCPP_INFO(this->get_logger(), "trajectory_generation started");
    RCLCPP_INFO(this->get_logger(), "reference_object = %s", reference_object_.c_str());
    RCLCPP_INFO(this->get_logger(), "normal_estimator_method = %s", normal_estimator_method_.c_str());
    RCLCPP_INFO(this->get_logger(), "EE offset d_B = [%.4f %.4f %.4f]",
                d_B_.x(), d_B_.y(), d_B_.z());
  }

private:
  // ==================================================
  // Utils
  // ==================================================
  static inline double wrap_pi(double a)
  {
    while (a > M_PI) a -= 2.0 * M_PI;
    while (a < -M_PI) a += 2.0 * M_PI;
    return a;
  }

  static Eigen::Quaterniond quatMsgToEigen(const geometry_msgs::msg::Quaternion &q)
  {
    return Eigen::Quaterniond(q.w, q.x, q.y, q.z);
  }

  static geometry_msgs::msg::Quaternion yawToQuatMsg(double yaw)
  {
    geometry_msgs::msg::Quaternion q;
    q.w = std::cos(0.5 * yaw);
    q.x = 0.0;
    q.y = 0.0;
    q.z = std::sin(0.5 * yaw);
    return q;
  }

  // ==================================================
  // Reference state
  // ==================================================
  struct ReferenceState
  {
    Eigen::Vector3d pos_w = Eigen::Vector3d::Zero();
    Eigen::Vector3d vel_w = Eigen::Vector3d::Zero();
    Eigen::Vector3d acc_w = Eigen::Vector3d::Zero();
    double yaw_w = 0.0;
    bool valid = false;
  };

  ReferenceState getReferenceStateLocked() const
  {
    ReferenceState ref;
    const bool use_ee = (reference_object_ == "end_effector");

    if (!use_ee) {
      if (!(pose_received_ && vel_received_ && acc_received_)) {
        ref.valid = false;
        return ref;
      }

      ref.pos_w = Eigen::Vector3d(pose_w_[0], pose_w_[1], pose_w_[2]);
      ref.vel_w = Eigen::Vector3d(vel_w_[0], vel_w_[1], vel_w_[2]);
      ref.acc_w = Eigen::Vector3d(acc_w_[0], acc_w_[1], acc_w_[2]);
      ref.yaw_w = yaw_w_;
      ref.valid = true;
      return ref;
    }

    if (!(ee_pose_received_ && ee_vel_received_ && ee_acc_received_ && pose_received_)) {
      ref.valid = false;
      return ref;
    }

    ref.pos_w = Eigen::Vector3d(ee_pose_w_[0], ee_pose_w_[1], ee_pose_w_[2]);
    ref.vel_w = Eigen::Vector3d(ee_vel_w_[0], ee_vel_w_[1], ee_vel_w_[2]);
    ref.acc_w = Eigen::Vector3d(ee_acc_w_[0], ee_acc_w_[1], ee_acc_w_[2]);
    ref.yaw_w = yaw_w_;
    ref.valid = true;
    return ref;
  }

  // ==================================================
  // Contact frame estimation
  // ==================================================
  struct ContactFrame
  {
    Eigen::Vector3d n_w = Eigen::Vector3d::UnitX();
    Eigen::Matrix3d R_C = Eigen::Matrix3d::Identity(); // contact -> world
    bool valid = false;
  };

  bool estimateNormalVector_Force_directly(
    ContactFrame &cf_out,
    const ReferenceState & /*ref*/)
  {
    std::array<double,3> Fw_arr{0.0, 0.0, 0.0};
    {
      std::lock_guard<std::mutex> lk(force_mtx_);
      Fw_arr = contact_F_;
    }

    Eigen::Vector3d Fw(Fw_arr[0], Fw_arr[1], Fw_arr[2]);
    const double Fnorm = Fw.norm();

    if (Fnorm < normal_force_threshold_) {
      cf_out.valid = false;
      return false;
    }

    Eigen::Vector3d n_new = Fw / (Fnorm + 1e-12);

    if (flip_measured_force_) {
      n_new = -n_new;
    }

    if (!cf_out.valid) {
      cf_out.n_w = n_new;
    } else {
      cf_out.n_w =
        (1.0 - normal_lpf_alpha_) * cf_out.n_w +
        normal_lpf_alpha_ * n_new;

      const double nn = cf_out.n_w.norm();
      if (nn > 1e-9) {
        cf_out.n_w /= nn;
      } else {
        cf_out.n_w = n_new;
      }
    }

    return true;
  }

  bool estimateNormalVector_KROC(
    ContactFrame &cf_out,
    const ReferenceState &ref)
  {
    std::array<double,3> Fw_arr{0.0, 0.0, 0.0};
    {
      std::lock_guard<std::mutex> lk(force_mtx_);
      Fw_arr = contact_F_;
    }

    Eigen::Vector3d nf(Fw_arr[0], Fw_arr[1], Fw_arr[2]);
    const double Fnorm = nf.norm();
    if (Fnorm < normal_force_threshold_) {
      cf_out.valid = false;
      return false;
    }

    Eigen::Vector3d v_ref = ref.vel_w;
    const double vnorm_sq = v_ref.squaredNorm();

    Eigen::Vector3d n_new = nf;
    const double vel_eps_sq = 1e-8;
    if (ref.valid && vnorm_sq > vel_eps_sq) {
      const double alpha = nf.dot(v_ref) / vnorm_sq;
      n_new = nf - alpha * v_ref;
    }

    const double n_norm = n_new.norm();
    if (n_norm < 1e-9) {
      cf_out.valid = false;
      return false;
    }
    n_new /= n_norm;

    if (flip_measured_force_) {
      n_new = -n_new;
    }

    if (!cf_out.valid) {
      cf_out.n_w = n_new;
    } else {
      if (cf_out.n_w.dot(n_new) < 0.0) {
        n_new = -n_new;
      }

      cf_out.n_w =
        (1.0 - normal_lpf_alpha_) * cf_out.n_w +
        normal_lpf_alpha_ * n_new;

      const double nn = cf_out.n_w.norm();
      if (nn > 1e-9) {
        cf_out.n_w /= nn;
      } else {
        cf_out.n_w = n_new;
      }
    }

    return true;
  }

  bool estimateNormalVector_ActionNormal_0326(
    ContactFrame &cf_out,
    const ReferenceState &ref)
  {
    std::array<double,3> Fw_arr{0.0, 0.0, 0.0};
    {
      std::lock_guard<std::mutex> lk(force_mtx_);
      Fw_arr = contact_F_;
    }

    Eigen::Vector3d Fw(Fw_arr[0], Fw_arr[1], Fw_arr[2]);
    Eigen::Vector3d vw = ref.vel_w;
    Eigen::Vector3d aw = ref.acc_w;

    if (flip_measured_force_) {
      Fw = -Fw;
    }

    const double Fnorm = Fw.norm();
    if (Fnorm < normal_force_threshold_) {
      cf_out.valid = false;
      return false;
    }

    Eigen::Vector3d n_nom = Fw / (Fnorm + 1e-12);

    if (cf_out.valid && cf_out.n_w.dot(n_nom) < 0.0) {
      n_nom = -n_nom;
    }

    Eigen::Vector3d ez = Eigen::Vector3d::UnitZ();
    Eigen::Vector3d t1 = ez.cross(n_nom);
    if (t1.norm() < 1e-6) {
      t1 = Eigen::Vector3d::UnitX();
    } else {
      t1.normalize();
    }

    const double w_tau = 1.0;
    const double w_f   = 1.0;
    const double w_a   = 0.2;

    const double theta_max = 20.0 * M_PI / 180.0;
    const double delta_max = std::tan(theta_max);

    const int N = 9;
    const double vel_xy_eps = 1e-6;
    const double eps = 1e-9;

    double best_J = std::numeric_limits<double>::infinity();
    Eigen::Vector3d n_best = n_nom;
    bool found = false;

    for (int i = 0; i < N; ++i) {
      const double d1 = -delta_max + 2.0 * delta_max * static_cast<double>(i) / static_cast<double>(N - 1);

      Eigen::Vector3d n_cand_unnorm = n_nom + d1 * t1;
      const double nn = n_cand_unnorm.norm();
      if (nn < eps) {
        continue;
      }

      Eigen::Vector3d n_cand = n_cand_unnorm / nn;

      if (n_cand.dot(n_nom) < 0.0) {
        n_cand = -n_cand;
      }

      if (n_cand.dot(n_nom) < std::cos(theta_max)) {
        continue;
      }

      const double F_normal = Fw.dot(n_cand);
      if (F_normal <= normal_force_threshold_) {
        continue;
      }

      const Eigen::Vector3d F_tan = Fw - F_normal * n_cand;
      const double F_n_leak = F_tan.norm();

      double a_t_int = 0.0;
      if (ref.valid) {
        const Eigen::Vector3d a_tan = aw - n_cand * (n_cand.dot(aw));
        a_t_int = a_tan.norm();
      }

      double tau_z = 0.0;
      if (ref.valid) {
        Eigen::Vector3d v_tan = vw - n_cand * (n_cand.dot(vw));
        Eigen::Vector2d n_xy(n_cand.x(), n_cand.y());
        Eigen::Vector2d v_xy(v_tan.x(), v_tan.y());

        const double nxy = n_xy.norm();
        const double vxy = v_xy.norm();

        if (nxy > vel_xy_eps && vxy > vel_xy_eps) {
          n_xy /= nxy;
          v_xy /= vxy;
          tau_z = std::abs(n_xy.x() * v_xy.y() - n_xy.y() * v_xy.x());
        }
      }

      const double J =
        w_tau * tau_z * tau_z +
        w_f   * F_n_leak * F_n_leak +
        w_a   * a_t_int * a_t_int;

      if (J < best_J) {
        best_J = J;
        n_best = n_cand;
        found = true;
      }
    }

    if (!found) {
      n_best = n_nom;
    }

    if (!cf_out.valid) {
      cf_out.n_w = n_best;
    } else {
      if (cf_out.n_w.dot(n_best) < 0.0) {
        n_best = -n_best;
      }

      cf_out.n_w =
        (1.0 - normal_lpf_alpha_) * cf_out.n_w +
        normal_lpf_alpha_ * n_best;

      const double nlp = cf_out.n_w.norm();
      if (nlp > 1e-9) {
        cf_out.n_w /= nlp;
      } else {
        cf_out.n_w = n_best;
      }
    }

    return true;
  }

  bool buildContactFrameFromNormal(ContactFrame &cf_out)
  {
    const double n_norm = cf_out.n_w.norm();
    if (n_norm < 1e-9) {
      cf_out.valid = false;
      return false;
    }

    const Eigen::Vector3d x_c = cf_out.n_w.normalized();
    const Eigen::Vector3d g_axis = Eigen::Vector3d::UnitZ();

    Eigen::Vector3d y_c = x_c.cross(g_axis);
    double y_norm = y_c.norm();

    if (y_norm < 1e-6) {
      Eigen::Vector3d a = Eigen::Vector3d::UnitY();
      if (std::abs(x_c.dot(a)) > 0.9) {
        a = Eigen::Vector3d::UnitX();
      }

      y_c = x_c.cross(a);
      y_norm = y_c.norm();

      if (y_norm < 1e-9) {
        cf_out.valid = false;
        return false;
      }
    }
    y_c /= (y_norm + 1e-12);
    y_c = -y_c;

    Eigen::Vector3d z_c = x_c.cross(y_c);
    const double z_norm = z_c.norm();
    if (z_norm < 1e-9) {
      cf_out.valid = false;
      return false;
    }
    z_c /= z_norm;

    cf_out.R_C.col(0) = x_c;
    cf_out.R_C.col(1) = y_c;
    cf_out.R_C.col(2) = z_c;
    cf_out.valid = true;
    return true;
  }

  bool updateContactFrame(ContactFrame &cf_out, const ReferenceState &ref)
  {
    bool ok = false;

    if (normal_estimator_method_ == "kroc") {
      ok = estimateNormalVector_KROC(cf_out, ref);
    } else if (normal_estimator_method_ == "action_normal") {
      ok = estimateNormalVector_ActionNormal_0326(cf_out, ref);
    } else {
      ok = estimateNormalVector_Force_directly(cf_out, ref);
    }

    if (!ok) {
      cf_out.valid = false;
      return false;
    }

    if (!buildContactFrameFromNormal(cf_out)) {
      cf_out.valid = false;
      return false;
    }

    return true;
  }

  void publishContactQuat(const Eigen::Matrix3d &R_C, const rclcpp::Time &stamp)
  {
    Eigen::Quaterniond q(R_C);
    q.normalize();

    geometry_msgs::msg::QuaternionStamped msg;
    msg.header.stamp = stamp;
    msg.header.frame_id = "world";
    msg.quaternion.w = q.w();
    msg.quaternion.x = q.x();
    msg.quaternion.y = q.y();
    msg.quaternion.z = q.z();
    pub_contact_quat_->publish(msg);
  }

  // ==================================================
  // Debug command pose publishing
  // ==================================================
  void publishDebugCmdPoses(
    const Eigen::Vector3d &p_drone_des_w,
    const Eigen::Vector3d &p_ee_des_w,
    double yaw_des,
    const rclcpp::Time &stamp)
  {
    geometry_msgs::msg::PoseStamped msg_drone;
    msg_drone.header.stamp = stamp;
    msg_drone.header.frame_id = "world";
    msg_drone.pose.position.x = p_drone_des_w.x();
    msg_drone.pose.position.y = p_drone_des_w.y();
    msg_drone.pose.position.z = p_drone_des_w.z();
    msg_drone.pose.orientation = yawToQuatMsg(yaw_des);
    pub_cmd_drone_pose_->publish(msg_drone);

    geometry_msgs::msg::PoseStamped msg_ee;
    msg_ee.header.stamp = stamp;
    msg_ee.header.frame_id = "world";
    msg_ee.pose.position.x = p_ee_des_w.x();
    msg_ee.pose.position.y = p_ee_des_w.y();
    msg_ee.pose.position.z = p_ee_des_w.z();
    msg_ee.pose.orientation = yawToQuatMsg(yaw_des);
    pub_cmd_ee_pose_->publish(msg_ee);

    geometry_msgs::msg::PoseStamped msg_active;
    msg_active.header.stamp = stamp;
    msg_active.header.frame_id = "world";

    if (reference_object_ == "end_effector") {
      msg_active.pose.position.x = p_ee_des_w.x();
      msg_active.pose.position.y = p_ee_des_w.y();
      msg_active.pose.position.z = p_ee_des_w.z();
    } else {
      msg_active.pose.position.x = p_drone_des_w.x();
      msg_active.pose.position.y = p_drone_des_w.y();
      msg_active.pose.position.z = p_drone_des_w.z();
    }
    msg_active.pose.orientation = yawToQuatMsg(yaw_des);
    pub_cmd_active_pose_->publish(msg_active);
  }

  void publishContactVelocityDebug(
    const Eigen::Vector3d &v_cmd_c,
    const Eigen::Vector3d &v_act_c,
    const rclcpp::Time &stamp)
  {
    geometry_msgs::msg::Vector3Stamped cmd_msg;
    cmd_msg.header.stamp = stamp;
    cmd_msg.header.frame_id = "contact";
    cmd_msg.vector.x = v_cmd_c.x();
    cmd_msg.vector.y = v_cmd_c.y();
    cmd_msg.vector.z = v_cmd_c.z();
    pub_contact_vel_cmd_->publish(cmd_msg);

    geometry_msgs::msg::Vector3Stamped act_msg;
    act_msg.header.stamp = stamp;
    act_msg.header.frame_id = "contact";
    act_msg.vector.x = v_act_c.x();
    act_msg.vector.y = v_act_c.y();
    act_msg.vector.z = v_act_c.z();
    pub_contact_vel_actual_->publish(act_msg);
  }

  // ==================================================
  // Callbacks
  // ==================================================
  void keyboardCb(const std_msgs::msg::Float64MultiArray::SharedPtr msg)
  {
    if (msg->data.size() < 4) return;

    sp_in_[0] = msg->data[0];
    sp_in_[1] = msg->data[1];
    sp_in_[2] = msg->data[2];
    sp_in_yaw_ = msg->data[3];
    sp_received_ = true;
  }

  void useVelModeCb(const std_msgs::msg::Float32::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lk(force_mtx_);
    su_cmd_use_vel_mode_ = msg->data;
  }

  void cmdForceCb(const std_msgs::msg::Float32::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lk(force_mtx_);
    su_cmd_fx_ = msg->data;
  }

  void contactForceCb(const geometry_msgs::msg::WrenchStamped::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lk(force_mtx_);
    contact_F_[0] = msg->wrench.force.x;
    contact_F_[1] = msg->wrench.force.y;
    contact_F_[2] = msg->wrench.force.z;
    f_ext_received_ = true;
  }

  void poseCb(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lk(state_mtx_);

    pose_w_[0] = msg->pose.position.x;
    pose_w_[1] = msg->pose.position.y;
    pose_w_[2] = msg->pose.position.z;

    const auto &q = msg->pose.orientation;
    q_WB_ = quatMsgToEigen(q).normalized();

    const double siny_cosp = 2.0 * (q.w * q.z + q.x * q.y);
    const double cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
    yaw_w_ = std::atan2(siny_cosp, cosy_cosp);

    pose_received_ = true;
  }

  void velCb(const geometry_msgs::msg::Vector3Stamped::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lk(state_mtx_);
    vel_w_[0] = msg->vector.x;
    vel_w_[1] = msg->vector.y;
    vel_w_[2] = msg->vector.z;
    vel_received_ = true;
  }

  void accCb(const geometry_msgs::msg::Vector3Stamped::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lk(state_mtx_);
    acc_w_[0] = msg->vector.x;
    acc_w_[1] = msg->vector.y;
    acc_w_[2] = msg->vector.z;
    acc_received_ = true;
  }

  void eePoseCb(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lk(state_mtx_);
    ee_pose_w_[0] = msg->pose.position.x;
    ee_pose_w_[1] = msg->pose.position.y;
    ee_pose_w_[2] = msg->pose.position.z;
    ee_pose_received_ = true;
  }

  void eeVelCb(const geometry_msgs::msg::Vector3Stamped::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lk(state_mtx_);
    ee_vel_w_[0] = msg->vector.x;
    ee_vel_w_[1] = msg->vector.y;
    ee_vel_w_[2] = msg->vector.z;
    ee_vel_received_ = true;
  }

  void eeAccCb(const geometry_msgs::msg::Vector3Stamped::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lk(state_mtx_);
    ee_acc_w_[0] = msg->vector.x;
    ee_acc_w_[1] = msg->vector.y;
    ee_acc_w_[2] = msg->vector.z;
    ee_acc_received_ = true;
  }

  // ==================================================
  // Publish controller output + debug command poses
  // ==================================================
  void publishOut()
  {
    Eigen::Quaterniond q_WB;
    bool pose_ok = false;
    {
      std::lock_guard<std::mutex> lk(state_mtx_);
      q_WB = q_WB_;
      pose_ok = pose_received_;
    }

    if (!pose_ok) {
      return;
    }

    const Eigen::Matrix3d R_WB = q_WB.toRotationMatrix();

    Eigen::Vector3d p_active_des_w(
      su_int_ref_pos_[0],
      su_int_ref_pos_[1],
      su_int_ref_pos_[2]);

    Eigen::Vector3d p_drone_des_w = Eigen::Vector3d::Zero();
    Eigen::Vector3d p_ee_des_w = Eigen::Vector3d::Zero();

    if (reference_object_ == "end_effector") {
      p_ee_des_w = p_active_des_w;
      p_drone_des_w = p_ee_des_w - R_WB * d_B_;
    } else {
      p_drone_des_w = p_active_des_w;
      p_ee_des_w = p_drone_des_w + R_WB * d_B_;
    }

    // controller input: always drone command
    std_msgs::msg::Float64MultiArray out;
    out.data.resize(4);
    out.data[0] = p_drone_des_w.x();
    out.data[1] = p_drone_des_w.y();
    out.data[2] = p_drone_des_w.z();
    out.data[3] = su_int_ref_yaw_;
    pub_pos_cmd_->publish(out);

    // debug poses
    publishDebugCmdPoses(
      p_drone_des_w,
      p_ee_des_w,
      su_int_ref_yaw_,
      this->now());
  }

  // ==================================================
  // Force update
  // ==================================================
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

    float su_cmd_fx_local = 0.0f;
    std::array<double,3> contact_F_local{0.0,0.0,0.0};
    bool f_ok = false;

    {
      std::lock_guard<std::mutex> lk(force_mtx_);
      su_cmd_fx_local = su_cmd_fx_;
      contact_F_local = contact_F_;
      f_ok = f_ext_received_;
    }
    if (!f_ok) return;

    ReferenceState ref;
    {
      std::lock_guard<std::mutex> lk(state_mtx_);
      ref = getReferenceStateLocked();
    }

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

    Eigen::Vector3d Fw(contact_F_local[0], contact_F_local[1], contact_F_local[2]);
    if (flip_measured_force_) {
      Fw = -Fw;
    }

    ContactFrame cf_local = cf_;
    const bool cf_ok = updateContactFrame(cf_local, ref);
    if (!(cf_ok && cf_local.valid)) {
      std_msgs::msg::Float32 fcx_msg;
      fcx_msg.data = std::numeric_limits<float>::quiet_NaN();
      pub_contact_force_x_->publish(fcx_msg);
      eF_state_initialized_ = false;
      return;
    }
    cf_ = cf_local;

    const Eigen::Vector3d Fc = cf_local.R_C.transpose() * Fw;
    const double Fcx = Fc.x();

    std_msgs::msg::Float32 fcx_msg;
    fcx_msg.data = static_cast<float>(Fcx);
    pub_contact_force_x_->publish(fcx_msg);

    std::array<double,3> eF{0.0,0.0,0.0};
    eF[0] = static_cast<double>(su_cmd_fx_local) - Fcx;

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

    std::array<double,3> eF_dot_raw{0.0,0.0,0.0};
    eF_dot_raw[0] = (eF[0] - eF_prev_[0]) / dt;

    const double wc = 3.0;
    const double alpha = wc * dt;
    const double a = std::clamp(alpha, 0.0, 1.0);

    std::array<double,3> eF_dot_filt{0.0,0.0,0.0};
    eF_dot_filt[0] = (1.0 - a) * eF_dot_filt_prev_[0] + a * eF_dot_raw[0];

    eF_prev_ = eF;
    eF_dot_filt_prev_ = eF_dot_filt;

    {
      std::lock_guard<std::mutex> lk(force_mtx_);
      eF_ = eF;
      eF_dot_filt_ = eF_dot_filt;
    }

    std_msgs::msg::Float64MultiArray lpf;
    lpf.data.resize(2);
    lpf.data[0] = eF_dot_raw[0];
    lpf.data[1] = eF_dot_filt[0];
    pub_force_lpf_->publish(lpf);
  }

  // ==================================================
  // Main update
  // ==================================================
  void update()
  {
    if (!sp_received_) {
      return;
    }

    const rclcpp::Time now = this->now();
    double dt = 0.0;
    if (last_time_.nanoseconds() != 0) {
      dt = (now - last_time_).seconds();
    }
    last_time_ = now;

    if (dt <= 1e-5 || dt > 0.1) {
      publishOut();
      return;
    }

    float use_vel_mode_local = 0.0f;
    {
      std::lock_guard<std::mutex> lk(force_mtx_);
      use_vel_mode_local = su_cmd_use_vel_mode_;
    }
    const bool vel_mode_on = (use_vel_mode_local > 0.5f);

    ReferenceState ref;
    {
      std::lock_guard<std::mutex> lk(state_mtx_);
      ref = getReferenceStateLocked();
    }

    if (!su_int_initialized_) {
      if (!vel_mode_on) {
        su_int_ref_pos_ = sp_in_;
        su_int_ref_yaw_ = sp_in_yaw_;
      } else {
        if (ref.valid) {
          su_int_ref_pos_[0] = ref.pos_w.x();
          su_int_ref_pos_[1] = ref.pos_w.y();
          su_int_ref_pos_[2] = ref.pos_w.z();
          su_int_ref_yaw_ = ref.yaw_w;
        } else {
          su_int_ref_pos_ = {0.0, 0.0, 0.0};
          su_int_ref_yaw_ = 0.0;
        }
      }

      su_int_initialized_ = true;
      su_vel_mode_prev_ = vel_mode_on;
      su_pos_base_valid_ = false;
    }

    if (vel_mode_on && !su_vel_mode_prev_) {
      if (ref.valid) {
        su_int_ref_pos_[0] = ref.pos_w.x();
        su_int_ref_pos_[1] = ref.pos_w.y();
        su_int_ref_pos_[2] = ref.pos_w.z();
        su_int_ref_yaw_ = ref.yaw_w;
      }
      su_pos_base_valid_ = false;
    }
    else if (!vel_mode_on && su_vel_mode_prev_) {
      su_pos_base_ = su_int_ref_pos_;
      su_yaw_base_ = su_int_ref_yaw_;
      su_pos_base_valid_ = true;
    }
    su_vel_mode_prev_ = vel_mode_on;

    if (!vel_mode_on) {
      geometry_msgs::msg::Vector3Stamped nan_msg;
      nan_msg.header.stamp = now;
      nan_msg.header.frame_id = "contact";
      nan_msg.vector.x = std::numeric_limits<double>::quiet_NaN();
      nan_msg.vector.y = std::numeric_limits<double>::quiet_NaN();
      nan_msg.vector.z = std::numeric_limits<double>::quiet_NaN();
      pub_contact_vel_cmd_->publish(nan_msg);
      pub_contact_vel_actual_->publish(nan_msg);

      if (su_pos_base_valid_) {
        su_int_ref_pos_[0] = su_pos_base_[0] + sp_in_[0];
        su_int_ref_pos_[1] = su_pos_base_[1] + sp_in_[1];
        su_int_ref_pos_[2] = su_pos_base_[2] + sp_in_[2];
        su_int_ref_yaw_    = su_yaw_base_    + sp_in_yaw_;
      } else {
        su_int_ref_pos_ = sp_in_;
        su_int_ref_yaw_ = sp_in_yaw_;
      }
    }
    else {
      double vx_cmd   = sp_in_[0];
      double vy_cmd   = sp_in_[1];
      double vz_cmd   = sp_in_[2];
      double vyaw_cmd = sp_in_yaw_;

      std::array<double,3> eF{0.0, 0.0, 0.0};
      std::array<double,3> eF_dot_filt{0.0, 0.0, 0.0};
      {
        std::lock_guard<std::mutex> lk(force_mtx_);
        eF = eF_;
        eF_dot_filt = eF_dot_filt_;
      }

      const double vel_adm_x = force_adm_kp_ * eF[0] + force_adm_kd_ * eF_dot_filt[0];
      vx_cmd += vel_adm_x;

      Eigen::Vector3d v_c(vx_cmd, vy_cmd, vz_cmd);
      Eigen::Vector3d v_w = v_c;

      ContactFrame cf_local = cf_;
      const bool cf_ok = updateContactFrame(cf_local, ref);
      if (cf_ok && cf_local.valid) {
        v_w = cf_local.R_C * v_c;
        cf_ = cf_local;
        publishContactQuat(cf_local.R_C, now);

        Eigen::Vector3d v_act_w = ref.valid ? ref.vel_w : Eigen::Vector3d::Zero();
        Eigen::Vector3d v_act_c = cf_local.R_C.transpose() * v_act_w;
        publishContactVelocityDebug(v_c, v_act_c, now);
      } else {
        geometry_msgs::msg::Vector3Stamped nan_msg;
        nan_msg.header.stamp = now;
        nan_msg.header.frame_id = "contact";
        nan_msg.vector.x = std::numeric_limits<double>::quiet_NaN();
        nan_msg.vector.y = std::numeric_limits<double>::quiet_NaN();
        nan_msg.vector.z = std::numeric_limits<double>::quiet_NaN();
        pub_contact_vel_cmd_->publish(nan_msg);
        pub_contact_vel_actual_->publish(nan_msg);
      }

      su_int_ref_pos_[0] += v_w.x() * dt;
      su_int_ref_pos_[1] += v_w.y() * dt;
      su_int_ref_pos_[2] += v_w.z() * dt;

      double yaw_next = su_int_ref_yaw_;
      if (cf_ok && cf_local.valid) {
        const Eigen::Vector3d n_w = cf_local.n_w;
        const double nxy = std::hypot(n_w.x(), n_w.y());

        if (nxy > normal_xy_min_) {
          const double psi_align = std::atan2(n_w.y(), n_w.x());
          const double e_yaw = wrap_pi(psi_align - su_int_ref_yaw_);
          const double psi_dot = yaw_align_kp_ * e_yaw;
          yaw_next = su_int_ref_yaw_ + psi_dot * dt;
        } else {
          yaw_next = su_int_ref_yaw_ + vyaw_cmd * dt;
        }
      } else {
        yaw_next = su_int_ref_yaw_ + vyaw_cmd * dt;
      }
      su_int_ref_yaw_ = yaw_next;
    }

    publishOut();
  }

  // ==================================================
  // ROS interfaces
  // ==================================================
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr sub_keyboard_;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr sub_use_vel_mode_;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr sub_cmd_force_;
  rclcpp::Subscription<geometry_msgs::msg::WrenchStamped>::SharedPtr sub_contact_force_;

  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr sub_pose_;
  rclcpp::Subscription<geometry_msgs::msg::Vector3Stamped>::SharedPtr sub_vel_;
  rclcpp::Subscription<geometry_msgs::msg::Vector3Stamped>::SharedPtr sub_acc_;

  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr sub_ee_pose_;
  rclcpp::Subscription<geometry_msgs::msg::Vector3Stamped>::SharedPtr sub_ee_vel_;
  rclcpp::Subscription<geometry_msgs::msg::Vector3Stamped>::SharedPtr sub_ee_acc_;

  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr pub_pos_cmd_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr pub_force_lpf_;
  rclcpp::Publisher<geometry_msgs::msg::QuaternionStamped>::SharedPtr pub_contact_quat_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr pub_contact_force_x_;
  rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr pub_contact_vel_cmd_;
  rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr pub_contact_vel_actual_;

  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pub_cmd_drone_pose_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pub_cmd_ee_pose_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pub_cmd_active_pose_;

  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::TimerBase::SharedPtr force_timer_;

  // ==================================================
  // Parameters / config
  // ==================================================
  std::string reference_object_;
  std::string normal_estimator_method_;

  bool flip_measured_force_{false};
  double yaw_align_kp_{3.0};
  double normal_xy_min_{1e-3};
  double normal_force_threshold_{0.005};
  double normal_lpf_alpha_{0.2};

  double force_adm_kp_{5.0};
  double force_adm_kd_{0.005};

  Eigen::Vector3d d_B_{0.0, 0.0, 0.0};

  std::string pose_topic_;
  std::string vel_topic_;
  std::string acc_topic_;
  std::string ee_pose_topic_;
  std::string ee_vel_topic_;
  std::string ee_acc_topic_;
  std::string contact_force_topic_;

  // ==================================================
  // Internal state
  // ==================================================
  ContactFrame cf_;

  std::array<double, 3> sp_in_{0.0, 0.0, 0.0};
  double sp_in_yaw_{0.0};
  bool sp_received_{false};

  std::array<double, 3> su_int_ref_pos_{0.0, 0.0, 0.0};
  double su_int_ref_yaw_{0.0};
  bool su_int_initialized_{false};

  bool su_vel_mode_prev_{false};
  std::array<double, 3> su_pos_base_{0.0, 0.0, 0.0};
  double su_yaw_base_{0.0};
  bool su_pos_base_valid_{false};

  std::mutex force_mtx_;
  float su_cmd_use_vel_mode_{0.0f};
  float su_cmd_fx_{0.0f};

  std::array<double,3> eF_{0.0, 0.0, 0.0};
  std::array<double,3> eF_dot_filt_{0.0, 0.0, 0.0};
  std::array<double,3> eF_prev_{0.0, 0.0, 0.0};
  std::array<double,3> eF_dot_filt_prev_{0.0, 0.0, 0.0};

  std::array<double,3> contact_F_{0.0, 0.0, 0.0};
  bool eF_state_initialized_{false};
  bool f_ext_received_{false};

  rclcpp::Time eF_last_time_;
  rclcpp::Time last_time_;

  std::mutex state_mtx_;

  std::array<double,3> pose_w_{0.0, 0.0, 0.0};
  std::array<double,3> vel_w_{0.0, 0.0, 0.0};
  std::array<double,3> acc_w_{0.0, 0.0, 0.0};
  double yaw_w_{0.0};
  Eigen::Quaterniond q_WB_{1.0, 0.0, 0.0, 0.0};

  std::array<double,3> ee_pose_w_{0.0, 0.0, 0.0};
  std::array<double,3> ee_vel_w_{0.0, 0.0, 0.0};
  std::array<double,3> ee_acc_w_{0.0, 0.0, 0.0};

  bool pose_received_{false};
  bool vel_received_{false};
  bool acc_received_{false};

  bool ee_pose_received_{false};
  bool ee_vel_received_{false};
  bool ee_acc_received_{false};
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TrajectoryGeneration>());
  rclcpp::shutdown();
  return 0;
}
