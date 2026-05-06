#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/quaternion_stamped.hpp>
#include <geometry_msgs/msg/vector3_stamped.hpp>
#include <geometry_msgs/msg/wrench_stamped.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>

#include <Eigen/Dense>

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <mutex>
#include <string>
#include <vector>

using namespace std::chrono_literals;

class NormalVectorEstimation : public rclcpp::Node
{
public:
  NormalVectorEstimation()
  : Node("normal_vector_estimation")
  {
    reference_object_ = this->declare_parameter<std::string>(
      "reference_object", "drone");

    normal_estimator_method_ = this->declare_parameter<std::string>(
      "normal_estimator_method", "direction");

    flip_measured_force_ = this->declare_parameter<bool>(
      "flip_measured_force", false);

    normal_force_threshold_ = this->declare_parameter<double>(
      "normal_force_threshold", 0.005);

    normal_lpf_alpha_ = this->declare_parameter<double>(
      "normal_lpf_alpha", 0.2);

    force_observation_source_ = this->declare_parameter<std::string>(
      "force_observation_source", "contact_force_filt");

    use_vel_mode_topic_ = this->declare_parameter<std::string>(
      "use_vel_mode_topic", "su/use_vel_mode");

    force_based_velocity_epsilon_ = this->declare_parameter<double>(
      "normal_force_based.velocity_epsilon", 0.005);
    force_based_force_epsilon_ = this->declare_parameter<double>(
      "normal_force_based.force_epsilon", 0.01);
    force_based_algebraic_force_epsilon_ = this->declare_parameter<double>(
      "normal_force_based.algebraic_force_epsilon", 5.0e-3);
    const double declared_gamma_epsilon = this->declare_parameter<double>(
      "normal_force_based.gamma_epsilon",
      std::numeric_limits<double>::quiet_NaN());
    const double declared_compare_gamma_epsilon = this->declare_parameter<double>(
      "normal_force_based.compare_gamma_epsilon",
      std::numeric_limits<double>::quiet_NaN());
    if (std::isfinite(declared_gamma_epsilon)) {
      force_based_gamma_epsilon_ = declared_gamma_epsilon;
    } else if (std::isfinite(declared_compare_gamma_epsilon)) {
      force_based_gamma_epsilon_ = declared_compare_gamma_epsilon;
    } else {
      force_based_gamma_epsilon_ = 1.0e-2;
    }
    force_based_candidate_lpf_cutoff_hz_ = this->declare_parameter<double>(
      "normal_force_based.candidate_lpf_cutoff_hz", 63.66197723675813);
    force_based_output_lpf_cutoff_rad_s_ = this->declare_parameter<double>(
      "normal_force_based.output_lpf_cutoff_rad_s", 3.0);
    force_based_beta_n_ = this->declare_parameter<double>(
      "normal_force_based.beta_n", 1.0);
    force_based_sigma_n_ = this->declare_parameter<double>(
      "normal_force_based.sigma_n", 1.0);

    publish_hz_ = this->declare_parameter<double>(
      "publish_hz", 100.0);

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

    contact_force_topic_ = resolveForceObservationTopic(force_observation_source_);

    contact_frame_quat_topic_ = this->declare_parameter<std::string>(
      "contact_frame_quat_topic", "/estimated_contact_frame_quat");
    contact_force_x_topic_ = this->declare_parameter<std::string>(
      "contact_force_x_topic", "/su/contact_force_x");
    normal_debug_metrics_topic_ = this->declare_parameter<std::string>(
      "normal_debug_metrics_topic", "/normal_vector/debug_metrics");

    sub_contact_force_ = this->create_subscription<geometry_msgs::msg::WrenchStamped>(
      contact_force_topic_, 10,
      std::bind(&NormalVectorEstimation::contactForceCb, this, std::placeholders::_1));

    sub_use_vel_mode_ = this->create_subscription<std_msgs::msg::Float32>(
      use_vel_mode_topic_, 10,
      std::bind(&NormalVectorEstimation::useVelModeCb, this, std::placeholders::_1));

    sub_pose_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
      pose_topic_, 10,
      std::bind(&NormalVectorEstimation::poseCb, this, std::placeholders::_1));

    sub_vel_ = this->create_subscription<geometry_msgs::msg::Vector3Stamped>(
      vel_topic_, 10,
      std::bind(&NormalVectorEstimation::velCb, this, std::placeholders::_1));

    sub_acc_ = this->create_subscription<geometry_msgs::msg::Vector3Stamped>(
      acc_topic_, 10,
      std::bind(&NormalVectorEstimation::accCb, this, std::placeholders::_1));

    sub_ee_pose_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
      ee_pose_topic_, 10,
      std::bind(&NormalVectorEstimation::eePoseCb, this, std::placeholders::_1));

    sub_ee_vel_ = this->create_subscription<geometry_msgs::msg::Vector3Stamped>(
      ee_vel_topic_, 10,
      std::bind(&NormalVectorEstimation::eeVelCb, this, std::placeholders::_1));

    sub_ee_acc_ = this->create_subscription<geometry_msgs::msg::Vector3Stamped>(
      ee_acc_topic_, 10,
      std::bind(&NormalVectorEstimation::eeAccCb, this, std::placeholders::_1));

    pub_contact_quat_ = this->create_publisher<geometry_msgs::msg::QuaternionStamped>(
      contact_frame_quat_topic_, 10);
    pub_contact_force_x_ = this->create_publisher<std_msgs::msg::Float32>(
      contact_force_x_topic_, 10);
    pub_normal_debug_metrics_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(
      normal_debug_metrics_topic_, 10);

    const double safe_hz = std::max(1.0, publish_hz_);
    timer_ = this->create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::duration<double>(1.0 / safe_hz)),
      std::bind(&NormalVectorEstimation::update, this));

    RCLCPP_INFO(this->get_logger(), "normal_vector_estimation started");
    RCLCPP_INFO(this->get_logger(), "reference_object = %s", reference_object_.c_str());
    RCLCPP_INFO(this->get_logger(), "normal_estimator_method = %s", normal_estimator_method_.c_str());
    RCLCPP_INFO(this->get_logger(), "force_observation_source = %s", force_observation_source_.c_str());
    RCLCPP_INFO(this->get_logger(), "force_observation_topic = %s", contact_force_topic_.c_str());
  }

private:
  struct ReferenceState
  {
    Eigen::Vector3d pos_w = Eigen::Vector3d::Zero();
    Eigen::Vector3d vel_w = Eigen::Vector3d::Zero();
    Eigen::Vector3d acc_w = Eigen::Vector3d::Zero();
    double yaw_w = 0.0;
    bool valid = false;
  };

  struct ContactFrame
  {
    Eigen::Vector3d n_w = Eigen::Vector3d::UnitX();
    Eigen::Matrix3d R_C = Eigen::Matrix3d::Identity();
    bool valid = false;
  };

  struct ForceBasedNormalState
  {
    Eigen::Matrix3d l_n = Eigen::Matrix3d::Zero();
    Eigen::Vector3d w_s = Eigen::Vector3d::Zero();
    Eigen::Vector3d n_f = Eigen::Vector3d::Zero();
    Eigen::Vector3d f_g = Eigen::Vector3d::Zero();
    Eigen::Vector3d f_g_soft = Eigen::Vector3d::Zero();
    Eigen::Vector3d n_alg = Eigen::Vector3d::Zero();
    Eigen::Vector3d n_alg_soft = Eigen::Vector3d::Zero();
    Eigen::Vector3d n_geo = Eigen::Vector3d::Zero();
    Eigen::Vector3d n_est = Eigen::Vector3d::Zero();
    double vel_norm = 0.0;
    double force_norm = 0.0;
    double corrected_force_norm = 0.0;
    bool initialized = false;
  };

  struct EigenSpectrum
  {
    Eigen::Vector3d values = Eigen::Vector3d::Zero();
    Eigen::Matrix3d vectors = Eigen::Matrix3d::Identity();
  };

  static double clamp01(double value)
  {
    return std::clamp(value, 0.0, 1.0);
  }

  static double lpfAlphaFromCutoff(double dt, double cutoff_hz)
  {
    if (!(std::isfinite(dt) && dt > 0.0 && std::isfinite(cutoff_hz) && cutoff_hz > 0.0)) {
      return 1.0;
    }
    const double tau = 1.0 / (2.0 * M_PI * cutoff_hz);
    return std::clamp(dt / (tau + dt), 0.0, 1.0);
  }

  static double lpfAlphaFromCutoffRadPerSec(double dt, double cutoff_rad_s)
  {
    if (!(std::isfinite(dt) && dt > 0.0 && std::isfinite(cutoff_rad_s) && cutoff_rad_s > 0.0)) {
      return 1.0;
    }
    const double tau = 1.0 / cutoff_rad_s;
    return std::clamp(dt / (tau + dt), 0.0, 1.0);
  }

  static EigenSpectrum eigenDecomposeDescending(const Eigen::Matrix3d & mat)
  {
    EigenSpectrum spectrum;
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> solver(0.5 * (mat + mat.transpose()));
    if (solver.info() != Eigen::Success) {
      return spectrum;
    }

    const auto values_asc = solver.eigenvalues();
    const auto vectors_asc = solver.eigenvectors();
    for (int i = 0; i < 3; ++i) {
      spectrum.values(i) = values_asc(2 - i);
      spectrum.vectors.col(i) = vectors_asc.col(2 - i);
    }
    return spectrum;
  }

  static Eigen::Vector3d lowPassNormalizedDirection(
    const Eigen::Vector3d & prev,
    const Eigen::Vector3d & current,
    double alpha)
  {
    if (current.norm() < 1e-12) {
      return Eigen::Vector3d::Zero();
    }

    if (prev.norm() < 1e-12) {
      return current.normalized();
    }

    Eigen::Vector3d aligned_current = current;
    if (prev.dot(aligned_current) < 0.0) {
      aligned_current = -aligned_current;
    }

    Eigen::Vector3d filtered = prev + clamp01(alpha) * (aligned_current - prev);
    const double filtered_norm = filtered.norm();
    if (filtered_norm < 1e-12) {
      return aligned_current.normalized();
    }
    return filtered / filtered_norm;
  }

  static Eigen::Vector3d normalizedOrFallback(
    const Eigen::Vector3d & vec,
    const Eigen::Vector3d & fallback,
    double epsilon)
  {
    const double vec_norm = vec.norm();
    if (std::isfinite(vec_norm) && vec_norm > epsilon) {
      return vec / (vec_norm + 1e-12);
    }
    const double fallback_norm = fallback.norm();
    if (std::isfinite(fallback_norm) && fallback_norm > epsilon) {
      return fallback / (fallback_norm + 1e-12);
    }
    return Eigen::Vector3d::Zero();
  }

  std::string resolveForceObservationTopic(const std::string & source) const
  {
    if (
      source == "contact_force" || source == "contact_force_filt" ||
      source == "filtered" || source == "filt" || source == "raw")
    {
      return "/crazyflie/out/EE_contact_force_filt";
    }
    if (source == "mob_2nd" || source == "mob2") {
      return "/crazyflie/out/ee_applied_mob_2nd";
    }
    if (source == "mob_2nd_tau" || source == "mob_tau" || source == "consistency") {
      return "/crazyflie/out/ee_applied_mob_2nd_tau";
    }

    RCLCPP_WARN(
      this->get_logger(),
      "Unknown force_observation_source '%s'. Falling back to '/crazyflie/out/EE_contact_force_filt'.",
      source.c_str());
    return "/crazyflie/out/EE_contact_force_filt";
  }

  ReferenceState getReferenceStateLocked() const
  {
    ReferenceState ref;
    const bool use_ee = (reference_object_ == "end_effector");

    if (!use_ee) {
      if (!(pose_received_ && vel_received_)) {
        return ref;
      }

      ref.pos_w = Eigen::Vector3d(pose_w_[0], pose_w_[1], pose_w_[2]);
      ref.vel_w = Eigen::Vector3d(vel_w_[0], vel_w_[1], vel_w_[2]);
      if (acc_received_) {
        ref.acc_w = Eigen::Vector3d(acc_w_[0], acc_w_[1], acc_w_[2]);
      }
      ref.yaw_w = yaw_w_;
      ref.valid = true;
      return ref;
    }

    if (!(ee_pose_received_ && ee_vel_received_ && pose_received_)) {
      return ref;
    }

    ref.pos_w = Eigen::Vector3d(ee_pose_w_[0], ee_pose_w_[1], ee_pose_w_[2]);
    ref.vel_w = Eigen::Vector3d(ee_vel_w_[0], ee_vel_w_[1], ee_vel_w_[2]);
    if (ee_acc_received_) {
      ref.acc_w = Eigen::Vector3d(ee_acc_w_[0], ee_acc_w_[1], ee_acc_w_[2]);
    }
    ref.yaw_w = yaw_w_;
    ref.valid = true;
    return ref;
  }

  bool estimateNormalVectorForceDirectly(
    ContactFrame & cf_out,
    const ReferenceState & /*ref*/,
    const Eigen::Vector3d & force_world)
  {
    const double force_norm = force_world.norm();
    if (force_norm < normal_force_threshold_) {
      cf_out.valid = false;
      return false;
    }

    Eigen::Vector3d n_new = force_world / (force_norm + 1e-12);
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
      cf_out.n_w = (nn > 1e-9) ? (cf_out.n_w / nn) : n_new;
    }

    return true;
  }

  bool estimateNormalVectorForceBased(
    ContactFrame & cf_out,
    const ReferenceState & ref,
    Eigen::Vector3d force_world,
    double dt)
  {
    if (flip_measured_force_) {
      force_world = -force_world;
    }

    dt = std::clamp(dt, 1e-4, 0.1);

    const double force_norm = force_world.norm();
    if (force_norm < normal_force_threshold_) {
      if (force_based_state_.initialized && force_based_state_.n_est.norm() > 1e-12) {
        cf_out.n_w = force_based_state_.n_est;
        return true;
      }
      cf_out.valid = false;
      return false;
    }

    const Eigen::Vector3d v_c_world = ref.vel_w;
    const double vel_norm = v_c_world.norm();
    Eigen::Vector3d w_s = Eigen::Vector3d::Zero();
    if (vel_norm > force_based_velocity_epsilon_) {
      w_s = v_c_world / (vel_norm + 1e-12);
    }

    Eigen::Vector3d n_f = Eigen::Vector3d::Zero();
    if (force_norm > force_based_force_epsilon_) {
      n_f = force_world / (force_norm + 1e-12);
    }

    Eigen::Vector3d f_g = force_world;
    if (w_s.squaredNorm() > 1e-12) {
      const Eigen::Matrix3d projector =
        Eigen::Matrix3d::Identity() - (w_s * w_s.transpose());
      f_g = projector * force_world;
    }
    const double corrected_force_norm = f_g.norm();

    Eigen::Vector3d f_g_soft = force_world;
    double gamma_v = 0.0;
    if (w_s.squaredNorm() > 1e-12) {
      const double vel_norm_sq = vel_norm * vel_norm;
      gamma_v = vel_norm_sq / (vel_norm_sq + force_based_gamma_epsilon_ + 1e-12);
      const Eigen::Matrix3d soft_projector =
        Eigen::Matrix3d::Identity() - gamma_v * (w_s * w_s.transpose());
      f_g_soft = soft_projector * force_world;
    }
    if (n_f.squaredNorm() <= 1e-12) {
      if (force_based_state_.initialized && force_based_state_.n_est.norm() > 1e-12) {
        cf_out.n_w = force_based_state_.n_est;
        return true;
      }
      cf_out.valid = false;
      return false;
    }

    Eigen::Vector3d n_alg = n_f;
    if (vel_norm > force_based_velocity_epsilon_) {
      n_alg = normalizedOrFallback(f_g, n_f, force_based_algebraic_force_epsilon_);
    }

    Eigen::Vector3d n_alg_soft = normalizedOrFallback(f_g_soft, n_f, force_based_algebraic_force_epsilon_);

    if (n_alg.squaredNorm() > 1e-12 && n_alg_soft.squaredNorm() > 1e-12 && n_alg.dot(n_alg_soft) < 0.0) {
      n_alg_soft = -n_alg_soft;
    } else if (n_f.squaredNorm() > 1e-12 && n_alg_soft.squaredNorm() > 1e-12 && n_f.dot(n_alg_soft) < 0.0) {
      n_alg_soft = -n_alg_soft;
    }

    if (cf_out.valid && cf_out.n_w.dot(n_alg) < 0.0) {
      n_alg = -n_alg;
    } else if (n_f.squaredNorm() > 1e-12 && n_f.dot(n_alg) < 0.0) {
      n_alg = -n_alg;
    }

    const double output_lpf_alpha =
      lpfAlphaFromCutoffRadPerSec(dt, force_based_output_lpf_cutoff_rad_s_);

    n_f = lowPassNormalizedDirection(
      force_based_state_.n_f,
      n_f,
      output_lpf_alpha);
    n_alg = lowPassNormalizedDirection(
      force_based_state_.n_alg,
      n_alg,
      output_lpf_alpha);
    n_alg_soft = lowPassNormalizedDirection(
      force_based_state_.n_alg_soft,
      n_alg_soft,
      output_lpf_alpha);

    const Eigen::Vector3d n_alg_memory = n_alg;

    Eigen::Matrix3d l_n_dot = -force_based_beta_n_ * force_based_state_.l_n;
    if (n_alg_memory.squaredNorm() > 1e-12) {
      l_n_dot += force_based_sigma_n_ * (n_alg_memory * n_alg_memory.transpose());
    }
    force_based_state_.l_n += dt * l_n_dot;

    const EigenSpectrum spec_n = eigenDecomposeDescending(force_based_state_.l_n);
    Eigen::Vector3d n_geo = spec_n.vectors.col(0);
    if (n_geo.norm() < 1e-9) {
      n_geo = n_alg_memory;
    }
    n_geo.normalize();

    if (n_alg_memory.squaredNorm() > 1e-12 && n_geo.dot(n_alg_memory) < 0.0) {
      n_geo = -n_geo;
    } else if (n_f.squaredNorm() > 1e-12 && n_geo.dot(n_f) < 0.0) {
      n_geo = -n_geo;
    } else if (cf_out.valid && n_geo.dot(cf_out.n_w) < 0.0) {
      n_geo = -n_geo;
    } else if (n_geo.dot(force_world) < 0.0) {
      n_geo = -n_geo;
    }

    force_based_state_.w_s = w_s;
    force_based_state_.n_f = n_f;
    force_based_state_.n_alg = n_alg;
    force_based_state_.f_g = f_g;
    force_based_state_.f_g_soft = f_g_soft;
    force_based_state_.n_geo = n_geo;
    force_based_state_.n_alg_soft = n_alg_soft;
    force_based_state_.vel_norm = vel_norm;
    force_based_state_.force_norm = force_norm;
    force_based_state_.corrected_force_norm = corrected_force_norm;
    force_based_state_.n_est = n_alg;
    force_based_state_.initialized = true;

    // Use the instantaneous epsilon-gated projection result for the published
    // contact frame; raw/projected/gamma variants are logged separately.
    cf_out.n_w = n_alg;
    return true;
  }

  bool buildContactFrameFromNormal(ContactFrame & cf_out)
  {
    const double n_norm = cf_out.n_w.norm();
    if (n_norm < 1e-9) {
      cf_out.valid = false;
      return false;
    }

    const Eigen::Vector3d x_c = cf_out.n_w.normalized();
    Eigen::Vector3d y_c = x_c.cross(Eigen::Vector3d::UnitZ());
    double y_norm = y_c.norm();

    if (y_norm < 1e-6) {
      Eigen::Vector3d axis = Eigen::Vector3d::UnitY();
      if (std::abs(x_c.dot(axis)) > 0.9) {
        axis = Eigen::Vector3d::UnitX();
      }

      y_c = x_c.cross(axis);
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

  bool updateContactFrame(
    ContactFrame & cf_out,
    const ReferenceState & ref,
    const Eigen::Vector3d & force_world,
    double dt)
  {
    if (
      normal_estimator_method_ == "None" ||
      normal_estimator_method_ == "none" ||
      normal_estimator_method_ == "NONE")
    {
      cf_out.valid = false;
      return false;
    }

    bool ok = false;
    if (
      normal_estimator_method_ == "normal_force_based" ||
      normal_estimator_method_ == "force_based")
    {
      ok = estimateNormalVectorForceBased(cf_out, ref, force_world, dt);
    } else if (
      normal_estimator_method_ == "direction" ||
      normal_estimator_method_ == "direct")
    {
      ok = estimateNormalVectorForceDirectly(cf_out, ref, force_world);
    } else {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 5000,
        "Unsupported normal_estimator_method '%s'. "
        "Supported methods are: direction, normal_force_based, None. "
        "Falling back to direction.",
        normal_estimator_method_.c_str());
      ok = estimateNormalVectorForceDirectly(cf_out, ref, force_world);
    }

    if (!ok) {
      cf_out.valid = false;
      return false;
    }

    return buildContactFrameFromNormal(cf_out);
  }

  void publishContactQuat(const Eigen::Matrix3d & contact_rotation, const rclcpp::Time & stamp)
  {
    Eigen::Quaterniond q(contact_rotation);
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

  void publishNormalDebugMetrics()
  {
    std_msgs::msg::Float64MultiArray msg;
    const auto nan = std::numeric_limits<double>::quiet_NaN();

    msg.data.resize(53, nan);
    if (!force_based_state_.initialized) {
      pub_normal_debug_metrics_->publish(msg);
      return;
    }
    msg.data[0] = force_based_state_.vel_norm;
    msg.data[1] = force_based_state_.force_norm;
    msg.data[2] = force_based_state_.corrected_force_norm;
    for (int r = 0; r < 3; ++r) {
      for (int c = 0; c < 3; ++c) {
        msg.data[10 + r * 3 + c] = force_based_state_.l_n(r, c);
      }
    }
    msg.data[28] = force_based_state_.n_geo.x();
    msg.data[29] = force_based_state_.n_geo.y();
    msg.data[30] = force_based_state_.n_geo.z();
    msg.data[31] = force_based_state_.n_f.x();
    msg.data[32] = force_based_state_.n_f.y();
    msg.data[33] = force_based_state_.n_f.z();
    msg.data[34] = force_based_state_.n_alg.x();
    msg.data[35] = force_based_state_.n_alg.y();
    msg.data[36] = force_based_state_.n_alg.z();
    msg.data[37] = force_based_state_.w_s.x();
    msg.data[38] = force_based_state_.w_s.y();
    msg.data[39] = force_based_state_.w_s.z();
    msg.data[40] = force_based_state_.f_g.x();
    msg.data[41] = force_based_state_.f_g.y();
    msg.data[42] = force_based_state_.f_g.z();
    msg.data[43] = force_based_state_.n_alg_soft.x();
    msg.data[44] = force_based_state_.n_alg_soft.y();
    msg.data[45] = force_based_state_.n_alg_soft.z();
    msg.data[46] = force_based_state_.f_g_soft.x();
    msg.data[47] = force_based_state_.f_g_soft.y();
    msg.data[48] = force_based_state_.f_g_soft.z();
    pub_normal_debug_metrics_->publish(msg);
  }

  void clearNormalDebugMetrics()
  {
    force_based_state_.w_s = Eigen::Vector3d::Zero();
    force_based_state_.n_f = Eigen::Vector3d::Zero();
    force_based_state_.f_g = Eigen::Vector3d::Zero();
    force_based_state_.f_g_soft = Eigen::Vector3d::Zero();
    force_based_state_.n_alg = Eigen::Vector3d::Zero();
    force_based_state_.n_alg_soft = Eigen::Vector3d::Zero();
    force_based_state_.n_geo = Eigen::Vector3d::Zero();
    force_based_state_.n_est = Eigen::Vector3d::Zero();
    force_based_state_.vel_norm = 0.0;
    force_based_state_.force_norm = 0.0;
    force_based_state_.corrected_force_norm = 0.0;
    force_based_state_.initialized = false;
  }

  void contactForceCb(const geometry_msgs::msg::WrenchStamped::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lk(force_mtx_);
    contact_force_[0] = msg->wrench.force.x;
    contact_force_[1] = msg->wrench.force.y;
    contact_force_[2] = msg->wrench.force.z;
    force_received_ = true;
  }

  void useVelModeCb(const std_msgs::msg::Float32::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lk(mode_mtx_);
    use_vel_mode_ = (msg->data > 0.5f);
  }

  void poseCb(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lk(state_mtx_);
    pose_w_[0] = msg->pose.position.x;
    pose_w_[1] = msg->pose.position.y;
    pose_w_[2] = msg->pose.position.z;

    const auto & q = msg->pose.orientation;
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

  void update()
  {
    {
      std::lock_guard<std::mutex> lk(mode_mtx_);
      if (!use_vel_mode_) {
        std_msgs::msg::Float32 force_x_msg;
        force_x_msg.data = 0.0f;
        pub_contact_force_x_->publish(force_x_msg);
        clearNormalDebugMetrics();
        publishNormalDebugMetrics();
        return;
      }
    }

    std::array<double, 3> force_arr{0.0, 0.0, 0.0};
    const rclcpp::Time stamp = this->now();
    double dt = 1.0 / std::max(1.0, publish_hz_);
    if (last_update_time_.nanoseconds() > 0) {
      const double dt_meas = (stamp - last_update_time_).seconds();
      if (std::isfinite(dt_meas) && dt_meas > 1e-4 && dt_meas < 0.2) {
        dt = dt_meas;
      }
    }
    last_update_time_ = stamp;

    {
      std::lock_guard<std::mutex> lk(force_mtx_);
      if (!force_received_) {
        return;
      }
      force_arr = contact_force_;
    }

    const Eigen::Vector3d force_world(force_arr[0], force_arr[1], force_arr[2]);

    ReferenceState ref;
    {
      std::lock_guard<std::mutex> lk(state_mtx_);
      ref = getReferenceStateLocked();
    }

    ContactFrame cf_local;
    {
      std::lock_guard<std::mutex> lk(contact_frame_mtx_);
      cf_local = contact_frame_;
    }

    if (!updateContactFrame(cf_local, ref, force_world, dt) || !cf_local.valid) {
      std_msgs::msg::Float32 force_x_msg;
      force_x_msg.data = 0.0f;
      pub_contact_force_x_->publish(force_x_msg);
      clearNormalDebugMetrics();
      publishNormalDebugMetrics();
      return;
    }

    const Eigen::Vector3d force_contact = cf_local.R_C.transpose() * force_world;

    {
      std::lock_guard<std::mutex> lk(contact_frame_mtx_);
      contact_frame_ = cf_local;
    }

    force_based_state_.n_est = cf_local.n_w;

    publishContactQuat(cf_local.R_C, stamp);
    publishNormalDebugMetrics();

    std_msgs::msg::Float32 force_x_msg;
    force_x_msg.data = static_cast<float>(force_contact.x());
    pub_contact_force_x_->publish(force_x_msg);
  }

  rclcpp::Subscription<geometry_msgs::msg::WrenchStamped>::SharedPtr sub_contact_force_;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr sub_use_vel_mode_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr sub_pose_;
  rclcpp::Subscription<geometry_msgs::msg::Vector3Stamped>::SharedPtr sub_vel_;
  rclcpp::Subscription<geometry_msgs::msg::Vector3Stamped>::SharedPtr sub_acc_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr sub_ee_pose_;
  rclcpp::Subscription<geometry_msgs::msg::Vector3Stamped>::SharedPtr sub_ee_vel_;
  rclcpp::Subscription<geometry_msgs::msg::Vector3Stamped>::SharedPtr sub_ee_acc_;

  rclcpp::Publisher<geometry_msgs::msg::QuaternionStamped>::SharedPtr pub_contact_quat_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr pub_contact_force_x_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr pub_normal_debug_metrics_;

  rclcpp::TimerBase::SharedPtr timer_;

  std::string reference_object_;
  std::string normal_estimator_method_;
  std::string force_observation_source_;
  bool flip_measured_force_{false};
  double normal_force_threshold_{0.005};
  double normal_lpf_alpha_{0.2};
  double publish_hz_{100.0};

  double force_based_velocity_epsilon_{0.005};
  double force_based_force_epsilon_{0.01};
  double force_based_algebraic_force_epsilon_{5.0e-3};
  double force_based_gamma_epsilon_{1.0e-2};
  double force_based_candidate_lpf_cutoff_hz_{63.66197723675813};
  double force_based_output_lpf_cutoff_rad_s_{3.0};
  double force_based_beta_n_{1.0};
  double force_based_sigma_n_{1.0};

  std::string pose_topic_;
  std::string vel_topic_;
  std::string acc_topic_;
  std::string ee_pose_topic_;
  std::string ee_vel_topic_;
  std::string ee_acc_topic_;
  std::string contact_force_topic_;
  std::string contact_frame_quat_topic_;
  std::string contact_force_x_topic_;
  std::string normal_debug_metrics_topic_;
  std::string use_vel_mode_topic_;

  std::mutex force_mtx_;
  std::array<double, 3> contact_force_{0.0, 0.0, 0.0};
  bool force_received_{false};

  std::mutex contact_frame_mtx_;
  ContactFrame contact_frame_;
  ForceBasedNormalState force_based_state_;

  std::mutex state_mtx_;
  std::array<double, 3> pose_w_{0.0, 0.0, 0.0};
  std::array<double, 3> vel_w_{0.0, 0.0, 0.0};
  std::array<double, 3> acc_w_{0.0, 0.0, 0.0};
  double yaw_w_{0.0};

  std::array<double, 3> ee_pose_w_{0.0, 0.0, 0.0};
  std::array<double, 3> ee_vel_w_{0.0, 0.0, 0.0};
  std::array<double, 3> ee_acc_w_{0.0, 0.0, 0.0};

  bool pose_received_{false};
  bool vel_received_{false};
  bool acc_received_{false};
  bool ee_pose_received_{false};
  bool ee_vel_received_{false};
  bool ee_acc_received_{false};

  std::mutex mode_mtx_;
  bool use_vel_mode_{false};

  rclcpp::Time last_update_time_{0, 0, RCL_ROS_TIME};
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<NormalVectorEstimation>());
  rclcpp::shutdown();
  return 0;
}
