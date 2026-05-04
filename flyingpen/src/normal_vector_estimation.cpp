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
      "normal_estimator_method", "direct");

    flip_measured_force_ = this->declare_parameter<bool>(
      "flip_measured_force", false);

    normal_force_threshold_ = this->declare_parameter<double>(
      "normal_force_threshold", 0.005);

    normal_lpf_alpha_ = this->declare_parameter<double>(
      "normal_lpf_alpha", 0.2);

    force_observation_source_ = this->declare_parameter<std::string>(
      "force_observation_source", "contact_force_filt");

    consistency_match_topic_ = this->declare_parameter<std::string>(
      "consistency_match_topic", "/crazyflie/out/mob_2nd_tau_consistency");

    use_vel_mode_topic_ = this->declare_parameter<std::string>(
      "use_vel_mode_topic", "su/use_vel_mode");

    new_normal_velocity_epsilon_ = this->declare_parameter<double>(
      "Lf_Lv_fusion.velocity_epsilon", 0.015);
    new_normal_beta_v_ = this->declare_parameter<double>(
      "Lf_Lv_fusion.beta_v", 4.0);
    new_normal_sigma_v_ = this->declare_parameter<double>(
      "Lf_Lv_fusion.sigma_v", 4.0);

    new_normal_slip_blend_velocity_ = this->declare_parameter<double>(
      "Lf_Lv_fusion.slip_blend_velocity", 0.04);
    new_normal_force_epsilon_ = this->declare_parameter<double>(
      "Lf_Lv_fusion.force_epsilon", 0.004);
    new_normal_beta_f_ = this->declare_parameter<double>(
      "Lf_Lv_fusion.beta_f", 8.0);
    new_normal_sigma_f_ = this->declare_parameter<double>(
      "Lf_Lv_fusion.sigma_f", 8.0);

    new_normal_c_v_ = this->declare_parameter<double>(
      "Lf_Lv_fusion.c_v", 0.08);
    new_normal_eps_lambda_ = this->declare_parameter<double>(
      "Lf_Lv_fusion.eps_lambda", 1e-4);
    new_normal_c_f_ = this->declare_parameter<double>(
      "Lf_Lv_fusion.c_f", 0.015);
    new_normal_eta_tau_ = this->declare_parameter<double>(
      "Lf_Lv_fusion.eta_tau", 0.4);
    new_normal_sigma_tau_ = this->declare_parameter<double>(
      "Lf_Lv_fusion.sigma_tau", 0.004);
    new_normal_use_raw_mob_tau_residual_ = this->declare_parameter<bool>(
      "Lf_Lv_fusion.use_raw_mob_tau_residual", false);
    raw_mob_wrench_topic_ = this->declare_parameter<std::string>(
      "raw_mob_wrench_topic", "/crazyflie/out/mob_2nd");

    new_normal_eps_alpha_ = this->declare_parameter<double>(
      "Lf_Lv_fusion.eps_alpha", 1e-6);
    new_normal_eps_l_ = this->declare_parameter<double>(
      "Lf_Lv_fusion.eps_l", 1e-6);
    new_normal_min_confidence_ = this->declare_parameter<double>(
      "Lf_Lv_fusion.min_confidence", 0.08);

    force_based_velocity_epsilon_ = this->declare_parameter<double>(
      "normal_force_based.velocity_epsilon", 0.005);
    force_based_gamma_epsilon_ = this->declare_parameter<double>(
      "normal_force_based.gamma_epsilon", 5.0e-4);
    force_based_force_epsilon_ = this->declare_parameter<double>(
      "normal_force_based.force_epsilon", 0.01);
    force_based_algebraic_force_epsilon_ = this->declare_parameter<double>(
      "normal_force_based.algebraic_force_epsilon", 5.0e-3);
    force_based_candidate_lpf_alpha_ = this->declare_parameter<double>(
      "normal_force_based.candidate_lpf_alpha", 0.2);
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

    sub_consistency_match_ = this->create_subscription<geometry_msgs::msg::WrenchStamped>(
      consistency_match_topic_, 10,
      std::bind(&NormalVectorEstimation::consistencyMatchCb, this, std::placeholders::_1));

    sub_raw_mob_wrench_ = this->create_subscription<geometry_msgs::msg::WrenchStamped>(
      raw_mob_wrench_topic_, 10,
      std::bind(&NormalVectorEstimation::rawMobWrenchCb, this, std::placeholders::_1));

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
    RCLCPP_INFO(
      this->get_logger(), "Lf_Lv_fusion.use_raw_mob_tau_residual = %s",
      new_normal_use_raw_mob_tau_residual_ ? "true" : "false");
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

  struct NewNormalState
  {
    Eigen::Matrix3d l_v = Eigen::Matrix3d::Zero();
    Eigen::Matrix3d l_f = Eigen::Matrix3d::Zero();
    Eigen::Matrix3d l_v_bar = Eigen::Matrix3d::Zero();
    Eigen::Matrix3d l_f_bar = Eigen::Matrix3d::Zero();
    Eigen::Vector3d n_f = Eigen::Vector3d::Zero();
    Eigen::Vector3d n_geo = Eigen::Vector3d::Zero();
    Eigen::Vector3d n_est = Eigen::Vector3d::Zero();
    double rho_v = 0.0;
    double rho_f = 0.0;
    double alpha_v = 0.0;
    double alpha_f = 0.0;
    double lambda1 = 0.0;
    double lambda2 = 0.0;
    double lambda3 = 0.0;
    double m_tau = 1.0;
    double e_tau_norm = 0.0;
    double vel_norm = 0.0;
    double force_norm = 0.0;
    double angle_n_geo_deg = 0.0;
    double angle_n_f_deg = 0.0;
    bool initialized = false;
  };

  struct ForceBasedNormalState
  {
    Eigen::Matrix3d l_n = Eigen::Matrix3d::Zero();
    Eigen::Vector3d w_s = Eigen::Vector3d::Zero();
    Eigen::Vector3d n_f = Eigen::Vector3d::Zero();
    Eigen::Vector3d f_g = Eigen::Vector3d::Zero();
    Eigen::Vector3d n_alg = Eigen::Vector3d::Zero();
    Eigen::Vector3d n_geo = Eigen::Vector3d::Zero();
    Eigen::Vector3d n_est = Eigen::Vector3d::Zero();
    double gamma_v = 0.0;
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

  static Eigen::Quaterniond quatMsgToEigen(const geometry_msgs::msg::Quaternion & q)
  {
    return Eigen::Quaterniond(q.w, q.x, q.y, q.z);
  }

  static double clamp01(double value)
  {
    return std::clamp(value, 0.0, 1.0);
  }

  static Eigen::Vector3d safeNormalized(const Eigen::Vector3d & vec, double eps)
  {
    const double norm = vec.norm();
    if (!(std::isfinite(norm) && norm > eps)) {
      return Eigen::Vector3d::Zero();
    }
    return vec / norm;
  }

  static bool projectNormalToXYPlane(ContactFrame & cf_out)
  {
    Eigen::Vector3d n_xy(cf_out.n_w.x(), cf_out.n_w.y(), 0.0);
    const double n_xy_norm = n_xy.norm();
    if (!(std::isfinite(n_xy_norm) && n_xy_norm > 1e-9)) {
      cf_out.valid = false;
      return false;
    }

    cf_out.n_w = n_xy / n_xy_norm;
    return true;
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

  static Eigen::Matrix3d normalizedEvidenceMatrix(const Eigen::Matrix3d & mat, double eps_trace)
  {
    const double trace = std::max(0.0, mat.trace());
    if (trace <= eps_trace) {
      return Eigen::Matrix3d::Zero();
    }
    return mat / (trace + eps_trace);
  }

  static double angleDegreesBetween(const Eigen::Vector3d & a, const Eigen::Vector3d & b)
  {
    const double na = a.norm();
    const double nb = b.norm();
    if (!(std::isfinite(na) && std::isfinite(nb) && na > 1e-12 && nb > 1e-12)) {
      return std::numeric_limits<double>::quiet_NaN();
    }

    double cosine = a.dot(b) / (na * nb);
    cosine = std::clamp(cosine, -1.0, 1.0);
    return std::acos(cosine) * 180.0 / M_PI;
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

    Eigen::Vector3d filtered = alpha * prev + (1.0 - alpha) * aligned_current;
    const double filtered_norm = filtered.norm();
    if (filtered_norm < 1e-12) {
      return aligned_current.normalized();
    }
    return filtered / filtered_norm;
  }

  std::string resolveForceObservationTopic(const std::string & source) const
  {
    if (
      source == "contact_force" || source == "contact_force_filt" ||
      source == "filtered" || source == "filt" || source == "raw")
    {
      return "/crazyflie/out/EE_contact_force_filt";
    }
    if (source == "mob") {
      return "/crazyflie/out/ee_applied_mob";
    }
    if (source == "mob_2nd" || source == "mob2") {
      return "/crazyflie/out/ee_applied_mob_2nd";
    }
    if (source == "mob_2nd_tau" || source == "mob_tau" || source == "consistency") {
      return "/crazyflie/out/ee_applied_mob_2nd_tau";
    }
    if (
      source == "mob_2nd_tau_ke_alt" || source == "mob_2nd_tau_ke2" ||
      source == "mob_tau_ke_alt" || source == "ke_alt" || source == "ke2")
    {
      return "/crazyflie/out/ee_applied_mob_2nd_tau_ke_alt";
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
      if (!(pose_received_ && vel_received_ && acc_received_)) {
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
      return ref;
    }

    ref.pos_w = Eigen::Vector3d(ee_pose_w_[0], ee_pose_w_[1], ee_pose_w_[2]);
    ref.vel_w = Eigen::Vector3d(ee_vel_w_[0], ee_vel_w_[1], ee_vel_w_[2]);
    ref.acc_w = Eigen::Vector3d(ee_acc_w_[0], ee_acc_w_[1], ee_acc_w_[2]);
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

  bool estimateNormalVectorKROC(
    ContactFrame & cf_out,
    const ReferenceState & ref,
    const Eigen::Vector3d & force_world)
  {
    const double force_norm = force_world.norm();
    if (force_norm < normal_force_threshold_) {
      cf_out.valid = false;
      return false;
    }

    Eigen::Vector3d n_new = force_world;
    const Eigen::Vector3d v_ref = ref.vel_w;
    const double vel_norm_sq = v_ref.squaredNorm();
    const double vel_eps_sq = 1e-8;

    if (ref.valid && vel_norm_sq > vel_eps_sq) {
      const double alpha = force_world.dot(v_ref) / vel_norm_sq;
      n_new = force_world - alpha * v_ref;
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
      cf_out.n_w = (nn > 1e-9) ? (cf_out.n_w / nn) : n_new;
    }

    return true;
  }

  bool estimateNormalVectorActionNormal0326(
    ContactFrame & cf_out,
    const ReferenceState & ref,
    Eigen::Vector3d force_world)
  {
    if (flip_measured_force_) {
      force_world = -force_world;
    }

    const double force_norm = force_world.norm();
    if (force_norm < normal_force_threshold_) {
      cf_out.valid = false;
      return false;
    }

    const Eigen::Vector3d v_world = ref.vel_w;
    const Eigen::Vector3d a_world = ref.acc_w;
    Eigen::Vector3d n_nom = force_world / (force_norm + 1e-12);

    if (cf_out.valid && cf_out.n_w.dot(n_nom) < 0.0) {
      n_nom = -n_nom;
    }

    Eigen::Vector3d tangent_axis = Eigen::Vector3d::UnitZ().cross(n_nom);
    if (tangent_axis.norm() < 1e-6) {
      tangent_axis = Eigen::Vector3d::UnitX();
    } else {
      tangent_axis.normalize();
    }

    const double w_tau = 1.0;
    const double w_f = 1.0;
    const double w_a = 0.2;
    const double theta_max = 20.0 * M_PI / 180.0;
    const double delta_max = std::tan(theta_max);
    const int num_samples = 9;
    const double vel_xy_eps = 1e-6;
    const double eps = 1e-9;

    double best_cost = std::numeric_limits<double>::infinity();
    Eigen::Vector3d n_best = n_nom;
    bool found = false;

    for (int i = 0; i < num_samples; ++i) {
      const double delta =
        -delta_max + 2.0 * delta_max * static_cast<double>(i) / static_cast<double>(num_samples - 1);

      const Eigen::Vector3d n_cand_unnorm = n_nom + delta * tangent_axis;
      const double cand_norm = n_cand_unnorm.norm();
      if (cand_norm < eps) {
        continue;
      }

      Eigen::Vector3d n_cand = n_cand_unnorm / cand_norm;
      if (n_cand.dot(n_nom) < 0.0) {
        n_cand = -n_cand;
      }
      if (n_cand.dot(n_nom) < std::cos(theta_max)) {
        continue;
      }

      const double force_normal = force_world.dot(n_cand);
      if (force_normal <= normal_force_threshold_) {
        continue;
      }

      const Eigen::Vector3d force_tangent = force_world - force_normal * n_cand;
      const double force_leak = force_tangent.norm();

      double accel_tangent_norm = 0.0;
      if (ref.valid) {
        const Eigen::Vector3d accel_tangent = a_world - n_cand * (n_cand.dot(a_world));
        accel_tangent_norm = accel_tangent.norm();
      }

      double tau_z = 0.0;
      if (ref.valid) {
        Eigen::Vector3d vel_tangent = v_world - n_cand * (n_cand.dot(v_world));
        Eigen::Vector2d n_xy(n_cand.x(), n_cand.y());
        Eigen::Vector2d v_xy(vel_tangent.x(), vel_tangent.y());

        const double nxy = n_xy.norm();
        const double vxy = v_xy.norm();
        if (nxy > vel_xy_eps && vxy > vel_xy_eps) {
          n_xy /= nxy;
          v_xy /= vxy;
          tau_z = std::abs(n_xy.x() * v_xy.y() - n_xy.y() * v_xy.x());
        }
      }

      const double cost =
        w_tau * tau_z * tau_z +
        w_f * force_leak * force_leak +
        w_a * accel_tangent_norm * accel_tangent_norm;

      if (cost < best_cost) {
        best_cost = cost;
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

      const double nn = cf_out.n_w.norm();
      cf_out.n_w = (nn > 1e-9) ? (cf_out.n_w / nn) : n_best;
    }

    return true;
  }

  bool estimateNormalVectorLfLvFusion(
    ContactFrame & cf_out,
    const ReferenceState & ref,
    Eigen::Vector3d force_world,
    double dt)
  {
    if (flip_measured_force_) {
      force_world = -force_world;
    }

    const double force_norm = force_world.norm();
    if (force_norm < normal_force_threshold_) {
      cf_out.valid = false;
      return false;
    }

    dt = std::clamp(dt, 1e-4, 0.1);

    const double vel_norm = ref.vel_w.norm();
    Eigen::Vector3d s_v = Eigen::Vector3d::Zero();
    if (vel_norm > new_normal_velocity_epsilon_) {
      s_v = ref.vel_w / (vel_norm + 1e-12);
    }
    new_normal_state_.vel_norm = vel_norm;
    new_normal_state_.force_norm = force_norm;

    const Eigen::Matrix3d l_v_dot =
      -new_normal_beta_v_ * new_normal_state_.l_v +
      new_normal_sigma_v_ * (s_v * s_v.transpose());

    new_normal_state_.l_v += dt * l_v_dot;

    const Eigen::Vector3d f_hat_ext = safeNormalized(force_world, new_normal_force_epsilon_);
    Eigen::Matrix3d proj_s = Eigen::Matrix3d::Identity();
    if (s_v.squaredNorm() > 1e-12) {
      proj_s -= s_v * s_v.transpose();
    }

    const double alpha_slip =
      (vel_norm * vel_norm) /
      (new_normal_slip_blend_velocity_ * new_normal_slip_blend_velocity_ +
      vel_norm * vel_norm + 1e-12);
    const Eigen::Vector3d f_perp =
      (1.0 - alpha_slip) * f_hat_ext +
      alpha_slip * (proj_s * f_hat_ext);
    const double f_perp_norm = f_perp.norm();
    const Eigen::Vector3d n_f = safeNormalized(f_perp, new_normal_force_epsilon_);

    if (n_f.squaredNorm() > 1e-12) {
      const Eigen::Matrix3d l_f_dot =
        -new_normal_beta_f_ * new_normal_state_.l_f +
        new_normal_sigma_f_ * (n_f * n_f.transpose());
      new_normal_state_.l_f += dt * l_f_dot;
      new_normal_state_.n_f = n_f;
    } else {
      const Eigen::Matrix3d l_f_dot =
        -new_normal_beta_f_ * new_normal_state_.l_f;
      new_normal_state_.l_f += dt * l_f_dot;
    }

    const EigenSpectrum spec_v = eigenDecomposeDescending(new_normal_state_.l_v);

    const double lambda1 = std::max(0.0, spec_v.values(0));
    const double lambda2 = std::max(0.0, spec_v.values(1));
    const double lambda3 = std::max(0.0, spec_v.values(2));
    new_normal_state_.lambda1 = lambda1;
    new_normal_state_.lambda2 = lambda2;
    new_normal_state_.lambda3 = lambda3;

    const double rho_v_mag = lambda2 / (lambda2 + new_normal_c_v_ + 1e-12);
    const double rho_v_gap =
      std::max(0.0, lambda2 - lambda3) /
      (lambda1 + new_normal_eps_lambda_);
    const double rho_v = clamp01(rho_v_mag * rho_v_gap);

    const double rho_f_bar = f_perp_norm / (f_perp_norm + new_normal_c_f_ + 1e-12);

    double e_tau_norm = 0.0;
    bool have_consistency = true;
    if (new_normal_use_raw_mob_tau_residual_) {
      Eigen::Vector3d raw_force_world = Eigen::Vector3d::Zero();
      Eigen::Vector3d raw_torque_world = Eigen::Vector3d::Zero();
      bool raw_received = false;
      {
        std::lock_guard<std::mutex> lk(raw_mob_mtx_);
        if (raw_mob_received_) {
          raw_force_world = raw_mob_force_world_;
          raw_torque_world = raw_mob_torque_world_;
          raw_received = true;
        }
      }

      Eigen::Vector3d ee_offset_world = Eigen::Vector3d::Zero();
      bool have_offset = false;
      {
        std::lock_guard<std::mutex> lk(state_mtx_);
        if (pose_received_ && ee_pose_received_) {
          const Eigen::Vector3d drone_pos_w(pose_w_[0], pose_w_[1], pose_w_[2]);
          const Eigen::Vector3d ee_pos_w(ee_pose_w_[0], ee_pose_w_[1], ee_pose_w_[2]);
          ee_offset_world = ee_pos_w - drone_pos_w;
          have_offset = true;
        }
      }

      if (raw_received && have_offset) {
        e_tau_norm = (raw_torque_world - ee_offset_world.cross(raw_force_world)).norm();
        have_consistency = true;
      }
    } else {
      std::lock_guard<std::mutex> lk(consistency_mtx_);
      if (consistency_received_) {
        e_tau_norm = (torque_hat_world_ - moment_from_force_world_).norm();
        have_consistency = true;
      }
    }
    const double sigma_tau_sq = new_normal_sigma_tau_ * new_normal_sigma_tau_;
    const double m_tau_raw =
      have_consistency ?
      (new_normal_eta_tau_ + (1.0 - new_normal_eta_tau_) *
      std::exp(-(e_tau_norm * e_tau_norm) / (sigma_tau_sq + 1e-12))) :
      1.0;
    const double tau_lpf = 1.0 / (2.0 * M_PI * kMTauLpfCutoffHz);
    const double m_tau_alpha = std::clamp(dt / (tau_lpf + dt), 0.0, 1.0);
    if (!m_tau_lpf_initialized_) {
      m_tau_lpf_ = m_tau_raw;
      m_tau_lpf_initialized_ = true;
    } else {
      m_tau_lpf_ += m_tau_alpha * (m_tau_raw - m_tau_lpf_);
    }
    const double m_tau = m_tau_lpf_;
    new_normal_state_.m_tau = m_tau;
    new_normal_state_.e_tau_norm = e_tau_norm;

    const double rho_f = clamp01(rho_f_bar * ((1.0 - rho_v) + rho_v * m_tau));
    const double alpha_denom = rho_v + rho_f + new_normal_eps_alpha_;
    const double alpha_v = rho_v / alpha_denom;
    const double alpha_f = rho_f / alpha_denom;

    const Eigen::Matrix3d l_v_bar = normalizedEvidenceMatrix(new_normal_state_.l_v, new_normal_eps_l_);
    const Eigen::Matrix3d l_f_bar = normalizedEvidenceMatrix(new_normal_state_.l_f, new_normal_eps_l_);
    const Eigen::Matrix3d m_n = alpha_f * l_f_bar - alpha_v * l_v_bar;
    const EigenSpectrum spec_n = eigenDecomposeDescending(m_n);

    Eigen::Vector3d n_geo = spec_n.vectors.col(0);
    if (n_geo.norm() < 1e-9) {
      cf_out.valid = false;
      return false;
    }
    n_geo.normalize();

    if (new_normal_state_.n_f.squaredNorm() > 1e-12 && n_geo.dot(new_normal_state_.n_f) < 0.0) {
      n_geo = -n_geo;
    } else if (cf_out.valid && n_geo.dot(cf_out.n_w) < 0.0) {
      n_geo = -n_geo;
    } else if (n_geo.dot(force_world) < 0.0) {
      n_geo = -n_geo;
    }

    new_normal_state_.n_geo = n_geo;
    new_normal_state_.rho_v = rho_v;
    new_normal_state_.rho_f = rho_f;
    new_normal_state_.alpha_v = alpha_v;
    new_normal_state_.alpha_f = alpha_f;
    new_normal_state_.l_v_bar = l_v_bar;
    new_normal_state_.l_f_bar = l_f_bar;
    new_normal_state_.initialized = true;

    if (std::max(rho_v, rho_f) < new_normal_min_confidence_) {
      cf_out.valid = false;
      return false;
    }

    cf_out.n_w = n_geo;

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
      cf_out.valid = false;
      return false;
    }

    const Eigen::Vector3d v_c_world = ref.vel_w;
    const double vel_norm = v_c_world.norm();
    Eigen::Vector3d w_s = Eigen::Vector3d::Zero();
    if (vel_norm > force_based_velocity_epsilon_) {
      w_s = v_c_world / (vel_norm + 1e-12);
    }

    const double vel_norm_sq = vel_norm * vel_norm;
    const double gamma_v =
      vel_norm_sq / (vel_norm_sq + force_based_gamma_epsilon_ + 1e-12);

    Eigen::Vector3d n_f = Eigen::Vector3d::Zero();
    if (force_norm > force_based_force_epsilon_) {
      n_f = force_world / (force_norm + 1e-12);
    }

    Eigen::Matrix3d projector = Eigen::Matrix3d::Identity();
    if (w_s.squaredNorm() > 1e-12) {
      projector -= gamma_v * (w_s * w_s.transpose());
    }
    const Eigen::Vector3d f_g = projector * force_world;
    const double corrected_force_norm = f_g.norm();

    Eigen::Vector3d n_alg = Eigen::Vector3d::Zero();
    if (corrected_force_norm > force_based_algebraic_force_epsilon_) {
      n_alg = f_g / (corrected_force_norm + 1e-12);
    } else if (n_f.squaredNorm() > 1e-12) {
      n_alg = n_f;
    } else {
      cf_out.valid = false;
      return false;
    }

    if (cf_out.valid && cf_out.n_w.dot(n_alg) < 0.0) {
      n_alg = -n_alg;
    } else if (n_f.squaredNorm() > 1e-12 && n_f.dot(n_alg) < 0.0) {
      n_alg = -n_alg;
    }

    force_based_state_.n_alg = lowPassNormalizedDirection(
      force_based_state_.n_alg, n_alg, clamp01(force_based_candidate_lpf_alpha_));

    const Eigen::Vector3d n_alg_memory =
      (force_based_state_.n_alg.norm() > 1e-12) ? force_based_state_.n_alg : n_alg;

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
    force_based_state_.f_g = f_g;
    force_based_state_.n_geo = n_geo;
    force_based_state_.gamma_v = gamma_v;
    force_based_state_.vel_norm = vel_norm;
    force_based_state_.force_norm = force_norm;
    force_based_state_.corrected_force_norm = corrected_force_norm;
    force_based_state_.initialized = true;

    cf_out.n_w = n_geo;
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
    if (normal_estimator_method_ == "kroc") {
      ok = estimateNormalVectorKROC(cf_out, ref, force_world);
    } else if (normal_estimator_method_ == "action_normal") {
      ok = estimateNormalVectorActionNormal0326(cf_out, ref, force_world);
    } else if (
      normal_estimator_method_ == "Lf_Lv_fusion" ||
      normal_estimator_method_ == "lf_lv_fusion" ||
      normal_estimator_method_ == "new_normal")
    {
      ok = estimateNormalVectorLfLvFusion(cf_out, ref, force_world, dt);
    } else if (
      normal_estimator_method_ == "normal_force_based" ||
      normal_estimator_method_ == "force_based")
    {
      ok = estimateNormalVectorForceBased(cf_out, ref, force_world, dt);
    } else {
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
    if (
      normal_estimator_method_ == "normal_force_based" ||
      normal_estimator_method_ == "force_based")
    {
      if (!force_based_state_.initialized) {
        pub_normal_debug_metrics_->publish(msg);
        return;
      }
      msg.data[0] = force_based_state_.gamma_v;
      msg.data[1] = force_based_state_.vel_norm;
      msg.data[2] = force_based_state_.force_norm;
      msg.data[3] = force_based_state_.corrected_force_norm;
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
      pub_normal_debug_metrics_->publish(msg);
      return;
    }

    msg.data[0] = new_normal_state_.lambda1;
    msg.data[1] = new_normal_state_.lambda2;
    msg.data[2] = new_normal_state_.lambda3;
    msg.data[3] = new_normal_state_.m_tau;
    msg.data[4] = new_normal_state_.vel_norm;
    msg.data[5] = new_normal_state_.force_norm;
    msg.data[6] = new_normal_state_.rho_v;
    msg.data[7] = new_normal_state_.rho_f;
    msg.data[8] = new_normal_state_.alpha_v;
    msg.data[9] = new_normal_state_.alpha_f;

    int idx = 10;
    for (int r = 0; r < 3; ++r) {
      for (int c = 0; c < 3; ++c) {
        msg.data[idx++] = new_normal_state_.l_v_bar(r, c);
      }
    }
    for (int r = 0; r < 3; ++r) {
      for (int c = 0; c < 3; ++c) {
        msg.data[idx++] = new_normal_state_.l_f_bar(r, c);
      }
    }
    msg.data[28] = new_normal_state_.n_geo.x();
    msg.data[29] = new_normal_state_.n_geo.y();
    msg.data[30] = new_normal_state_.n_geo.z();
    msg.data[31] = new_normal_state_.n_f.x();
    msg.data[32] = new_normal_state_.n_f.y();
    msg.data[33] = new_normal_state_.n_f.z();

    idx = 34;
    for (int r = 0; r < 3; ++r) {
      for (int c = 0; c < 3; ++c) {
        msg.data[idx++] = new_normal_state_.l_v(r, c);
      }
    }
    for (int r = 0; r < 3; ++r) {
      for (int c = 0; c < 3; ++c) {
        msg.data[idx++] = new_normal_state_.l_f(r, c);
      }
    }

    msg.data[52] = new_normal_state_.e_tau_norm;
    pub_normal_debug_metrics_->publish(msg);
  }

  void clearNormalDebugMetrics()
  {
    const auto nan = std::numeric_limits<double>::quiet_NaN();
    new_normal_state_.lambda1 = nan;
    new_normal_state_.lambda2 = nan;
    new_normal_state_.lambda3 = nan;
    new_normal_state_.m_tau = nan;
    new_normal_state_.e_tau_norm = nan;
    new_normal_state_.vel_norm = nan;
    new_normal_state_.force_norm = nan;
    new_normal_state_.rho_v = nan;
    new_normal_state_.rho_f = nan;
    new_normal_state_.alpha_v = nan;
    new_normal_state_.alpha_f = nan;
    new_normal_state_.l_v_bar = Eigen::Matrix3d::Constant(nan);
    new_normal_state_.l_f_bar = Eigen::Matrix3d::Constant(nan);
    new_normal_state_.angle_n_geo_deg = nan;
    new_normal_state_.angle_n_f_deg = nan;
    new_normal_state_.n_est = Eigen::Vector3d::Zero();
    new_normal_state_.initialized = false;
    m_tau_lpf_initialized_ = false;

    force_based_state_.w_s = Eigen::Vector3d::Zero();
    force_based_state_.n_f = Eigen::Vector3d::Zero();
    force_based_state_.f_g = Eigen::Vector3d::Zero();
    force_based_state_.n_alg = Eigen::Vector3d::Zero();
    force_based_state_.n_geo = Eigen::Vector3d::Zero();
    force_based_state_.n_est = Eigen::Vector3d::Zero();
    force_based_state_.gamma_v = 0.0;
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

  void consistencyMatchCb(const geometry_msgs::msg::WrenchStamped::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lk(consistency_mtx_);
    torque_hat_world_ =
      Eigen::Vector3d(msg->wrench.force.x, msg->wrench.force.y, msg->wrench.force.z);
    moment_from_force_world_ =
      Eigen::Vector3d(msg->wrench.torque.x, msg->wrench.torque.y, msg->wrench.torque.z);
    consistency_received_ = true;
  }

  void rawMobWrenchCb(const geometry_msgs::msg::WrenchStamped::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lk(raw_mob_mtx_);
    raw_mob_force_world_ =
      Eigen::Vector3d(msg->wrench.force.x, msg->wrench.force.y, msg->wrench.force.z);
    raw_mob_torque_world_ =
      Eigen::Vector3d(msg->wrench.torque.x, msg->wrench.torque.y, msg->wrench.torque.z);
    raw_mob_received_ = true;
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

    if (
      normal_estimator_method_ == "normal_force_based" ||
      normal_estimator_method_ == "force_based")
    {
      force_based_state_.n_est = cf_local.n_w;
    } else {
      new_normal_state_.n_est = cf_local.n_w;
      new_normal_state_.angle_n_geo_deg =
        angleDegreesBetween(new_normal_state_.n_geo, cf_local.n_w);
      new_normal_state_.angle_n_f_deg =
        angleDegreesBetween(new_normal_state_.n_f, cf_local.n_w);
    }

    publishContactQuat(cf_local.R_C, stamp);
    publishNormalDebugMetrics();

    std_msgs::msg::Float32 force_x_msg;
    force_x_msg.data = static_cast<float>(force_contact.x());
    pub_contact_force_x_->publish(force_x_msg);
  }

  rclcpp::Subscription<geometry_msgs::msg::WrenchStamped>::SharedPtr sub_contact_force_;
  rclcpp::Subscription<geometry_msgs::msg::WrenchStamped>::SharedPtr sub_consistency_match_;
  rclcpp::Subscription<geometry_msgs::msg::WrenchStamped>::SharedPtr sub_raw_mob_wrench_;
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

  double new_normal_velocity_epsilon_{0.015};
  double new_normal_beta_v_{4.0};
  double new_normal_sigma_v_{4.0};
  double new_normal_slip_blend_velocity_{0.04};
  double new_normal_force_epsilon_{0.004};
  double new_normal_beta_f_{8.0};
  double new_normal_sigma_f_{8.0};
  double new_normal_c_v_{0.08};
  double new_normal_eps_lambda_{1e-4};
  double new_normal_c_f_{0.015};
  double new_normal_eta_tau_{0.4};
  double new_normal_sigma_tau_{0.004};
  bool new_normal_use_raw_mob_tau_residual_{false};
  double new_normal_eps_alpha_{1e-6};
  double new_normal_eps_l_{1e-6};
  double new_normal_min_confidence_{0.08};
  double force_based_velocity_epsilon_{0.005};
  double force_based_gamma_epsilon_{5.0e-4};
  double force_based_force_epsilon_{0.01};
  double force_based_algebraic_force_epsilon_{5.0e-3};
  double force_based_candidate_lpf_alpha_{0.2};
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
  std::string consistency_match_topic_;
  std::string raw_mob_wrench_topic_;
  std::string use_vel_mode_topic_;

  std::mutex force_mtx_;
  std::array<double, 3> contact_force_{0.0, 0.0, 0.0};
  bool force_received_{false};

  std::mutex contact_frame_mtx_;
  ContactFrame contact_frame_;
  NewNormalState new_normal_state_;
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

  std::mutex consistency_mtx_;
  Eigen::Vector3d torque_hat_world_{Eigen::Vector3d::Zero()};
  Eigen::Vector3d moment_from_force_world_{Eigen::Vector3d::Zero()};
  bool consistency_received_{false};

  std::mutex raw_mob_mtx_;
  Eigen::Vector3d raw_mob_force_world_{Eigen::Vector3d::Zero()};
  Eigen::Vector3d raw_mob_torque_world_{Eigen::Vector3d::Zero()};
  bool raw_mob_received_{false};

  static constexpr double kMTauLpfCutoffHz = 2.0;
  double m_tau_lpf_{1.0};
  bool m_tau_lpf_initialized_{false};

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
