#include "mujoco_bridge/mujoco_contact.hpp"

#include <mujoco/mujoco.h>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/wrench_stamped.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>

#include <array>
#include <algorithm>
#include <cmath>
#include <cstring>
#include <limits>
#include <stdexcept>

namespace mujoco_bridge
{

namespace
{

inline void mat3_from_contact_frame(const mjContact& con, double R[9])
{
  for (int i = 0; i < 9; ++i) {
    R[i] = static_cast<double>(con.frame[i]);
  }
}

inline void mat3_vec_mul(const double R[9], const double v[3], double out[3])
{
  out[0] = R[0] * v[0] + R[1] * v[1] + R[2] * v[2];
  out[1] = R[3] * v[0] + R[4] * v[1] + R[5] * v[2];
  out[2] = R[6] * v[0] + R[7] * v[1] + R[8] * v[2];
}

inline void mat3_transpose_vec_mul(const double R[9], const double v[3], double out[3])
{
  out[0] = R[0] * v[0] + R[3] * v[1] + R[6] * v[2];
  out[1] = R[1] * v[0] + R[4] * v[1] + R[7] * v[2];
  out[2] = R[2] * v[0] + R[5] * v[1] + R[8] * v[2];
}

inline double norm3(const double v[3])
{
  return std::sqrt(v[0]*v[0] + v[1]*v[1] + v[2]*v[2]);
}

inline void normalize3(double v[3])
{
  const double n = norm3(v);
  if (n < 1e-12) {
    v[0] = 0.0;
    v[1] = 0.0;
    v[2] = 1.0;
    return;
  }
  v[0] /= n;
  v[1] /= n;
  v[2] /= n;
}

inline void cross3(const double a[3], const double b[3], double out[3])
{
  out[0] = a[1]*b[2] - a[2]*b[1];
  out[1] = a[2]*b[0] - a[0]*b[2];
  out[2] = a[0]*b[1] - a[1]*b[0];
}

}  // namespace

MujocoContact::MujocoContact(
  rclcpp::Node* node,
  mjModel* model,
  mjData* data,
  const rclcpp::Publisher<geometry_msgs::msg::WrenchStamped>::SharedPtr& pub_contact_force,
  const rclcpp::Publisher<geometry_msgs::msg::WrenchStamped>::SharedPtr& pub_contact_force_filt)
: node_(node),
  model_(model),
  data_(data),
  pub_contact_force_(pub_contact_force),
  pub_contact_force_filt_(pub_contact_force_filt)
{
  node_->declare_parameter("viz.contact_arrows.enable", true);
  node_->declare_parameter("viz.contact_arrows.scale", 4.5);
  node_->declare_parameter("viz.contact_arrows.width", 0.008);
  node_->declare_parameter("viz.contact_arrows.max", 64);

  viz_contact_enable_ = node_->get_parameter("viz.contact_arrows.enable").as_bool();
  viz_contact_scale_ = node_->get_parameter("viz.contact_arrows.scale").as_double();
  viz_contact_width_ = node_->get_parameter("viz.contact_arrows.width").as_double();
  viz_contact_max_ = node_->get_parameter("viz.contact_arrows.max").as_int();

  node_->declare_parameter("contact_filter.enable", true);
  node_->declare_parameter("contact_filter.cutoff_hz", 5.0);
  node_->declare_parameter("contact_filter.timer_hz", 100.0);
  node_->declare_parameter("contact_filter.use_exp_alpha", true);

  contact_filter_enable_ = node_->get_parameter("contact_filter.enable").as_bool();
  contact_cutoff_hz_ = node_->get_parameter("contact_filter.cutoff_hz").as_double();
  contact_timer_hz_ = node_->get_parameter("contact_filter.timer_hz").as_double();
  use_exp_alpha_ = node_->get_parameter("contact_filter.use_exp_alpha").as_bool();

  gid_tip_ = mj_name2id(model_, mjOBJ_GEOM, "ee_tip_sphere");

  if (gid_tip_ < 0) {
    throw std::runtime_error("geom not found: ee_tip_sphere");
  }

  const int tip_body_id = model_->geom_bodyid[gid_tip_];
  if (tip_body_id < 0) {
    throw std::runtime_error("failed to resolve body for ee_tip_sphere");
  }
  tip_root_body_id_ = model_->body_rootid[tip_body_id];

  gid_small_cylinder_ = mj_name2id(model_, mjOBJ_GEOM, "small_cylinder_collision");
  gid_large_cylinder_ = mj_name2id(model_, mjOBJ_GEOM, "large_cylinder_collision");
  gid_belt_right_ = mj_name2id(model_, mjOBJ_GEOM, "cylinder_belt_right_collision");
  gid_belt_left_ = mj_name2id(model_, mjOBJ_GEOM, "cylinder_belt_left_collision");
  pub_geometry_metrics_ = node_->create_publisher<std_msgs::msg::Float64MultiArray>(
    "/mujoco/ground_truth/geometry_metrics", 10);

  if (contact_filter_enable_) {
    const auto period = std::chrono::duration<double>(1.0 / std::max(1e-6, contact_timer_hz_));
    timer_contact_ = node_->create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(period),
      std::bind(&MujocoContact::contact_filter_timer_cb, this));
  }
}

bool MujocoContact::is_external_geom(int geom_id) const
{
  if (geom_id < 0 || geom_id >= model_->ngeom) {
    return false;
  }
  if (geom_id == gid_tip_) {
    return false;
  }

  const int body_id = model_->geom_bodyid[geom_id];
  if (body_id < 0 || body_id >= model_->nbody) {
    return false;
  }

  return model_->body_rootid[body_id] != tip_root_body_id_;
}

void MujocoContact::update_raw_and_publish(const rclcpp::Time& stamp)
{
  std::lock_guard<std::mutex> lock(mtx_);

  compute_contact_resultant_locked();

  f_raw_latest_ = fw_;

  geometry_msgs::msg::WrenchStamped msg;
  msg.header.stamp = stamp;
  msg.header.frame_id = "world";
  msg.wrench.force.x = fw_[0];
  msg.wrench.force.y = fw_[1];
  msg.wrench.force.z = fw_[2];
  msg.wrench.torque.x = 0.0;
  msg.wrench.torque.y = 0.0;
  msg.wrench.torque.z = 0.0;
  pub_contact_force_->publish(msg);

  std_msgs::msg::Float64MultiArray geometry_msg;
  geometry_msg.data.assign(geometry_metrics_.begin(), geometry_metrics_.end());
  pub_geometry_metrics_->publish(geometry_msg);
}

void MujocoContact::contact_filter_timer_cb()
{
  std::lock_guard<std::mutex> lock(mtx_);

  const auto now = node_->get_clock()->now();
  if (last_contact_timer_time_.nanoseconds() == 0) {
    last_contact_timer_time_ = now;
    f_raw_prev_ = f_raw_latest_;
    return;
  }

  const double dt = (now - last_contact_timer_time_).seconds();
  last_contact_timer_time_ = now;

  if (dt <= 1e-6 || dt > 0.05) {
    return;
  }

  std::array<double, 3> fdot_raw{};
  for (int i = 0; i < 3; ++i) {
    fdot_raw[i] = (f_raw_latest_[i] - f_raw_prev_[i]) / dt;
  }

  f_raw_prev_ = f_raw_latest_;
  fdot_raw_latest_ = fdot_raw;

  double a = 1.0;
  if (contact_cutoff_hz_ > 0.0) {
    if (use_exp_alpha_) {
      const double wc = 2.0 * M_PI * contact_cutoff_hz_;
      a = 1.0 - std::exp(-wc * dt);
    } else {
      a = std::clamp(contact_cutoff_hz_ * dt, 0.0, 1.0);
    }
  }

  for (int i = 0; i < 3; ++i) {
    fdot_filt_latest_[i] = (1.0 - a) * fdot_filt_latest_[i] + a * fdot_raw[i];
    f_filt_latest_[i] = (1.0 - a) * f_filt_latest_[i] + a * f_raw_latest_[i];
  }

  geometry_msgs::msg::WrenchStamped msg;
  msg.header.stamp = now;
  msg.header.frame_id = "world";
  msg.wrench.force.x = f_filt_latest_[0];
  msg.wrench.force.y = f_filt_latest_[1];
  msg.wrench.force.z = f_filt_latest_[2];
  msg.wrench.torque.x = 0.0;
  msg.wrench.torque.y = 0.0;
  msg.wrench.torque.z = 0.0;
  pub_contact_force_filt_->publish(msg);
}

void MujocoContact::compute_contact_resultant_locked()
{
  fcn_ = 0.0;
  rf_ = {0.0, 0.0, 0.0};
  fw_ = {0.0, 0.0, 0.0};
  const double nan = std::numeric_limits<double>::quiet_NaN();
  geometry_metrics_ = {nan, nan, nan, nan, nan, 0.0, nan, nan};

  int active_geom = -1;
  double active_weight = -1.0;
  std::array<double, 3> active_contact_pos{{0.0, 0.0, 0.0}};

  const auto now = node_->get_clock()->now();
  bool do_log = false;
  if ((now - last_contact_log_time_).seconds() > 1.0) {
    last_contact_log_time_ = now;
    do_log = true;
  }

  for (int i = 0; i < data_->ncon; ++i) {
    const mjContact& con = data_->contact[i];
    const int g1 = con.geom1;
    const int g2 = con.geom2;

    const bool tip_is_g1 = (g1 == gid_tip_);
    const bool tip_is_g2 = (g2 == gid_tip_);
    if (!tip_is_g1 && !tip_is_g2) {
      continue;
    }

    const int other_geom = tip_is_g1 ? g2 : g1;
    if (!is_external_geom(other_geom)) {
      continue;
    }

    if (do_log && i < 5) {
      const char* name1 = mj_id2name(model_, mjOBJ_GEOM, g1);
      const char* name2 = mj_id2name(model_, mjOBJ_GEOM, g2);
      RCLCPP_DEBUG(
        node_->get_logger(),
        "contact geom pair accepted: %s / %s",
        name1 ? name1 : "(null)",
        name2 ? name2 : "(null)");
    }

    mjtNum fci[6] = {0, 0, 0, 0, 0, 0};
    mj_contactForce(model_, data_, i, fci);

    double R[9];
    mat3_from_contact_frame(con, R);

    const double fc[3] = {
      static_cast<double>(fci[0]),
      static_cast<double>(fci[1]),
      static_cast<double>(fci[2])
    };

    double fw_local[3];
    mat3_transpose_vec_mul(R, fc, fw_local);

    const double fn_mag = norm3(fw_local);
    if (fn_mag < 1e-12) {
      continue;
    }

    fcn_ += fn_mag;
    rf_[0] += static_cast<double>(con.pos[0]) * fn_mag;
    rf_[1] += static_cast<double>(con.pos[1]) * fn_mag;
    rf_[2] += static_cast<double>(con.pos[2]) * fn_mag;

    fw_[0] += fw_local[0];
    fw_[1] += fw_local[1];
    fw_[2] += fw_local[2];

    const double normal_contribution = std::abs(fc[0]);
    if (normal_contribution > active_weight &&
      (other_geom == gid_small_cylinder_ || other_geom == gid_large_cylinder_ ||
      other_geom == gid_belt_right_ || other_geom == gid_belt_left_))
    {
      active_weight = normal_contribution;
      active_geom = other_geom;
      active_contact_pos = {
        static_cast<double>(con.pos[0]), static_cast<double>(con.pos[1]),
        static_cast<double>(con.pos[2])};
    }
  }

  if (active_geom < 0) {
    return;
  }

  const bool is_small = active_geom == gid_small_cylinder_;
  const bool is_large = active_geom == gid_large_cylinder_;
  const bool is_cylinder = is_small || is_large;
  const int surface_id = is_small ? 1 : (is_large ? 2 : (active_geom == gid_belt_right_ ? 3 : 4));
  double n_true[3] = {0.0, 0.0, 0.0};
  if (is_cylinder) {
    const mjtNum * center = data_->geom_xpos + 3 * active_geom;
    n_true[0] = active_contact_pos[0] - static_cast<double>(center[0]);
    n_true[1] = active_contact_pos[1] - static_cast<double>(center[1]);
    normalize3(n_true);
    n_true[2] = 0.0;
  } else {
    const mjtNum * center = data_->geom_xpos + 3 * active_geom;
    const mjtNum * rot = data_->geom_xmat + 9 * active_geom;
    // The thin box's local y axis is its face normal; select the contacted side.
    double face_normal[3] = {
      static_cast<double>(rot[1]), static_cast<double>(rot[4]), static_cast<double>(rot[7])};
    const double side =
      (active_contact_pos[0] - center[0]) * face_normal[0] +
      (active_contact_pos[1] - center[1]) * face_normal[1] +
      (active_contact_pos[2] - center[2]) * face_normal[2];
    const double sign = side >= 0.0 ? 1.0 : -1.0;
    for (int j = 0; j < 3; ++j) n_true[j] = sign * face_normal[j];
    normalize3(n_true);
  }

  mjtNum spatial_vel[6] = {0, 0, 0, 0, 0, 0};
  mj_objectVelocity(model_, data_, mjOBJ_GEOM, gid_tip_, spatial_vel, 0);
  // cf21B_500.xml places ee_tip_site and ee_tip_sphere at the ee_tip body
  // origin (no geom-local pos), so the geom-center linear velocity is the
  // same fixed EE reference-point velocity used by /EE_velocity and Eq. (12).
  const double vc[3] = {
    static_cast<double>(spatial_vel[3]),
    static_cast<double>(spatial_vel[4]),
    static_cast<double>(spatial_vel[5])};
  const double normal_speed = n_true[0]*vc[0] + n_true[1]*vc[1] + n_true[2]*vc[2];
  const double vt[3] = {
    vc[0] - n_true[0]*normal_speed, vc[1] - n_true[1]*normal_speed,
    vc[2] - n_true[2]*normal_speed};
  const double vt_norm = norm3(vt);
  double kappa_true = 0.0;
  double a_true = 0.0;
  double v_theta = 0.0;
  if (is_cylinder && vt_norm > 1e-6) {
    const double e_theta[3] = {-n_true[1], n_true[0], 0.0};
    v_theta = e_theta[0]*vt[0] + e_theta[1]*vt[1];
    const double radius = static_cast<double>(model_->geom_size[3 * active_geom]);
    kappa_true = (v_theta * v_theta) / (radius * vt_norm * vt_norm);
    a_true = (v_theta * v_theta) / radius;
  }
  geometry_metrics_ = {
    kappa_true, a_true, n_true[0], n_true[1], n_true[2],
    static_cast<double>(surface_id), v_theta, vt_norm};
}

void MujocoContact::update_contact_resultant_arrow_in_viewer(mjvScene* scn)
{
  if (!scn) {
    return;
  }

  std::lock_guard<std::mutex> lock(mtx_);

  if (!viz_contact_enable_ || viz_contact_scale_ <= 0.0) {
    return;
  }
  if (fcn_ <= 1e-12) {
    return;
  }
  if (scn->ngeom >= scn->maxgeom) {
    return;
  }

  const double fn = std::sqrt(fw_[0]*fw_[0] + fw_[1]*fw_[1] + fw_[2]*fw_[2]);
  if (fn <= 1e-12) {
    return;
  }

  double p0[3] = {
    rf_[0] / fcn_,
    rf_[1] / fcn_,
    rf_[2] / fcn_
  };

  double z[3] = {fw_[0], fw_[1], fw_[2]};
  normalize3(z);

  double up[3] = {0.0, 0.0, 1.0};
  if (std::fabs(z[2]) > 0.95) {
    up[0] = 0.0;
    up[1] = 1.0;
    up[2] = 0.0;
  }

  double x[3], y[3];
  cross3(up, z, x);
  normalize3(x);
  cross3(z, x, y);

  mjvGeom* g = scn->geoms + scn->ngeom;

  const float rgba[4] = {1.f, 0.f, 0.f, 1.f};
  mjv_initGeom(g, mjGEOM_ARROW, nullptr, nullptr, nullptr, rgba);

  g->pos[0] = static_cast<float>(p0[0]);
  g->pos[1] = static_cast<float>(p0[1]);
  g->pos[2] = static_cast<float>(p0[2]);

  g->mat[0] = static_cast<float>(x[0]);
  g->mat[1] = static_cast<float>(y[0]);
  g->mat[2] = static_cast<float>(z[0]);

  g->mat[3] = static_cast<float>(x[1]);
  g->mat[4] = static_cast<float>(y[1]);
  g->mat[5] = static_cast<float>(z[1]);

  g->mat[6] = static_cast<float>(x[2]);
  g->mat[7] = static_cast<float>(y[2]);
  g->mat[8] = static_cast<float>(z[2]);

  g->size[0] = static_cast<float>(viz_contact_width_);
  g->size[1] = static_cast<float>(viz_contact_width_);
  g->size[2] = static_cast<float>(viz_contact_scale_ * fn);

  scn->ngeom += 1;
}

std::array<double, 3> MujocoContact::latest_raw_force_world() const
{
  std::lock_guard<std::mutex> lock(mtx_);
  return f_raw_latest_;
}

std::array<double, 3> MujocoContact::latest_filtered_force_world() const
{
  std::lock_guard<std::mutex> lock(mtx_);
  return f_filt_latest_;
}

}  // namespace mujoco_bridge
