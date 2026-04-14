#include "mujoco_bridge/mujoco_contact.hpp"

#include <mujoco/mujoco.h>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/wrench_stamped.hpp>

#include <array>
#include <algorithm>
#include <cmath>
#include <cstring>
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
  }
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
