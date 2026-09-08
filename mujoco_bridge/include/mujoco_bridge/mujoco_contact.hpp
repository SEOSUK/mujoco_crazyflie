#pragma once

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/wrench_stamped.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <mujoco/mujoco.h>

#include <array>
#include <mutex>

namespace mujoco_bridge
{

class MujocoContact
{
public:
  MujocoContact(
    rclcpp::Node* node,
    mjModel* model,
    mjData* data,
    const rclcpp::Publisher<geometry_msgs::msg::WrenchStamped>::SharedPtr& pub_contact_force,
    const rclcpp::Publisher<geometry_msgs::msg::WrenchStamped>::SharedPtr& pub_contact_force_filt);

  void update_raw_and_publish(const rclcpp::Time& stamp);
  void contact_filter_timer_cb();
  void update_contact_resultant_arrow_in_viewer(mjvScene* scn);

  std::array<double, 3> latest_raw_force_world() const;
  std::array<double, 3> latest_filtered_force_world() const;

private:
  void compute_contact_resultant_locked();
  bool is_external_geom(int geom_id) const;

private:
  rclcpp::Node* node_;
  mjModel* model_;
  mjData* data_;

  rclcpp::Publisher<geometry_msgs::msg::WrenchStamped>::SharedPtr pub_contact_force_;
  rclcpp::Publisher<geometry_msgs::msg::WrenchStamped>::SharedPtr pub_contact_force_filt_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr pub_geometry_metrics_;
  rclcpp::TimerBase::SharedPtr timer_contact_;

  bool viz_contact_enable_{true};
  double viz_contact_scale_{4.5};
  double viz_contact_width_{0.008};
  int viz_contact_max_{64};

  bool contact_filter_enable_{true};
  double contact_cutoff_hz_{5.0};
  double contact_timer_hz_{100.0};
  bool use_exp_alpha_{true};

  int gid_tip_{-1};
  int tip_root_body_id_{-1};
  int gid_small_cylinder_{-1};
  int gid_large_cylinder_{-1};
  int gid_belt_right_{-1};
  int gid_belt_left_{-1};

  mutable std::mutex mtx_;

  std::array<double,3> f_raw_latest_{{0.0, 0.0, 0.0}};
  std::array<double,3> f_raw_prev_{{0.0, 0.0, 0.0}};
  std::array<double,3> f_filt_latest_{{0.0, 0.0, 0.0}};
  std::array<double,3> fdot_raw_latest_{{0.0, 0.0, 0.0}};
  std::array<double,3> fdot_filt_latest_{{0.0, 0.0, 0.0}};

  std::array<double,3> rf_{{0.0, 0.0, 0.0}};
  std::array<double,3> fw_{{0.0, 0.0, 0.0}};
  double fcn_{0.0};
  std::array<double,8> geometry_metrics_{{
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0}};

  rclcpp::Time last_contact_timer_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_contact_log_time_{0, 0, RCL_ROS_TIME};
};

}  // namespace mujoco_bridge
