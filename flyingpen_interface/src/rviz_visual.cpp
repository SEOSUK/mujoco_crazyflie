// rviz_visual.cpp
#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/vector3_stamped.hpp>   // ✅ EE vel
#include <geometry_msgs/msg/wrench_stamped.hpp>
#include <geometry_msgs/msg/quaternion_stamped.hpp> // ✅ contact frame quat
#include <std_msgs/msg/float64_multi_array.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>

#include <visualization_msgs/msg/marker.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>

#include <mutex>
#include <array>
#include <chrono>
#include <cmath>
#include <string>

using namespace std::chrono_literals;

class RvizVisual : public rclcpp::Node
{
public:
  RvizVisual()
  : Node("rviz_visual")
  {
    // -------------------------
    // Parameters
    // -------------------------
    parent_frame_ = this->declare_parameter<std::string>("parent_frame", "world");
    cf_frame_     = this->declare_parameter<std::string>("cf_frame", "crazyflie");
    cmd_frame_    = this->declare_parameter<std::string>("cmd_frame", "cmd_position");

    // ✅ EE frames/topics
    ee_frame_ = this->declare_parameter<std::string>("ee_frame", "end_effector");
    ee_pose_topic_ = this->declare_parameter<std::string>("ee_pose_topic", "/crazyflie/out/EE_pose");
    ee_vel_topic_  = this->declare_parameter<std::string>("ee_vel_topic",  "/crazyflie/out/EE_velocity");

    // ✅ estimated contact frame quat topic
    contact_frame_quat_topic_ = this->declare_parameter<std::string>(
      "contact_frame_quat_topic", "/estimated_contact_frame_quat");

    // ✅ TF child frame name for estimated contact frame
    est_contact_frame_ = this->declare_parameter<std::string>(
      "est_contact_frame", "estimated_contact_frame");

    // Force vector (N) -> arrow length (m) scale
    force_scale_  = this->declare_parameter<double>("force_scale", 10.);

    // contact force arrow scale
    contact_force_scale_ = this->declare_parameter<double>("contact_force_scale", 10.);

    // ✅ EE velocity arrow scale
    ee_vel_scale_ = this->declare_parameter<double>("ee_vel_scale", 1.0);

    // Marker appearance
    arrow_shaft_diam_ = this->declare_parameter<double>("arrow_shaft_diam", 0.01);
    arrow_head_diam_  = this->declare_parameter<double>("arrow_head_diam", 0.02);
    arrow_head_len_   = this->declare_parameter<double>("arrow_head_len",  0.04);

    publish_hz_ = this->declare_parameter<double>("publish_hz", 60.0);

    // -------------------------
    // TF broadcaster
    // -------------------------
    tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

    // -------------------------
    // Subscribers
    // -------------------------
    sub_pose_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
      "/crazyflie/out/pose", 10,
      std::bind(&RvizVisual::cb_pose, this, std::placeholders::_1));

    // ✅ EE pose/vel from fk_ik_transform
    sub_ee_pose_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
      ee_pose_topic_, 10,
      std::bind(&RvizVisual::cb_ee_pose, this, std::placeholders::_1));

    sub_ee_vel_ = this->create_subscription<geometry_msgs::msg::Vector3Stamped>(
      ee_vel_topic_, 10,
      std::bind(&RvizVisual::cb_ee_vel, this, std::placeholders::_1));

    sub_pos_cmd_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
      "/crazyflie/in/pos_cmd", 10,
      std::bind(&RvizVisual::cb_pos_cmd, this, std::placeholders::_1));

    sub_mob_wrench_ = this->create_subscription<std_msgs::msg::Float32MultiArray>(
      "/crazyflie/out/mob_wrench", 10,
      std::bind(&RvizVisual::cb_mob_wrench, this, std::placeholders::_1));

    sub_contact_force_ = this->create_subscription<geometry_msgs::msg::WrenchStamped>(
      "/crazyflie/out/EE_contact_force", 10,
      std::bind(&RvizVisual::cb_contact_force, this, std::placeholders::_1));

    // ✅ estimated contact frame quaternion (R_C as quat)
    sub_contact_frame_quat_ = this->create_subscription<geometry_msgs::msg::QuaternionStamped>(
      contact_frame_quat_topic_, 10,
      std::bind(&RvizVisual::cb_contact_frame_quat, this, std::placeholders::_1));

    // -------------------------
    // Publishers (Markers)
    // -------------------------
    pub_mob_arrow_ = this->create_publisher<visualization_msgs::msg::Marker>(
      "/rviz/mob_Fext", 10);

    pub_contact_arrow_ = this->create_publisher<visualization_msgs::msg::Marker>(
      "/rviz/EE_contact_force", 10);

    // ✅ EE velocity arrow marker
    pub_ee_vel_arrow_ = this->create_publisher<visualization_msgs::msg::Marker>(
      "/rviz/EE_velocity", 10);

    // -------------------------
    // Timer
    // -------------------------
    const auto period = std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::duration<double>(1.0 / std::max(1e-6, publish_hz_))
    );

    timer_ = this->create_wall_timer(
      period, std::bind(&RvizVisual::loop_publish, this));

    RCLCPP_INFO(this->get_logger(), "rviz_visual started.");
    RCLCPP_INFO(this->get_logger(), "Sub EE pose/vel: %s , %s",
                ee_pose_topic_.c_str(), ee_vel_topic_.c_str());
    RCLCPP_INFO(this->get_logger(), "Sub contact frame quat: %s",
                contact_frame_quat_topic_.c_str());
  }

private:
  // =========================
  // Callbacks (store only)
  // =========================
  void cb_pose(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lk(mtx_);
    pose_ = *msg;
    have_pose_ = true;
  }

  // ✅ EE pose
  void cb_ee_pose(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lk(mtx_);
    ee_pose_ = *msg;
    have_ee_pose_ = true;
  }

  // ✅ EE velocity
  void cb_ee_vel(const geometry_msgs::msg::Vector3Stamped::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lk(mtx_);
    ee_vel_ = *msg;
    have_ee_vel_ = true;
  }

  void cb_pos_cmd(const std_msgs::msg::Float64MultiArray::SharedPtr msg)
  {
    if (msg->data.size() < 4) return;

    std::lock_guard<std::mutex> lk(mtx_);
    cmd_pos_[0] = msg->data[0];
    cmd_pos_[1] = msg->data[1];
    cmd_pos_[2] = msg->data[2];
    cmd_yaw_    = msg->data[3];
    have_cmd_ = true;
  }

  void cb_mob_wrench(const std_msgs::msg::Float32MultiArray::SharedPtr msg)
  {
    if (msg->data.size() < 3) return;

    double mob_visual_scale = 500.0; // 좀 보이게 키워놓음
    std::lock_guard<std::mutex> lk(mtx_);
    mob_Fext_[0] = mob_visual_scale * msg->data[0];
    mob_Fext_[1] = mob_visual_scale * msg->data[1];
    mob_Fext_[2] = mob_visual_scale * msg->data[2];
    have_mob_ = true;
  }

  void cb_contact_force(const geometry_msgs::msg::WrenchStamped::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lk(mtx_);
    contact_F_[0] = static_cast<float>(msg->wrench.force.x);
    contact_F_[1] = static_cast<float>(msg->wrench.force.y);
    contact_F_[2] = static_cast<float>(msg->wrench.force.z);
    have_contact_ = true;
    contact_frame_id_ = msg->header.frame_id;
  }

  // ✅ estimated contact frame quaternion
  void cb_contact_frame_quat(const geometry_msgs::msg::QuaternionStamped::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lk(mtx_);
    contact_q_ = msg->quaternion;
    have_contact_q_ = true;
  }

  // =========================
  // Timer loop
  // =========================
  void loop_publish()
  {
    // snapshot
    geometry_msgs::msg::PoseStamped pose, ee_pose;
    geometry_msgs::msg::Vector3Stamped ee_vel;
    std::array<double, 3> cmd_pos;
    double cmd_yaw;
    std::array<float, 3> Fext;
    std::array<float, 3> Fcontact;
    geometry_msgs::msg::Quaternion contact_q;

    bool have_pose, have_cmd, have_mob, have_contact;
    bool have_ee_pose, have_ee_vel;
    bool have_contact_q;
    std::string contact_frame_id;

    {
      std::lock_guard<std::mutex> lk(mtx_);
      pose = pose_;
      ee_pose = ee_pose_;
      ee_vel = ee_vel_;

      cmd_pos = cmd_pos_;
      cmd_yaw = cmd_yaw_;

      Fext = mob_Fext_;
      Fcontact = contact_F_;

      contact_q = contact_q_;

      have_pose = have_pose_;
      have_cmd = have_cmd_;
      have_mob = have_mob_;
      have_contact = have_contact_;

      have_ee_pose = have_ee_pose_;
      have_ee_vel  = have_ee_vel_;

      have_contact_q = have_contact_q_;

      contact_frame_id = contact_frame_id_;
    }

    const auto stamp = this->now();

    // 1) TF: world -> crazyflie
    if (have_pose) {
      geometry_msgs::msg::TransformStamped tf;
      tf.header.stamp = stamp;
      tf.header.frame_id = parent_frame_;
      tf.child_frame_id = cf_frame_;

      tf.transform.translation.x = pose.pose.position.x;
      tf.transform.translation.y = pose.pose.position.y;
      tf.transform.translation.z = pose.pose.position.z;
      tf.transform.rotation = pose.pose.orientation;
      tf_broadcaster_->sendTransform(tf);
    }

    // 2) TF: world -> cmd_position
    if (have_cmd) {
      geometry_msgs::msg::TransformStamped tf;
      tf.header.stamp = stamp;
      tf.header.frame_id = parent_frame_;
      tf.child_frame_id = cmd_frame_;

      tf.transform.translation.x = cmd_pos[0];
      tf.transform.translation.y = cmd_pos[1];
      tf.transform.translation.z = cmd_pos[2];

      tf2::Quaternion q;
      q.setRPY(0.0, 0.0, cmd_yaw);
      tf.transform.rotation.x = q.x();
      tf.transform.rotation.y = q.y();
      tf.transform.rotation.z = q.z();
      tf.transform.rotation.w = q.w();
      tf_broadcaster_->sendTransform(tf);
    }

    // 3) TF: world -> end_effector
    if (have_ee_pose) {
      geometry_msgs::msg::TransformStamped tf;
      tf.header.stamp = stamp;
      tf.header.frame_id = parent_frame_;
      tf.child_frame_id = ee_frame_;

      tf.transform.translation.x = ee_pose.pose.position.x;
      tf.transform.translation.y = ee_pose.pose.position.y;
      tf.transform.translation.z = ee_pose.pose.position.z;
      tf.transform.rotation = ee_pose.pose.orientation;
      tf_broadcaster_->sendTransform(tf);
    }

    // 4) TF: world -> estimated_contact_frame
    if (have_ee_pose && have_contact_q) {
      geometry_msgs::msg::TransformStamped tf;
      tf.header.stamp = stamp;
      tf.header.frame_id = parent_frame_;        // world
      tf.child_frame_id  = est_contact_frame_;   // estimated_contact_frame

      // origin at EE position (so the axes appear at EE)
      tf.transform.translation.x = ee_pose.pose.position.x;
      tf.transform.translation.y = ee_pose.pose.position.y;
      tf.transform.translation.z = ee_pose.pose.position.z;

      // contact_q is contact->world (R_C), so world->contact is inverse
      tf2::Quaternion q_wc(contact_q.x, contact_q.y, contact_q.z, contact_q.w);
      q_wc.normalize();
      // tf2::Quaternion q_cw = q_wc.inverse();     // world->contact

      tf.transform.rotation.x = q_wc.x();
      tf.transform.rotation.y = q_wc.y();
      tf.transform.rotation.z = q_wc.z();
      tf.transform.rotation.w = q_wc.w();

      tf_broadcaster_->sendTransform(tf);
    }




    // 5) Marker Arrow: mob wrench (origin=drone position)
    if (have_pose && have_mob) {
      visualization_msgs::msg::Marker mk;
      mk.header.stamp = stamp;
      mk.header.frame_id = parent_frame_;
      mk.ns = "mob_wrench";
      mk.id = 0;
      mk.type = visualization_msgs::msg::Marker::ARROW;
      mk.action = visualization_msgs::msg::Marker::ADD;

      geometry_msgs::msg::Point p0, p1;
      p0.x = pose.pose.position.x;
      p0.y = pose.pose.position.y;
      p0.z = pose.pose.position.z;

      p1.x = p0.x + force_scale_ * static_cast<double>(Fext[0]);
      p1.y = p0.y + force_scale_ * static_cast<double>(Fext[1]);
      p1.z = p0.z + force_scale_ * static_cast<double>(Fext[2]);

      mk.points = {p0, p1};
      mk.scale.x = arrow_shaft_diam_;
      mk.scale.y = arrow_head_diam_;
      mk.scale.z = arrow_head_len_;

      mk.color.a = 1.0;
      mk.color.r = 1.0;
      mk.color.g = 0.3;
      mk.color.b = 0.3;

      mk.lifetime = rclcpp::Duration::from_seconds(0.2);
      pub_mob_arrow_->publish(mk);
    }

    // 6) Marker Arrow: contact force (origin=EE position)
    if (have_ee_pose && have_contact) {
      visualization_msgs::msg::Marker mk;
      mk.header.stamp = stamp;
      mk.header.frame_id = parent_frame_;

      mk.ns = "contact_force";
      mk.id = 0;
      mk.type = visualization_msgs::msg::Marker::ARROW;
      mk.action = visualization_msgs::msg::Marker::ADD;

      geometry_msgs::msg::Point p0, p1;
      p0.x = ee_pose.pose.position.x;
      p0.y = ee_pose.pose.position.y;
      p0.z = ee_pose.pose.position.z;

      p1.x = p0.x + contact_force_scale_ * static_cast<double>(Fcontact[0]);
      p1.y = p0.y + contact_force_scale_ * static_cast<double>(Fcontact[1]);
      p1.z = p0.z + contact_force_scale_ * static_cast<double>(Fcontact[2]);

      mk.points = {p0, p1};
      mk.scale.x = arrow_shaft_diam_;
      mk.scale.y = arrow_head_diam_;
      mk.scale.z = arrow_head_len_;

      mk.color.a = 1.0;
      mk.color.r = 0.3;
      mk.color.g = 0.3;
      mk.color.b = 1.0;

      mk.lifetime = rclcpp::Duration::from_seconds(0.2);
      pub_contact_arrow_->publish(mk);
    }

    // 7) EE velocity arrow marker (origin=EE position)
    if (have_ee_pose && have_ee_vel) {
      visualization_msgs::msg::Marker mk;
      mk.header.stamp = stamp;
      mk.header.frame_id = parent_frame_;

      mk.ns = "ee_velocity";
      mk.id = 0;
      mk.type = visualization_msgs::msg::Marker::ARROW;
      mk.action = visualization_msgs::msg::Marker::ADD;

      geometry_msgs::msg::Point p0, p1;
      p0.x = ee_pose.pose.position.x;
      p0.y = ee_pose.pose.position.y;
      p0.z = ee_pose.pose.position.z;

      p1.x = p0.x + ee_vel_scale_ * static_cast<double>(ee_vel.vector.x);
      p1.y = p0.y + ee_vel_scale_ * static_cast<double>(ee_vel.vector.y);
      p1.z = p0.z + ee_vel_scale_ * static_cast<double>(ee_vel.vector.z);

      mk.points = {p0, p1};
      mk.scale.x = arrow_shaft_diam_;
      mk.scale.y = arrow_head_diam_;
      mk.scale.z = arrow_head_len_;

      mk.color.a = 1.0;
      mk.color.r = 0.2;
      mk.color.g = 1.0;
      mk.color.b = 0.2;

      mk.lifetime = rclcpp::Duration::from_seconds(0.2);
      pub_ee_vel_arrow_->publish(mk);
    }
  }

private:
  // =========================
  // ROS
  // =========================
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr sub_pose_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr sub_ee_pose_;
  rclcpp::Subscription<geometry_msgs::msg::Vector3Stamped>::SharedPtr sub_ee_vel_;
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr sub_pos_cmd_;
  rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr sub_mob_wrench_;
  rclcpp::Subscription<geometry_msgs::msg::WrenchStamped>::SharedPtr sub_contact_force_;

  // ✅ new: estimated contact frame quaternion
  rclcpp::Subscription<geometry_msgs::msg::QuaternionStamped>::SharedPtr sub_contact_frame_quat_;

  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr pub_mob_arrow_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr pub_contact_arrow_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr pub_ee_vel_arrow_;

  rclcpp::TimerBase::SharedPtr timer_;

  // =========================
  // State
  // =========================
  std::mutex mtx_;

  geometry_msgs::msg::PoseStamped pose_;
  bool have_pose_{false};

  geometry_msgs::msg::PoseStamped ee_pose_;
  geometry_msgs::msg::Vector3Stamped ee_vel_;
  bool have_ee_pose_{false};
  bool have_ee_vel_{false};

  std::array<double, 3> cmd_pos_{0.0, 0.0, 0.0};
  double cmd_yaw_{0.0};
  bool have_cmd_{false};

  std::array<float, 3> mob_Fext_{0.0f, 0.0f, 0.0f};
  bool have_mob_{false};

  std::array<float, 3> contact_F_{0.0f, 0.0f, 0.0f};
  bool have_contact_{false};
  std::string contact_frame_id_;

  // ✅ new: estimated contact frame quaternion
  geometry_msgs::msg::Quaternion contact_q_;
  bool have_contact_q_{false};

  // =========================
  // Params
  // =========================
  std::string parent_frame_;
  std::string cf_frame_;
  std::string cmd_frame_;

  std::string ee_frame_;
  std::string ee_pose_topic_;
  std::string ee_vel_topic_;

  // ✅ new params
  std::string contact_frame_quat_topic_;
  std::string est_contact_frame_;

  double force_scale_{10.};
  double contact_force_scale_{10.};
  double ee_vel_scale_{1.0};

  double arrow_shaft_diam_{0.01};
  double arrow_head_diam_{0.02};
  double arrow_head_len_{0.04};
  double publish_hz_{60.0};
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<RvizVisual>());
  rclcpp::shutdown();
  return 0;
}
