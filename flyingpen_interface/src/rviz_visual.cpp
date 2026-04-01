#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/vector3_stamped.hpp>
#include <geometry_msgs/msg/wrench_stamped.hpp>
#include <geometry_msgs/msg/quaternion_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>

#include <std_msgs/msg/float32_multi_array.hpp>

#include <visualization_msgs/msg/marker.hpp>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>

#include <mutex>
#include <array>
#include <chrono>
#include <string>
#include <algorithm>

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
    parent_frame_     = this->declare_parameter<std::string>("parent_frame", "world");
    cf_frame_         = this->declare_parameter<std::string>("cf_frame", "crazyflie");
    cmd_drone_frame_  = this->declare_parameter<std::string>("cmd_drone_frame", "cmd_drone");
    cmd_ee_frame_     = this->declare_parameter<std::string>("cmd_ee_frame", "cmd_end_effector");
    cmd_active_frame_ = this->declare_parameter<std::string>("cmd_active_frame", "cmd_active");
    ee_frame_         = this->declare_parameter<std::string>("ee_frame", "end_effector");

    pose_topic_   = this->declare_parameter<std::string>("pose_topic", "/crazyflie/out/pose");
    vel_topic_    = this->declare_parameter<std::string>("vel_topic", "/crazyflie/out/vel");
    acc_topic_    = this->declare_parameter<std::string>("acc_topic", "/crazyflie/out/acc");

    ee_pose_topic_ = this->declare_parameter<std::string>("ee_pose_topic", "/crazyflie/out/EE_pose");
    ee_vel_topic_  = this->declare_parameter<std::string>("ee_vel_topic", "/crazyflie/out/EE_velocity");
    ee_acc_topic_  = this->declare_parameter<std::string>("ee_acc_topic", "/crazyflie/out/EE_acceleration");

    cmd_drone_topic_ = this->declare_parameter<std::string>(
      "cmd_drone_topic", "/crazyflie/debug/cmd_drone");
    cmd_ee_topic_ = this->declare_parameter<std::string>(
      "cmd_ee_topic", "/crazyflie/debug/cmd_ee");
    cmd_active_topic_ = this->declare_parameter<std::string>(
      "cmd_active_topic", "/crazyflie/debug/cmd_active");

    contact_force_topic_ = this->declare_parameter<std::string>(
      "contact_force_topic", "/crazyflie/out/EE_contact_force_filt");

    contact_frame_quat_topic_ = this->declare_parameter<std::string>(
      "contact_frame_quat_topic", "/estimated_contact_frame_quat");

    est_contact_frame_ = this->declare_parameter<std::string>(
      "est_contact_frame", "estimated_contact_frame");

    force_scale_         = this->declare_parameter<double>("force_scale", 10.0);
    contact_force_scale_ = this->declare_parameter<double>("contact_force_scale", 10.0);

    drone_vel_scale_ = this->declare_parameter<double>("drone_vel_scale", 1.0);
    drone_acc_scale_ = this->declare_parameter<double>("drone_acc_scale", 1.0);
    ee_vel_scale_    = this->declare_parameter<double>("ee_vel_scale", 1.0);
    ee_acc_scale_    = this->declare_parameter<double>("ee_acc_scale", 1.0);

    arrow_shaft_diam_ = this->declare_parameter<double>("arrow_shaft_diam", 0.01);
    arrow_head_diam_  = this->declare_parameter<double>("arrow_head_diam", 0.02);
    arrow_head_len_   = this->declare_parameter<double>("arrow_head_len", 0.04);

    publish_hz_ = this->declare_parameter<double>("publish_hz", 60.0);

    // -------------------------
    // TF broadcaster
    // -------------------------
    tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

    // -------------------------
    // Subscribers
    // -------------------------
    sub_pose_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
      pose_topic_, 10,
      std::bind(&RvizVisual::cb_pose, this, std::placeholders::_1));

    sub_vel_ = this->create_subscription<geometry_msgs::msg::Vector3Stamped>(
      vel_topic_, 10,
      std::bind(&RvizVisual::cb_vel, this, std::placeholders::_1));

    sub_acc_ = this->create_subscription<geometry_msgs::msg::Vector3Stamped>(
      acc_topic_, 10,
      std::bind(&RvizVisual::cb_acc, this, std::placeholders::_1));

    sub_ee_pose_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
      ee_pose_topic_, 10,
      std::bind(&RvizVisual::cb_ee_pose, this, std::placeholders::_1));

    sub_ee_vel_ = this->create_subscription<geometry_msgs::msg::Vector3Stamped>(
      ee_vel_topic_, 10,
      std::bind(&RvizVisual::cb_ee_vel, this, std::placeholders::_1));

    sub_ee_acc_ = this->create_subscription<geometry_msgs::msg::Vector3Stamped>(
      ee_acc_topic_, 10,
      std::bind(&RvizVisual::cb_ee_acc, this, std::placeholders::_1));

    sub_cmd_drone_pose_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
      cmd_drone_topic_, 10,
      std::bind(&RvizVisual::cb_cmd_drone_pose, this, std::placeholders::_1));

    sub_cmd_ee_pose_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
      cmd_ee_topic_, 10,
      std::bind(&RvizVisual::cb_cmd_ee_pose, this, std::placeholders::_1));

    sub_cmd_active_pose_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
      cmd_active_topic_, 10,
      std::bind(&RvizVisual::cb_cmd_active_pose, this, std::placeholders::_1));

    sub_mob_wrench_ = this->create_subscription<std_msgs::msg::Float32MultiArray>(
      "/crazyflie/out/mob_wrench", 10,
      std::bind(&RvizVisual::cb_mob_wrench, this, std::placeholders::_1));

    sub_contact_force_ = this->create_subscription<geometry_msgs::msg::WrenchStamped>(
      contact_force_topic_, 10,
      std::bind(&RvizVisual::cb_contact_force, this, std::placeholders::_1));

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

    pub_drone_vel_arrow_ = this->create_publisher<visualization_msgs::msg::Marker>(
      "/rviz/drone_velocity", 10);

    pub_drone_acc_arrow_ = this->create_publisher<visualization_msgs::msg::Marker>(
      "/rviz/drone_acceleration", 10);

    pub_ee_vel_arrow_ = this->create_publisher<visualization_msgs::msg::Marker>(
      "/rviz/EE_velocity", 10);

    pub_ee_acc_arrow_ = this->create_publisher<visualization_msgs::msg::Marker>(
      "/rviz/EE_acceleration", 10);

    // -------------------------
    // Timer
    // -------------------------
    const auto period = std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::duration<double>(1.0 / std::max(1e-6, publish_hz_)));

    timer_ = this->create_wall_timer(
      period, std::bind(&RvizVisual::loop_publish, this));

    RCLCPP_INFO(this->get_logger(), "rviz_visual started.");
    RCLCPP_INFO(this->get_logger(), "Sub drone: %s | %s | %s",
                pose_topic_.c_str(), vel_topic_.c_str(), acc_topic_.c_str());
    RCLCPP_INFO(this->get_logger(), "Sub EE: %s | %s | %s",
                ee_pose_topic_.c_str(), ee_vel_topic_.c_str(), ee_acc_topic_.c_str());
    RCLCPP_INFO(this->get_logger(), "Sub cmd pose: %s | %s | %s",
                cmd_drone_topic_.c_str(), cmd_ee_topic_.c_str(), cmd_active_topic_.c_str());
  }

private:
  // =========================
  // Callbacks
  // =========================
  void cb_pose(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lk(mtx_);
    pose_ = *msg;
    have_pose_ = true;
  }

  void cb_vel(const geometry_msgs::msg::Vector3Stamped::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lk(mtx_);
    vel_ = *msg;
    have_vel_ = true;
  }

  void cb_acc(const geometry_msgs::msg::Vector3Stamped::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lk(mtx_);
    acc_ = *msg;
    have_acc_ = true;
  }

  void cb_ee_pose(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lk(mtx_);
    ee_pose_ = *msg;
    have_ee_pose_ = true;
  }

  void cb_ee_vel(const geometry_msgs::msg::Vector3Stamped::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lk(mtx_);
    ee_vel_ = *msg;
    have_ee_vel_ = true;
  }

  void cb_ee_acc(const geometry_msgs::msg::Vector3Stamped::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lk(mtx_);
    ee_acc_ = *msg;
    have_ee_acc_ = true;
  }

  void cb_cmd_drone_pose(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lk(mtx_);
    cmd_drone_pose_ = *msg;
    have_cmd_drone_pose_ = true;
  }

  void cb_cmd_ee_pose(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lk(mtx_);
    cmd_ee_pose_ = *msg;
    have_cmd_ee_pose_ = true;
  }

  void cb_cmd_active_pose(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lk(mtx_);
    cmd_active_pose_ = *msg;
    have_cmd_active_pose_ = true;
  }

  void cb_mob_wrench(const std_msgs::msg::Float32MultiArray::SharedPtr msg)
  {
    if (msg->data.size() < 3) return;
    constexpr double mob_visual_scale = 500.0;
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

  void cb_contact_frame_quat(const geometry_msgs::msg::QuaternionStamped::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lk(mtx_);
    contact_q_ = msg->quaternion;
    have_contact_q_ = true;
  }

  // =========================
  // Marker helper
  // =========================
  visualization_msgs::msg::Marker make_arrow_marker(
    const std::string &ns,
    int id,
    const std::string &frame_id,
    const rclcpp::Time &stamp,
    double x0, double y0, double z0,
    double vx, double vy, double vz,
    double scale,
    double r, double g, double b) const
  {
    visualization_msgs::msg::Marker mk;
    mk.header.stamp = stamp;
    mk.header.frame_id = frame_id;
    mk.ns = ns;
    mk.id = id;
    mk.type = visualization_msgs::msg::Marker::ARROW;
    mk.action = visualization_msgs::msg::Marker::ADD;

    geometry_msgs::msg::Point p0, p1;
    p0.x = x0; p0.y = y0; p0.z = z0;
    p1.x = x0 + scale * vx;
    p1.y = y0 + scale * vy;
    p1.z = z0 + scale * vz;

    mk.points = {p0, p1};
    mk.scale.x = arrow_shaft_diam_;
    mk.scale.y = arrow_head_diam_;
    mk.scale.z = arrow_head_len_;

    mk.color.a = 1.0;
    mk.color.r = r;
    mk.color.g = g;
    mk.color.b = b;

    mk.lifetime = rclcpp::Duration::from_seconds(0.2);
    return mk;
  }

  // =========================
  // Timer loop
  // =========================
  void loop_publish()
  {
    geometry_msgs::msg::PoseStamped pose, ee_pose;
    geometry_msgs::msg::Vector3Stamped vel, acc, ee_vel, ee_acc;
    geometry_msgs::msg::PoseStamped cmd_drone_pose, cmd_ee_pose, cmd_active_pose;
    std::array<float, 3> Fext;
    std::array<float, 3> Fcontact;
    geometry_msgs::msg::Quaternion contact_q;

    bool have_pose, have_vel, have_acc;
    bool have_ee_pose, have_ee_vel, have_ee_acc;
    bool have_cmd_drone_pose, have_cmd_ee_pose, have_cmd_active_pose;
    bool have_mob, have_contact, have_contact_q;
    std::string contact_frame_id;

    {
      std::lock_guard<std::mutex> lk(mtx_);
      pose = pose_;
      vel = vel_;
      acc = acc_;
      ee_pose = ee_pose_;
      ee_vel = ee_vel_;
      ee_acc = ee_acc_;

      cmd_drone_pose = cmd_drone_pose_;
      cmd_ee_pose = cmd_ee_pose_;
      cmd_active_pose = cmd_active_pose_;

      Fext = mob_Fext_;
      Fcontact = contact_F_;
      contact_q = contact_q_;

      have_pose = have_pose_;
      have_vel = have_vel_;
      have_acc = have_acc_;
      have_ee_pose = have_ee_pose_;
      have_ee_vel = have_ee_vel_;
      have_ee_acc = have_ee_acc_;

      have_cmd_drone_pose = have_cmd_drone_pose_;
      have_cmd_ee_pose = have_cmd_ee_pose_;
      have_cmd_active_pose = have_cmd_active_pose_;

      have_mob = have_mob_;
      have_contact = have_contact_;
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

    // 2) TF: world -> cmd_drone
    if (have_cmd_drone_pose) {
      geometry_msgs::msg::TransformStamped tf;
      tf.header.stamp = stamp;
      tf.header.frame_id = parent_frame_;
      tf.child_frame_id = cmd_drone_frame_;
      tf.transform.translation.x = cmd_drone_pose.pose.position.x;
      tf.transform.translation.y = cmd_drone_pose.pose.position.y;
      tf.transform.translation.z = cmd_drone_pose.pose.position.z;
      tf.transform.rotation = cmd_drone_pose.pose.orientation;
      tf_broadcaster_->sendTransform(tf);
    }

    // 3) TF: world -> cmd_end_effector
    if (have_cmd_ee_pose) {
      geometry_msgs::msg::TransformStamped tf;
      tf.header.stamp = stamp;
      tf.header.frame_id = parent_frame_;
      tf.child_frame_id = cmd_ee_frame_;
      tf.transform.translation.x = cmd_ee_pose.pose.position.x;
      tf.transform.translation.y = cmd_ee_pose.pose.position.y;
      tf.transform.translation.z = cmd_ee_pose.pose.position.z;
      tf.transform.rotation = cmd_ee_pose.pose.orientation;
      tf_broadcaster_->sendTransform(tf);
    }

    // 4) TF: world -> cmd_active
    if (have_cmd_active_pose) {
      geometry_msgs::msg::TransformStamped tf;
      tf.header.stamp = stamp;
      tf.header.frame_id = parent_frame_;
      tf.child_frame_id = cmd_active_frame_;
      tf.transform.translation.x = cmd_active_pose.pose.position.x;
      tf.transform.translation.y = cmd_active_pose.pose.position.y;
      tf.transform.translation.z = cmd_active_pose.pose.position.z;
      tf.transform.rotation = cmd_active_pose.pose.orientation;
      tf_broadcaster_->sendTransform(tf);
    }

    // 5) TF: world -> end_effector
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

    // 6) TF: world -> estimated_contact_frame
    if (have_ee_pose && have_contact_q) {
      geometry_msgs::msg::TransformStamped tf;
      tf.header.stamp = stamp;
      tf.header.frame_id = parent_frame_;
      tf.child_frame_id = est_contact_frame_;
      tf.transform.translation.x = ee_pose.pose.position.x;
      tf.transform.translation.y = ee_pose.pose.position.y;
      tf.transform.translation.z = ee_pose.pose.position.z;

      tf2::Quaternion q_wc(contact_q.x, contact_q.y, contact_q.z, contact_q.w);
      q_wc.normalize();
      tf.transform.rotation.x = q_wc.x();
      tf.transform.rotation.y = q_wc.y();
      tf.transform.rotation.z = q_wc.z();
      tf.transform.rotation.w = q_wc.w();
      tf_broadcaster_->sendTransform(tf);
    }

    // 7) Marker: mob wrench
    if (have_pose && have_mob) {
      auto mk = make_arrow_marker(
        "mob_wrench", 0, parent_frame_, stamp,
        pose.pose.position.x, pose.pose.position.y, pose.pose.position.z,
        static_cast<double>(Fext[0]), static_cast<double>(Fext[1]), static_cast<double>(Fext[2]),
        force_scale_, 1.0, 0.3, 0.3);
      pub_mob_arrow_->publish(mk);
    }

    // 8) Marker: contact force
    if (have_ee_pose && have_contact) {
      auto mk = make_arrow_marker(
        "contact_force", 0, parent_frame_, stamp,
        ee_pose.pose.position.x, ee_pose.pose.position.y, ee_pose.pose.position.z,
        static_cast<double>(Fcontact[0]), static_cast<double>(Fcontact[1]), static_cast<double>(Fcontact[2]),
        contact_force_scale_, 0.3, 0.3, 1.0);
      pub_contact_arrow_->publish(mk);
    }

    // 9) Marker: drone velocity
    if (have_pose && have_vel) {
      auto mk = make_arrow_marker(
        "drone_velocity", 0, parent_frame_, stamp,
        pose.pose.position.x, pose.pose.position.y, pose.pose.position.z,
        vel.vector.x, vel.vector.y, vel.vector.z,
        drone_vel_scale_, 1.0, 1.0, 0.2);
      pub_drone_vel_arrow_->publish(mk);
    }

    // 10) Marker: drone acceleration
    if (have_pose && have_acc) {
      auto mk = make_arrow_marker(
        "drone_acceleration", 0, parent_frame_, stamp,
        pose.pose.position.x, pose.pose.position.y, pose.pose.position.z,
        acc.vector.x, acc.vector.y, acc.vector.z,
        drone_acc_scale_, 1.0, 0.5, 0.0);
      pub_drone_acc_arrow_->publish(mk);
    }

    // 11) Marker: EE velocity
    if (have_ee_pose && have_ee_vel) {
      auto mk = make_arrow_marker(
        "ee_velocity", 0, parent_frame_, stamp,
        ee_pose.pose.position.x, ee_pose.pose.position.y, ee_pose.pose.position.z,
        ee_vel.vector.x, ee_vel.vector.y, ee_vel.vector.z,
        ee_vel_scale_, 0.2, 1.0, 0.2);
      pub_ee_vel_arrow_->publish(mk);
    }

    // 12) Marker: EE acceleration
    if (have_ee_pose && have_ee_acc) {
      auto mk = make_arrow_marker(
        "ee_acceleration", 0, parent_frame_, stamp,
        ee_pose.pose.position.x, ee_pose.pose.position.y, ee_pose.pose.position.z,
        ee_acc.vector.x, ee_acc.vector.y, ee_acc.vector.z,
        ee_acc_scale_, 0.2, 1.0, 1.0);
      pub_ee_acc_arrow_->publish(mk);
    }
  }

private:
  // =========================
  // ROS
  // =========================
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr sub_pose_;
  rclcpp::Subscription<geometry_msgs::msg::Vector3Stamped>::SharedPtr sub_vel_;
  rclcpp::Subscription<geometry_msgs::msg::Vector3Stamped>::SharedPtr sub_acc_;

  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr sub_ee_pose_;
  rclcpp::Subscription<geometry_msgs::msg::Vector3Stamped>::SharedPtr sub_ee_vel_;
  rclcpp::Subscription<geometry_msgs::msg::Vector3Stamped>::SharedPtr sub_ee_acc_;

  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr sub_cmd_drone_pose_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr sub_cmd_ee_pose_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr sub_cmd_active_pose_;

  rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr sub_mob_wrench_;
  rclcpp::Subscription<geometry_msgs::msg::WrenchStamped>::SharedPtr sub_contact_force_;
  rclcpp::Subscription<geometry_msgs::msg::QuaternionStamped>::SharedPtr sub_contact_frame_quat_;

  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr pub_mob_arrow_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr pub_contact_arrow_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr pub_drone_vel_arrow_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr pub_drone_acc_arrow_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr pub_ee_vel_arrow_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr pub_ee_acc_arrow_;

  rclcpp::TimerBase::SharedPtr timer_;

  // =========================
  // State
  // =========================
  std::mutex mtx_;

  geometry_msgs::msg::PoseStamped pose_;
  geometry_msgs::msg::Vector3Stamped vel_;
  geometry_msgs::msg::Vector3Stamped acc_;
  bool have_pose_{false};
  bool have_vel_{false};
  bool have_acc_{false};

  geometry_msgs::msg::PoseStamped ee_pose_;
  geometry_msgs::msg::Vector3Stamped ee_vel_;
  geometry_msgs::msg::Vector3Stamped ee_acc_;
  bool have_ee_pose_{false};
  bool have_ee_vel_{false};
  bool have_ee_acc_{false};

  geometry_msgs::msg::PoseStamped cmd_drone_pose_;
  geometry_msgs::msg::PoseStamped cmd_ee_pose_;
  geometry_msgs::msg::PoseStamped cmd_active_pose_;
  bool have_cmd_drone_pose_{false};
  bool have_cmd_ee_pose_{false};
  bool have_cmd_active_pose_{false};

  std::array<float, 3> mob_Fext_{0.0f, 0.0f, 0.0f};
  bool have_mob_{false};

  std::array<float, 3> contact_F_{0.0f, 0.0f, 0.0f};
  bool have_contact_{false};
  std::string contact_frame_id_;

  geometry_msgs::msg::Quaternion contact_q_;
  bool have_contact_q_{false};

  // =========================
  // Params
  // =========================
  std::string parent_frame_;
  std::string cf_frame_;
  std::string cmd_drone_frame_;
  std::string cmd_ee_frame_;
  std::string cmd_active_frame_;
  std::string ee_frame_;

  std::string pose_topic_;
  std::string vel_topic_;
  std::string acc_topic_;

  std::string ee_pose_topic_;
  std::string ee_vel_topic_;
  std::string ee_acc_topic_;

  std::string cmd_drone_topic_;
  std::string cmd_ee_topic_;
  std::string cmd_active_topic_;

  std::string contact_force_topic_;
  std::string contact_frame_quat_topic_;
  std::string est_contact_frame_;

  double force_scale_{10.0};
  double contact_force_scale_{10.0};
  double drone_vel_scale_{1.0};
  double drone_acc_scale_{1.0};
  double ee_vel_scale_{1.0};
  double ee_acc_scale_{1.0};

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