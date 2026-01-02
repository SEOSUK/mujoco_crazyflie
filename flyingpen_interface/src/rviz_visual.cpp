// rviz_visual.cpp
#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/pose_stamped.hpp>
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

    // Force vector (N) -> arrow length (m) scale
    force_scale_  = this->declare_parameter<double>("force_scale", 0.05);

    // Marker appearance
    arrow_shaft_diam_ = this->declare_parameter<double>("arrow_shaft_diam", 0.01);
    arrow_head_diam_  = this->declare_parameter<double>("arrow_head_diam", 0.02);
    arrow_head_len_   = this->declare_parameter<double>("arrow_head_len",  0.04);

    publish_hz_ = this->declare_parameter<double>("publish_hz", 60.0);

    // -------------------------
    // ✅ Mesh visual params
    // -------------------------
    // STL only is fine for now
    mesh_resource_ = this->declare_parameter<std::string>(
      "mesh_resource",
      "package://plant/data/assets/cf21B/cf21B_full.stl"
    );
    mesh_scale_ = this->declare_parameter<double>("mesh_scale", 1.0);
    mesh_alpha_ = this->declare_parameter<double>("mesh_alpha", 1.0);

    // -------------------------
    // TF broadcaster
    // -------------------------
    tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

    // -------------------------
    // Subscribers
    // -------------------------
    // (sub) /crazyflie/out/pose : PoseStamped (attitude feedback)
    sub_pose_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
      "/crazyflie/out/pose", 10,
      std::bind(&RvizVisual::cb_pose, this, std::placeholders::_1));

    // (sub) /crazyflie/in/pos_cmd : Float64MultiArray [x,y,z,yaw]
    sub_pos_cmd_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
      "/crazyflie/in/pos_cmd", 10,
      std::bind(&RvizVisual::cb_pos_cmd, this, std::placeholders::_1));

    // (sub) /crazyflie/out/mob_wrench : Float32MultiArray [Fx,Fy,Fz] in world frame
    sub_mob_wrench_ = this->create_subscription<std_msgs::msg::Float32MultiArray>(
      "/crazyflie/out/mob_wrench", 10,
      std::bind(&RvizVisual::cb_mob_wrench, this, std::placeholders::_1));

    // -------------------------
    // Publisher (Marker)
    // -------------------------
    // (pub) /rviz/mob_Fext : Marker arrow
    pub_mob_arrow_ = this->create_publisher<visualization_msgs::msg::Marker>(
      "/rviz/mob_Fext", 10);

    // ✅ (pub) /rviz/crazyflie_mesh : Marker mesh
    pub_cf_mesh_ = this->create_publisher<visualization_msgs::msg::Marker>(
      "/rviz/crazyflie_mesh", 10);

    // -------------------------
    // Timer: publish TFs + marker at fixed rate
    // -------------------------
    const auto period = std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::duration<double>(1.0 / std::max(1e-6, publish_hz_))
    );

    timer_ = this->create_wall_timer(
      period, std::bind(&RvizVisual::loop_publish, this));

    RCLCPP_INFO(this->get_logger(), "rviz_visual started.");
    RCLCPP_INFO(this->get_logger(), "mesh_resource: %s", mesh_resource_.c_str());
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

  void cb_pos_cmd(const std_msgs::msg::Float64MultiArray::SharedPtr msg)
  {
    if (msg->data.size() < 4) return;

    std::lock_guard<std::mutex> lk(mtx_);
    cmd_pos_[0] = msg->data[0];
    cmd_pos_[1] = msg->data[1];
    cmd_pos_[2] = msg->data[2];
    cmd_yaw_    = msg->data[3];  // (assumed rad; if deg, convert here)
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

  // =========================
  // Timer loop
  // =========================
  void loop_publish()
  {
    // snapshot
    geometry_msgs::msg::PoseStamped pose;
    std::array<double, 3> cmd_pos;
    double cmd_yaw;
    std::array<float, 3> Fext;
    bool have_pose, have_cmd, have_mob;

    {
      std::lock_guard<std::mutex> lk(mtx_);
      pose = pose_;
      cmd_pos = cmd_pos_;
      cmd_yaw = cmd_yaw_;
      Fext = mob_Fext_;
      have_pose = have_pose_;
      have_cmd = have_cmd_;
      have_mob = have_mob_;
    }

    const auto stamp = this->now();

    // 1) TF: world -> crazyflie (from pose feedback)
    if (have_pose) {
      geometry_msgs::msg::TransformStamped tf;
      tf.header.stamp = stamp;
      tf.header.frame_id = parent_frame_;
      tf.child_frame_id = cf_frame_;

      tf.transform.translation.x = pose.pose.position.x;
      tf.transform.translation.y = pose.pose.position.y;
      tf.transform.translation.z = pose.pose.position.z;

      tf.transform.rotation = pose.pose.orientation; // 그대로 사용 (x,y,z,w)
      tf_broadcaster_->sendTransform(tf);
    }

    // 2) TF: world -> cmd_position (from pos_cmd)
    if (have_cmd) {
      geometry_msgs::msg::TransformStamped tf;
      tf.header.stamp = stamp;
      tf.header.frame_id = parent_frame_;
      tf.child_frame_id = cmd_frame_;

      tf.transform.translation.x = cmd_pos[0];
      tf.transform.translation.y = cmd_pos[1];
      tf.transform.translation.z = cmd_pos[2];

      // yaw only
      tf2::Quaternion q;
      q.setRPY(0.0, 0.0, cmd_yaw);
      tf.transform.rotation.x = q.x();
      tf.transform.rotation.y = q.y();
      tf.transform.rotation.z = q.z();
      tf.transform.rotation.w = q.w();

      tf_broadcaster_->sendTransform(tf);
    }

    // ✅ 2.5) Mesh marker: cf21B_full.stl anchored at crazyflie frame
    //     - Put marker in cf_frame_ and identity pose so TF drives it.
    if (have_pose) {
      visualization_msgs::msg::Marker mesh;
      mesh.header.stamp = stamp;
      mesh.header.frame_id = cf_frame_;
      mesh.ns = "cf_mesh";
      mesh.id = 0;
      mesh.type = visualization_msgs::msg::Marker::MESH_RESOURCE;
      mesh.action = visualization_msgs::msg::Marker::ADD;

      mesh.mesh_resource = mesh_resource_;
      mesh.mesh_use_embedded_materials = false;

      // Identity pose in cf frame
      mesh.pose.position.x = 0.0;
      mesh.pose.position.y = 0.0;
      mesh.pose.position.z = 0.0;
      mesh.pose.orientation.x = 0.0;
      mesh.pose.orientation.y = 0.0;
      mesh.pose.orientation.z = 0.0;
      mesh.pose.orientation.w = 1.0;

      mesh.scale.x = mesh_scale_;
      mesh.scale.y = mesh_scale_;
      mesh.scale.z = mesh_scale_;

      // visible color
      mesh.color.a = static_cast<float>(std::max(0.0, std::min(1.0, mesh_alpha_)));
      mesh.color.r = 0.8f;
      mesh.color.g = 0.8f;
      mesh.color.b = 0.8f;

      // keep alive
      mesh.lifetime = rclcpp::Duration::from_seconds(0.0);

      pub_cf_mesh_->publish(mesh);
    }

    // 3) Marker Arrow: /rviz/mob_Fext
    //    start = current pose position, end = start + force_scale * Fext
    if (have_pose && have_mob) {
      visualization_msgs::msg::Marker mk;
      mk.header.stamp = stamp;
      mk.header.frame_id = parent_frame_;
      mk.ns = "mob_wrench";
      mk.id = 0;
      mk.type = visualization_msgs::msg::Marker::ARROW;
      mk.action = visualization_msgs::msg::Marker::ADD;

      // Arrow points (start/end)
      geometry_msgs::msg::Point p0, p1;
      p0.x = pose.pose.position.x;
      p0.y = pose.pose.position.y;
      p0.z = pose.pose.position.z;

      p1.x = p0.x + force_scale_ * static_cast<double>(Fext[0]);
      p1.y = p0.y + force_scale_ * static_cast<double>(Fext[1]);
      p1.z = p0.z + force_scale_ * static_cast<double>(Fext[2]);

      mk.points.clear();
      mk.points.push_back(p0);
      mk.points.push_back(p1);

      // Marker scale for ARROW when using points:
      // scale.x = shaft diameter, scale.y = head diameter, scale.z = head length
      mk.scale.x = arrow_shaft_diam_;
      mk.scale.y = arrow_head_diam_;
      mk.scale.z = arrow_head_len_;

      // color
      mk.color.a = 1.0;
      mk.color.r = 1.0;
      mk.color.g = 0.3;
      mk.color.b = 0.3;

      // lifetime (0 = forever; 짧게 하면 끊김 없이 갱신되는 느낌)
      mk.lifetime = rclcpp::Duration::from_seconds(0.2);

      pub_mob_arrow_->publish(mk);
    }
  }

private:
  // =========================
  // ROS
  // =========================
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr sub_pose_;
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr sub_pos_cmd_;
  rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr sub_mob_wrench_;

  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr pub_mob_arrow_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr pub_cf_mesh_;
  rclcpp::TimerBase::SharedPtr timer_;

  // =========================
  // State
  // =========================
  std::mutex mtx_;

  geometry_msgs::msg::PoseStamped pose_;
  bool have_pose_{false};

  std::array<double, 3> cmd_pos_{0.0, 0.0, 0.0};
  double cmd_yaw_{0.0};
  bool have_cmd_{false};

  std::array<float, 3> mob_Fext_{0.0f, 0.0f, 0.0f};
  bool have_mob_{false};

  // =========================
  // Params
  // =========================
  std::string parent_frame_;
  std::string cf_frame_;
  std::string cmd_frame_;

  double force_scale_{0.05};
  double arrow_shaft_diam_{0.01};
  double arrow_head_diam_{0.02};
  double arrow_head_len_{0.04};
  double publish_hz_{60.0};

  // ✅ mesh params
  std::string mesh_resource_;
  double mesh_scale_{1.0};
  double mesh_alpha_{1.0};
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<RvizVisual>());
  rclcpp::shutdown();
  return 0;
}
