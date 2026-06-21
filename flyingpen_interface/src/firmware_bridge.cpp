#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/quaternion_stamped.hpp>
#include <geometry_msgs/msg/vector3_stamped.hpp>
#include <geometry_msgs/msg/wrench_stamped.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>

#include <functional>
#include <string>
#include <vector>

class FirmwareBridge : public rclcpp::Node
{
public:
  FirmwareBridge()
  : Node("firmware_bridge")
  {
    const auto qos = rclcpp::SensorDataQoS();

    add_passthrough_subscription<geometry_msgs::msg::PoseStamped>(
      "/crazyflie/out/pose", "/cf2/pose", qos);
    add_passthrough_subscription<geometry_msgs::msg::PoseStamped>(
      "/crazyflie/out/EE_pose", "/cf2/ee_pose", qos);
    add_passthrough_subscription<geometry_msgs::msg::Vector3Stamped>(
      "/crazyflie/out/vel", "/cf2/stateEstimate_velocity", qos);
    add_passthrough_subscription<geometry_msgs::msg::Vector3Stamped>(
      "/crazyflie/out/acc", "/cf2/stateEstimate_acc", qos);
    add_passthrough_subscription<geometry_msgs::msg::Vector3Stamped>(
      "/crazyflie/out/ang_vel", "/cf2/gyro_feedback", qos);
    add_passthrough_subscription<geometry_msgs::msg::PoseStamped>(
      "/crazyflie/debug/cmd_active", "/cf2/cf_ctrl_target_pos", qos);
    add_passthrough_subscription<geometry_msgs::msg::WrenchStamped>(
      "/crazyflie/out/mob_2nd", "/cf2/cf_Fext_MOB_pure", qos);
    add_passthrough_subscription<geometry_msgs::msg::WrenchStamped>(
      "/crazyflie/out/mob_2nd_tau", "/cf2/cf_Fext_MOB", qos);
    add_passthrough_subscription<geometry_msgs::msg::WrenchStamped>(
      "/crazyflie/out/mob_2nd_tau_consistency", "/cf2/cf_MOB_consistency", qos);
    add_passthrough_subscription<geometry_msgs::msg::QuaternionStamped>(
      "/estimated_contact_frame_quat", "/cf2/contact_frame_quat", qos);
    add_passthrough_subscription<std_msgs::msg::Float32>(
      "/su/contact_force_x", "/cf2/preload_feedback", qos);
    add_passthrough_subscription<std_msgs::msg::Float64MultiArray>(
      "/normal_vector/debug_metrics", "/cf2/normal_debug_metrics", qos);
    add_passthrough_subscription<std_msgs::msg::Float64MultiArray>(
      "/data_logging_msgs", "/data_logging_msg_debug", qos);

    RCLCPP_INFO(
      get_logger(),
      "firmware_bridge started with firmware-style aliases under /cf2 and /data_logging_msg_debug");
  }

private:
  template<typename MsgT>
  void add_passthrough_subscription(
    const std::string & input_topic,
    const std::string & output_topic,
    const rclcpp::QoS & qos)
  {
    auto pub = create_publisher<MsgT>(output_topic, qos);
    auto sub = create_subscription<MsgT>(
      input_topic,
      qos,
      [pub](const typename MsgT::SharedPtr msg) {
        pub->publish(*msg);
      });

    passthrough_publishers_.push_back(pub);
    passthrough_subscriptions_.push_back(sub);
  }

  std::vector<rclcpp::PublisherBase::SharedPtr> passthrough_publishers_;
  std::vector<rclcpp::SubscriptionBase::SharedPtr> passthrough_subscriptions_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<FirmwareBridge>());
  rclcpp::shutdown();
  return 0;
}
