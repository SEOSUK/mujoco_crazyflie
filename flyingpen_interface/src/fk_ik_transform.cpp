// fk_ik_transform.cpp
#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/vector3_stamped.hpp>
#include <geometry_msgs/msg/wrench_stamped.hpp>

#include <Eigen/Dense>

#include <mutex>
#include <optional>
#include <string>
#include <chrono>

class FkIkTransformNode : public rclcpp::Node
{
public:
  FkIkTransformNode()
  : Node("fk_ik_transform")
  {
    // ---- params ----
    pose_topic_   = this->declare_parameter<std::string>("pose_topic", "/crazyflie/out/pose");
    vel_topic_    = this->declare_parameter<std::string>("vel_topic", "/crazyflie/out/vel");
    angvel_topic_ = this->declare_parameter<std::string>("angvel_topic", "/crazyflie/out/ang_vel");
    cf_topic_     = this->declare_parameter<std::string>("contact_force_topic", "/crazyflie/out/EE_contact_force");
    cf_f_topic_   = this->declare_parameter<std::string>("contact_force_filt_topic", "/crazyflie/out/EE_contact_force_filt");

    ee_pose_topic_ = this->declare_parameter<std::string>("ee_pose_topic", "/crazyflie/out/EE_pose");
    ee_vel_topic_  = this->declare_parameter<std::string>("ee_vel_topic",  "/crazyflie/out/EE_velocity");

    // end-effector static offset (body frame) [m]
    // 예: pen tip이 body 원점에서 +x로 0.10m, +z로 -0.02m 등
    auto ee_off = this->declare_parameter<std::vector<double>>("end_effector_offset", {0.09, 0.0, 0.085});
    if (ee_off.size() != 3) {
      RCLCPP_WARN(this->get_logger(), "end_effector_offset must be size 3. Fallback to [0,0,0].");
      ee_off = {0.0, 0.0, 0.0};
    }
    end_effector_offset = Eigen::Vector3d(ee_off[0], ee_off[1], ee_off[2]);

    double compute_hz = this->declare_parameter<double>("compute_hz", 200.0);
    double print_hz   = this->declare_parameter<double>("debug_print_hz", 10.0);

    // Sensor QoS 권장 (plant 쪽이 sensor_data QoS로 쏘는 경우 잘 맞음)
    auto qos = rclcpp::SensorDataQoS();

    sub_pose_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
      pose_topic_, qos,
      std::bind(&FkIkTransformNode::cbPose, this, std::placeholders::_1));

    sub_vel_ = this->create_subscription<geometry_msgs::msg::Vector3Stamped>(
      vel_topic_, qos,
      std::bind(&FkIkTransformNode::cbVel, this, std::placeholders::_1));

    sub_angvel_ = this->create_subscription<geometry_msgs::msg::Vector3Stamped>(
      angvel_topic_, qos,
      std::bind(&FkIkTransformNode::cbAngVel, this, std::placeholders::_1));

    sub_cf_ = this->create_subscription<geometry_msgs::msg::WrenchStamped>(
      cf_topic_, qos,
      std::bind(&FkIkTransformNode::cbContactForce, this, std::placeholders::_1));

    sub_cf_filt_ = this->create_subscription<geometry_msgs::msg::WrenchStamped>(
      cf_f_topic_, qos,
      std::bind(&FkIkTransformNode::cbContactForceFilt, this, std::placeholders::_1));

    // ---- publishers (NEW) ----
    pub_ee_pose_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(ee_pose_topic_, 10);
    pub_ee_vel_  = this->create_publisher<geometry_msgs::msg::Vector3Stamped>(ee_vel_topic_, 10);

    // ---- compute timer (FK + EE vel publish) ----
    if (compute_hz > 0.0) {
      auto period = std::chrono::duration<double>(1.0 / compute_hz);
      timer_compute_ = this->create_wall_timer(
        std::chrono::duration_cast<std::chrono::nanoseconds>(period),
        std::bind(&FkIkTransformNode::onComputeTimer, this));
    }

    // ---- debug timer (optional) ----
    if (print_hz > 0.0) {
      auto period = std::chrono::duration<double>(1.0 / print_hz);
      timer_debug_ = this->create_wall_timer(
        std::chrono::duration_cast<std::chrono::nanoseconds>(period),
        std::bind(&FkIkTransformNode::onDebugTimer, this));
    }

    RCLCPP_INFO(this->get_logger(), "fk_ik_transform started.");
    RCLCPP_INFO(this->get_logger(), "EE offset (body): [%.4f %.4f %.4f]",
                end_effector_offset.x(), end_effector_offset.y(), end_effector_offset.z());
    RCLCPP_INFO(this->get_logger(), "Subscribing:\n  %s\n  %s\n  %s\n  %s\n  %s",
                pose_topic_.c_str(), vel_topic_.c_str(), angvel_topic_.c_str(),
                cf_topic_.c_str(), cf_f_topic_.c_str());
    RCLCPP_INFO(this->get_logger(), "Publishing:\n  %s\n  %s",
                ee_pose_topic_.c_str(), ee_vel_topic_.c_str());
  }

private:
  // ---------------- math helpers ----------------
  static Eigen::Quaterniond quatMsgToEigen(const geometry_msgs::msg::Quaternion &q)
  {
    // geometry_msgs: (x,y,z,w) -> Eigen(w,x,y,z)
    return Eigen::Quaterniond(q.w, q.x, q.y, q.z);
  }

  static Eigen::Vector3d vecMsgToEigen(const geometry_msgs::msg::Vector3 &v)
  {
    return Eigen::Vector3d(v.x, v.y, v.z);
  }

  static geometry_msgs::msg::Vector3 eigenToVecMsg(const Eigen::Vector3d &v)
  {
    geometry_msgs::msg::Vector3 out;
    out.x = v.x();
    out.y = v.y();
    out.z = v.z();
    return out;
  }

  // ---------------- callbacks ----------------
  void cbPose(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lk(mtx_);
    last_pose_ = *msg;
  }

  void cbVel(const geometry_msgs::msg::Vector3Stamped::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lk(mtx_);
    last_vel_ = *msg;
  }

  void cbAngVel(const geometry_msgs::msg::Vector3Stamped::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lk(mtx_);
    last_angvel_ = *msg;
  }

  void cbContactForce(const geometry_msgs::msg::WrenchStamped::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lk(mtx_);
    last_cf_ = *msg;
  }

  void cbContactForceFilt(const geometry_msgs::msg::WrenchStamped::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lk(mtx_);
    last_cf_filt_ = *msg;
  }

  // ---------------- FK + EE vel publish ----------------
  void onComputeTimer()
  {
    std::optional<geometry_msgs::msg::PoseStamped> pose;
    std::optional<geometry_msgs::msg::Vector3Stamped> vel;
    std::optional<geometry_msgs::msg::Vector3Stamped> angvel;

    {
      std::lock_guard<std::mutex> lk(mtx_);
      pose   = last_pose_;
      vel    = last_vel_;
      angvel = last_angvel_;
    }

    if (!pose || !vel || !angvel) {
      return;
    }

    // ---- naming requested by you ----
    Eigen::Vector3d global_drone_position(
      pose->pose.position.x,
      pose->pose.position.y,
      pose->pose.position.z
    );

    // drone attitude (world<-body). Eigen quat represents rotation from body to world
    Eigen::Quaterniond q_WB = quatMsgToEigen(pose->pose.orientation).normalized();
    Eigen::Matrix3d R_WB = q_WB.toRotationMatrix();

    // ---- FK: EE position in world ----
    // p_EE = p_drone + R_WB * r_BE (offset in BODY)
    Eigen::Vector3d global_EE_position = global_drone_position + R_WB * end_effector_offset;

    // ---- EE velocity in world ----
    Eigen::Vector3d v_W = Eigen::Vector3d(vel->vector.x, vel->vector.y, vel->vector.z);

    // angvel is BODY frame (as your plant publishes with frame_id="body")
    Eigen::Vector3d omega_B(angvel->vector.x, angvel->vector.y, angvel->vector.z);

    // ω in world
    Eigen::Vector3d omega_W = R_WB * omega_B;

    // v_EE = v + ω × r   (all in world)
    Eigen::Vector3d r_W = R_WB * end_effector_offset;
    Eigen::Vector3d global_EE_velocity = v_W + omega_W.cross(r_W);

    // ---- publish EE pose ----
    geometry_msgs::msg::PoseStamped ee_pose;
    ee_pose.header = pose->header;          // stamp + frame_id 그대로(world)
    ee_pose.pose.position.x = global_EE_position.x();
    ee_pose.pose.position.y = global_EE_position.y();
    ee_pose.pose.position.z = global_EE_position.z();

    // EE orientation: offset이 pure translation이면 orientation은 drone과 동일
    ee_pose.pose.orientation = pose->pose.orientation;

    pub_ee_pose_->publish(ee_pose);

    // ---- publish EE velocity ----
    geometry_msgs::msg::Vector3Stamped ee_vel;
    ee_vel.header = vel->header;            // stamp + frame_id 그대로(world)
    ee_vel.vector = eigenToVecMsg(global_EE_velocity);
    pub_ee_vel_->publish(ee_vel);
  }

  // ---------------- periodic debug ----------------
  void onDebugTimer()
  {
    std::optional<geometry_msgs::msg::PoseStamped> pose;
    std::optional<geometry_msgs::msg::Vector3Stamped> vel;
    std::optional<geometry_msgs::msg::Vector3Stamped> angvel;
    std::optional<geometry_msgs::msg::WrenchStamped> cf;
    std::optional<geometry_msgs::msg::WrenchStamped> cf_f;

    {
      std::lock_guard<std::mutex> lk(mtx_);
      pose = last_pose_;
      vel = last_vel_;
      angvel = last_angvel_;
      cf = last_cf_;
      cf_f = last_cf_filt_;
    }

    if (!pose || !vel || !angvel) {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                           "Waiting for messages... (pose/vel/angvel not ready)");
      return;
    }

    const auto &p = pose->pose.position;
    const auto &q = pose->pose.orientation;
    const auto &v = vel->vector;
    const auto &w = angvel->vector;

    if (cf) {
      const auto &F = cf->wrench.force;
      RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 500,
        "p=(%.3f %.3f %.3f) q=(%.3f %.3f %.3f %.3f) v=(%.3f %.3f %.3f) wB=(%.3f %.3f %.3f) | Fraw=(%.3f %.3f %.3f)",
        p.x, p.y, p.z, q.x, q.y, q.z, q.w,
        v.x, v.y, v.z, w.x, w.y, w.z,
        F.x, F.y, F.z
      );
    } else {
      RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 500,
        "p=(%.3f %.3f %.3f) q=(%.3f %.3f %.3f %.3f) v=(%.3f %.3f %.3f) wB=(%.3f %.3f %.3f)",
        p.x, p.y, p.z, q.x, q.y, q.z, q.w,
        v.x, v.y, v.z, w.x, w.y, w.z
      );
    }

    if (cf_f) {
      const auto &Ff = cf_f->wrench.force;
      RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 500,
        "Ffilt=(%.3f %.3f %.3f)", Ff.x, Ff.y, Ff.z);
    }
  }

private:
  // ---- params/topics ----
  std::string pose_topic_;
  std::string vel_topic_;
  std::string angvel_topic_;
  std::string cf_topic_;
  std::string cf_f_topic_;

  std::string ee_pose_topic_;
  std::string ee_vel_topic_;

public:
  // requested by you: static offset vector (body->EE)
  Eigen::Vector3d end_effector_offset;

private:
  // ---- subscribers ----
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr sub_pose_;
  rclcpp::Subscription<geometry_msgs::msg::Vector3Stamped>::SharedPtr sub_vel_;
  rclcpp::Subscription<geometry_msgs::msg::Vector3Stamped>::SharedPtr sub_angvel_;
  rclcpp::Subscription<geometry_msgs::msg::WrenchStamped>::SharedPtr sub_cf_;
  rclcpp::Subscription<geometry_msgs::msg::WrenchStamped>::SharedPtr sub_cf_filt_;

  // ---- publishers (NEW) ----
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pub_ee_pose_;
  rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr pub_ee_vel_;

  // ---- timers ----
  rclcpp::TimerBase::SharedPtr timer_compute_;
  rclcpp::TimerBase::SharedPtr timer_debug_;

  // ---- latest buffers ----
  std::mutex mtx_;
  std::optional<geometry_msgs::msg::PoseStamped> last_pose_;
  std::optional<geometry_msgs::msg::Vector3Stamped> last_vel_;
  std::optional<geometry_msgs::msg::Vector3Stamped> last_angvel_;
  std::optional<geometry_msgs::msg::WrenchStamped> last_cf_;
  std::optional<geometry_msgs::msg::WrenchStamped> last_cf_filt_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<FkIkTransformNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
