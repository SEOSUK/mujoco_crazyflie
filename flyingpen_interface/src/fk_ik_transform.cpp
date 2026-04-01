#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/vector3_stamped.hpp>
#include <geometry_msgs/msg/wrench_stamped.hpp>

#include <Eigen/Dense>

#include <mutex>
#include <optional>
#include <string>
#include <chrono>
#include <vector>

class FkIkTransformNode : public rclcpp::Node
{
public:
  FkIkTransformNode()
  : Node("fk_ik_transform")
  {
    // ---------------- params ----------------
    pose_topic_   = this->declare_parameter<std::string>("pose_topic",   "/crazyflie/out/pose");
    vel_topic_    = this->declare_parameter<std::string>("vel_topic",    "/crazyflie/out/vel");
    acc_topic_    = this->declare_parameter<std::string>("acc_topic",    "/crazyflie/out/acc");
    angvel_topic_ = this->declare_parameter<std::string>("angvel_topic", "/crazyflie/out/ang_vel");
    angacc_topic_ = this->declare_parameter<std::string>("angacc_topic", "/crazyflie/out/ang_acc");

    cf_topic_   = this->declare_parameter<std::string>("contact_force_topic",      "/crazyflie/out/EE_contact_force");
    cf_f_topic_ = this->declare_parameter<std::string>("contact_force_filt_topic", "/crazyflie/out/EE_contact_force_filt");

    ee_pose_topic_ = this->declare_parameter<std::string>("ee_pose_topic", "/crazyflie/out/EE_pose");
    ee_vel_topic_  = this->declare_parameter<std::string>("ee_vel_topic",  "/crazyflie/out/EE_velocity");
    ee_acc_topic_  = this->declare_parameter<std::string>("ee_acc_topic",  "/crazyflie/out/EE_acceleration");

    auto ee_off = this->declare_parameter<std::vector<double>>(
      "end_effector_offset", {0.09, 0.0, 0.085});

    if (ee_off.size() != 3) {
      RCLCPP_WARN(this->get_logger(),
                  "end_effector_offset must be size 3. Fallback to [0, 0, 0].");
      ee_off = {0.0, 0.0, 0.0};
    }
    d_B_ = Eigen::Vector3d(ee_off[0], ee_off[1], ee_off[2]);

    const double compute_hz = this->declare_parameter<double>("compute_hz", 200.0);
    const double print_hz   = this->declare_parameter<double>("debug_print_hz", 10.0);

    auto qos = rclcpp::SensorDataQoS();

    // ---------------- subscribers ----------------
    sub_pose_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
      pose_topic_, qos,
      std::bind(&FkIkTransformNode::cbPose, this, std::placeholders::_1));

    sub_vel_ = this->create_subscription<geometry_msgs::msg::Vector3Stamped>(
      vel_topic_, qos,
      std::bind(&FkIkTransformNode::cbVel, this, std::placeholders::_1));

    sub_acc_ = this->create_subscription<geometry_msgs::msg::Vector3Stamped>(
      acc_topic_, qos,
      std::bind(&FkIkTransformNode::cbAcc, this, std::placeholders::_1));

    sub_angvel_ = this->create_subscription<geometry_msgs::msg::Vector3Stamped>(
      angvel_topic_, qos,
      std::bind(&FkIkTransformNode::cbAngVel, this, std::placeholders::_1));

    sub_angacc_ = this->create_subscription<geometry_msgs::msg::Vector3Stamped>(
      angacc_topic_, qos,
      std::bind(&FkIkTransformNode::cbAngAcc, this, std::placeholders::_1));

    sub_cf_ = this->create_subscription<geometry_msgs::msg::WrenchStamped>(
      cf_topic_, qos,
      std::bind(&FkIkTransformNode::cbContactForce, this, std::placeholders::_1));

    sub_cf_filt_ = this->create_subscription<geometry_msgs::msg::WrenchStamped>(
      cf_f_topic_, qos,
      std::bind(&FkIkTransformNode::cbContactForceFilt, this, std::placeholders::_1));

    // ---------------- publishers ----------------
    pub_ee_pose_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(ee_pose_topic_, 10);
    pub_ee_vel_  = this->create_publisher<geometry_msgs::msg::Vector3Stamped>(ee_vel_topic_, 10);
    pub_ee_acc_  = this->create_publisher<geometry_msgs::msg::Vector3Stamped>(ee_acc_topic_, 10);

    // ---------------- timers ----------------
    if (compute_hz > 0.0) {
      auto period = std::chrono::duration<double>(1.0 / compute_hz);
      timer_compute_ = this->create_wall_timer(
        std::chrono::duration_cast<std::chrono::nanoseconds>(period),
        std::bind(&FkIkTransformNode::onComputeTimer, this));
    }

    if (print_hz > 0.0) {
      auto period = std::chrono::duration<double>(1.0 / print_hz);
      timer_debug_ = this->create_wall_timer(
        std::chrono::duration_cast<std::chrono::nanoseconds>(period),
        std::bind(&FkIkTransformNode::onDebugTimer, this));
    }

    RCLCPP_INFO(this->get_logger(), "fk_ik_transform started.");
    RCLCPP_INFO(this->get_logger(), "EE offset d_B = [%.4f %.4f %.4f]",
                d_B_.x(), d_B_.y(), d_B_.z());

    RCLCPP_INFO(this->get_logger(),
                "Subscribing:\n  %s\n  %s\n  %s\n  %s\n  %s",
                pose_topic_.c_str(),
                vel_topic_.c_str(),
                acc_topic_.c_str(),
                angvel_topic_.c_str(),
                angacc_topic_.c_str());

    RCLCPP_INFO(this->get_logger(),
                "Publishing:\n  %s\n  %s\n  %s",
                ee_pose_topic_.c_str(),
                ee_vel_topic_.c_str(),
                ee_acc_topic_.c_str());
  }

private:
  // --------------------------------------------------
  // math helpers
  // --------------------------------------------------
  static Eigen::Quaterniond quatMsgToEigen(const geometry_msgs::msg::Quaternion &q)
  {
    // geometry_msgs: (x, y, z, w) -> Eigen(w, x, y, z)
    return Eigen::Quaterniond(q.w, q.x, q.y, q.z);
  }

  static Eigen::Vector3d vecMsgToEigen(const geometry_msgs::msg::Vector3 &v)
  {
    return Eigen::Vector3d(v.x, v.y, v.z);
  }

  static geometry_msgs::msg::Vector3 eigenToVecMsg(const Eigen::Vector3d &v)
  {
    geometry_msgs::msg::Vector3 msg;
    msg.x = v.x();
    msg.y = v.y();
    msg.z = v.z();
    return msg;
  }

  // --------------------------------------------------
  // callbacks
  // --------------------------------------------------
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

  void cbAcc(const geometry_msgs::msg::Vector3Stamped::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lk(mtx_);
    last_acc_ = *msg;
  }

  void cbAngVel(const geometry_msgs::msg::Vector3Stamped::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lk(mtx_);
    last_angvel_ = *msg;
  }

  void cbAngAcc(const geometry_msgs::msg::Vector3Stamped::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lk(mtx_);
    last_angacc_ = *msg;
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

  // --------------------------------------------------
  // compute EE kinematics
  //
  // Forward kinematics (body-offset d_B):
  //   p_E = p_B + R_B d_B
  //   v_E = v_B + R_B (w_B x d_B)
  //   a_E = a_B + R_B (alpha_B x d_B + w_B x (w_B x d_B))
  // --------------------------------------------------
  void onComputeTimer()
  {
    std::optional<geometry_msgs::msg::PoseStamped> pose;
    std::optional<geometry_msgs::msg::Vector3Stamped> vel;
    std::optional<geometry_msgs::msg::Vector3Stamped> acc;
    std::optional<geometry_msgs::msg::Vector3Stamped> angvel;
    std::optional<geometry_msgs::msg::Vector3Stamped> angacc;

    {
      std::lock_guard<std::mutex> lk(mtx_);
      pose   = last_pose_;
      vel    = last_vel_;
      acc    = last_acc_;
      angvel = last_angvel_;
      angacc = last_angacc_;
    }

    if (!pose || !vel || !acc || !angvel || !angacc) {
      return;
    }

    // ---------------- drone state ----------------
    const Eigen::Vector3d p_B_W(
      pose->pose.position.x,
      pose->pose.position.y,
      pose->pose.position.z);

    const Eigen::Quaterniond q_WB = quatMsgToEigen(pose->pose.orientation).normalized();
    const Eigen::Matrix3d R_WB = q_WB.toRotationMatrix();

    const Eigen::Vector3d v_B_W(
      vel->vector.x,
      vel->vector.y,
      vel->vector.z);

    const Eigen::Vector3d a_B_W(
      acc->vector.x,
      acc->vector.y,
      acc->vector.z);

    const Eigen::Vector3d w_B(
      angvel->vector.x,
      angvel->vector.y,
      angvel->vector.z);

    const Eigen::Vector3d alpha_B(
      angacc->vector.x,
      angacc->vector.y,
      angacc->vector.z);

    // ---------------- EE kinematics ----------------
    const Eigen::Vector3d p_E_W = p_B_W + R_WB * d_B_;

    const Eigen::Vector3d v_E_W =
      v_B_W + R_WB * (w_B.cross(d_B_));

    const Eigen::Vector3d a_E_W =
      a_B_W + R_WB * (alpha_B.cross(d_B_) + w_B.cross(w_B.cross(d_B_)));

    // ---------------- publish EE pose ----------------
    geometry_msgs::msg::PoseStamped ee_pose_msg;
    ee_pose_msg.header = pose->header;
    ee_pose_msg.pose.position.x = p_E_W.x();
    ee_pose_msg.pose.position.y = p_E_W.y();
    ee_pose_msg.pose.position.z = p_E_W.z();

    // pure translation offset -> orientation unchanged
    ee_pose_msg.pose.orientation = pose->pose.orientation;
    pub_ee_pose_->publish(ee_pose_msg);

    // ---------------- publish EE velocity ----------------
    geometry_msgs::msg::Vector3Stamped ee_vel_msg;
    ee_vel_msg.header = vel->header;
    ee_vel_msg.header.frame_id = "world";
    ee_vel_msg.vector = eigenToVecMsg(v_E_W);
    pub_ee_vel_->publish(ee_vel_msg);

    // ---------------- publish EE acceleration ----------------
    geometry_msgs::msg::Vector3Stamped ee_acc_msg;
    ee_acc_msg.header = acc->header;
    ee_acc_msg.header.frame_id = "world";
    ee_acc_msg.vector = eigenToVecMsg(a_E_W);
    pub_ee_acc_->publish(ee_acc_msg);
  }

  // --------------------------------------------------
  // debug print
  // --------------------------------------------------
  void onDebugTimer()
  {
    std::optional<geometry_msgs::msg::PoseStamped> pose;
    std::optional<geometry_msgs::msg::Vector3Stamped> vel;
    std::optional<geometry_msgs::msg::Vector3Stamped> acc;
    std::optional<geometry_msgs::msg::Vector3Stamped> angvel;
    std::optional<geometry_msgs::msg::Vector3Stamped> angacc;
    std::optional<geometry_msgs::msg::WrenchStamped> cf;
    std::optional<geometry_msgs::msg::WrenchStamped> cf_f;

    {
      std::lock_guard<std::mutex> lk(mtx_);
      pose   = last_pose_;
      vel    = last_vel_;
      acc    = last_acc_;
      angvel = last_angvel_;
      angacc = last_angacc_;
      cf     = last_cf_;
      cf_f   = last_cf_filt_;
    }

    if (!pose || !vel || !acc || !angvel || !angacc) {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                           "Waiting for messages... (pose/vel/acc/angvel/angacc not ready)");
      return;
    }

    const auto &p = pose->pose.position;
    const auto &q = pose->pose.orientation;
    const auto &v = vel->vector;
    const auto &a = acc->vector;
    const auto &w = angvel->vector;
    const auto &al = angacc->vector;

    if (cf) {
      const auto &F = cf->wrench.force;
      RCLCPP_INFO_THROTTLE(
        this->get_logger(), *this->get_clock(), 500,
        "p=(%.3f %.3f %.3f) q=(%.3f %.3f %.3f %.3f) "
        "v=(%.3f %.3f %.3f) a=(%.3f %.3f %.3f) "
        "wB=(%.3f %.3f %.3f) alphaB=(%.3f %.3f %.3f) | "
        "Fraw=(%.3f %.3f %.3f)",
        p.x, p.y, p.z,
        q.x, q.y, q.z, q.w,
        v.x, v.y, v.z,
        a.x, a.y, a.z,
        w.x, w.y, w.z,
        al.x, al.y, al.z,
        F.x, F.y, F.z);
    } else {
      RCLCPP_INFO_THROTTLE(
        this->get_logger(), *this->get_clock(), 500,
        "p=(%.3f %.3f %.3f) q=(%.3f %.3f %.3f %.3f) "
        "v=(%.3f %.3f %.3f) a=(%.3f %.3f %.3f) "
        "wB=(%.3f %.3f %.3f) alphaB=(%.3f %.3f %.3f)",
        p.x, p.y, p.z,
        q.x, q.y, q.z, q.w,
        v.x, v.y, v.z,
        a.x, a.y, a.z,
        w.x, w.y, w.z,
        al.x, al.y, al.z);
    }

    if (cf_f) {
      const auto &Ff = cf_f->wrench.force;
      RCLCPP_INFO_THROTTLE(
        this->get_logger(), *this->get_clock(), 500,
        "Ffilt=(%.3f %.3f %.3f)",
        Ff.x, Ff.y, Ff.z);
    }
  }

private:
  // ---------------- topic names ----------------
  std::string pose_topic_;
  std::string vel_topic_;
  std::string acc_topic_;
  std::string angvel_topic_;
  std::string angacc_topic_;
  std::string cf_topic_;
  std::string cf_f_topic_;

  std::string ee_pose_topic_;
  std::string ee_vel_topic_;
  std::string ee_acc_topic_;

  // body-frame offset from drone body origin to end-effector
  Eigen::Vector3d d_B_;

  // ---------------- subscribers ----------------
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr sub_pose_;
  rclcpp::Subscription<geometry_msgs::msg::Vector3Stamped>::SharedPtr sub_vel_;
  rclcpp::Subscription<geometry_msgs::msg::Vector3Stamped>::SharedPtr sub_acc_;
  rclcpp::Subscription<geometry_msgs::msg::Vector3Stamped>::SharedPtr sub_angvel_;
  rclcpp::Subscription<geometry_msgs::msg::Vector3Stamped>::SharedPtr sub_angacc_;
  rclcpp::Subscription<geometry_msgs::msg::WrenchStamped>::SharedPtr sub_cf_;
  rclcpp::Subscription<geometry_msgs::msg::WrenchStamped>::SharedPtr sub_cf_filt_;

  // ---------------- publishers ----------------
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pub_ee_pose_;
  rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr pub_ee_vel_;
  rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr pub_ee_acc_;

  // ---------------- timers ----------------
  rclcpp::TimerBase::SharedPtr timer_compute_;
  rclcpp::TimerBase::SharedPtr timer_debug_;

  // ---------------- latest message buffers ----------------
  std::mutex mtx_;
  std::optional<geometry_msgs::msg::PoseStamped> last_pose_;
  std::optional<geometry_msgs::msg::Vector3Stamped> last_vel_;
  std::optional<geometry_msgs::msg::Vector3Stamped> last_acc_;
  std::optional<geometry_msgs::msg::Vector3Stamped> last_angvel_;
  std::optional<geometry_msgs::msg::Vector3Stamped> last_angacc_;
  std::optional<geometry_msgs::msg::WrenchStamped> last_cf_;
  std::optional<geometry_msgs::msg::WrenchStamped> last_cf_filt_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<FkIkTransformNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}