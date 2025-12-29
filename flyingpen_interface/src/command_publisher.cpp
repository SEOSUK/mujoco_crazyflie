#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <ncurses.h>
#include <array>
#include <chrono>
#include <string>

using namespace std::chrono_literals;

class CommandPublisher : public rclcpp::Node
{
public:
  CommandPublisher()
  : Node("command_publisher")
  {
    // ===============================
    // position 명령 (Float64MultiArray)
    // ===============================
    pos_cmd_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(
      "/su/keyboard_input", 10);

    // su_interface 로 보내는 모드/force 커맨드
    use_vel_mode_pub_ = this->create_publisher<std_msgs::msg::Float32>(
      "su/use_vel_mode", 10);
    force_pub_ = this->create_publisher<std_msgs::msg::Float32>(
      "su/cmd_force", 10);

    // 키보드 control step size
    pos_delta_[0] = this->declare_parameter<double>("dx", 0.03);
    pos_delta_[1] = this->declare_parameter<double>("dy", 0.03);
    pos_delta_[2] = this->declare_parameter<double>("dz", 0.03);
    yaw_delta_deg_ = this->declare_parameter<double>("dyaw_deg", 0.1);
    force_delta_   = this->declare_parameter<double>("df", 0.01);

    cmd_xyz_yaw_.fill(0.0);
    force_des_ = 0.0;
    use_vel_mode_ = 0.0;   // 기본: position mode
    status_msg_ = "ready";

    // ncurses 초기화
    initscr();
    cbreak();
    noecho();
    nodelay(stdscr, TRUE);
    keypad(stdscr, TRUE);

    mvprintw(0, 0, "command_publisher running. press 't' to quit.");
    mvprintw(2, 0, "keys: w/s(x), a/d(y), e/q(z), z/c(yaw), x(reset pos)");
    mvprintw(3, 0, "      j/k/l(force), i(toggle pos/vel+reset)");
    refresh();

    RCLCPP_INFO(this->get_logger(), "command_publisher started.");

    // 50ms timer
    timer_ = this->create_wall_timer(
      50ms, std::bind(&CommandPublisher::timerCallback, this));
  }

  ~CommandPublisher() override
  {
    endwin();
  }

private:
  void timerCallback()
  {
    int ch = getch();
    if (ch != ERR) {
      handleKey(static_cast<char>(ch));
    }

    publishPositionCmd();
    drawStatusLine();
  }

  void handleKey(char c)
  {
    if (c == 'w')       cmd_xyz_yaw_[0] += pos_delta_[0];
    else if (c == 's')  cmd_xyz_yaw_[0] -= pos_delta_[0];
    else if (c == 'a')  cmd_xyz_yaw_[1] += pos_delta_[1];
    else if (c == 'd')  cmd_xyz_yaw_[1] -= pos_delta_[1];
    else if (c == 'e')  cmd_xyz_yaw_[2] += pos_delta_[2];
    else if (c == 'q')  cmd_xyz_yaw_[2] -= pos_delta_[2];
    else if (c == 'z')  cmd_xyz_yaw_[3] += yaw_delta_deg_;
    else if (c == 'c')  cmd_xyz_yaw_[3] -= yaw_delta_deg_;
    else if (c == 'x')  cmd_xyz_yaw_.fill(0.0);

    else if (c == 'j')  { force_des_ += force_delta_; publishForce(); }
    else if (c == 'k')  { force_des_ -= force_delta_; publishForce(); }
    else if (c == 'l')  { force_des_ = 0.0;           publishForce(); }

    else if (c == 'i')  { toggleVelMode(); }

    else if (c == 't') {
      rclcpp::shutdown();
    }
  }

  // ===============================
  // Float64MultiArray publish
  // data = [x, y, z, yaw]
  // ===============================
  void publishPositionCmd()
  {
    std_msgs::msg::Float64MultiArray msg;
    msg.data.resize(4);
    msg.data[0] = cmd_xyz_yaw_[0];
    msg.data[1] = cmd_xyz_yaw_[1];
    msg.data[2] = cmd_xyz_yaw_[2];
    msg.data[3] = cmd_xyz_yaw_[3];  // yaw [deg]

    pos_cmd_pub_->publish(msg);
  }

  void publishForce()
  {
    std_msgs::msg::Float32 msg;
    msg.data = static_cast<float>(force_des_);
    force_pub_->publish(msg);
  }

  void toggleVelMode()
  {
    use_vel_mode_ = (use_vel_mode_ < 0.5) ? 1.0 : 0.0;

    std_msgs::msg::Float32 msg;
    msg.data = static_cast<float>(use_vel_mode_);
    use_vel_mode_pub_->publish(msg);

    cmd_xyz_yaw_.fill(0.0);
  }

  void drawStatusLine()
  {
    const char* mode_str = (use_vel_mode_ > 0.5) ? "VELOCITY" : "POSITION";

    mvprintw(5, 0,
      "cmd: x=%6.2f y=%6.2f z=%6.2f yaw=%6.2f | f=%5.2f      ",
      cmd_xyz_yaw_[0], cmd_xyz_yaw_[1], cmd_xyz_yaw_[2],
      cmd_xyz_yaw_[3], force_des_);

    mvprintw(6, 0,
      "mode: %s  (i: toggle pos/vel)                         ",
      mode_str);

    refresh();
  }


  // ============================
  // 멤버 변수
  // ============================
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr pos_cmd_pub_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr          use_vel_mode_pub_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr          force_pub_;
  rclcpp::TimerBase::SharedPtr                                  timer_;

  std::array<double, 4> cmd_xyz_yaw_;
  std::array<double, 3> pos_delta_;
  double yaw_delta_deg_;
  double force_des_;
  double force_delta_;
  double use_vel_mode_;
  std::string status_msg_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<CommandPublisher>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
