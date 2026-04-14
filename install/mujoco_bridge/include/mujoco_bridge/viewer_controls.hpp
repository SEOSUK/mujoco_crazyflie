#pragma once

#include <mujoco/mujoco.h>

#include <GLFW/glfw3.h>

namespace mujoco_bridge
{

struct ViewerControlsConfig
{
  double orbit_sensitivity{1.0};
  double pan_sensitivity{1.0};
  double zoom_sensitivity{1.0};
  double min_distance{0.08};
  double max_distance{20.0};
};

class ViewerControls
{
public:
  void configure(const ViewerControlsConfig & config);
  void syncButtonState(GLFWwindow * window);
  void onMouseButton(
    GLFWwindow * window, mjModel * model, mjData * data, const mjvOption * option,
    const mjvScene * scene, mjvCamera * camera);
  void onCursorPos(
    GLFWwindow * window, mjModel * model, mjData * data, const mjvOption * option,
    const mjvScene * scene, mjvCamera * camera,
    double xpos, double ypos);
  void onScroll(GLFWwindow * window, mjModel * model, mjData * data, mjvCamera * camera, double yoffset);

private:
  static bool isShiftPressed(GLFWwindow * window);
  static double clampDouble(double value, double lo, double hi);

  bool button_left_{false};
  bool button_middle_{false};
  bool button_right_{false};
  double last_x_{0.0};
  double last_y_{0.0};

  ViewerControlsConfig config_{};
};

}  // namespace mujoco_bridge
