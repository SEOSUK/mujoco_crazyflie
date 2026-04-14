#include "mujoco_bridge/viewer_controls.hpp"

#include <algorithm>
#include <cmath>

namespace mujoco_bridge
{

void ViewerControls::configure(const ViewerControlsConfig & config)
{
  config_ = config;
}

void ViewerControls::syncButtonState(GLFWwindow * window)
{
  button_left_ = (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_LEFT) == GLFW_PRESS);
  button_middle_ = (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_MIDDLE) == GLFW_PRESS);
  button_right_ = (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_RIGHT) == GLFW_PRESS);
}

void ViewerControls::onMouseButton(
  GLFWwindow * window, mjModel * model, mjData * data, const mjvOption * option,
  const mjvScene * scene, mjvCamera * camera)
{
  (void)model;
  (void)data;
  (void)option;
  (void)scene;
  (void)camera;

  syncButtonState(window);
  glfwGetCursorPos(window, &last_x_, &last_y_);
}

void ViewerControls::onCursorPos(
  GLFWwindow * window, mjModel * model, mjData * data, const mjvOption * option,
  const mjvScene * scene, mjvCamera * camera, double xpos, double ypos)
{
  (void)data;
  (void)option;

  const double dx = xpos - last_x_;
  const double dy = ypos - last_y_;
  last_x_ = xpos;
  last_y_ = ypos;

  syncButtonState(window);
  if (!button_left_ && !button_middle_ && !button_right_) {
    return;
  }

  int width = 0;
  int height = 0;
  glfwGetWindowSize(window, &width, &height);
  if (width <= 0 || height <= 0) {
    return;
  }

  const bool shift = isShiftPressed(window);

  // SolidWorks-like feel:
  // - wheel: zoom
  // - middle drag: orbit
  // - shift + middle drag: pan
  // - right drag: pan
  if ((button_right_ && !shift) || (button_middle_ && shift) || (button_left_ && shift)) {
    mjv_moveCamera(
      model,
      mjMOUSE_MOVE_H,
      config_.pan_sensitivity * dx / static_cast<double>(height),
      0.0,
      scene,
      camera);

    mjv_moveCamera(
      model,
      mjMOUSE_MOVE_V,
      0.0,
      config_.pan_sensitivity * dy / static_cast<double>(height),
      scene,
      camera);
    return;
  }

  if ((button_middle_ && !shift) || (button_left_ && !shift)) {
    mjv_moveCamera(
      model,
      mjMOUSE_ROTATE_H,
      config_.orbit_sensitivity * dx / static_cast<double>(height),
      0.0,
      scene,
      camera);

    mjv_moveCamera(
      model,
      mjMOUSE_ROTATE_V,
      0.0,
      config_.orbit_sensitivity * dy / static_cast<double>(height),
      scene,
      camera);
  }
}

void ViewerControls::onScroll(
  GLFWwindow * window, mjModel * model, mjData * data, mjvCamera * camera, double yoffset)
{
  (void)window;
  (void)model;
  (void)data;

  const double zoom_gain = 0.90;
  const double scale = std::pow(zoom_gain, config_.zoom_sensitivity * yoffset);
  camera->distance = clampDouble(
    camera->distance * scale,
    config_.min_distance,
    config_.max_distance);
}

bool ViewerControls::isShiftPressed(GLFWwindow * window)
{
  return
    (glfwGetKey(window, GLFW_KEY_LEFT_SHIFT) == GLFW_PRESS) ||
    (glfwGetKey(window, GLFW_KEY_RIGHT_SHIFT) == GLFW_PRESS);
}

double ViewerControls::clampDouble(double value, double lo, double hi)
{
  return std::max(lo, std::min(hi, value));
}

}  // namespace mujoco_bridge
