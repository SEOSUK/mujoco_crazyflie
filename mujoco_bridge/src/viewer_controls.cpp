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
  const double norm_dx = dx / static_cast<double>(height);
  const double norm_dy = dy / static_cast<double>(height);

  // CAD/SolidWorks-inspired navigation:
  // - left drag: orbit
  // - right drag: pan
  // - middle drag: pan
  // - shift + left drag: pan
  // - shift + right drag: dolly zoom
  if (button_right_ && shift) {
    mjv_moveCamera(
      model,
      mjMOUSE_ZOOM,
      0.0,
      -config_.zoom_sensitivity * norm_dy,
      scene,
      camera);
    return;
  }

  if (button_middle_ || (button_right_ && !shift) || (button_left_ && shift)) {
    mjv_moveCamera(
      model,
      mjMOUSE_MOVE_H,
      config_.pan_sensitivity * norm_dx,
      0.0,
      scene,
      camera);

    mjv_moveCamera(
      model,
      mjMOUSE_MOVE_V,
      0.0,
      config_.pan_sensitivity * norm_dy,
      scene,
      camera);
    return;
  }

  if (button_left_ && !shift) {
    mjv_moveCamera(
      model,
      mjMOUSE_ROTATE_H,
      config_.orbit_sensitivity * norm_dx,
      0.0,
      scene,
      camera);

    mjv_moveCamera(
      model,
      mjMOUSE_ROTATE_V,
      0.0,
      config_.orbit_sensitivity * norm_dy,
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

  const double azimuth_rad = camera->azimuth * M_PI / 180.0;
  const double elevation_rad = camera->elevation * M_PI / 180.0;

  // Move the whole camera rig forward/backward along the current view ray.
  double forward_x = std::sin(azimuth_rad) * std::cos(elevation_rad);
  double forward_y = std::cos(azimuth_rad) * std::cos(elevation_rad);
  double forward_z = std::sin(elevation_rad);
  const double forward_norm = std::sqrt(
    forward_x * forward_x + forward_y * forward_y + forward_z * forward_z);

  if (forward_norm < 1.0e-9) {
    return;
  }
  forward_x /= forward_norm;
  forward_y /= forward_norm;
  forward_z /= forward_norm;

  const double dolly_step = config_.zoom_sensitivity *
    clampDouble(0.15 * camera->distance, 0.03, 0.75);
  camera->lookat[0] += dolly_step * yoffset * forward_x;
  camera->lookat[1] += dolly_step * yoffset * forward_y;
  camera->lookat[2] += dolly_step * yoffset * forward_z;
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
