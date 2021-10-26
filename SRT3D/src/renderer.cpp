// SPDX-License-Identifier: MIT
// Copyright (c) 2021 Manuel Stoiber, German Aerospace Center (DLR)

#include <srt3d/renderer.h>

namespace srt3d {

Renderer::Renderer(const std::string &name,
                   std::shared_ptr<RendererGeometry> renderer_geometry_ptr,
                   const Transform3fA &world2camera_pose,
                   const Intrinsics &intrinsics, float z_min, float z_max)
    : name_{name},
      renderer_geometry_ptr_{std::move(renderer_geometry_ptr)},
      intrinsics_{intrinsics},
      z_min_{z_min},
      z_max_{z_max} {
  world2camera_pose_ = world2camera_pose;
  camera2world_pose_ = world2camera_pose.inverse();
}

Renderer::Renderer(const std::string &name,
                   std::shared_ptr<RendererGeometry> renderer_geometry_ptr,
                   std::shared_ptr<Camera> camera_ptr, float z_min, float z_max)
    : name_{name},
      renderer_geometry_ptr_{std::move(renderer_geometry_ptr)},
      camera_ptr_{std::move(camera_ptr)},
      z_min_{z_min},
      z_max_{z_max} {}

void Renderer::set_name(const std::string &name) {
  const std::lock_guard<std::mutex> lock{mutex_};
  name_ = name;
}

void Renderer::set_world2camera_pose(const Transform3fA &world2camera_pose) {
  const std::lock_guard<std::mutex> lock{mutex_};
  world2camera_pose_ = world2camera_pose;
  camera2world_pose_ = world2camera_pose.inverse();
}

void Renderer::set_camera2world_pose(const Transform3fA &camera2world_pose) {
  const std::lock_guard<std::mutex> lock{mutex_};
  camera2world_pose_ = camera2world_pose;
  world2camera_pose_ = camera2world_pose.inverse();
}

void Renderer::set_camera_ptr(std::shared_ptr<Camera> camera_ptr) {
  const std::lock_guard<std::mutex> lock{mutex_};
  camera_ptr_ = std::move(camera_ptr);
  set_up_ = false;
}

void Renderer::set_renderer_geometry_ptr(
    std::shared_ptr<RendererGeometry> renderer_geometry_ptr) {
  const std::lock_guard<std::mutex> lock{mutex_};
  renderer_geometry_ptr_ = std::move(renderer_geometry_ptr);
  set_up_ = false;
}

void Renderer::set_intrinsics(const Intrinsics &intrinsics) {
  const std::lock_guard<std::mutex> lock{mutex_};
  intrinsics_ = intrinsics;
  set_up_ = false;
}

void Renderer::set_z_min(float z_min) {
  const std::lock_guard<std::mutex> lock{mutex_};
  z_min_ = z_min;
  set_up_ = false;
}

void Renderer::set_z_max(float z_max) {
  const std::lock_guard<std::mutex> lock{mutex_};
  z_max_ = z_max;
  set_up_ = false;
}

const std::string &Renderer::name() const { return name_; }

std::shared_ptr<Camera> Renderer::camera_ptr() const { return camera_ptr_; }

std::shared_ptr<RendererGeometry> Renderer::renderer_geometry_ptr() const {
  return renderer_geometry_ptr_;
}

const Transform3fA &Renderer::world2camera_pose() const {
  return world2camera_pose_;
}

const Transform3fA &Renderer::camera2world_pose() const {
  return camera2world_pose_;
}

const Intrinsics &Renderer::intrinsics() const { return intrinsics_; }

float Renderer::z_min() const { return z_min_; }

float Renderer::z_max() const { return z_max_; }

bool Renderer::set_up() const { return set_up_; };

bool Renderer::InitParametersFromCamera() {
  if (!camera_ptr_->set_up()) {
    std::cerr << "Camera " << camera_ptr_->name() << " was not set up"
              << std::endl;
    return false;
  }
  intrinsics_ = camera_ptr_->intrinsics();
  world2camera_pose_ = camera_ptr_->world2camera_pose();
  camera2world_pose_ = camera_ptr_->camera2world_pose();
  return true;
}

void Renderer::CalculateProjectionMatrix() {
  projection_matrix_ << 2.0f * intrinsics_.fu / float(intrinsics_.width), 0.0f,
      2.0f * (intrinsics_.ppu + 0.5f) / float(intrinsics_.width) - 1.0f, 0.0f,
      0.0f, 2.0f * intrinsics_.fv / float(intrinsics_.height),
      2.0f * (intrinsics_.ppv + 0.5f) / float(intrinsics_.height) - 1.0f, 0.0f,
      0.0f, 0.0f, (z_max_ + z_min_) / (z_max_ - z_min_),
      -2.0f * z_max_ * z_min_ / (z_max_ - z_min_), 0.0f, 0.0f, 1.0f, 0.0f;
}

bool Renderer::CreateShaderProgram(const char *vertex_shader_code,
                                   const char *fragment_shader_code,
                                   unsigned *shader_program) {
  renderer_geometry_ptr_->MakeContextCurrent();

  // Create shader
  unsigned vertex_shader = glCreateShader(GL_VERTEX_SHADER);
  glShaderSource(vertex_shader, 1, &vertex_shader_code, nullptr);
  glCompileShader(vertex_shader);
  if (!CheckCompileErrors(vertex_shader, "VERTEX")) return false;

  unsigned fragment_shader = glCreateShader(GL_FRAGMENT_SHADER);
  glShaderSource(fragment_shader, 1, &fragment_shader_code, nullptr);
  glCompileShader(fragment_shader);
  if (!CheckCompileErrors(fragment_shader, "FRAGMENT")) return false;

  // Create shader programs
  *shader_program = glCreateProgram();
  glAttachShader(*shader_program, vertex_shader);
  glAttachShader(*shader_program, fragment_shader);
  glLinkProgram(*shader_program);
  if (!CheckCompileErrors(*shader_program, "PROGRAM")) return false;

  glDeleteShader(vertex_shader);
  glDeleteShader(fragment_shader);
  renderer_geometry_ptr_->DetachContext();
  return true;
}

bool Renderer::CheckCompileErrors(unsigned shader, const std::string &type) {
  int success;
  char info_log[1024];
  if (type != "PROGRAM") {
    glGetShaderiv(shader, GL_COMPILE_STATUS, &success);
    if (!success) {
      glGetShaderInfoLog(shader, 1024, nullptr, info_log);
      std::cerr << "Shader compilation error of type: " << type << std::endl
                << info_log << std::endl;
      return false;
    }
  } else {
    glGetProgramiv(shader, GL_LINK_STATUS, &success);
    if (!success) {
      glGetProgramInfoLog(shader, 1024, nullptr, info_log);
      std::cerr << "Shader linking error of type: " << type << std::endl
                << info_log << std::endl;
      return false;
    }
  }
  return true;
}

}  // namespace srt3d
