// SPDX-License-Identifier: MIT
// Copyright (c) 2020 Manuel Stoiber, German Aerospace Center (DLR)

#include <rbgt/renderer.h>

namespace rbgt {

bool Renderer::InitFromCamera(
    const std::string &name,
    std::shared_ptr<RendererGeometry> renderer_geometry_ptr,
    const Camera &camera, float z_min, float z_max) {
  if (!camera.initialized()) {
    std::cerr << "Camera is not initialized" << std::endl;
    return false;
  }
  return Init(name, std::move(renderer_geometry_ptr),
              camera.world2camera_pose(), camera.intrinsics(), z_min, z_max);
}

void Renderer::set_world2camera_pose(const Transform3fA &world2camera_pose) {
  world2camera_pose_ = world2camera_pose;
  camera2world_pose_ = world2camera_pose.inverse();
}

void Renderer::set_camera2world_pose(const Transform3fA &camera2world_pose) {
  camera2world_pose_ = camera2world_pose;
  world2camera_pose_ = camera2world_pose.inverse();
}

const std::string &Renderer::name() const { return name_; }

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

bool Renderer::initialized() const { return initialized_; }

void Renderer::CalculateProjectionMatrix() {
  projection_matrix_ << 2 * intrinsics_.fu / float(intrinsics_.width), 0,
      2 * (intrinsics_.ppu + 0.5f) / float(intrinsics_.width) - 1, 0, 0,
      2 * intrinsics_.fv / float(intrinsics_.height),
      2 * (intrinsics_.ppv + 0.5f) / float(intrinsics_.height) - 1, 0, 0, 0,
      (z_max_ + z_min_) / (z_max_ - z_min_),
      -2 * z_max_ * z_min_ / (z_max_ - z_min_), 0, 0, 1, 0;
}

bool Renderer::CreateShaderProgram(const char *vertex_shader_code,
                                   const char *fragment_shader_code,
                                   unsigned int *shader_program) {
  glfwMakeContextCurrent(renderer_geometry_ptr_->window());

  // Create shader
  unsigned int vertex_shader = glCreateShader(GL_VERTEX_SHADER);
  glShaderSource(vertex_shader, 1, &vertex_shader_code, nullptr);
  glCompileShader(vertex_shader);
  if (!CheckCompileErrors(vertex_shader, "VERTEX")) return false;

  unsigned int fragment_shader = glCreateShader(GL_FRAGMENT_SHADER);
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
  glfwMakeContextCurrent(nullptr);
  return true;
}

bool Renderer::CheckCompileErrors(unsigned int shader,
                                  const std::string &type) {
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

}  // namespace rbgt
