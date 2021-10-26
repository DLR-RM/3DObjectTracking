// SPDX-License-Identifier: MIT
// Copyright (c) 2021 Manuel Stoiber, German Aerospace Center (DLR)

#ifndef OBJECT_TRACKING_INCLUDE_SRT3D_RENDERER_H_
#define OBJECT_TRACKING_INCLUDE_SRT3D_RENDERER_H_

#include <GL/glew.h>
#include <GLFW/glfw3.h>
#include <srt3d/camera.h>
#include <srt3d/common.h>
#include <srt3d/renderer_geometry.h>

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <iostream>
#include <memory>
#include <opencv2/opencv.hpp>
#include <string>
#include <vector>

namespace srt3d {

// Abstract class that defines a renderer as a single camera at a defined
// location. Specifics with respect to the type of image rendered are
// implemented in the derived class
class Renderer {
 public:
  // Constructors and setup methods
  Renderer(const std::string &name,
           std::shared_ptr<RendererGeometry> renderer_geometry_ptr,
           const Transform3fA &world2camera_pose, const Intrinsics &intrinsics,
           float z_min, float z_max);
  Renderer(const std::string &name,
           std::shared_ptr<RendererGeometry> renderer_geometry_ptr,
           std::shared_ptr<Camera> camera_ptr, float z_min, float z_max);
  virtual bool SetUp() = 0;

  // Setters
  void set_name(const std::string &name);
  void set_world2camera_pose(const Transform3fA &world2camera_pose);
  void set_camera2world_pose(const Transform3fA &camera2world_pose);
  void set_camera_ptr(std::shared_ptr<Camera> camera_ptr);
  void set_renderer_geometry_ptr(
      std::shared_ptr<RendererGeometry> renderer_geometry_ptr);
  void set_intrinsics(const Intrinsics &intrinsics);
  void set_z_min(float z_min);
  void set_z_max(float z_max);

  // Main methods
  virtual bool StartRendering() = 0;

  // Getters
  const std::string &name() const;
  const Transform3fA &world2camera_pose() const;
  const Transform3fA &camera2world_pose() const;
  std::shared_ptr<Camera> camera_ptr() const;
  std::shared_ptr<RendererGeometry> renderer_geometry_ptr() const;
  const Intrinsics &intrinsics() const;
  float z_min() const;
  float z_max() const;
  bool set_up() const;

 protected:
  // Helper Methods
  bool InitParametersFromCamera();
  void CalculateProjectionMatrix();
  bool CreateShaderProgram(const char *vertex_shader_code,
                           const char *fragment_shader_code,
                           unsigned *shader_program);
  static bool CheckCompileErrors(unsigned shader, const std::string &type);

  // Data
  std::string name_;
  std::shared_ptr<RendererGeometry> renderer_geometry_ptr_ = nullptr;
  std::shared_ptr<Camera> camera_ptr_ = nullptr;
  Transform3fA world2camera_pose_;
  Transform3fA camera2world_pose_;
  Intrinsics intrinsics_{};
  float z_min_ = 0.01f;  // min and max z-distance considered in clip space
  float z_max_ = 5.0f;
  Eigen::Matrix4f projection_matrix_;  // projects 3d data into clip space

  // State variables
  std::mutex mutex_;
  bool image_rendered_ = false;
  bool initial_set_up_ = false;
  bool set_up_ = false;
};

}  // namespace srt3d

#endif  // OBJECT_TRACKING_INCLUDE_SRT3D_RENDERER_H_
