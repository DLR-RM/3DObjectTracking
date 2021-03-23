// SPDX-License-Identifier: MIT
// Copyright (c) 2020 Manuel Stoiber, German Aerospace Center (DLR)

#ifndef OBJECT_TRACKING_INCLUDE_RBGT_RENDERER_H_
#define OBJECT_TRACKING_INCLUDE_RBGT_RENDERER_H_

#include <GL/glew.h>
#include <GLFW/glfw3.h>
#include <rbgt/camera.h>
#include <rbgt/common.h>
#include <rbgt/renderer_geometry.h>

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <iostream>
#include <memory>
#include <opencv2/opencv.hpp>
#include <string>
#include <vector>

namespace rbgt {

// Abstract class that defines a renderer as a single camera at a defined
// location. Specifics with respect to the type of image rendered are
// implemented in the derived class
class Renderer {
 public:
  // Initialization method and setters
  virtual bool Init(const std::string &name,
                    std::shared_ptr<RendererGeometry> renderer_geometry_ptr,
                    const Transform3fA &world2camera_pose,
                    const Intrinsics &intrinsics, float z_min, float z_max) = 0;
  bool InitFromCamera(const std::string &name,
                      std::shared_ptr<RendererGeometry> renderer_geometry_ptr,
                      const Camera &camera, float z_min = 0.01f,
                      float z_max = 5.0f);
  void set_world2camera_pose(const Transform3fA &world2camera_pose);
  void set_camera2world_pose(const Transform3fA &camera2world_pose);
  virtual bool set_intrinsics(const Intrinsics &intrinsics) = 0;
  virtual bool set_z_min(float z_min) = 0;
  virtual bool set_z_max(float z_max) = 0;

  // Main methods
  virtual bool StartRendering() = 0;

  // Getters
  const std::string &name() const;
  std::shared_ptr<RendererGeometry> renderer_geometry_ptr() const;
  const Transform3fA &world2camera_pose() const;
  const Transform3fA &camera2world_pose() const;
  const Intrinsics &intrinsics() const;
  float z_min() const;
  float z_max() const;
  bool initialized() const;

 protected:
  // Helper Methods
  void CalculateProjectionMatrix();
  bool CreateShaderProgram(const char *vertex_shader_code,
                           const char *fragment_shader_code,
                           unsigned int *shader_program);
  static bool CheckCompileErrors(unsigned int shader, const std::string &type);

  std::string name_;
  std::shared_ptr<RendererGeometry> renderer_geometry_ptr_ = nullptr;
  Transform3fA world2camera_pose_;
  Transform3fA camera2world_pose_;
  Intrinsics intrinsics_{};
  float z_min_ = 0.01f;  // min and max z-distance considered in clip space
  float z_max_ = 5.0f;
  Eigen::Matrix4f projection_matrix_;  // projects 3d data into clip space
  bool initialized_ = false;
};

}  // namespace rbgt

#endif  // OBJECT_TRACKING_INCLUDE_RBGT_RENDERER_H_
