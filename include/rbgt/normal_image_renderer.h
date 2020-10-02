// SPDX-License-Identifier: MIT
// Copyright (c) 2020 Manuel Stoiber, German Aerospace Center (DLR)

#ifndef OBJECT_TRACKING_INCLUDE_RBGT_NORMAL_IMAGE_RENDERER_H_
#define OBJECT_TRACKING_INCLUDE_RBGT_NORMAL_IMAGE_RENDERER_H_

#include <GL/glew.h>
#include <GLFW/glfw3.h>
#include <rbgt/body.h>
#include <rbgt/camera.h>
#include <rbgt/common.h>
#include <rbgt/renderer.h>
#include <rbgt/renderer_geometry.h>

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <iostream>
#include <memory>
#include <opencv2/opencv.hpp>
#include <string>
#include <vector>

namespace rbgt {

// Renderer that is able to render both a depth image and an image where the
// normal vector of the surface is encoded in the color of each pixel
class NormalImageRenderer : public Renderer {
 public:
  // Constructors and destructors
  NormalImageRenderer();
  ~NormalImageRenderer();

  // Initialization method and setters
  bool Init(const std::string &name,
            std::shared_ptr<RendererGeometry> renderer_geometry_ptr,
            const Transform3fA &world2camera_pose, const Intrinsics &intrinsics,
            float z_min, float z_max) override;
  bool set_intrinsics(const Intrinsics &intrinsics) override;
  bool set_z_min(float z_min) override;
  bool set_z_max(float z_max) override;
  void set_depth_scale(float depth_scale);

  // Main method
  bool StartRendering() override;
  void FetchNormalImage();
  void FetchDepthImage();

  // Getters for images and internal variables
  const cv::Mat &normal_image() const;
  const cv::Mat &depth_image() const;
  float depth_scale() const;

  // Getter that calculates a point vector based on a rendered depth image
  Eigen::Vector3f GetPointVector(const cv::Point2i &image_coordinate) const;

 private:
  // Helper methods
  void ClearImages();
  void CalculateProjectionTerms();
  void CreateBufferObjects();
  void DeleteBufferObjects();

  // Image data
  cv::Mat normal_image_;
  cv::Mat depth_image_;

  // Parameters
  float depth_scale_ = 1000;  // in units per meter

  // Shader code
  std::string vertex_shader_code_{};
  std::string fragment_shader_code_{};

  // OpenGL variables
  unsigned int fbo_ = 0;
  unsigned int rbo_normal_ = 0;
  unsigned int rbo_depth_ = 0;
  unsigned int shader_program_ = 0;

  // Internal variables
  float projection_term_a_ = 0;
  float projection_term_b_ = 0;

  // Internal state variables
  bool normal_image_fetched_ = false;
  bool depth_image_fetched_ = false;
  bool image_rendered_ = false;
};

}  // namespace rbgt

#endif  // OBJECT_TRACKING_INCLUDE_RBGT_NORMAL_IMAGE_RENDERER_H_
