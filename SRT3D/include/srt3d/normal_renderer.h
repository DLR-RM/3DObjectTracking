// SPDX-License-Identifier: MIT
// Copyright (c) 2021 Manuel Stoiber, German Aerospace Center (DLR)

#ifndef OBJECT_TRACKING_INCLUDE_SRT3D_NORMAL_RENDERER_H_
#define OBJECT_TRACKING_INCLUDE_SRT3D_NORMAL_RENDERER_H_

#include <GL/glew.h>
#include <GLFW/glfw3.h>
#include <srt3d/body.h>
#include <srt3d/camera.h>
#include <srt3d/common.h>
#include <srt3d/renderer.h>
#include <srt3d/renderer_geometry.h>

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <iostream>
#include <memory>
#include <mutex>
#include <opencv2/opencv.hpp>
#include <string>
#include <vector>

namespace srt3d {

// Renderer that is able to render both a depth image and an image where the
// normal vector of the surface is encoded in the color of each pixel
class NormalRenderer : public Renderer {
 public:
  // Constructors, destructors, and setup method
  NormalRenderer(const std::string &name,
                 std::shared_ptr<RendererGeometry> renderer_geometry_ptr,
                 const Transform3fA &world2camera_pose,
                 const Intrinsics &intrinsics, float z_min = 0.01f,
                 float z_max = 5.0f, float depth_scale = 1000.0f);
  NormalRenderer(const std::string &name,
                 std::shared_ptr<RendererGeometry> renderer_geometry_ptr,
                 std::shared_ptr<Camera> camera_ptr, float z_min = 0.01f,
                 float z_max = 5.0f, float depth_scale = 1000.0f);
  ~NormalRenderer();
  bool SetUp() override;

  // Setters
  void set_depth_scale(float depth_scale);

  // Main method
  bool StartRendering() override;
  bool FetchNormalImage();
  bool FetchDepthImage();

  // Getters
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
  float depth_scale_{};  // in units per meter

  // Shader code
  static std::string vertex_shader_code_;
  static std::string fragment_shader_code_;

  // OpenGL variables
  unsigned fbo_ = 0;
  unsigned rbo_normal_ = 0;
  unsigned rbo_depth_ = 0;
  unsigned shader_program_ = 0;

  // Internal variables
  float projection_term_a_ = 0;
  float projection_term_b_ = 0;

  // Internal state variables
  bool normal_image_fetched_ = false;
  bool depth_image_fetched_ = false;
};

}  // namespace srt3d

#endif  // OBJECT_TRACKING_INCLUDE_SRT3D_NORMAL_RENDERER_H_
