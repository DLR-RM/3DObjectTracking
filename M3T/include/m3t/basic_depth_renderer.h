// SPDX-License-Identifier: MIT
// Copyright (c) 2023 Manuel Stoiber, German Aerospace Center (DLR)

#ifndef M3T_INCLUDE_M3T_BASIC_DEPTH_RENDERER_H_
#define M3T_INCLUDE_M3T_BASIC_DEPTH_RENDERER_H_

#include <m3t/body.h>
#include <m3t/camera.h>
#include <m3t/common.h>
#include <m3t/renderer.h>
#include <m3t/renderer_geometry.h>

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <iostream>
#include <memory>
#include <mutex>
#include <opencv2/opencv.hpp>
#include <string>
#include <vector>

namespace m3t {

/**
 * \brief Class that implements the main functionality for a basic depth
 * renderer and is used by \ref FullBasicDepthRenderer and \ref
 * FocusedBasicDepthRenderer.
 */
class BasicDepthRendererCore {
 public:
  // Destructor and setup method
  BasicDepthRendererCore() = default;
  BasicDepthRendererCore(const BasicDepthRendererCore &) = delete;
  BasicDepthRendererCore &operator=(const BasicDepthRendererCore &) = delete;
  ~BasicDepthRendererCore();
  bool SetUp(const std::shared_ptr<RendererGeometry> &renderer_geometry_ptr,
             int image_width, int image_height);

  // Main methods
  bool StartRendering(const Eigen::Matrix4f &projection_matrix,
                      const Transform3fA &world2camera_pose);
  bool FetchDepthImage(cv::Mat *depth_image);

 private:
  // Helper methods
  void CreateBufferObjects();
  void DeleteBufferObjects();

  // Internal data
  std::shared_ptr<RendererGeometry> renderer_geometry_ptr_;
  int image_width_;
  int image_height_;

  // Shader code
  static std::string vertex_shader_code_;

  // OpenGL variables
  unsigned fbo_ = 0;
  unsigned rbo_ = 0;
  unsigned shader_program_ = 0;

  // Internal state
  bool image_rendered_ = false;
  bool image_fetched_ = false;
  bool initial_set_up_ = false;
};

/**
 * \brief Renderer that extends the full depth renderer class with functionality
 * from \ref BasicDepthRendererCore to render a depth image.
 *
 * \details Rendering is started using `StartRendering()`. Images are fetched
 * from the GPU using `FetchDepthImage()` and can then be accessed using the
 * `depth_image()` getter. Setters and all main methods are thread-safe.
 */
class FullBasicDepthRenderer : public FullDepthRenderer {
 public:
  // Constructors, destructors, and setup method
  FullBasicDepthRenderer(
      const std::string &name,
      const std::shared_ptr<RendererGeometry> &renderer_geometry_ptr,
      const Transform3fA &world2camera_pose, const Intrinsics &intrinsics,
      float z_min = 0.02f, float z_max = 10.0f);
  FullBasicDepthRenderer(
      const std::string &name,
      const std::shared_ptr<RendererGeometry> &renderer_geometry_ptr,
      const std::shared_ptr<Camera> &camera_ptr, float z_min = 0.02f,
      float z_max = 10.0f);
  FullBasicDepthRenderer(
      const std::string &name, const std::filesystem::path &metafile_path,
      const std::shared_ptr<RendererGeometry> &renderer_geometry_ptr,
      const std::shared_ptr<Camera> &camera_ptr);
  bool SetUp() override;

  // Main methods
  bool StartRendering() override;
  bool FetchDepthImage() override;

 private:
  // Helper methods
  bool LoadMetaData();

  // Data
  BasicDepthRendererCore core_{};
};

/**
 * \brief Renderer that extends the focused depth renderer class with
 * functionality from \ref BasicDepthRendererCore to render an image with a
 * defined size that is focused on referenced bodies.
 *
 * \details Rendering is started using `StartRendering()`. Images are fetched
 * from the GPU using `FetchDepthImage()` and can then be accessed using the
 * `depth_image()` getter. Setters and all main methods are thread-safe.
 */
class FocusedBasicDepthRenderer : public FocusedDepthRenderer {
 public:
  // Constructors, destructors, and setup method
  FocusedBasicDepthRenderer(
      const std::string &name,
      const std::shared_ptr<RendererGeometry> &renderer_geometry_ptr,
      const Transform3fA &world2camera_pose, const Intrinsics &intrinsics,
      int image_size = 200, float z_min = 0.02f, float z_max = 10.0f);
  FocusedBasicDepthRenderer(
      const std::string &name,
      const std::shared_ptr<RendererGeometry> &renderer_geometry_ptr,
      const std::shared_ptr<Camera> &camera_ptr, int image_size = 200,
      float z_min = 0.02f, float z_max = 10.0f);
  FocusedBasicDepthRenderer(
      const std::string &name, const std::filesystem::path &metafile_path,
      const std::shared_ptr<RendererGeometry> &renderer_geometry_ptr,
      const std::shared_ptr<Camera> &camera_ptr);
  bool SetUp() override;

  // Main methods
  bool StartRendering() override;
  bool FetchDepthImage() override;

 private:
  // Helper methods
  bool LoadMetaData();

  // Data
  BasicDepthRendererCore core_{};
};

}  // namespace m3t

#endif  // M3T_INCLUDE_M3T_BASIC_DEPTH_RENDERER_H_
