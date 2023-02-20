// SPDX-License-Identifier: MIT
// Copyright (c) 2023 Manuel Stoiber, German Aerospace Center (DLR)

#ifndef M3T_INCLUDE_M3T_NORMAL_RENDERER_H_
#define M3T_INCLUDE_M3T_NORMAL_RENDERER_H_

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
 * \brief Class that implements the main functionality for a normal renderer and
 * is used by \ref FullBasicDepthRenderer and \ref FocusedBasicDepthRenderer.
 */
class NormalRendererCore {
 public:
  // Destructor and setup method
  NormalRendererCore() = default;
  NormalRendererCore(const NormalRendererCore &) = delete;
  NormalRendererCore &operator=(const NormalRendererCore &) = delete;
  ~NormalRendererCore();
  bool SetUp(const std::shared_ptr<RendererGeometry> &renderer_geometry_ptr,
             int image_width, int image_height);

  // Main methods
  bool StartRendering(const Eigen::Matrix4f &projection_matrix,
                      const Transform3fA &world2camera_pose);
  bool FetchNormalImage(cv::Mat *normal_image);
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
  static std::string fragment_shader_code_;

  // OpenGL variables
  unsigned fbo_ = 0;
  unsigned rbo_normal_ = 0;
  unsigned rbo_depth_ = 0;
  unsigned shader_program_ = 0;

  // Internal state
  bool image_rendered_ = false;
  bool normal_image_fetched_ = false;
  bool depth_image_fetched_ = false;
  bool initial_set_up_ = false;
};

/**
 * \brief Renderer that extends the full depth renderer class with functionality
 * from \ref NormalRendererCore to render both a depth image and an image where
 * the normal vector of the surface is encoded in the color of each pixel.
 *
 * \details Rendering is started using `StartRendering()`. Images are fetched
 * from the GPU using `FetchNormalImage()` and `FetchDepthImage()`. They can
 * then be accessed using the `normal_image()` and `depth_image()` getter.
 * Setters and all main methods are thread-safe.
 */
class FullNormalRenderer : public FullDepthRenderer {
 public:
  // Constructors, destructors, and setup method
  FullNormalRenderer(
      const std::string &name,
      const std::shared_ptr<RendererGeometry> &renderer_geometry_ptr,
      const Transform3fA &world2camera_pose, const Intrinsics &intrinsics,
      float z_min = 0.02f, float z_max = 10.0f);
  FullNormalRenderer(
      const std::string &name,
      const std::shared_ptr<RendererGeometry> &renderer_geometry_ptr,
      const std::shared_ptr<Camera> &camera_ptr, float z_min = 0.02f,
      float z_max = 10.0f);
  FullNormalRenderer(
      const std::string &name, const std::filesystem::path &metafile_path,
      const std::shared_ptr<RendererGeometry> &renderer_geometry_ptr,
      const std::shared_ptr<Camera> &camera_ptr);
  bool SetUp() override;

  // Main methods
  bool StartRendering() override;
  bool FetchNormalImage();
  bool FetchDepthImage() override;

  // Getters
  const cv::Mat &normal_image() const;

  // Getters that calculate values based on the rendered normal image
  Eigen::Vector3f NormalVector(cv::Vec4b normal_image_value) const;
  Eigen::Vector3f NormalVector(const cv::Point2i &image_coordinate) const;
  cv::Vec4b NormalImageValue(const cv::Point2i &image_coordinate) const;

 private:
  // Helper methods
  bool LoadMetaData();
  void ClearNormalImage();

  // Data
  cv::Mat normal_image_;
  NormalRendererCore core_{};
};

/**
 * \brief Renderer that extends the focused depth renderer class with
 * functionality from \ref NormalRendererCore to render images with a defined
 * size that is focused on referenced bodies.
 *
 * \details It is able to render both a depth image and an image where the
 * normal vector of the surface is encoded in the color of each pixel. Rendering
 * is started using `StartRendering()`. Images are fetched from the GPU using
 * `FetchNormalImage()` and `FetchDepthImage()`. They can then be accessed using
 * the `normal_image()` and `depth_image()` getter. Setters and all main methods
 * are thread-safe.
 */
class FocusedNormalRenderer : public FocusedDepthRenderer {
 public:
  // Constructors, destructors, and setup method
  FocusedNormalRenderer(
      const std::string &name,
      const std::shared_ptr<RendererGeometry> &renderer_geometry_ptr,
      const Transform3fA &world2camera_pose, const Intrinsics &intrinsics,
      int image_size = 200, float z_min = 0.02f, float z_max = 10.0f);
  FocusedNormalRenderer(
      const std::string &name,
      const std::shared_ptr<RendererGeometry> &renderer_geometry_ptr,
      const std::shared_ptr<Camera> &camera_ptr, int image_size = 200,
      float z_min = 0.02f, float z_max = 10.0f);
  FocusedNormalRenderer(
      const std::string &name, const std::filesystem::path &metafile_path,
      const std::shared_ptr<RendererGeometry> &renderer_geometry_ptr,
      const std::shared_ptr<Camera> &camera_ptr);
  bool SetUp() override;

  // Main methods
  bool StartRendering() override;
  bool FetchNormalImage();
  bool FetchDepthImage() override;

  // Getters
  const cv::Mat &focused_normal_image() const;

  // Getters that calculate values based on the rendered normal image for the
  // original image coordinates
  Eigen::Vector3f NormalVector(cv::Vec4b normal_image_value) const;
  Eigen::Vector3f NormalVector(const cv::Point2i &image_coordinate) const;
  cv::Vec4b NormalImageValue(const cv::Point2i &image_coordinate) const;

 private:
  // Helper methods
  bool LoadMetaData();
  void ClearNormalImage();

  // Data
  cv::Mat focused_normal_image_;
  NormalRendererCore core_{};
};

}  // namespace m3t

#endif  // M3T_INCLUDE_M3T_NORMAL_RENDERER_H_
