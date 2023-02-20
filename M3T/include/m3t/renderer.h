// SPDX-License-Identifier: MIT
// Copyright (c) 2023 Manuel Stoiber, German Aerospace Center (DLR)

#ifndef M3T_INCLUDE_M3T_RENDERER_H_
#define M3T_INCLUDE_M3T_RENDERER_H_

#include <m3t/camera.h>
#include <m3t/common.h>
#include <m3t/renderer_geometry.h>

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <iostream>
#include <memory>
#include <opencv2/opencv.hpp>
#include <string>
#include <vector>

namespace m3t {

// General functions to create shader programs
bool CreateShaderProgram(RendererGeometry *renderer_geometry,
                         const char *vertex_shader_code,
                         unsigned int *shader_program);
bool CreateShaderProgram(RendererGeometry *renderer_geometry,
                         const char *vertex_shader_code,
                         const char *fragment_shader_code,
                         unsigned *shader_program);
bool CheckCompileErrors(unsigned shader, const std::string &type);

/**
 * \brief Abstract class that defines a renderer as a single camera at a defined
 * location.
 *
 * \details It creates a rendering based on the geometry stored in the
 * `RendererGeometry`, the pose of `Body` objects referenced in the
 * `RendererGeometry`, and the view of the renderer on the scene defined by
 * intrinsics and a pose. Rendering is started calling the main method
 * `StartRendering()`.
 *
 * @param renderer_geometry_ptr referenced \ref RendererGeometry object that
 * defines the geometry that is rendered and provides a *GLFW* context.
 * @param world2camera_pose pose of the camera relative to the world frame.
 * @param intrinsics focal length, principal point, and image size that define
 * the view of the renderer on the scene.
 * @param camera_ptr optional parameter that references a \ref Camera object.
 * Executing the `SetUp()` method writes the `world2camera_pose` and
 * `intrinsics` parameters from the camera to the renderer.
 * @param z_min minimum z-distance that is rendered.
 * @param z_max maximum z-distance that is rendered.
 */
class Renderer {
 public:
  // Setup method
  virtual bool SetUp() = 0;

  // Setters
  void set_name(const std::string &name);
  void set_metafile_path(const std::filesystem::path &metafile_path);
  void set_world2camera_pose(const Transform3fA &world2camera_pose);
  void set_camera2world_pose(const Transform3fA &camera2world_pose);
  void set_camera_ptr(const std::shared_ptr<Camera> &camera_ptr);
  void set_renderer_geometry_ptr(
      const std::shared_ptr<RendererGeometry> &renderer_geometry_ptr);
  void set_intrinsics(const Intrinsics &intrinsics);
  void set_z_min(float z_min);
  void set_z_max(float z_max);

  // Main methods
  virtual bool StartRendering() = 0;

  // Getters
  const std::string &name() const;
  const std::filesystem::path &metafile_path() const;
  const Transform3fA &world2camera_pose() const;
  const Transform3fA &camera2world_pose() const;
  const std::shared_ptr<Camera> &camera_ptr() const;
  const std::shared_ptr<RendererGeometry> &renderer_geometry_ptr() const;
  const Intrinsics &intrinsics() const;
  float z_min() const;
  float z_max() const;
  bool set_up() const;

  // Getters optional data
  virtual const std::vector<std::shared_ptr<Body>> &referenced_body_ptrs()
      const;

 protected:
  // Constructors
  Renderer(const std::string &name,
           const std::shared_ptr<RendererGeometry> &renderer_geometry_ptr,
           const Transform3fA &world2camera_pose, const Intrinsics &intrinsics,
           float z_min, float z_max);
  Renderer(const std::string &name,
           const std::shared_ptr<RendererGeometry> &renderer_geometry_ptr,
           const std::shared_ptr<Camera> &camera_ptr, float z_min, float z_max);
  Renderer(const std::string &name, const std::filesystem::path &metafile_path,
           const std::shared_ptr<RendererGeometry> &renderer_geometry_ptr,
           const std::shared_ptr<Camera> &camera_ptr);

  // Helper Methods
  bool InitParametersFromCamera();

  // Data
  std::string name_;
  std::filesystem::path metafile_path_{};
  std::shared_ptr<RendererGeometry> renderer_geometry_ptr_ = nullptr;
  std::shared_ptr<Camera> camera_ptr_ = nullptr;
  Transform3fA world2camera_pose_{};
  Transform3fA camera2world_pose_{};
  Intrinsics intrinsics_{};
  float z_min_ = 0.02f;  // min and max z-distance considered in clip space
  float z_max_ = 10.0f;

  // State variables
  std::mutex mutex_{};
  bool set_up_ = false;
};

/**
 * \brief Abstract \ref Renderer class that defines a full renderer that renders
 * the image according to the image width and height specified in the
 * intrinsics.
 */
class FullRenderer : public Renderer {
 public:
  // Setup method
  virtual bool SetUp() override = 0;

  // Main methods
  virtual bool StartRendering() override = 0;

 protected:
  // Constructors
  FullRenderer(const std::string &name,
               const std::shared_ptr<RendererGeometry> &renderer_geometry_ptr,
               const Transform3fA &world2camera_pose,
               const Intrinsics &intrinsics, float z_min, float z_max);
  FullRenderer(const std::string &name,
               const std::shared_ptr<RendererGeometry> &renderer_geometry_ptr,
               const std::shared_ptr<Camera> &camera_ptr, float z_min,
               float z_max);
  FullRenderer(const std::string &name,
               const std::filesystem::path &metafile_path,
               const std::shared_ptr<RendererGeometry> &renderer_geometry_ptr,
               const std::shared_ptr<Camera> &camera_ptr);

  // Helper Methods
  void CalculateProjectionMatrix();

  // Data
  Eigen::Matrix4f projection_matrix_;  // projects 3d data into clip space
};

/**
 * \brief Abstract \ref Renderer class that defines a focused renderer that
 * crops and scales the scene to focus on referenced objects with a defined
 * image size.
 *
 * \details Objects are only considered if they would be visible in the image
 * defined by the intrinsics. The left upper corner of the crop relative to the
 * image defined by the intrinsics is given by `corner_u` and `corner_v`. The
 * relative scale of the image is provided by `scale`. The visibility of bodies
 * in the rendered image can be checked using `IsBodyVisible()`.
 *
 * @param image_size image size that is rendered.
 * @param referenced_body_ptrs referenced \ref Body objects on which the scene
 * is focused.
 */
class FocusedRenderer : public Renderer {
 private:
  static constexpr float kImageSizeSafetyMargin = 1.05f;

 public:
  // Setup method
  virtual bool SetUp() override = 0;

  // Configure focused bodies
  bool AddReferencedBody(const std::shared_ptr<Body> &referenced_body_ptr);
  bool DeleteReferencedBody(const std::string &name);
  void ClearReferencedBodies();

  // Setters
  void set_image_size(int image_size);

  // Main methods
  virtual bool StartRendering() override = 0;
  bool IsBodyReferenced(const std::string &body_name) const;
  bool IsBodyVisible(const std::string &body_name) const;

  // Getters
  const std::vector<std::shared_ptr<Body>> &referenced_body_ptrs()
      const override;
  int image_size() const;
  float corner_u() const;
  float corner_v() const;
  float scale() const;

 protected:
  // Constructors
  FocusedRenderer(
      const std::string &name,
      const std::shared_ptr<RendererGeometry> &renderer_geometry_ptr,
      const Transform3fA &world2camera_pose, const Intrinsics &intrinsics,
      int image_size, float z_min, float z_max);
  FocusedRenderer(
      const std::string &name,
      const std::shared_ptr<RendererGeometry> &renderer_geometry_ptr,
      const std::shared_ptr<Camera> &camera_ptr, int image_size, float z_min,
      float z_max);
  FocusedRenderer(
      const std::string &name, const std::filesystem::path &metafile_path,
      const std::shared_ptr<RendererGeometry> &renderer_geometry_ptr,
      const std::shared_ptr<Camera> &camera_ptr);

  // Helper Methods
  void CalculateProjectionMatrix();

  // Data
  std::vector<std::shared_ptr<Body>> referenced_body_ptrs_{};
  std::vector<std::string> visible_body_names_{};
  int image_size_ = 200;
  float corner_u_{};
  float corner_v_{};
  float scale_{};
  Eigen::Matrix4f projection_matrix_{};
};

/**
 * \brief Abstract \ref Renderer class that defines a full depth renderer that
 * extends the \ref FullRenderer class with functionality specific to depth
 * renderings.
 */
class FullDepthRenderer : public FullRenderer {
 public:
  // Setup method
  virtual bool SetUp() override = 0;

  // Main methods
  virtual bool StartRendering() override = 0;
  virtual bool FetchDepthImage() = 0;
  cv::Mat NormalizedDepthImage(float min_depth, float max_depth) const;

  // Getters
  const cv::Mat &depth_image() const;

  // Getters that calculate values based on the rendered depth image
  float Depth(ushort depth_image_value) const;
  float Depth(const cv::Point2i &image_coordinate) const;
  ushort DepthImageValue(const cv::Point2i &image_coordinate) const;
  Eigen::Vector3f PointVector(const cv::Point2i &image_coordinate) const;

 protected:
  // Constructors
  FullDepthRenderer(
      const std::string &name,
      const std::shared_ptr<RendererGeometry> &renderer_geometry_ptr,
      const Transform3fA &world2camera_pose, const Intrinsics &intrinsics,
      float z_min, float z_max);
  FullDepthRenderer(
      const std::string &name,
      const std::shared_ptr<RendererGeometry> &renderer_geometry_ptr,
      const std::shared_ptr<Camera> &camera_ptr, float z_min, float z_max);
  FullDepthRenderer(
      const std::string &name, const std::filesystem::path &metafile_path,
      const std::shared_ptr<RendererGeometry> &renderer_geometry_ptr,
      const std::shared_ptr<Camera> &camera_ptr);

  // Helper methods
  void CalculateProjectionTerms();
  void ClearDepthImage();

  // Data
  cv::Mat depth_image_{};
  float projection_term_a_ = 0;
  float projection_term_b_ = 0;
};

/**
 * \brief Abstract \ref Renderer class that defines a focused depth renderer
 * that extends the \ref FocusedRenderer class with functionality specific to
 * depth renderings.
 */
class FocusedDepthRenderer : public FocusedRenderer {
 public:
  // Setup methods
  virtual bool SetUp() override = 0;

  // Main methods
  virtual bool StartRendering() override = 0;
  virtual bool FetchDepthImage() = 0;
  cv::Mat NormalizedFocusedDepthImage(float min_depth, float max_depth) const;

  // Getters
  const cv::Mat &focused_depth_image() const;

  // Getters that calculate values based on the rendered depth image for the
  // original image coordinates
  float Depth(ushort depth_image_value) const;
  float Depth(const cv::Point2i &image_coordinate) const;
  ushort DepthImageValue(const cv::Point2i &image_coordinate) const;
  Eigen::Vector3f PointVector(const cv::Point2i &image_coordinate) const;

 protected:
  // Constructors
  FocusedDepthRenderer(
      const std::string &name,
      const std::shared_ptr<RendererGeometry> &renderer_geometry_ptr,
      const Transform3fA &world2camera_pose, const Intrinsics &intrinsics,
      int image_size, float z_min, float z_max);
  FocusedDepthRenderer(
      const std::string &name,
      const std::shared_ptr<RendererGeometry> &renderer_geometry_ptr,
      const std::shared_ptr<Camera> &camera_ptr, int image_size, float z_min,
      float z_max);
  FocusedDepthRenderer(
      const std::string &name, const std::filesystem::path &metafile_path,
      const std::shared_ptr<RendererGeometry> &renderer_geometry_ptr,
      const std::shared_ptr<Camera> &camera_ptr);

  // Helper methods
  void CalculateProjectionTerms();
  void ClearDepthImage();

  // Data
  cv::Mat focused_depth_image_{};
  float projection_term_a_ = 0;
  float projection_term_b_ = 0;
};

}  // namespace m3t

#endif  // M3T_INCLUDE_M3T_RENDERER_H_
