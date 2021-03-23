// SPDX-License-Identifier: MIT
// Copyright (c) 2020 Manuel Stoiber, German Aerospace Center (DLR)

#ifndef OBJECT_TRACKING_INCLUDE_RBGT_OCCLUSION_MASK_RENDERER_H_
#define OBJECT_TRACKING_INCLUDE_RBGT_OCCLUSION_MASK_RENDERER_H_

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

// Renderer that is able to render an occlusion mask, where all bodies are
// enlarged by a certain dilation radius. The occlusion mask uses a binary
// encoding. A body is considered occluded if the bit corresponding to a body's
// occlusion_mask_id is zero and unoccluded if the bit is one. To improve
// efficiency, a resolution can be set to decrease the amount of computation
// that is required.
class OcclusionMaskRenderer : public Renderer {
 private:
  // Constants that define maximum radius in pixel for mask fragment shader
  // set to minimum to improve benchmarks
  static constexpr int kMaxEffectiveRadius = 1;
  static constexpr int kMaxTextureIterations =
      (kMaxEffectiveRadius * 2 + 1) * (kMaxEffectiveRadius * 2 + 1);

 public:
  // Constructors and destructors
  OcclusionMaskRenderer();
  ~OcclusionMaskRenderer();

  // Initialization method and setters
  bool Init(const std::string &name,
            std::shared_ptr<RendererGeometry> renderer_geometry_ptr,
            const Transform3fA &world2camera_pose, const Intrinsics &intrinsics,
            float z_min, float z_max) override;
  bool set_intrinsics(const Intrinsics &intrinsics) override;
  bool set_z_min(float z_min) override;
  bool set_z_max(float z_max) override;
  bool set_mask_resolution(int mask_resolution);
  bool set_dilation_radius(float dilation_radius);

  // Main method
  bool StartRendering() override;
  void FetchOcclusionMask();

  // Getters for mask and internal variables
  const cv::Mat &occlusion_mask() const;
  int mask_resolution() const;
  float dilation_radius() const;

 private:
  // Helper methods
  void ClearImages();
  void CalculateMaskDimensions();
  void CreateBufferObjects();
  void DeleteBufferObjects();
  void CreateVertexArrayAndBufferObjects();
  void DeleteVertexArrayAndBufferObjects();
  void GenerateTextureSteps();
  void AssignUniformVariablesToShader();

  // Mask data
  cv::Mat occlusion_mask_;

  // Parameters
  int mask_resolution_ = 4;
  float dilation_radius_ = 4.0f;

  // Shader code
  std::string vertex_shader_code_body_id_;
  std::string fragment_shader_code_body_id_;
  std::string vertex_shader_code_mask_;
  std::string fragment_shader_code_mask_;

  // OpenGL variables
  unsigned int fbo_body_id_ = 0;
  unsigned int fbo_mask_ = 0;
  unsigned int tex_body_id_ = 0;
  unsigned int tex_depth_ = 0;
  unsigned int rbo_mask_ = 0;
  unsigned int shader_program_body_id_ = 0;
  unsigned int shader_program_mask_ = 0;
  GLuint vao_texture_ = 0;
  GLuint vbo_texture_ = 0;

  // Internal variables
  int mask_width_ = 0;
  int mask_height_ = 0;
  int iterations_ = 0;
  float texture_steps_[2 * kMaxTextureIterations]{};

  // Internal state variables
  bool occlusion_mask_fetched_ = false;
  bool image_rendered_ = false;
};

}  // namespace rbgt

#endif  // OBJECT_TRACKING_INCLUDE_RBGT_OCCLUSION_MASK_RENDERER_H_
