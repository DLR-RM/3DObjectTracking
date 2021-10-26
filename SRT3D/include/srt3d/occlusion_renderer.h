// SPDX-License-Identifier: MIT
// Copyright (c) 2021 Manuel Stoiber, German Aerospace Center (DLR)

#ifndef OBJECT_TRACKING_INCLUDE_SRT3D_OCCLUSION_RENDERER_H_
#define OBJECT_TRACKING_INCLUDE_SRT3D_OCCLUSION_RENDERER_H_

#include <GL/glew.h>
#include <GLFW/glfw3.h>
#include <srt3d/body.h>
#include <srt3d/camera.h>
#include <srt3d/common.h>
#include <srt3d/renderer.h>
#include <srt3d/renderer_geometry.h>

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <array>
#include <iostream>
#include <memory>
#include <opencv2/opencv.hpp>
#include <string>
#include <vector>

namespace srt3d {

// Renderer that is able to render an occlusion mask, where all bodies are
// enlarged by a certain dilation radius. The occlusion mask uses a binary
// encoding. A body is considered occluded if the bit corresponding to a body's
// occlusion_mask_id is zero and unoccluded if the bit is one. To improve
// efficiency, a resolution can be set to decrease the amount of computation
// that is required.
class OcclusionRenderer : public Renderer {
 private:
  // Constants that define maximum radius in pixel for mask fragment shader
  // set to minimum to improve benchmarks
  static constexpr int kMaxEffectiveRadius = 1;
  static constexpr int kMaxTextureIterations =
      (kMaxEffectiveRadius * 2 + 1) * (kMaxEffectiveRadius * 2 + 1);

 public:
  // Constructors, destructor, and setup method
  OcclusionRenderer(const std::string &name,
                    std::shared_ptr<RendererGeometry> renderer_geometry_ptr,
                    const Transform3fA &world2camera_pose,
                    const Intrinsics &intrinsics, float z_min = 0.01f,
                    float z_max = 5.0f, int mask_resolution = 4,
                    float dilation_radius = 4.0f);
  OcclusionRenderer(const std::string &name,
                    std::shared_ptr<RendererGeometry> renderer_geometry_ptr,
                    std::shared_ptr<Camera> camera_ptr, float z_min = 0.01f,
                    float z_max = 5.0f, int mask_resolution = 4,
                    float dilation_radius = 4.0f);
  ~OcclusionRenderer();
  bool SetUp() override;

  // Setters
  bool set_mask_resolution(int mask_resolution);
  bool set_dilation_radius(float dilation_radius);

  // Main method
  bool StartRendering() override;
  bool FetchOcclusionMask();

  // Getters
  const cv::Mat &occlusion_mask() const;
  int mask_resolution() const;
  float dilation_radius() const;

  // Getter that considers the mask resolution to return a single mask value
  uchar GetValue(int v_unscaled, int u_unscaled) const;

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
  static std::string vertex_shader_code_body_id_;
  static std::string fragment_shader_code_body_id_;
  static std::string vertex_shader_code_mask_;
  static std::string fragment_shader_code_mask_;

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
  std::array<float, 2 * kMaxTextureIterations> texture_steps_{};

  // Internal state variables
  bool occlusion_mask_fetched_ = false;
};

}  // namespace srt3d

#endif  // OBJECT_TRACKING_INCLUDE_SRT3D_OCCLUSION_RENDERER_H_
