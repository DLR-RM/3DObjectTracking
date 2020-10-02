// SPDX-License-Identifier: MIT
// Copyright (c) 2020 Manuel Stoiber, German Aerospace Center (DLR)

#include <rbgt/occlusion_mask_renderer.h>

namespace rbgt {

OcclusionMaskRenderer::OcclusionMaskRenderer() {
  vertex_shader_code_body_id_ =
      "#version 330 core\n"
      "layout(location = 0) in vec3 aPos;\n"
      "uniform mat4 Trans;\n"
      "void main()\n"
      "{\n"
      "  gl_Position = Trans * vec4(aPos, 1.0);\n"
      "}";

  fragment_shader_code_body_id_ =
      "#version 330 core\n"
      "uniform float NormalizedBodyID;\n"
      "out float FragColor;\n"
      "void main()\n"
      "{\n"
      "  FragColor = NormalizedBodyID;\n"
      "}";

  vertex_shader_code_mask_ =
      "#version 330 core\n"
      "layout(location = 0) in vec2 aPos;\n"
      "layout(location = 1) in vec2 aTexCoord;\n"
      "out vec2 TexCoord;\n"
      "void main()\n"
      "{\n"
      "	 gl_Position = vec4(aPos, 0.0, 1.0);\n"
      "	 TexCoord = aTexCoord;\n"
      "}";

  fragment_shader_code_mask_ =
      "#version 330 core\n"
      "in vec2 TexCoord;\n"
      "out float FragColor;\n"
      "uniform sampler2D BodyIDTexture;\n"
      "uniform sampler2D DepthTexture;\n"
      "uniform vec2 TextureSteps[9];\n"
      "uniform int Iterations;\n"
      ""
      "void main()\n"
      "{\n"
      "  int MinBodyID = int(texture2D(BodyIDTexture, TexCoord.st).r * "
      "                      255.0);\n"
      "  float MinDepth = texture2D(DepthTexture, TexCoord.st).r;\n"
      "	 for (int i = 0; i < Iterations; i++)\n"
      "	 {\n"
      "    int BodyID = int(texture2D(BodyIDTexture, TexCoord.st + "
      "                               TextureSteps[i]).r * 255.0);\n"
      "    float Depth = texture2D(DepthTexture, TexCoord.st + "
      "                            TextureSteps[i]).r;\n"
      "    MinBodyID = Depth < MinDepth ? BodyID : MinBodyID;\n"
      "	   MinDepth = Depth < MinDepth ? Depth : MinDepth;\n"
      "  }\n"
      "  uint MaskValue = bool(MinBodyID) ? uint(1) << MinBodyID : uint(255);\n"
      "  FragColor = float(MaskValue) / 255.0;\n"
      "}";
}

OcclusionMaskRenderer::~OcclusionMaskRenderer() {
  DeleteBufferObjects();
  DeleteVertexArrayAndBufferObjects();
}

bool OcclusionMaskRenderer::Init(
    const std::string &name,
    std::shared_ptr<RendererGeometry> renderer_geometry_ptr,
    const Transform3fA &world2camera_pose, const Intrinsics &intrinsics,
    float z_min, float z_max) {
  if (initialized_) {
    DeleteBufferObjects();
    DeleteVertexArrayAndBufferObjects();
  }
  initialized_ = false;
  image_rendered_ = false;

  if (!renderer_geometry_ptr->initialized()) {
    if (!renderer_geometry_ptr->Init()) return false;
  }

  name_ = name;
  renderer_geometry_ptr_ = renderer_geometry_ptr;
  world2camera_pose_ = world2camera_pose;
  intrinsics_ = intrinsics;
  z_min_ = z_min;
  z_max_ = z_max;

  CalculateProjectionMatrix();
  CalculateMaskDimensions();
  ClearImages();
  CreateBufferObjects();
  CreateVertexArrayAndBufferObjects();
  if (!CreateShaderProgram(vertex_shader_code_body_id_.c_str(),
                           fragment_shader_code_body_id_.c_str(),
                           &shader_program_body_id_))
    return false;
  if (!CreateShaderProgram(vertex_shader_code_mask_.c_str(),
                           fragment_shader_code_mask_.c_str(),
                           &shader_program_mask_))
    return false;
  GenerateTextureSteps();
  AssignUniformVariablesToShader();

  initialized_ = true;
  return true;
}

bool OcclusionMaskRenderer::set_intrinsics(const Intrinsics &intrinsics) {
  if (!initialized_) {
    std::cerr << "Initialize renderer first" << std::endl;
    return false;
  }
  image_rendered_ = false;
  intrinsics_ = intrinsics;
  CalculateProjectionMatrix();
  CalculateMaskDimensions();
  DeleteBufferObjects();
  CreateBufferObjects();
  ClearImages();
  GenerateTextureSteps();
  AssignUniformVariablesToShader();
  return true;
}

bool OcclusionMaskRenderer::set_z_min(float z_min) {
  if (!initialized_) {
    std::cerr << "Initialize renderer first" << std::endl;
    return false;
  }
  image_rendered_ = false;
  z_min_ = z_min;
  CalculateProjectionMatrix();
  ClearImages();
  AssignUniformVariablesToShader();
  return true;
}

bool OcclusionMaskRenderer::set_z_max(float z_max) {
  if (!initialized_) {
    std::cerr << "Initialize renderer first" << std::endl;
    return false;
  }
  image_rendered_ = false;
  z_max_ = z_max;
  CalculateProjectionMatrix();
  ClearImages();
  AssignUniformVariablesToShader();
  return true;
}

bool OcclusionMaskRenderer::set_mask_resolution(int mask_resolution) {
  int effective_radius = int(dilation_radius_ / float(mask_resolution));
  if (effective_radius > kMaxEffectiveRadius) {
    std::cerr << "dilation_radius too big or mask_resolution too small"
              << std::endl
              << "dilation_radius / mask_resolution has to be be smaller than "
              << kMaxEffectiveRadius << std::endl
              << "To increase the possible range, change the value of "
                 "kMaxEffectiveRadius in the source code and change the size "
                 "of the shader variable TextureSteps to kMaxTextureIterations."
              << std::endl;
    return false;
  }
  image_rendered_ = false;
  mask_resolution_ = mask_resolution;
  if (initialized_) {
    CalculateMaskDimensions();
    DeleteBufferObjects();
    CreateBufferObjects();
    ClearImages();
    GenerateTextureSteps();
    AssignUniformVariablesToShader();
  }
  return true;
}

bool OcclusionMaskRenderer::set_dilation_radius(float dilation_radius) {
  int effective_radius = int(dilation_radius / float(mask_resolution_));
  if (effective_radius > kMaxEffectiveRadius) {
    std::cerr << "dilation_radius too big or mask_resolution too small"
              << std::endl
              << "dilation_radius / mask_resolution has to be be smaller than "
              << kMaxEffectiveRadius << std::endl
              << "To increase the possible range, change the value of "
                 "kMaxEffectiveRadius in the source code and change the size "
                 "of the shader variable TextureSteps to kMaxTextureIterations."
              << std::endl;
    return false;
  }
  image_rendered_ = false;
  dilation_radius_ = dilation_radius;
  if (initialized_) {
    CalculateMaskDimensions();
    DeleteBufferObjects();
    CreateBufferObjects();
    ClearImages();
    GenerateTextureSteps();
    AssignUniformVariablesToShader();
  }
  return true;
}

bool OcclusionMaskRenderer::StartRendering() {
  if (!initialized_) {
    std::cerr << "Initialize renderer first" << std::endl;
    return false;
  }

  glfwMakeContextCurrent(renderer_geometry_ptr_->window());
  glViewport(0, 0, mask_width_, mask_height_);

  // Render depth image and body ids to textures
  glBindFramebuffer(GL_FRAMEBUFFER, fbo_body_id_);
  glClearColor(0.0f, 0.0f, 0.0f, 0.0f);
  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
  glEnable(GL_DEPTH_TEST);
  glFrontFace(GL_CCW);
  glCullFace(GL_FRONT);

  glUseProgram(shader_program_body_id_);
  for (const auto &render_data_body :
       renderer_geometry_ptr_->render_data_bodies()) {
    Eigen::Matrix4f trans{
        projection_matrix_ *
        (world2camera_pose_ * render_data_body.body_ptr->geometry2world_pose())
            .matrix()};

    unsigned int loc;
    loc = glGetUniformLocation(shader_program_body_id_, "Trans");
    glUniformMatrix4fv(loc, 1, GL_FALSE, trans.data());
    loc = glGetUniformLocation(shader_program_body_id_, "NormalizedBodyID");
    glUniform1f(loc,
                float(render_data_body.body_ptr->occlusion_mask_id()) / 255.0f);

    if (render_data_body.body_ptr->geometry_enable_culling())
      glEnable(GL_CULL_FACE);
    else
      glDisable(GL_CULL_FACE);

    glBindVertexArray(render_data_body.vao);
    glDrawArrays(GL_TRIANGLES, 0, render_data_body.n_vertices);
    glBindVertexArray(0);
  }

  // Compute occlusion mask from textures
  glBindFramebuffer(GL_FRAMEBUFFER, fbo_mask_);
  glDisable(GL_DEPTH_TEST);
  glDisable(GL_CULL_FACE);

  glUseProgram(shader_program_mask_);
  glBindVertexArray(vao_texture_);
  glActiveTexture(GL_TEXTURE0);
  glBindTexture(GL_TEXTURE_2D, tex_body_id_);
  glActiveTexture(GL_TEXTURE1);
  glBindTexture(GL_TEXTURE_2D, tex_depth_);

  glDrawArrays(GL_TRIANGLES, 0, 6);
  glBindVertexArray(0);

  glBindFramebuffer(GL_FRAMEBUFFER, 0);
  glfwMakeContextCurrent(nullptr);

  image_rendered_ = true;
  occlusion_mask_fetched_ = false;
  return true;
}

void OcclusionMaskRenderer::FetchOcclusionMask() {
  if (image_rendered_ && !occlusion_mask_fetched_) {
    cv::Mat mask_reduced =
        cv::Mat{cv::Size{mask_width_, mask_height_}, CV_8UC1};

    glfwMakeContextCurrent(renderer_geometry_ptr_->window());
    glPixelStorei(GL_PACK_ALIGNMENT, (mask_reduced.step & 3) ? 1 : 4);
    glPixelStorei(GL_PACK_ROW_LENGTH,
                  (GLint)(mask_reduced.step / mask_reduced.elemSize()));
    glBindFramebuffer(GL_FRAMEBUFFER, fbo_mask_);
    glBindRenderbuffer(GL_RENDERBUFFER, rbo_mask_);
    glReadPixels(0, 0, mask_reduced.cols, mask_reduced.rows, GL_RED,
                 GL_UNSIGNED_BYTE, mask_reduced.data);
    glBindRenderbuffer(GL_RENDERBUFFER, 0);
    glBindFramebuffer(GL_FRAMEBUFFER, 0);
    glfwMakeContextCurrent(nullptr);

    cv::resize(mask_reduced, occlusion_mask_,
               cv::Size{intrinsics_.width, intrinsics_.height},
               mask_resolution_, mask_resolution_,
               cv::InterpolationFlags::INTER_NEAREST);
    occlusion_mask_fetched_ = true;
  }
}

const cv::Mat &OcclusionMaskRenderer::occlusion_mask() const {
  return occlusion_mask_;
}

int OcclusionMaskRenderer::mask_resolution() const { return mask_resolution_; }

float OcclusionMaskRenderer::dilation_radius() const {
  return dilation_radius_;
}

void OcclusionMaskRenderer::ClearImages() {
  occlusion_mask_.create(cv::Size{intrinsics_.width, intrinsics_.height},
                         CV_8UC1);
  occlusion_mask_.setTo(cv::Scalar{0});
}

void OcclusionMaskRenderer::CalculateMaskDimensions() {
  mask_width_ = intrinsics_.width / mask_resolution_;
  mask_height_ = intrinsics_.height / mask_resolution_;
}

void OcclusionMaskRenderer::CreateBufferObjects() {
  glfwMakeContextCurrent(renderer_geometry_ptr_->window());

  // Initialize texture and renderbuffer bodies_render_data
  glGenTextures(1, &tex_body_id_);
  glBindTexture(GL_TEXTURE_2D, tex_body_id_);
  glTexImage2D(GL_TEXTURE_2D, 0, GL_R8, mask_width_, mask_height_, 0, GL_RED,
               GL_UNSIGNED_BYTE, nullptr);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
  glBindTexture(GL_TEXTURE_2D, 0);

  glGenTextures(1, &tex_depth_);
  glBindTexture(GL_TEXTURE_2D, tex_depth_);
  glTexImage2D(GL_TEXTURE_2D, 0, GL_DEPTH_COMPONENT16, mask_width_,
               mask_height_, 0, GL_DEPTH_COMPONENT, GL_FLOAT, nullptr);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
  glBindTexture(GL_TEXTURE_2D, 0);

  glGenRenderbuffers(1, &rbo_mask_);
  glBindRenderbuffer(GL_RENDERBUFFER, rbo_mask_);
  glRenderbufferStorage(GL_RENDERBUFFER, GL_R8, mask_width_, mask_height_);
  glBindRenderbuffer(GL_RENDERBUFFER, 0);

  // Initialize framebuffer bodies_render_data
  glGenFramebuffers(1, &fbo_body_id_);
  glBindFramebuffer(GL_FRAMEBUFFER, fbo_body_id_);
  glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D,
                         tex_body_id_, 0);
  glFramebufferTexture2D(GL_FRAMEBUFFER, GL_DEPTH_ATTACHMENT, GL_TEXTURE_2D,
                         tex_depth_, 0);
  glBindFramebuffer(GL_FRAMEBUFFER, 0);

  glGenFramebuffers(1, &fbo_mask_);
  glBindFramebuffer(GL_FRAMEBUFFER, fbo_mask_);
  glFramebufferRenderbuffer(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0,
                            GL_RENDERBUFFER, rbo_mask_);
  glBindFramebuffer(GL_FRAMEBUFFER, 0);
  glfwMakeContextCurrent(nullptr);
}

void OcclusionMaskRenderer::DeleteBufferObjects() {
  if (renderer_geometry_ptr_ != nullptr) {
    glfwMakeContextCurrent(renderer_geometry_ptr_->window());
    glDeleteTextures(1, &tex_body_id_);
    glDeleteTextures(1, &tex_depth_);
    glDeleteRenderbuffers(1, &rbo_mask_);
    glDeleteFramebuffers(1, &fbo_body_id_);
    glDeleteFramebuffers(1, &fbo_mask_);
    glfwMakeContextCurrent(nullptr);
  }
}

void OcclusionMaskRenderer::CreateVertexArrayAndBufferObjects() {
  glfwMakeContextCurrent(renderer_geometry_ptr_->window());
  float vertices_texture[] = {// positions, texCoords
                              -1.0f, 1.0f, 0.0f, 1.0f,  -1.0f, -1.0f,
                              0.0f,  0.0f, 1.0f, -1.0f, 1.0f,  0.0f,
                              -1.0f, 1.0f, 0.0f, 1.0f,  1.0f,  -1.0f,
                              1.0f,  0.0f, 1.0f, 1.0f,  1.0f,  1.0f};

  glGenVertexArrays(1, &vao_texture_);
  glBindVertexArray(vao_texture_);

  glGenBuffers(1, &vbo_texture_);
  glBindBuffer(GL_ARRAY_BUFFER, vbo_texture_);
  glBufferData(GL_ARRAY_BUFFER, sizeof(vertices_texture), vertices_texture,
               GL_STATIC_DRAW);

  glVertexAttribPointer(0, 2, GL_FLOAT, GL_FALSE, 4 * sizeof(float), nullptr);
  glEnableVertexAttribArray(0);
  glVertexAttribPointer(1, 2, GL_FLOAT, GL_FALSE, 4 * sizeof(float),
                        (void *)(2 * sizeof(float)));
  glEnableVertexAttribArray(1);

  glBindBuffer(GL_ARRAY_BUFFER, 0);
  glBindVertexArray(0);
  glfwMakeContextCurrent(nullptr);
}

void OcclusionMaskRenderer::DeleteVertexArrayAndBufferObjects() {
  glDeleteBuffers(1, &vbo_texture_);
  glDeleteVertexArrays(1, &vao_texture_);
}

void OcclusionMaskRenderer::GenerateTextureSteps() {
  float radius = dilation_radius_ / float(mask_resolution_);
  int step = 2 * int(radius) + 1;
  iterations_ = 0;
  for (int i = 0; i < step * step; ++i) {
    int x_pos = int(i / step) - int(radius);
    int y_pos = int(i % step) - int(radius);
    if (pow(x_pos, 2) + pow(y_pos, 2) <= powf(radius, 2.0f)) {
      texture_steps_[iterations_ * 2] = float(x_pos) / float(mask_width_);
      texture_steps_[iterations_ * 2 + 1] = float(y_pos) / float(mask_height_);
      iterations_++;
    }
  }
}

void OcclusionMaskRenderer::AssignUniformVariablesToShader() {
  glfwMakeContextCurrent(renderer_geometry_ptr_->window());
  glUseProgram(shader_program_mask_);
  unsigned int loc;
  loc = glGetUniformLocation(shader_program_mask_, "BodyIDTexture");
  glUniform1i(loc, 0);
  loc = glGetUniformLocation(shader_program_mask_, "DepthTexture");
  glUniform1i(loc, 1);
  loc = glGetUniformLocation(shader_program_mask_, "TextureSteps");
  glUniform2fv(loc, kMaxTextureIterations, texture_steps_);
  loc = glGetUniformLocation(shader_program_mask_, "Iterations");
  glUniform1i(loc, iterations_);
  glUseProgram(0);
  glfwMakeContextCurrent(nullptr);
}

}  // namespace rbgt
