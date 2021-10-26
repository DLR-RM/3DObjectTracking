// SPDX-License-Identifier: MIT
// Copyright (c) 2021 Manuel Stoiber, German Aerospace Center (DLR)

#include <srt3d/occlusion_renderer.h>

namespace srt3d {

std::string OcclusionRenderer::vertex_shader_code_body_id_ =
    "#version 330 core\n"
    "layout(location = 0) in vec3 aPos;\n"
    "uniform mat4 Trans;\n"
    "void main()\n"
    "{\n"
    "  gl_Position = Trans * vec4(aPos, 1.0);\n"
    "}";

std::string OcclusionRenderer::fragment_shader_code_body_id_ =
    "#version 330 core\n"
    "uniform float NormalizedBodyID;\n"
    "out float FragColor;\n"
    "void main()\n"
    "{\n"
    "  FragColor = NormalizedBodyID;\n"
    "}";

std::string OcclusionRenderer::vertex_shader_code_mask_ =
    "#version 330 core\n"
    "layout(location = 0) in vec2 aPos;\n"
    "layout(location = 1) in vec2 aTexCoord;\n"
    "out vec2 TexCoord;\n"
    "void main()\n"
    "{\n"
    "	 gl_Position = vec4(aPos, 0.0, 1.0);\n"
    "	 TexCoord = aTexCoord;\n"
    "}";

std::string OcclusionRenderer::fragment_shader_code_mask_ =
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

OcclusionRenderer::OcclusionRenderer(
    const std::string &name,
    std::shared_ptr<RendererGeometry> renderer_geometry_ptr,
    const Transform3fA &world2camera_pose, const Intrinsics &intrinsics,
    float z_min, float z_max, int mask_resolution, float dilation_radius)
    : Renderer{name,
               std::move(renderer_geometry_ptr),
               world2camera_pose,
               intrinsics,
               z_min,
               z_max},
      mask_resolution_{mask_resolution},
      dilation_radius_{dilation_radius} {}

OcclusionRenderer::OcclusionRenderer(
    const std::string &name,
    std::shared_ptr<RendererGeometry> renderer_geometry_ptr,
    std::shared_ptr<Camera> camera_ptr, float z_min, float z_max,
    int mask_resolution, float dilation_radius)
    : Renderer{name, std::move(renderer_geometry_ptr), camera_ptr, z_min,
               z_max},
      mask_resolution_{mask_resolution},
      dilation_radius_{dilation_radius} {}

OcclusionRenderer::~OcclusionRenderer() {
  const std::lock_guard<std::mutex> lock{mutex_};
  if (initial_set_up_) {
    DeleteBufferObjects();
    DeleteVertexArrayAndBufferObjects();
  }
}

bool OcclusionRenderer::SetUp() {
  const std::lock_guard<std::mutex> lock{mutex_};
  set_up_ = false;
  image_rendered_ = false;

  // Check if all required objects are set up
  if (!renderer_geometry_ptr_->set_up()) {
    std::cerr << "Renderer geometry " << renderer_geometry_ptr_->name()
              << " was not set up" << std::endl;
    return false;
  }
  if (camera_ptr_) {
    if (!InitParametersFromCamera()) return false;
  }

  // Create shader programs
  if (!initial_set_up_) {
    if (!CreateShaderProgram(vertex_shader_code_body_id_.c_str(),
                             fragment_shader_code_body_id_.c_str(),
                             &shader_program_body_id_))
      return false;
    if (!CreateShaderProgram(vertex_shader_code_mask_.c_str(),
                             fragment_shader_code_mask_.c_str(),
                             &shader_program_mask_))
      return false;
  }

  // Set up everything
  CalculateProjectionMatrix();
  CalculateMaskDimensions();
  ClearImages();
  if (initial_set_up_) DeleteBufferObjects();
  CreateBufferObjects();
  if (initial_set_up_) DeleteVertexArrayAndBufferObjects();
  CreateVertexArrayAndBufferObjects();
  GenerateTextureSteps();
  AssignUniformVariablesToShader();

  initial_set_up_ = true;
  set_up_ = true;
  return true;
}

bool OcclusionRenderer::set_mask_resolution(int mask_resolution) {
  const std::lock_guard<std::mutex> lock{mutex_};
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
  mask_resolution_ = mask_resolution;
  set_up_ = false;
  return true;
}

bool OcclusionRenderer::set_dilation_radius(float dilation_radius) {
  const std::lock_guard<std::mutex> lock{mutex_};
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
  dilation_radius_ = dilation_radius;
  set_up_ = false;
  return true;
}

bool OcclusionRenderer::StartRendering() {
  const std::lock_guard<std::mutex> lock{mutex_};
  if (!set_up_) {
    std::cerr << "Set up renderer " << name_ << " first" << std::endl;
    return false;
  }

  renderer_geometry_ptr_->MakeContextCurrent();
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

    unsigned loc;
    loc = glGetUniformLocation(shader_program_body_id_, "Trans");
    glUniformMatrix4fv(loc, 1, GL_FALSE, trans.data());
    loc = glGetUniformLocation(shader_program_body_id_, "NormalizedBodyID");
    glUniform1f(loc, float(render_data_body.body_ptr->occlusion_id()) / 255.0f);

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
  renderer_geometry_ptr_->DetachContext();

  image_rendered_ = true;
  occlusion_mask_fetched_ = false;
  return true;
}

bool OcclusionRenderer::FetchOcclusionMask() {
  const std::lock_guard<std::mutex> lock{mutex_};
  if (!set_up_ || !image_rendered_) return false;
  if (occlusion_mask_fetched_) return true;
  renderer_geometry_ptr_->MakeContextCurrent();
  glPixelStorei(GL_PACK_ALIGNMENT, (occlusion_mask_.step & 3) ? 1 : 4);
  glPixelStorei(GL_PACK_ROW_LENGTH,
                GLint(occlusion_mask_.step / occlusion_mask_.elemSize()));
  glBindFramebuffer(GL_FRAMEBUFFER, fbo_mask_);
  glBindRenderbuffer(GL_RENDERBUFFER, rbo_mask_);
  glReadPixels(0, 0, occlusion_mask_.cols, occlusion_mask_.rows, GL_RED,
               GL_UNSIGNED_BYTE, occlusion_mask_.data);
  glBindRenderbuffer(GL_RENDERBUFFER, 0);
  glBindFramebuffer(GL_FRAMEBUFFER, 0);
  renderer_geometry_ptr_->DetachContext();
  occlusion_mask_fetched_ = true;
  return true;
}

const cv::Mat &OcclusionRenderer::occlusion_mask() const {
  return occlusion_mask_;
}

int OcclusionRenderer::mask_resolution() const { return mask_resolution_; }

float OcclusionRenderer::dilation_radius() const { return dilation_radius_; }

uchar OcclusionRenderer::GetValue(int v_unscaled, int u_unscaled) const {
  return occlusion_mask_.at<uchar>(v_unscaled / mask_resolution_,
                                   u_unscaled / mask_resolution_);
}

void OcclusionRenderer::ClearImages() {
  occlusion_mask_.create(cv::Size{mask_width_, mask_height_}, CV_8UC1);
  occlusion_mask_.setTo(cv::Scalar{0});
}

void OcclusionRenderer::CalculateMaskDimensions() {
  mask_width_ = intrinsics_.width / mask_resolution_;
  mask_height_ = intrinsics_.height / mask_resolution_;
}

void OcclusionRenderer::CreateBufferObjects() {
  renderer_geometry_ptr_->MakeContextCurrent();

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
  renderer_geometry_ptr_->DetachContext();
}

void OcclusionRenderer::DeleteBufferObjects() {
  if (renderer_geometry_ptr_ != nullptr) {
    renderer_geometry_ptr_->MakeContextCurrent();
    glDeleteTextures(1, &tex_body_id_);
    glDeleteTextures(1, &tex_depth_);
    glDeleteRenderbuffers(1, &rbo_mask_);
    glDeleteFramebuffers(1, &fbo_body_id_);
    glDeleteFramebuffers(1, &fbo_mask_);
    renderer_geometry_ptr_->DetachContext();
  }
}

void OcclusionRenderer::CreateVertexArrayAndBufferObjects() {
  renderer_geometry_ptr_->MakeContextCurrent();
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
  renderer_geometry_ptr_->DetachContext();
}

void OcclusionRenderer::DeleteVertexArrayAndBufferObjects() {
  renderer_geometry_ptr_->MakeContextCurrent();
  glDeleteBuffers(1, &vbo_texture_);
  glDeleteVertexArrays(1, &vao_texture_);
  renderer_geometry_ptr_->DetachContext();
}

void OcclusionRenderer::GenerateTextureSteps() {
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

void OcclusionRenderer::AssignUniformVariablesToShader() {
  renderer_geometry_ptr_->MakeContextCurrent();
  glUseProgram(shader_program_mask_);
  unsigned loc;
  loc = glGetUniformLocation(shader_program_mask_, "BodyIDTexture");
  glUniform1i(loc, 0);
  loc = glGetUniformLocation(shader_program_mask_, "DepthTexture");
  glUniform1i(loc, 1);
  loc = glGetUniformLocation(shader_program_mask_, "TextureSteps");
  glUniform2fv(loc, kMaxTextureIterations, texture_steps_.data());
  loc = glGetUniformLocation(shader_program_mask_, "Iterations");
  glUniform1i(loc, iterations_);
  glUseProgram(0);
  renderer_geometry_ptr_->DetachContext();
}

}  // namespace srt3d
