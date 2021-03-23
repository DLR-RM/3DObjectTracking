// SPDX-License-Identifier: MIT
// Copyright (c) 2020 Manuel Stoiber, German Aerospace Center (DLR)

#include <rbgt/normal_image_renderer.h>

namespace rbgt {

NormalImageRenderer::NormalImageRenderer() {
  vertex_shader_code_ =
      "#version 330 core\n"
      "layout(location = 0) in vec3 aPos;\n"
      "layout(location = 1) in vec3 aNormal;\n"
      "flat out vec3 Normal;\n"
      "uniform mat4 Trans;\n"
      "uniform mat3 Rot;\n"
      "void main()\n"
      "{\n"
      "  gl_Position = Trans * vec4(aPos, 1.0);\n"
      "  Normal = Rot * aNormal;\n"
      "}";

  fragment_shader_code_ =
      "#version 330 core\n"
      "flat in vec3 Normal;\n"
      "out vec4 FragColor;\n"
      "void main()\n"
      "{\n"
      "	 FragColor = vec4(0.5 - 0.5 * Normal, 1.0).zyxw;\n"
      "}";
}

NormalImageRenderer::~NormalImageRenderer() { DeleteBufferObjects(); }

bool NormalImageRenderer::Init(
    const std::string &name,
    std::shared_ptr<RendererGeometry> renderer_geometry_ptr,
    const Transform3fA &world2camera_pose, const Intrinsics &intrinsics,
    float z_min, float z_max) {
  if (initialized_) DeleteBufferObjects();
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
  CalculateProjectionTerms();
  ClearImages();
  CreateBufferObjects();
  if (!CreateShaderProgram(vertex_shader_code_.c_str(),
                           fragment_shader_code_.c_str(), &shader_program_))
    return false;

  initialized_ = true;
  return true;
}

bool NormalImageRenderer::set_intrinsics(const Intrinsics &intrinsics) {
  if (!initialized_) {
    std::cerr << "Initialize renderer first" << std::endl;
    return false;
  }
  image_rendered_ = false;
  intrinsics_ = intrinsics;
  CalculateProjectionMatrix();
  DeleteBufferObjects();
  CreateBufferObjects();
  ClearImages();
  return true;
}

bool NormalImageRenderer::set_z_min(float z_min) {
  if (!initialized_) {
    std::cerr << "Initialize renderer first" << std::endl;
    return false;
  }
  image_rendered_ = false;
  z_min_ = z_min;
  CalculateProjectionMatrix();
  CalculateProjectionTerms();
  ClearImages();
  return true;
}

bool NormalImageRenderer::set_z_max(float z_max) {
  if (!initialized_) {
    std::cerr << "Initialize renderer first" << std::endl;
    return false;
  }
  image_rendered_ = false;
  z_max_ = z_max;
  CalculateProjectionMatrix();
  CalculateProjectionTerms();
  ClearImages();
  return true;
}

void NormalImageRenderer::set_depth_scale(float depth_scale) {
  image_rendered_ = false;
  depth_scale_ = depth_scale;
  if (initialized_) {
    CalculateProjectionTerms();
    ClearImages();
  }
}

float NormalImageRenderer::depth_scale() const { return depth_scale_; }

bool NormalImageRenderer::StartRendering() {
  if (!initialized_) {
    std::cerr << "Initialize renderer first" << std::endl;
    return false;
  }

  glfwMakeContextCurrent(renderer_geometry_ptr_->window());
  glViewport(0, 0, intrinsics_.width, intrinsics_.height);

  glBindFramebuffer(GL_FRAMEBUFFER, fbo_);
  glClearColor(0.0f, 0.0f, 0.0f, 0.0f);
  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
  glEnable(GL_DEPTH_TEST);
  glFrontFace(GL_CCW);
  glCullFace(GL_FRONT);

  glUseProgram(shader_program_);
  for (const auto &render_data_body :
       renderer_geometry_ptr_->render_data_bodies()) {
    Transform3fA trans_without_projection{
        world2camera_pose_ * render_data_body.body_ptr->geometry2world_pose()};
    Eigen::Matrix4f trans{projection_matrix_ *
                          trans_without_projection.matrix()};
    Eigen::Matrix3f rot{trans_without_projection.rotation().matrix()};

    unsigned int loc;
    loc = glGetUniformLocation(shader_program_, "Trans");
    glUniformMatrix4fv(loc, 1, GL_FALSE, trans.data());
    loc = glGetUniformLocation(shader_program_, "Rot");
    glUniformMatrix3fv(loc, 1, GL_FALSE, rot.data());

    if (render_data_body.body_ptr->geometry_enable_culling())
      glEnable(GL_CULL_FACE);
    else
      glDisable(GL_CULL_FACE);

    glBindVertexArray(render_data_body.vao);
    glDrawArrays(GL_TRIANGLES, 0, render_data_body.n_vertices);
    glBindVertexArray(0);
  }
  glBindFramebuffer(GL_FRAMEBUFFER, 0);
  glfwMakeContextCurrent(nullptr);

  image_rendered_ = true;
  normal_image_fetched_ = false;
  depth_image_fetched_ = false;
  return true;
}

void NormalImageRenderer::FetchNormalImage() {
  if (image_rendered_ && !normal_image_fetched_) {
    glfwMakeContextCurrent(renderer_geometry_ptr_->window());
    glPixelStorei(GL_PACK_ALIGNMENT, (normal_image_.step & 3) ? 1 : 4);
    glPixelStorei(GL_PACK_ROW_LENGTH,
                  (GLint)(normal_image_.step / normal_image_.elemSize()));
    glBindFramebuffer(GL_FRAMEBUFFER, fbo_);
    glBindRenderbuffer(GL_RENDERBUFFER, rbo_normal_);
    glReadPixels(0, 0, normal_image_.cols, normal_image_.rows, GL_BGRA,
                 GL_UNSIGNED_BYTE, normal_image_.data);
    glBindRenderbuffer(GL_RENDERBUFFER, 0);
    glBindFramebuffer(GL_FRAMEBUFFER, 0);
    glfwMakeContextCurrent(nullptr);
    normal_image_fetched_ = true;
  }
}

void NormalImageRenderer::FetchDepthImage() {
  if (image_rendered_ && !depth_image_fetched_) {
    glfwMakeContextCurrent(renderer_geometry_ptr_->window());
    glPixelStorei(GL_PACK_ALIGNMENT, (depth_image_.step & 3) ? 1 : 4);
    glPixelStorei(GL_PACK_ROW_LENGTH,
                  (GLint)(depth_image_.step / depth_image_.elemSize()));
    glBindFramebuffer(GL_FRAMEBUFFER, fbo_);
    glBindRenderbuffer(GL_RENDERBUFFER, rbo_depth_);
    glReadPixels(0, 0, depth_image_.cols, depth_image_.rows, GL_DEPTH_COMPONENT,
                 GL_UNSIGNED_SHORT, depth_image_.data);
    glBindRenderbuffer(GL_RENDERBUFFER, 0);
    glBindFramebuffer(GL_FRAMEBUFFER, 0);
    glfwMakeContextCurrent(nullptr);
    depth_image_fetched_ = true;
  }
}

const cv::Mat &NormalImageRenderer::normal_image() const {
  return normal_image_;
}

const cv::Mat &NormalImageRenderer::depth_image() const { return depth_image_; }

Eigen::Vector3f NormalImageRenderer::GetPointVector(
    const cv::Point2i &image_coordinate) const {
  float depth = depth_image_.at<ushort>(image_coordinate);
  depth = (projection_term_a_ / (projection_term_b_ - depth)) / depth_scale_;
  return Eigen::Vector3f{
      depth * (image_coordinate.x - intrinsics_.ppu) / intrinsics_.fu,
      depth * (image_coordinate.y - intrinsics_.ppv) / intrinsics_.fv, depth};
}

void NormalImageRenderer::ClearImages() {
  normal_image_.create(cv::Size{intrinsics_.width, intrinsics_.height},
                       CV_8UC4);
  normal_image_.setTo(cv::Vec4b{0, 0, 0, 0});
  depth_image_.create(cv::Size{intrinsics_.width, intrinsics_.height}, CV_16U);
  depth_image_.setTo(cv::Scalar{0});
}

void NormalImageRenderer::CalculateProjectionTerms() {
  projection_term_a_ =
      depth_scale_ * z_max_ * z_min_ * USHRT_MAX / (z_max_ - z_min_);
  projection_term_b_ = z_max_ * USHRT_MAX / (z_max_ - z_min_);
}

void NormalImageRenderer::CreateBufferObjects() {
  glfwMakeContextCurrent(renderer_geometry_ptr_->window());

  // Initialize renderbuffer bodies_render_data
  glGenRenderbuffers(1, &rbo_normal_);
  glBindRenderbuffer(GL_RENDERBUFFER, rbo_normal_);
  glRenderbufferStorage(GL_RENDERBUFFER, GL_RGBA8, intrinsics_.width,
                        intrinsics_.height);
  glBindRenderbuffer(GL_RENDERBUFFER, 0);

  glGenRenderbuffers(1, &rbo_depth_);
  glBindRenderbuffer(GL_RENDERBUFFER, rbo_depth_);
  glRenderbufferStorage(GL_RENDERBUFFER, GL_DEPTH_COMPONENT16,
                        intrinsics_.width, intrinsics_.height);
  glBindRenderbuffer(GL_RENDERBUFFER, 0);

  // Initialize framebuffer bodies_render_data
  glGenFramebuffers(1, &fbo_);
  glBindFramebuffer(GL_FRAMEBUFFER, fbo_);
  glFramebufferRenderbuffer(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0,
                            GL_RENDERBUFFER, rbo_normal_);
  glFramebufferRenderbuffer(GL_FRAMEBUFFER, GL_DEPTH_ATTACHMENT,
                            GL_RENDERBUFFER, rbo_depth_);
  glBindFramebuffer(GL_FRAMEBUFFER, 0);
  glfwMakeContextCurrent(nullptr);
}

void NormalImageRenderer::DeleteBufferObjects() {
  if (renderer_geometry_ptr_ != nullptr) {
    glfwMakeContextCurrent(renderer_geometry_ptr_->window());
    glDeleteRenderbuffers(1, &rbo_normal_);
    glDeleteRenderbuffers(1, &rbo_depth_);
    glDeleteFramebuffers(1, &fbo_);
    glfwMakeContextCurrent(nullptr);
  }
}

}  // namespace rbgt
