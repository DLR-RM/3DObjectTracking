// SPDX-License-Identifier: MIT
// Copyright (c) 2021 Manuel Stoiber, German Aerospace Center (DLR)

#include <srt3d/normal_renderer.h>

namespace srt3d {

std::string NormalRenderer::vertex_shader_code_ =
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

std::string NormalRenderer::fragment_shader_code_ =
    "#version 330 core\n"
    "flat in vec3 Normal;\n"
    "out vec4 FragColor;\n"
    "void main()\n"
    "{\n"
    "	 FragColor = vec4(0.5 - 0.5 * Normal, 1.0).zyxw;\n"
    "}";

NormalRenderer::NormalRenderer(
    const std::string &name,
    std::shared_ptr<RendererGeometry> renderer_geometry_ptr,
    const Transform3fA &world2camera_pose, const Intrinsics &intrinsics,
    float z_min, float z_max, float depth_scale)
    : Renderer{name,
               std::move(renderer_geometry_ptr),
               world2camera_pose,
               intrinsics,
               z_min,
               z_max},
      depth_scale_{depth_scale} {}

NormalRenderer::NormalRenderer(
    const std::string &name,
    std::shared_ptr<RendererGeometry> renderer_geometry_ptr,
    std::shared_ptr<Camera> camera_ptr, float z_min, float z_max,
    float depth_scale)
    : Renderer{name, std::move(renderer_geometry_ptr), std::move(camera_ptr),
               z_min, z_max},
      depth_scale_{depth_scale} {}

NormalRenderer::~NormalRenderer() {
  const std::lock_guard<std::mutex> lock{mutex_};
  if (initial_set_up_) DeleteBufferObjects();
}

bool NormalRenderer::SetUp() {
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
    if (!CreateShaderProgram(vertex_shader_code_.c_str(),
                             fragment_shader_code_.c_str(), &shader_program_))
      return false;
  }

  // Set up everything
  CalculateProjectionMatrix();
  CalculateProjectionTerms();
  ClearImages();
  if (initial_set_up_) DeleteBufferObjects();
  CreateBufferObjects();

  initial_set_up_ = true;
  set_up_ = true;
  return true;
}

void NormalRenderer::set_depth_scale(float depth_scale) {
  const std::lock_guard<std::mutex> lock{mutex_};
  depth_scale_ = depth_scale;
  set_up_ = false;
}

bool NormalRenderer::StartRendering() {
  const std::lock_guard<std::mutex> lock{mutex_};
  if (!set_up_) {
    std::cerr << "Set up renderer " << name_ << " first" << std::endl;
    return false;
  }

  renderer_geometry_ptr_->MakeContextCurrent();
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

    unsigned loc;
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
  renderer_geometry_ptr_->DetachContext();

  image_rendered_ = true;
  normal_image_fetched_ = false;
  depth_image_fetched_ = false;
  return true;
}

bool NormalRenderer::FetchNormalImage() {
  const std::lock_guard<std::mutex> lock{mutex_};
  if (!set_up_ || !image_rendered_) return false;
  if (normal_image_fetched_) return true;
  renderer_geometry_ptr_->MakeContextCurrent();
  glPixelStorei(GL_PACK_ALIGNMENT, (normal_image_.step & 3) ? 1 : 4);
  glPixelStorei(GL_PACK_ROW_LENGTH,
                GLint(normal_image_.step / normal_image_.elemSize()));
  glBindFramebuffer(GL_FRAMEBUFFER, fbo_);
  glBindRenderbuffer(GL_RENDERBUFFER, rbo_normal_);
  glReadPixels(0, 0, normal_image_.cols, normal_image_.rows, GL_BGRA,
               GL_UNSIGNED_BYTE, normal_image_.data);
  glBindRenderbuffer(GL_RENDERBUFFER, 0);
  glBindFramebuffer(GL_FRAMEBUFFER, 0);
  renderer_geometry_ptr_->DetachContext();
  normal_image_fetched_ = true;
  return true;
}

bool NormalRenderer::FetchDepthImage() {
  const std::lock_guard<std::mutex> lock{mutex_};
  if (!set_up_ || !image_rendered_) return false;
  if (depth_image_fetched_) return true;
  renderer_geometry_ptr_->MakeContextCurrent();
  glPixelStorei(GL_PACK_ALIGNMENT, (depth_image_.step & 3) ? 1 : 4);
  glPixelStorei(GL_PACK_ROW_LENGTH,
                GLint(depth_image_.step / depth_image_.elemSize()));
  glBindFramebuffer(GL_FRAMEBUFFER, fbo_);
  glBindRenderbuffer(GL_RENDERBUFFER, rbo_depth_);
  glReadPixels(0, 0, depth_image_.cols, depth_image_.rows, GL_DEPTH_COMPONENT,
               GL_UNSIGNED_SHORT, depth_image_.data);
  glBindRenderbuffer(GL_RENDERBUFFER, 0);
  glBindFramebuffer(GL_FRAMEBUFFER, 0);
  renderer_geometry_ptr_->DetachContext();
  depth_image_fetched_ = true;
  return true;
}

const cv::Mat &NormalRenderer::normal_image() const { return normal_image_; }

const cv::Mat &NormalRenderer::depth_image() const { return depth_image_; }

float NormalRenderer::depth_scale() const { return depth_scale_; }

Eigen::Vector3f NormalRenderer::GetPointVector(
    const cv::Point2i &image_coordinate) const {
  float depth = depth_image_.at<ushort>(image_coordinate);
  depth = (projection_term_a_ / (projection_term_b_ - depth)) / depth_scale_;
  return Eigen::Vector3f{
      depth * (image_coordinate.x - intrinsics_.ppu) / intrinsics_.fu,
      depth * (image_coordinate.y - intrinsics_.ppv) / intrinsics_.fv, depth};
}

void NormalRenderer::ClearImages() {
  normal_image_.create(cv::Size{intrinsics_.width, intrinsics_.height},
                       CV_8UC4);
  normal_image_.setTo(cv::Vec4b{0, 0, 0, 0});
  depth_image_.create(cv::Size{intrinsics_.width, intrinsics_.height}, CV_16U);
  depth_image_.setTo(cv::Scalar{0});
}

void NormalRenderer::CalculateProjectionTerms() {
  projection_term_a_ =
      depth_scale_ * z_max_ * z_min_ * USHRT_MAX / (z_max_ - z_min_);
  projection_term_b_ = z_max_ * USHRT_MAX / (z_max_ - z_min_);
}

void NormalRenderer::CreateBufferObjects() {
  renderer_geometry_ptr_->MakeContextCurrent();

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
  renderer_geometry_ptr_->DetachContext();
}

void NormalRenderer::DeleteBufferObjects() {
  if (renderer_geometry_ptr_ != nullptr) {
    renderer_geometry_ptr_->MakeContextCurrent();
    glDeleteRenderbuffers(1, &rbo_normal_);
    glDeleteRenderbuffers(1, &rbo_depth_);
    glDeleteFramebuffers(1, &fbo_);
    renderer_geometry_ptr_->DetachContext();
  }
}

}  // namespace srt3d
