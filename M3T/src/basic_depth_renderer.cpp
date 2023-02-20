// SPDX-License-Identifier: MIT
// Copyright (c) 2023 Manuel Stoiber, German Aerospace Center (DLR)

#include <m3t/basic_depth_renderer.h>

#include <GL/glew.h>
#include <GLFW/glfw3.h>

namespace m3t {

std::string BasicDepthRendererCore::vertex_shader_code_ =
    "#version 330 core\n"
    "layout(location = 0) in vec3 aPos;\n"
    "uniform mat4 Trans;\n"
    "void main()\n"
    "{\n"
    "  gl_Position = Trans * vec4(aPos, 1.0);\n"
    "}";

BasicDepthRendererCore ::~BasicDepthRendererCore() {
  if (initial_set_up_) DeleteBufferObjects();
}

bool BasicDepthRendererCore::SetUp(
    const std::shared_ptr<RendererGeometry> &renderer_geometry_ptr,
    int image_width, int image_height) {
  renderer_geometry_ptr_ = renderer_geometry_ptr;
  image_width_ = image_width;
  image_height_ = image_height;
  image_rendered_ = false;

  // Create shader program
  if (!initial_set_up_ &&
      !CreateShaderProgram(renderer_geometry_ptr_.get(),
                           vertex_shader_code_.c_str(), &shader_program_))
    return false;

  // Create buffer objects
  if (initial_set_up_) DeleteBufferObjects();
  CreateBufferObjects();
  initial_set_up_ = true;
  return true;
}

bool BasicDepthRendererCore::StartRendering(
    const Eigen::Matrix4f &projection_matrix,
    const Transform3fA &world2camera_pose) {
  if (!initial_set_up_) return false;
  renderer_geometry_ptr_->MakeContextCurrent();
  glViewport(0, 0, image_width_, image_height_);

  glBindFramebuffer(GL_FRAMEBUFFER, fbo_);
  glClear(GL_DEPTH_BUFFER_BIT);
  glEnable(GL_DEPTH_TEST);
  glFrontFace(GL_CCW);
  glCullFace(GL_FRONT);

  glUseProgram(shader_program_);
  for (const auto &render_data_body :
       renderer_geometry_ptr_->render_data_bodies()) {
    Eigen::Matrix4f trans{
        projection_matrix *
        (world2camera_pose * render_data_body.body_ptr->geometry2world_pose())
            .matrix()};

    unsigned loc;
    loc = glGetUniformLocation(shader_program_, "Trans");
    glUniformMatrix4fv(loc, 1, GL_FALSE, trans.data());

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
  image_fetched_ = false;
  return true;
}

bool BasicDepthRendererCore::FetchDepthImage(cv::Mat *depth_image) {
  if (!initial_set_up_ || !image_rendered_) return false;
  if (image_fetched_) return true;
  renderer_geometry_ptr_->MakeContextCurrent();
  glPixelStorei(GL_PACK_ALIGNMENT, (depth_image->step & 3) ? 1 : 4);
  glPixelStorei(GL_PACK_ROW_LENGTH,
                GLint(depth_image->step / depth_image->elemSize()));
  glBindFramebuffer(GL_FRAMEBUFFER, fbo_);
  glBindRenderbuffer(GL_RENDERBUFFER, rbo_);
  glReadPixels(0, 0, image_width_, image_height_, GL_DEPTH_COMPONENT,
               GL_UNSIGNED_SHORT, depth_image->data);
  glBindRenderbuffer(GL_RENDERBUFFER, 0);
  glBindFramebuffer(GL_FRAMEBUFFER, 0);
  renderer_geometry_ptr_->DetachContext();
  image_fetched_ = true;
  return true;
}

void BasicDepthRendererCore::CreateBufferObjects() {
  renderer_geometry_ptr_->MakeContextCurrent();

  // Initialize renderbuffer bodies_render_data
  glGenRenderbuffers(1, &rbo_);
  glBindRenderbuffer(GL_RENDERBUFFER, rbo_);
  glRenderbufferStorage(GL_RENDERBUFFER, GL_DEPTH_COMPONENT16, image_width_,
                        image_height_);
  glBindRenderbuffer(GL_RENDERBUFFER, 0);

  // Initialize framebuffer bodies_render_data
  glGenFramebuffers(1, &fbo_);
  glBindFramebuffer(GL_FRAMEBUFFER, fbo_);
  glFramebufferRenderbuffer(GL_FRAMEBUFFER, GL_DEPTH_ATTACHMENT,
                            GL_RENDERBUFFER, rbo_);
  glBindFramebuffer(GL_FRAMEBUFFER, 0);
  renderer_geometry_ptr_->DetachContext();
}

void BasicDepthRendererCore::DeleteBufferObjects() {
  renderer_geometry_ptr_->MakeContextCurrent();
  glDeleteRenderbuffers(1, &rbo_);
  glDeleteFramebuffers(1, &fbo_);
  renderer_geometry_ptr_->DetachContext();
}

FullBasicDepthRenderer::FullBasicDepthRenderer(
    const std::string &name,
    const std::shared_ptr<RendererGeometry> &renderer_geometry_ptr,
    const Transform3fA &world2camera_pose, const Intrinsics &intrinsics,
    float z_min, float z_max)
    : FullDepthRenderer{
          name, renderer_geometry_ptr, world2camera_pose, intrinsics, z_min,
          z_max} {}

FullBasicDepthRenderer::FullBasicDepthRenderer(
    const std::string &name,
    const std::shared_ptr<RendererGeometry> &renderer_geometry_ptr,
    const std::shared_ptr<Camera> &camera_ptr, float z_min, float z_max)
    : FullDepthRenderer{name, renderer_geometry_ptr, camera_ptr, z_min, z_max} {
}

FullBasicDepthRenderer::FullBasicDepthRenderer(
    const std::string &name, const std::filesystem::path &metafile_path,
    const std::shared_ptr<RendererGeometry> &renderer_geometry_ptr,
    const std::shared_ptr<Camera> &camera_ptr)
    : FullDepthRenderer{name, metafile_path, renderer_geometry_ptr,
                        camera_ptr} {}

bool FullBasicDepthRenderer::SetUp() {
  const std::lock_guard<std::mutex> lock{mutex_};
  set_up_ = false;
  if (!metafile_path_.empty())
    if (!LoadMetaData()) return false;

  // Check if all required objects are set up
  if (!renderer_geometry_ptr_->set_up()) {
    std::cerr << "Renderer geometry " << renderer_geometry_ptr_->name()
              << " was not set up" << std::endl;
    return false;
  }
  if (camera_ptr_ && !InitParametersFromCamera()) return false;

  // Set up everything
  CalculateProjectionMatrix();
  CalculateProjectionTerms();
  ClearDepthImage();
  if (!core_.SetUp(renderer_geometry_ptr_, intrinsics_.width,
                   intrinsics_.height))
    return false;

  set_up_ = true;
  return true;
}

bool FullBasicDepthRenderer::StartRendering() {
  const std::lock_guard<std::mutex> lock{mutex_};
  if (!set_up_) {
    std::cerr << "Set up renderer " << name_ << " first" << std::endl;
    return false;
  }
  return core_.StartRendering(projection_matrix_, world2camera_pose_);
}

bool FullBasicDepthRenderer::FetchDepthImage() {
  const std::lock_guard<std::mutex> lock{mutex_};
  if (!set_up_) {
    std::cerr << "Set up renderer " << name_ << " first" << std::endl;
    return false;
  }
  return core_.FetchDepthImage(&depth_image_);
}

bool FullBasicDepthRenderer::LoadMetaData() {
  // Open file storage from yaml
  cv::FileStorage fs;
  if (!OpenYamlFileStorage(metafile_path_, &fs)) return false;

  // Read parameters from yaml
  ReadOptionalValueFromYaml(fs, "z_min", &z_min_);
  ReadOptionalValueFromYaml(fs, "z_max", &z_max_);
  fs.release();
  return true;
}

FocusedBasicDepthRenderer::FocusedBasicDepthRenderer(
    const std::string &name,
    const std::shared_ptr<RendererGeometry> &renderer_geometry_ptr,
    const Transform3fA &world2camera_pose, const Intrinsics &intrinsics,
    int image_size, float z_min, float z_max)
    : FocusedDepthRenderer{name,
                           renderer_geometry_ptr,
                           world2camera_pose,
                           intrinsics,
                           image_size,
                           z_min,
                           z_max} {}

FocusedBasicDepthRenderer::FocusedBasicDepthRenderer(
    const std::string &name,
    const std::shared_ptr<RendererGeometry> &renderer_geometry_ptr,
    const std::shared_ptr<Camera> &camera_ptr, int image_size, float z_min,
    float z_max)
    : FocusedDepthRenderer{
          name, renderer_geometry_ptr, camera_ptr, image_size, z_min, z_max} {}

FocusedBasicDepthRenderer::FocusedBasicDepthRenderer(
    const std::string &name, const std::filesystem::path &metafile_path,
    const std::shared_ptr<RendererGeometry> &renderer_geometry_ptr,
    const std::shared_ptr<Camera> &camera_ptr)
    : FocusedDepthRenderer{name, metafile_path, renderer_geometry_ptr,
                           camera_ptr} {}

bool FocusedBasicDepthRenderer::SetUp() {
  const std::lock_guard<std::mutex> lock{mutex_};
  set_up_ = false;
  if (!metafile_path_.empty())
    if (!LoadMetaData()) return false;

  // Check if all required objects are set up
  if (!renderer_geometry_ptr_->set_up()) {
    std::cerr << "Renderer geometry " << renderer_geometry_ptr_->name()
              << " was not set up" << std::endl;
    return false;
  }
  if (camera_ptr_ && !InitParametersFromCamera()) return false;
  if (referenced_body_ptrs_.empty()) {
    std::cerr << "No referenced bodys were assigned to renderer " << name_
              << std::endl;
    return false;
  }
  for (auto &referenced_body_ptr : referenced_body_ptrs_) {
    if (!referenced_body_ptr->set_up()) {
      std::cerr << "Body " << referenced_body_ptr->name() << " was not set up"
                << std::endl;
      return false;
    }
  }

  // Set up everything
  CalculateProjectionMatrix();
  CalculateProjectionTerms();
  ClearDepthImage();
  if (!core_.SetUp(renderer_geometry_ptr_, image_size_, image_size_))
    return false;

  set_up_ = true;
  return true;
}

bool FocusedBasicDepthRenderer::StartRendering() {
  const std::lock_guard<std::mutex> lock{mutex_};
  if (!set_up_) {
    std::cerr << "Set up renderer " << name_ << " first" << std::endl;
    return false;
  }
  CalculateProjectionMatrix();
  return core_.StartRendering(projection_matrix_, world2camera_pose_);
}

bool FocusedBasicDepthRenderer::FetchDepthImage() {
  const std::lock_guard<std::mutex> lock{mutex_};
  if (!set_up_) {
    std::cerr << "Set up renderer " << name_ << " first" << std::endl;
    return false;
  }
  return core_.FetchDepthImage(&focused_depth_image_);
}

bool FocusedBasicDepthRenderer::LoadMetaData() {
  // Open file storage from yaml
  cv::FileStorage fs;
  if (!OpenYamlFileStorage(metafile_path_, &fs)) return false;

  // Read parameters from yaml
  ReadOptionalValueFromYaml(fs, "z_min", &z_min_);
  ReadOptionalValueFromYaml(fs, "z_max", &z_max_);
  ReadOptionalValueFromYaml(fs, "image_size", &image_size_);
  fs.release();
  return true;
}

}  // namespace m3t
