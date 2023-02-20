// SPDX-License-Identifier: MIT
// Copyright (c) 2023 Manuel Stoiber, German Aerospace Center (DLR)

#include "m3t/silhouette_renderer.h"

#include <GL/glew.h>
#include <GLFW/glfw3.h>

namespace m3t {

std::string SilhouetteRendererCore::vertex_shader_code_ =
    "#version 330 core\n"
    "layout(location = 0) in vec3 aPos;\n"
    "uniform mat4 Trans;\n"
    "void main()\n"
    "{\n"
    "  gl_Position = Trans * vec4(aPos, 1.0);\n"
    "}";

std::string SilhouetteRendererCore::fragment_shader_code_ =
    "#version 330 core\n"
    "uniform float SilhouetteID;\n"
    "out float FragColor;\n"
    "void main()\n"
    "{\n"
    "  FragColor = SilhouetteID;\n"
    "}";

SilhouetteRendererCore::~SilhouetteRendererCore() {
  if (initial_set_up_) DeleteBufferObjects();
}

bool SilhouetteRendererCore::SetUp(
    const std::shared_ptr<RendererGeometry> &renderer_geometry_ptr,
    int image_width, int image_height) {
  renderer_geometry_ptr_ = renderer_geometry_ptr;
  image_width_ = image_width;
  image_height_ = image_height;
  image_rendered_ = false;

  // Create shader program
  if (!initial_set_up_ &&
      !CreateShaderProgram(renderer_geometry_ptr_.get(),
                           vertex_shader_code_.c_str(),
                           fragment_shader_code_.c_str(), &shader_program_))
    return false;

  // Create buffer objects
  if (initial_set_up_) DeleteBufferObjects();
  CreateBufferObjects();
  initial_set_up_ = true;
  return true;
}

bool SilhouetteRendererCore::StartRendering(
    const Eigen::Matrix4f &projection_matrix,
    const Transform3fA &world2camera_pose, IDType id_type) {
  if (!initial_set_up_) return false;
  renderer_geometry_ptr_->MakeContextCurrent();
  glViewport(0, 0, image_width_, image_height_);

  glBindFramebuffer(GL_FRAMEBUFFER, fbo_);
  glClearColor(0.0f, 0.0f, 0.0f, 0.0f);
  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
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
    loc = glGetUniformLocation(shader_program_, "SilhouetteID");
    // Map silhouette id from uchar [0, 255] to float [0.0, 1.0]
    glUniform1f(loc,
                float(render_data_body.body_ptr->get_id(id_type)) / 255.0f);

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
  silhouette_image_fetched_ = false;
  depth_image_fetched_ = false;
  return true;
}

bool SilhouetteRendererCore::FetchSilhouetteImage(cv::Mat *silhouette_image) {
  if (!initial_set_up_ || !image_rendered_) return false;
  if (silhouette_image_fetched_) return true;
  renderer_geometry_ptr_->MakeContextCurrent();
  glPixelStorei(GL_PACK_ALIGNMENT, (silhouette_image->step & 3) ? 1 : 4);
  glPixelStorei(GL_PACK_ROW_LENGTH,
                GLint(silhouette_image->step / silhouette_image->elemSize()));
  glBindFramebuffer(GL_FRAMEBUFFER, fbo_);
  glBindRenderbuffer(GL_RENDERBUFFER, rbo_silhouette_);
  glReadPixels(0, 0, image_width_, image_height_, GL_RED, GL_UNSIGNED_BYTE,
               silhouette_image->data);
  glBindRenderbuffer(GL_RENDERBUFFER, 0);
  glBindFramebuffer(GL_FRAMEBUFFER, 0);
  renderer_geometry_ptr_->DetachContext();
  silhouette_image_fetched_ = true;
  return true;
}

bool SilhouetteRendererCore::FetchDepthImage(cv::Mat *depth_image) {
  if (!initial_set_up_ || !image_rendered_) return false;
  if (depth_image_fetched_) return true;
  renderer_geometry_ptr_->MakeContextCurrent();
  glPixelStorei(GL_PACK_ALIGNMENT, (depth_image->step & 3) ? 1 : 4);
  glPixelStorei(GL_PACK_ROW_LENGTH,
                GLint(depth_image->step / depth_image->elemSize()));
  glBindFramebuffer(GL_FRAMEBUFFER, fbo_);
  glBindRenderbuffer(GL_RENDERBUFFER, rbo_depth_);
  glReadPixels(0, 0, image_width_, image_height_, GL_DEPTH_COMPONENT,
               GL_UNSIGNED_SHORT, depth_image->data);
  glBindRenderbuffer(GL_RENDERBUFFER, 0);
  glBindFramebuffer(GL_FRAMEBUFFER, 0);
  renderer_geometry_ptr_->DetachContext();
  depth_image_fetched_ = true;
  return true;
}

void SilhouetteRendererCore::CreateBufferObjects() {
  renderer_geometry_ptr_->MakeContextCurrent();

  // Initialize renderbuffer bodies_render_data
  glGenRenderbuffers(1, &rbo_silhouette_);
  glBindRenderbuffer(GL_RENDERBUFFER, rbo_silhouette_);
  glRenderbufferStorage(GL_RENDERBUFFER, GL_R8, image_width_, image_height_);
  glBindRenderbuffer(GL_RENDERBUFFER, 0);

  glGenRenderbuffers(1, &rbo_depth_);
  glBindRenderbuffer(GL_RENDERBUFFER, rbo_depth_);
  glRenderbufferStorage(GL_RENDERBUFFER, GL_DEPTH_COMPONENT16, image_width_,
                        image_height_);
  glBindRenderbuffer(GL_RENDERBUFFER, 0);

  // Initialize framebuffer bodies_render_data
  glGenFramebuffers(1, &fbo_);
  glBindFramebuffer(GL_FRAMEBUFFER, fbo_);
  glFramebufferRenderbuffer(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0,
                            GL_RENDERBUFFER, rbo_silhouette_);
  glFramebufferRenderbuffer(GL_FRAMEBUFFER, GL_DEPTH_ATTACHMENT,
                            GL_RENDERBUFFER, rbo_depth_);
  glBindFramebuffer(GL_FRAMEBUFFER, 0);
  renderer_geometry_ptr_->DetachContext();
}

void SilhouetteRendererCore::DeleteBufferObjects() {
  renderer_geometry_ptr_->MakeContextCurrent();
  glDeleteRenderbuffers(1, &rbo_silhouette_);
  glDeleteRenderbuffers(1, &rbo_depth_);
  glDeleteFramebuffers(1, &fbo_);
  renderer_geometry_ptr_->DetachContext();
}

FullSilhouetteRenderer::FullSilhouetteRenderer(
    const std::string &name,
    const std::shared_ptr<RendererGeometry> &renderer_geometry_ptr,
    const Transform3fA &world2camera_pose, const Intrinsics &intrinsics,
    IDType id_type, float z_min, float z_max)
    : FullDepthRenderer{name,
                        renderer_geometry_ptr,
                        world2camera_pose,
                        intrinsics,
                        z_min,
                        z_max},
      id_type_{id_type} {}

FullSilhouetteRenderer::FullSilhouetteRenderer(
    const std::string &name,
    const std::shared_ptr<RendererGeometry> &renderer_geometry_ptr,
    const std::shared_ptr<Camera> &camera_ptr, IDType id_type, float z_min,
    float z_max)
    : FullDepthRenderer{name, renderer_geometry_ptr, camera_ptr, z_min, z_max},
      id_type_{id_type} {}

FullSilhouetteRenderer::FullSilhouetteRenderer(
    const std::string &name, const std::filesystem::path &metafile_path,
    const std::shared_ptr<RendererGeometry> &renderer_geometry_ptr,
    const std::shared_ptr<Camera> &camera_ptr)
    : FullDepthRenderer{name, metafile_path, renderer_geometry_ptr,
                        camera_ptr} {}

bool FullSilhouetteRenderer::SetUp() {
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
  ClearSilhouetteImage();
  if (!core_.SetUp(renderer_geometry_ptr_, intrinsics_.width,
                   intrinsics_.height))
    return false;

  set_up_ = true;
  return true;
}

void FullSilhouetteRenderer::set_id_type(IDType id_type) { id_type_ = id_type; }

bool FullSilhouetteRenderer::StartRendering() {
  const std::lock_guard<std::mutex> lock{mutex_};
  if (!set_up_) {
    std::cerr << "Set up renderer " << name_ << " first" << std::endl;
    return false;
  }
  return core_.StartRendering(projection_matrix_, world2camera_pose_, id_type_);
}

bool FullSilhouetteRenderer::FetchSilhouetteImage() {
  const std::lock_guard<std::mutex> lock{mutex_};
  if (!set_up_) {
    std::cerr << "Set up renderer " << name_ << " first" << std::endl;
    return false;
  }
  return core_.FetchSilhouetteImage(&silhouette_image_);
}

bool FullSilhouetteRenderer::FetchDepthImage() {
  const std::lock_guard<std::mutex> lock{mutex_};
  if (!set_up_) {
    std::cerr << "Set up renderer " << name_ << " first" << std::endl;
    return false;
  }
  return core_.FetchDepthImage(&depth_image_);
}

IDType FullSilhouetteRenderer::id_type() const { return id_type_; }

const cv::Mat &FullSilhouetteRenderer::silhouette_image() const {
  return silhouette_image_;
}

uchar FullSilhouetteRenderer::SilhouetteValue(
    const cv::Point2i &image_coordinate) const {
  return silhouette_image_.at<uchar>(image_coordinate);
}

bool FullSilhouetteRenderer::LoadMetaData() {
  // Open file storage from yaml
  cv::FileStorage fs;
  if (!OpenYamlFileStorage(metafile_path_, &fs)) return false;

  // Read parameters from yaml
  ReadOptionalValueFromYaml(fs, "z_min", &z_min_);
  ReadOptionalValueFromYaml(fs, "z_max", &z_max_);
  ReadOptionalValueFromYaml(fs, "id_type", &id_type_);
  fs.release();
  return true;
}

void FullSilhouetteRenderer::ClearSilhouetteImage() {
  silhouette_image_.create(cv::Size{intrinsics_.width, intrinsics_.height},
                           CV_8U);
  silhouette_image_.setTo(cv::Scalar{0});
}

FocusedSilhouetteRenderer::FocusedSilhouetteRenderer(
    const std::string &name,
    const std::shared_ptr<RendererGeometry> &renderer_geometry_ptr,
    const Transform3fA &world2camera_pose, const Intrinsics &intrinsics,
    IDType id_type, int image_size, float z_min, float z_max)
    : FocusedDepthRenderer{name,
                           renderer_geometry_ptr,
                           world2camera_pose,
                           intrinsics,
                           image_size,
                           z_min,
                           z_max},
      id_type_{id_type} {}

FocusedSilhouetteRenderer::FocusedSilhouetteRenderer(
    const std::string &name,
    const std::shared_ptr<RendererGeometry> &renderer_geometry_ptr,
    const std::shared_ptr<Camera> &camera_ptr, IDType id_type, int image_size,
    float z_min, float z_max)
    : FocusedDepthRenderer{name,       renderer_geometry_ptr,
                           camera_ptr, image_size,
                           z_min,      z_max},
      id_type_{id_type} {}

FocusedSilhouetteRenderer::FocusedSilhouetteRenderer(
    const std::string &name, const std::filesystem::path &metafile_path,
    const std::shared_ptr<RendererGeometry> &renderer_geometry_ptr,
    const std::shared_ptr<Camera> &camera_ptr)
    : FocusedDepthRenderer{name, metafile_path, renderer_geometry_ptr,
                           camera_ptr} {}

bool FocusedSilhouetteRenderer::SetUp() {
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
    std::cerr << "No referenced bodies were assigned to renderer " << name_
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
  ClearSilhouetteImage();
  if (!core_.SetUp(renderer_geometry_ptr_, image_size_, image_size_))
    return false;

  set_up_ = true;
  return true;
}

void FocusedSilhouetteRenderer::set_id_type(IDType id_type) {
  id_type_ = id_type;
}

bool FocusedSilhouetteRenderer::StartRendering() {
  std::lock_guard<std::mutex> lock(mutex_);
  if (!set_up_) {
    std::cerr << "Set up renderer " << name_ << " first" << std::endl;
    return false;
  }
  CalculateProjectionMatrix();
  return core_.StartRendering(projection_matrix_, world2camera_pose_, id_type_);
}

bool FocusedSilhouetteRenderer::FetchSilhouetteImage() {
  const std::lock_guard<std::mutex> lock{mutex_};
  if (!set_up_) {
    std::cerr << "Set up renderer " << name_ << " first" << std::endl;
    return false;
  }
  return core_.FetchSilhouetteImage(&focused_silhouette_image_);
}

bool FocusedSilhouetteRenderer::FetchDepthImage() {
  const std::lock_guard<std::mutex> lock{mutex_};
  if (!set_up_) {
    std::cerr << "Set up renderer " << name_ << " first" << std::endl;
    return false;
  }
  return core_.FetchDepthImage(&focused_depth_image_);
}

IDType FocusedSilhouetteRenderer::id_type() const { return id_type_; }

const cv::Mat &FocusedSilhouetteRenderer::focused_silhouette_image() const {
  return focused_silhouette_image_;
}

uchar FocusedSilhouetteRenderer::SilhouetteValue(
    const cv::Point2i &image_coordinate) const {
  int u = int((image_coordinate.x - corner_u_) * scale_ + 0.5f);
  int v = int((image_coordinate.y - corner_v_) * scale_ + 0.5f);
  return focused_silhouette_image_.at<uchar>(v, u);
}

bool FocusedSilhouetteRenderer::LoadMetaData() {
  // Open file storage from yaml
  cv::FileStorage fs;
  if (!OpenYamlFileStorage(metafile_path_, &fs)) return false;

  // Read parameters from yaml
  ReadOptionalValueFromYaml(fs, "id_type", &id_type_);
  ReadOptionalValueFromYaml(fs, "image_size", &image_size_);
  ReadOptionalValueFromYaml(fs, "z_min", &z_min_);
  ReadOptionalValueFromYaml(fs, "z_max", &z_max_);
  fs.release();
  return true;
}

void FocusedSilhouetteRenderer::ClearSilhouetteImage() {
  focused_silhouette_image_.create(cv::Size{image_size_, image_size_}, CV_8U);
  focused_silhouette_image_.setTo(cv::Scalar{0});
}

}  // namespace m3t
