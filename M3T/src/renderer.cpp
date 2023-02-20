// SPDX-License-Identifier: MIT
// Copyright (c) 2023 Manuel Stoiber, German Aerospace Center (DLR)

#include <m3t/renderer.h>

#include <GL/glew.h>
#include <GLFW/glfw3.h>

namespace m3t {

bool CreateShaderProgram(RendererGeometry *renderer_geometry,
                         const char *vertex_shader_code,
                         unsigned int *shader_program) {
  renderer_geometry->MakeContextCurrent();

  // Create shader
  unsigned int vertex_shader = glCreateShader(GL_VERTEX_SHADER);
  glShaderSource(vertex_shader, 1, &vertex_shader_code, nullptr);
  glCompileShader(vertex_shader);
  if (!CheckCompileErrors(vertex_shader, "VERTEX")) {
    renderer_geometry->DetachContext();
    return false;
  }

  // Create shader programs
  *shader_program = glCreateProgram();
  glAttachShader(*shader_program, vertex_shader);
  glLinkProgram(*shader_program);
  if (!CheckCompileErrors(*shader_program, "PROGRAM")) {
    renderer_geometry->DetachContext();
    return false;
  }

  glDeleteShader(vertex_shader);
  renderer_geometry->DetachContext();
  return true;
}

bool CreateShaderProgram(RendererGeometry *renderer_geometry,
                         const char *vertex_shader_code,
                         const char *fragment_shader_code,
                         unsigned *shader_program) {
  renderer_geometry->MakeContextCurrent();

  // Create shader
  unsigned vertex_shader = glCreateShader(GL_VERTEX_SHADER);
  glShaderSource(vertex_shader, 1, &vertex_shader_code, nullptr);
  glCompileShader(vertex_shader);
  if (!CheckCompileErrors(vertex_shader, "VERTEX")) {
    renderer_geometry->DetachContext();
    return false;
  }

  unsigned fragment_shader = glCreateShader(GL_FRAGMENT_SHADER);
  glShaderSource(fragment_shader, 1, &fragment_shader_code, nullptr);
  glCompileShader(fragment_shader);
  if (!CheckCompileErrors(fragment_shader, "FRAGMENT")) {
    renderer_geometry->DetachContext();
    return false;
  }

  // Create shader programs
  *shader_program = glCreateProgram();
  glAttachShader(*shader_program, vertex_shader);
  glAttachShader(*shader_program, fragment_shader);
  glLinkProgram(*shader_program);
  if (!CheckCompileErrors(*shader_program, "PROGRAM")) {
    renderer_geometry->DetachContext();
    return false;
  }

  glDeleteShader(vertex_shader);
  glDeleteShader(fragment_shader);
  renderer_geometry->DetachContext();
  return true;
}

bool CheckCompileErrors(unsigned shader, const std::string &type) {
  int success;
  char info_log[1024];
  if (type != "PROGRAM") {
    glGetShaderiv(shader, GL_COMPILE_STATUS, &success);
    if (!success) {
      glGetShaderInfoLog(shader, 1024, nullptr, info_log);
      std::cerr << "Shader compilation error of type: " << type << std::endl
                << info_log << std::endl;
      return false;
    }
  } else {
    glGetProgramiv(shader, GL_LINK_STATUS, &success);
    if (!success) {
      glGetProgramInfoLog(shader, 1024, nullptr, info_log);
      std::cerr << "Shader linking error of type: " << type << std::endl
                << info_log << std::endl;
      return false;
    }
  }
  return true;
}

void Renderer::set_name(const std::string &name) {
  const std::lock_guard<std::mutex> lock{mutex_};
  name_ = name;
}

void Renderer::set_metafile_path(const std::filesystem::path &metafile_path) {
  const std::lock_guard<std::mutex> lock{mutex_};
  metafile_path_ = metafile_path;
  set_up_ = false;
}

void Renderer::set_world2camera_pose(const Transform3fA &world2camera_pose) {
  const std::lock_guard<std::mutex> lock{mutex_};
  world2camera_pose_ = world2camera_pose;
  camera2world_pose_ = world2camera_pose.inverse();
}

void Renderer::set_camera2world_pose(const Transform3fA &camera2world_pose) {
  const std::lock_guard<std::mutex> lock{mutex_};
  camera2world_pose_ = camera2world_pose;
  world2camera_pose_ = camera2world_pose.inverse();
}

void Renderer::set_camera_ptr(const std::shared_ptr<Camera> &camera_ptr) {
  const std::lock_guard<std::mutex> lock{mutex_};
  camera_ptr_ = camera_ptr;
  set_up_ = false;
}

void Renderer::set_renderer_geometry_ptr(
    const std::shared_ptr<RendererGeometry> &renderer_geometry_ptr) {
  const std::lock_guard<std::mutex> lock{mutex_};
  renderer_geometry_ptr_ = renderer_geometry_ptr;
  set_up_ = false;
}

void Renderer::set_intrinsics(const Intrinsics &intrinsics) {
  const std::lock_guard<std::mutex> lock{mutex_};
  intrinsics_ = intrinsics;
  set_up_ = false;
}

void Renderer::set_z_min(float z_min) {
  const std::lock_guard<std::mutex> lock{mutex_};
  z_min_ = z_min;
  set_up_ = false;
}

void Renderer::set_z_max(float z_max) {
  const std::lock_guard<std::mutex> lock{mutex_};
  z_max_ = z_max;
  set_up_ = false;
}

const std::string &Renderer::name() const { return name_; }

const std::filesystem::path &Renderer::metafile_path() const {
  return metafile_path_;
}

const std::shared_ptr<Camera> &Renderer::camera_ptr() const {
  return camera_ptr_;
}

const std::shared_ptr<RendererGeometry> &Renderer::renderer_geometry_ptr()
    const {
  return renderer_geometry_ptr_;
}

const Transform3fA &Renderer::world2camera_pose() const {
  return world2camera_pose_;
}

const Transform3fA &Renderer::camera2world_pose() const {
  return camera2world_pose_;
}

const Intrinsics &Renderer::intrinsics() const { return intrinsics_; }

float Renderer::z_min() const { return z_min_; }

float Renderer::z_max() const { return z_max_; }

bool Renderer::set_up() const { return set_up_; };

const std::vector<std::shared_ptr<Body>> &Renderer::referenced_body_ptrs()
    const {
  return {};
}

Renderer::Renderer(
    const std::string &name,
    const std::shared_ptr<RendererGeometry> &renderer_geometry_ptr,
    const Transform3fA &world2camera_pose, const Intrinsics &intrinsics,
    float z_min, float z_max)
    : name_{name},
      renderer_geometry_ptr_{renderer_geometry_ptr},
      intrinsics_{intrinsics},
      z_min_{z_min},
      z_max_{z_max} {
  world2camera_pose_ = world2camera_pose;
  camera2world_pose_ = world2camera_pose.inverse();
}

Renderer::Renderer(
    const std::string &name,
    const std::shared_ptr<RendererGeometry> &renderer_geometry_ptr,
    const std::shared_ptr<Camera> &camera_ptr, float z_min, float z_max)
    : name_{name},
      renderer_geometry_ptr_{renderer_geometry_ptr},
      camera_ptr_{camera_ptr},
      z_min_{z_min},
      z_max_{z_max} {}

Renderer::Renderer(
    const std::string &name, const std::filesystem::path &metafile_path,
    const std::shared_ptr<RendererGeometry> &renderer_geometry_ptr,
    const std::shared_ptr<Camera> &camera_ptr)
    : name_{name},
      metafile_path_{metafile_path},
      renderer_geometry_ptr_{renderer_geometry_ptr},
      camera_ptr_{camera_ptr} {}

bool Renderer::InitParametersFromCamera() {
  if (!camera_ptr_->set_up()) {
    std::cerr << "Camera " << camera_ptr_->name() << " was not set up"
              << std::endl;
    return false;
  }
  intrinsics_ = camera_ptr_->intrinsics();
  world2camera_pose_ = camera_ptr_->world2camera_pose();
  camera2world_pose_ = camera_ptr_->camera2world_pose();
  return true;
}

FullRenderer::FullRenderer(
    const std::string &name,
    const std::shared_ptr<RendererGeometry> &renderer_geometry_ptr,
    const Transform3fA &world2camera_pose, const Intrinsics &intrinsics,
    float z_min, float z_max)
    : Renderer{
          name, renderer_geometry_ptr, world2camera_pose, intrinsics, z_min,
          z_max} {}

FullRenderer::FullRenderer(
    const std::string &name,
    const std::shared_ptr<RendererGeometry> &renderer_geometry_ptr,
    const std::shared_ptr<Camera> &camera_ptr, float z_min, float z_max)
    : Renderer{name, renderer_geometry_ptr, camera_ptr, z_min, z_max} {}

FullRenderer::FullRenderer(
    const std::string &name, const std::filesystem::path &metafile_path,
    const std::shared_ptr<RendererGeometry> &renderer_geometry_ptr,
    const std::shared_ptr<Camera> &camera_ptr)
    : Renderer{name, metafile_path, renderer_geometry_ptr, camera_ptr} {}

void FullRenderer::CalculateProjectionMatrix() {
  projection_matrix_ << 2.0f * intrinsics_.fu / float(intrinsics_.width), 0.0f,
      2.0f * (intrinsics_.ppu + 0.5f) / float(intrinsics_.width) - 1.0f, 0.0f,
      0.0f, 2.0f * intrinsics_.fv / float(intrinsics_.height),
      2.0f * (intrinsics_.ppv + 0.5f) / float(intrinsics_.height) - 1.0f, 0.0f,
      0.0f, 0.0f, (z_max_ + z_min_) / (z_max_ - z_min_),
      -2.0f * z_max_ * z_min_ / (z_max_ - z_min_), 0.0f, 0.0f, 1.0f, 0.0f;
}

bool FocusedRenderer::AddReferencedBody(
    const std::shared_ptr<Body> &referenced_body_ptr) {
  set_up_ = false;
  if (!AddPtrIfNameNotExists(referenced_body_ptr, &referenced_body_ptrs_)) {
    std::cerr << "Referenced body " << referenced_body_ptr->name()
              << " already exists" << std::endl;
    return false;
  }
  return true;
}

bool FocusedRenderer::DeleteReferencedBody(const std::string &name) {
  set_up_ = false;
  if (!DeletePtrIfNameExists(name, &referenced_body_ptrs_)) {
    std::cerr << "Referenced body " << name << " not found" << std::endl;
    return false;
  }
  return true;
}

void FocusedRenderer::ClearReferencedBodies() {
  set_up_ = false;
  referenced_body_ptrs_.clear();
}

void FocusedRenderer::set_image_size(int image_size) {
  set_up_ = false;
  image_size_ = image_size;
}

bool FocusedRenderer::IsBodyReferenced(const std::string &body_name) const {
  return std::find_if(begin(referenced_body_ptrs_), end(referenced_body_ptrs_),
                      [&](const auto &body_ptr) {
                        return body_ptr->name() == body_name;
                      }) != end(referenced_body_ptrs_);
}

bool FocusedRenderer::IsBodyVisible(const std::string &body_name) const {
  return std::find(begin(visible_body_names_), end(visible_body_names_),
                   body_name) != end(visible_body_names_);
}

const std::vector<std::shared_ptr<Body>>
    &FocusedRenderer::referenced_body_ptrs() const {
  return referenced_body_ptrs_;
}

int FocusedRenderer::image_size() const { return image_size_; }

float FocusedRenderer::corner_u() const { return corner_u_; }

float FocusedRenderer::corner_v() const { return corner_v_; }

float FocusedRenderer::scale() const { return scale_; }

FocusedRenderer::FocusedRenderer(
    const std::string &name,
    const std::shared_ptr<RendererGeometry> &renderer_geometry_ptr,
    const Transform3fA &world2camera_pose, const Intrinsics &intrinsics,
    int image_size, float z_min, float z_max)
    : Renderer{name,
               renderer_geometry_ptr,
               world2camera_pose,
               intrinsics,
               z_min,
               z_max},
      image_size_{image_size} {}

FocusedRenderer::FocusedRenderer(
    const std::string &name,
    const std::shared_ptr<RendererGeometry> &renderer_geometry_ptr,
    const std::shared_ptr<Camera> &camera_ptr, int image_size, float z_min,
    float z_max)
    : Renderer{name, renderer_geometry_ptr, camera_ptr, z_min, z_max},
      image_size_{image_size} {}

FocusedRenderer::FocusedRenderer(
    const std::string &name, const std::filesystem::path &metafile_path,
    const std::shared_ptr<RendererGeometry> &renderer_geometry_ptr,
    const std::shared_ptr<Camera> &camera_ptr)
    : Renderer{name, metafile_path, renderer_geometry_ptr, camera_ptr} {}

void FocusedRenderer::CalculateProjectionMatrix() {
  // Calculate limits
  visible_body_names_.clear();
  float u_min = std::numeric_limits<float>::max();
  float u_max = std::numeric_limits<float>::min();
  float v_min = std::numeric_limits<float>::max();
  float v_max = std::numeric_limits<float>::min();
  for (auto &referenced_body_ptr : referenced_body_ptrs_) {
    float r = 0.5f * referenced_body_ptr->maximum_body_diameter();
    auto translation{world2camera_pose_ *
                     referenced_body_ptr->body2world_pose().translation()};
    float x = translation(0);
    float y = translation(1);
    float z = translation(2);
    if (z < r * 1.5f || z - r < z_min_ || z + r > z_max_) continue;
    float abs_x = std::abs(x);
    float abs_y = std::abs(y);
    float x2 = x * x;
    float y2 = y * y;
    float z2 = z * z;
    float r2 = r * r;
    float rz = r * z;
    float z2_r2 = z2 - r2;
    float z3_zr2 = z2_r2 * z;
    float r_u = intrinsics_.fu * (abs_x * r2 + rz * sqrtf(z2_r2 + x2)) / z3_zr2;
    float r_v = intrinsics_.fv * (abs_y * r2 + rz * sqrtf(z2_r2 + y2)) / z3_zr2;
    float center_u = x * intrinsics_.fu / z + intrinsics_.ppu;
    float center_v = y * intrinsics_.fv / z + intrinsics_.ppv;
    float u_min_body = center_u - r_u;
    float u_max_body = center_u + r_u;
    float v_min_body = center_v - r_v;
    float v_max_body = center_v + r_v;
    if (u_min_body > intrinsics_.width || u_max_body < 0 ||
        v_min_body > intrinsics_.height || v_max_body < 0)
      continue;
    u_min = std::min(u_min, u_min_body);
    u_max = std::max(u_max, u_max_body);
    v_min = std::min(v_min, v_min_body);
    v_max = std::max(v_max, v_max_body);
    visible_body_names_.push_back(referenced_body_ptr->name());
  }

  // Calculate parameters
  float d = std::max(u_max - u_min, v_max - v_min) * kImageSizeSafetyMargin;
  corner_u_ = 0.5f * (u_min + u_max - d);
  corner_v_ = 0.5f * (v_min + v_max - d);
  scale_ = float(image_size_) / d;

  // Calculate projection matrix
  float ppu_scaled = (intrinsics_.ppu - corner_u_) * scale_;
  float ppv_scaled = (intrinsics_.ppv - corner_v_) * scale_;
  projection_matrix_ << 2.0f * intrinsics_.fu / d, 0.0f,
      2.0f * (ppu_scaled + 0.5f) / float(image_size_) - 1.0f, 0.0f, 0.0f,
      2.0f * intrinsics_.fv / d,
      2.0f * (ppv_scaled + 0.5f) / float(image_size_) - 1.0f, 0.0f, 0.0f, 0.0f,
      (z_max_ + z_min_) / (z_max_ - z_min_),
      -2.0f * z_max_ * z_min_ / (z_max_ - z_min_), 0.0f, 0.0f, 1.0f, 0.0f;
}

cv::Mat FullDepthRenderer::NormalizedDepthImage(float min_depth,
                                                float max_depth) const {
  cv::Mat normalized_image{depth_image_.size(), CV_8UC1};
  float delta_depth = max_depth - min_depth;
  float scale = 255.0f / delta_depth;
  const ushort *ptr_depth_image;
  uchar *ptr_normalized_image;
  float depth;
  int u, v;
  for (v = 0; v < depth_image_.rows; ++v) {
    ptr_depth_image = depth_image_.ptr<ushort>(v);
    ptr_normalized_image = normalized_image.ptr<uchar>(v);
    for (u = 0; u < depth_image_.cols; ++u) {
      depth = float(ptr_depth_image[u]);
      depth = projection_term_a_ / (projection_term_b_ - depth);
      depth = std::min(std::max(depth - min_depth, 0.0f), delta_depth);
      ptr_normalized_image[u] = uchar(depth * scale);
    }
  }
  return normalized_image;
}

const cv::Mat &FullDepthRenderer::depth_image() const { return depth_image_; }

float FullDepthRenderer::Depth(ushort depth_image_value) const {
  return projection_term_a_ / (projection_term_b_ - float(depth_image_value));
}

float FullDepthRenderer::Depth(const cv::Point2i &image_coordinate) const {
  float depth_image_value = float(depth_image_.at<ushort>(image_coordinate));
  return projection_term_a_ / (projection_term_b_ - depth_image_value);
}

ushort FullDepthRenderer::DepthImageValue(
    const cv::Point2i &image_coordinate) const {
  return depth_image_.at<ushort>(image_coordinate);
}

Eigen::Vector3f FullDepthRenderer::PointVector(
    const cv::Point2i &image_coordinate) const {
  float depth_image_value = float(depth_image_.at<ushort>(image_coordinate));
  float depth = projection_term_a_ / (projection_term_b_ - depth_image_value);
  return Eigen::Vector3f{
      depth * (image_coordinate.x - intrinsics_.ppu) / intrinsics_.fu,
      depth * (image_coordinate.y - intrinsics_.ppv) / intrinsics_.fv, depth};
}

FullDepthRenderer::FullDepthRenderer(
    const std::string &name,
    const std::shared_ptr<RendererGeometry> &renderer_geometry_ptr,
    const Transform3fA &world2camera_pose, const Intrinsics &intrinsics,
    float z_min, float z_max)
    : FullRenderer{
          name, renderer_geometry_ptr, world2camera_pose, intrinsics, z_min,
          z_max} {}

FullDepthRenderer::FullDepthRenderer(
    const std::string &name,
    const std::shared_ptr<RendererGeometry> &renderer_geometry_ptr,
    const std::shared_ptr<Camera> &camera_ptr, float z_min, float z_max)
    : FullRenderer{name, renderer_geometry_ptr, camera_ptr, z_min, z_max} {}

FullDepthRenderer::FullDepthRenderer(
    const std::string &name, const std::filesystem::path &metafile_path,
    const std::shared_ptr<RendererGeometry> &renderer_geometry_ptr,
    const std::shared_ptr<Camera> &camera_ptr)
    : FullRenderer{name, metafile_path, renderer_geometry_ptr, camera_ptr} {}

void FullDepthRenderer::CalculateProjectionTerms() {
  projection_term_a_ = z_max_ * z_min_ * USHRT_MAX / (z_max_ - z_min_);
  projection_term_b_ = z_max_ * USHRT_MAX / (z_max_ - z_min_);
}

void FullDepthRenderer::ClearDepthImage() {
  depth_image_.create(cv::Size{intrinsics_.width, intrinsics_.height}, CV_16U);
  depth_image_.setTo(cv::Scalar{0});
}

cv::Mat FocusedDepthRenderer::NormalizedFocusedDepthImage(
    float min_depth, float max_depth) const {
  cv::Mat normalized_image{focused_depth_image_.size(), CV_8UC1};
  float delta_depth = max_depth - min_depth;
  float scale = 255.0f / delta_depth;
  const ushort *ptr_depth_image;
  uchar *ptr_normalized_image;
  float depth;
  int u, v;
  for (v = 0; v < focused_depth_image_.rows; ++v) {
    ptr_depth_image = focused_depth_image_.ptr<ushort>(v);
    ptr_normalized_image = normalized_image.ptr<uchar>(v);
    for (u = 0; u < focused_depth_image_.cols; ++u) {
      depth = float(ptr_depth_image[u]);
      depth = projection_term_a_ / (projection_term_b_ - depth);
      depth = std::min(std::max(depth - min_depth, 0.0f), delta_depth);
      ptr_normalized_image[u] = uchar(depth * scale);
    }
  }
  return normalized_image;
}

const cv::Mat &FocusedDepthRenderer::focused_depth_image() const {
  return focused_depth_image_;
}

float FocusedDepthRenderer::Depth(ushort depth_image_value) const {
  return projection_term_a_ / (projection_term_b_ - float(depth_image_value));
}

float FocusedDepthRenderer::Depth(const cv::Point2i &image_coordinate) const {
  int u = int((image_coordinate.x - corner_u_) * scale_ + 0.5f);
  int v = int((image_coordinate.y - corner_v_) * scale_ + 0.5f);
  float depth_image_value = float(focused_depth_image_.at<ushort>(v, u));
  return projection_term_a_ / (projection_term_b_ - depth_image_value);
}

ushort FocusedDepthRenderer::DepthImageValue(
    const cv::Point2i &image_coordinate) const {
  int u = int((image_coordinate.x - corner_u_) * scale_ + 0.5f);
  int v = int((image_coordinate.y - corner_v_) * scale_ + 0.5f);
  return focused_depth_image_.at<ushort>(v, u);
}

Eigen::Vector3f FocusedDepthRenderer::PointVector(
    const cv::Point2i &image_coordinate) const {
  int u = int((image_coordinate.x - corner_u_) * scale_ + 0.5f);
  int v = int((image_coordinate.y - corner_v_) * scale_ + 0.5f);
  float depth_image_value = float(focused_depth_image_.at<ushort>(v, u));
  float depth = projection_term_a_ / (projection_term_b_ - depth_image_value);
  return Eigen::Vector3f{
      depth * (image_coordinate.x - intrinsics_.ppu) / intrinsics_.fu,
      depth * (image_coordinate.y - intrinsics_.ppv) / intrinsics_.fv, depth};
}

FocusedDepthRenderer::FocusedDepthRenderer(
    const std::string &name,
    const std::shared_ptr<RendererGeometry> &renderer_geometry_ptr,
    const Transform3fA &world2camera_pose, const Intrinsics &intrinsics,
    int image_size, float z_min, float z_max)
    : FocusedRenderer{name,
                      renderer_geometry_ptr,
                      world2camera_pose,
                      intrinsics,
                      image_size,
                      z_min,
                      z_max} {}

FocusedDepthRenderer::FocusedDepthRenderer(
    const std::string &name,
    const std::shared_ptr<RendererGeometry> &renderer_geometry_ptr,
    const std::shared_ptr<Camera> &camera_ptr, int image_size, float z_min,
    float z_max)
    : FocusedRenderer{
          name, renderer_geometry_ptr, camera_ptr, image_size, z_min, z_max} {}

FocusedDepthRenderer::FocusedDepthRenderer(
    const std::string &name, const std::filesystem::path &metafile_path,
    const std::shared_ptr<RendererGeometry> &renderer_geometry_ptr,
    const std::shared_ptr<Camera> &camera_ptr)
    : FocusedRenderer{name, metafile_path, renderer_geometry_ptr, camera_ptr} {}

void FocusedDepthRenderer::CalculateProjectionTerms() {
  projection_term_a_ = z_max_ * z_min_ * USHRT_MAX / (z_max_ - z_min_);
  projection_term_b_ = z_max_ * USHRT_MAX / (z_max_ - z_min_);
}

void FocusedDepthRenderer::ClearDepthImage() {
  focused_depth_image_.create(cv::Size{image_size_, image_size_}, CV_16U);
  focused_depth_image_.setTo(cv::Scalar{0});
}

}  // namespace m3t
