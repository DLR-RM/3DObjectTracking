// SPDX-License-Identifier: MIT
// Copyright (c) 2023 Manuel Stoiber, German Aerospace Center (DLR)

#include <m3t/normal_viewer.h>

namespace m3t {

cv::Mat CalculateAlphaBlend(const cv::Mat &camera_image,
                            const cv::Mat &renderer_image, float opacity) {
  // Declare variables
  cv::Mat image{camera_image.size(), CV_8UC3};
  int v, u;
  const cv::Vec3b *ptr_camera_image;
  const cv::Vec4b *ptr_renderer_image;
  cv::Vec3b *ptr_image;
  const uchar *val_camera_image;
  const uchar *val_renderer_image;
  uchar *val_image;
  float alpha, alpha_inv;
  float alpha_scale = opacity / 255.0f;

  // Iterate over all pixels
  for (v = 0; v < camera_image.rows; ++v) {
    ptr_camera_image = camera_image.ptr<cv::Vec3b>(v);
    ptr_renderer_image = renderer_image.ptr<cv::Vec4b>(v);
    ptr_image = image.ptr<cv::Vec3b>(v);
    for (u = 0; u < camera_image.cols; ++u) {
      val_camera_image = ptr_camera_image[u].val;
      val_renderer_image = ptr_renderer_image[u].val;
      val_image = ptr_image[u].val;

      // Blend images
      alpha = float(val_renderer_image[3]) * alpha_scale;
      alpha_inv = 1.0f - alpha;
      val_image[0] =
          char(val_camera_image[0] * alpha_inv + val_renderer_image[0] * alpha);
      val_image[1] =
          char(val_camera_image[1] * alpha_inv + val_renderer_image[1] * alpha);
      val_image[2] =
          char(val_camera_image[2] * alpha_inv + val_renderer_image[2] * alpha);
    }
  }
  return image;
}

NormalColorViewer::NormalColorViewer(
    const std::string &name,
    const std::shared_ptr<ColorCamera> &color_camera_ptr,
    const std::shared_ptr<RendererGeometry> &renderer_geometry_ptr,
    float opacity)
    : ColorViewer{name, color_camera_ptr},
      renderer_geometry_ptr_{renderer_geometry_ptr},
      opacity_{opacity},
      renderer_{"renderer", renderer_geometry_ptr, color_camera_ptr} {}

NormalColorViewer::NormalColorViewer(
    const std::string &name, const std::filesystem::path &metafile_path,
    const std::shared_ptr<ColorCamera> &color_camera_ptr,
    const std::shared_ptr<RendererGeometry> &renderer_geometry_ptr)
    : ColorViewer{name, metafile_path, color_camera_ptr},
      renderer_geometry_ptr_{renderer_geometry_ptr},
      renderer_{"renderer", renderer_geometry_ptr, color_camera_ptr} {}

bool NormalColorViewer::SetUp() {
  set_up_ = false;
  if (!metafile_path_.empty())
    if (!LoadMetaData()) return false;

  // Check if all required objects are set up
  if (!color_camera_ptr_->set_up()) {
    std::cerr << "Color camera " << color_camera_ptr_->name()
              << " was not set up" << std::endl;
    return false;
  }
  if (!renderer_geometry_ptr_->set_up()) {
    std::cerr << "Renderer geometry " << renderer_geometry_ptr_->name()
              << " was not set up" << std::endl;
    return false;
  }

  // Configure and set up renderer
  renderer_.set_renderer_geometry_ptr(renderer_geometry_ptr_);
  renderer_.set_camera_ptr(color_camera_ptr_);
  if (!renderer_.SetUp()) return false;
  set_up_ = true;
  return true;
}

void NormalColorViewer::set_renderer_geometry_ptr(
    const std::shared_ptr<RendererGeometry> &renderer_geometry_ptr) {
  renderer_geometry_ptr_ = renderer_geometry_ptr;
  set_up_ = false;
}

void NormalColorViewer::set_opacity(float opacity) { opacity_ = opacity; }

bool NormalColorViewer::UpdateViewer(int save_index) {
  if (!set_up_) {
    std::cerr << "Set up normal color viewer " << name_ << " first"
              << std::endl;
    return false;
  }

  // Render overlay
  renderer_.StartRendering();
  renderer_.FetchNormalImage();

  // Display and save images
  DisplayAndSaveImage(save_index,
                      CalculateAlphaBlend(color_camera_ptr_->image(),
                                          renderer_.normal_image(), opacity_));
  return true;
}

std::shared_ptr<RendererGeometry> NormalColorViewer::renderer_geometry_ptr()
    const {
  return renderer_geometry_ptr_;
}

float NormalColorViewer::opacity() const { return opacity_; }

bool NormalColorViewer::LoadMetaData() {
  // Open file storage from yaml
  cv::FileStorage fs;
  if (!OpenYamlFileStorage(metafile_path_, &fs)) return false;

  // Read parameters from yaml
  ReadOptionalValueFromYaml(fs, "opacity", &opacity_);
  ReadOptionalValueFromYaml(fs, "display_images", &display_images_);
  ReadOptionalValueFromYaml(fs, "save_images", &save_images_);
  ReadOptionalValueFromYaml(fs, "save_directory", &save_directory_);
  ReadOptionalValueFromYaml(fs, "save_image_type", &save_image_type_);
  fs.release();

  // Process parameters
  if (save_directory_.is_relative())
    save_directory_ = metafile_path_.parent_path() / save_directory_;
  return true;
}

NormalDepthViewer::NormalDepthViewer(
    const std::string &name,
    const std::shared_ptr<DepthCamera> &depth_camera_ptr,
    const std::shared_ptr<RendererGeometry> &renderer_geometry_ptr,
    float min_depth, float max_depth, float opacity)
    : DepthViewer{name, depth_camera_ptr, min_depth, max_depth},
      renderer_geometry_ptr_{renderer_geometry_ptr},
      opacity_{opacity},
      renderer_{"renderer", renderer_geometry_ptr, depth_camera_ptr} {}

NormalDepthViewer::NormalDepthViewer(
    const std::string &name, const std::filesystem::path &metafile_path,
    const std::shared_ptr<DepthCamera> &depth_camera_ptr,
    const std::shared_ptr<RendererGeometry> &renderer_geometry_ptr)
    : DepthViewer{name, metafile_path, depth_camera_ptr},
      renderer_geometry_ptr_{renderer_geometry_ptr},
      renderer_{"renderer", renderer_geometry_ptr, depth_camera_ptr} {}

bool NormalDepthViewer::SetUp() {
  set_up_ = false;
  if (!metafile_path_.empty())
    if (!LoadMetaData()) return false;

  // Check if all required objects are set up
  if (!depth_camera_ptr_->set_up()) {
    std::cerr << "Depth camera " << depth_camera_ptr_->name()
              << " was not set up" << std::endl;
    return false;
  }
  if (!renderer_geometry_ptr_->set_up()) {
    std::cerr << "Renderer geometry " << renderer_geometry_ptr_->name()
              << " was not set up" << std::endl;
    return false;
  }

  // Configure and set up renderer
  renderer_.set_renderer_geometry_ptr(renderer_geometry_ptr_);
  renderer_.set_camera_ptr(depth_camera_ptr_);
  if (!renderer_.SetUp()) return false;
  set_up_ = true;
  return true;
}

void NormalDepthViewer::set_renderer_geometry_ptr(
    const std::shared_ptr<RendererGeometry> &renderer_geometry_ptr) {
  renderer_geometry_ptr_ = renderer_geometry_ptr;
  set_up_ = false;
}

void NormalDepthViewer::set_opacity(float opacity) { opacity_ = opacity; }

bool NormalDepthViewer::UpdateViewer(int save_index) {
  if (!set_up_) {
    std::cerr << "Set up normal depth viewer " << name_ << " first"
              << std::endl;
    return false;
  }

  // Render overlay
  renderer_.StartRendering();
  renderer_.FetchNormalImage();

  // Display and save images
  cv::Mat normalized_depth_image_rgb;
  cv::cvtColor(depth_camera_ptr_->NormalizedDepthImage(min_depth_, max_depth_),
               normalized_depth_image_rgb, cv::COLOR_GRAY2BGR);
  DisplayAndSaveImage(save_index,
                      CalculateAlphaBlend(normalized_depth_image_rgb,
                                          renderer_.normal_image(), opacity_));
  return true;
}

std::shared_ptr<RendererGeometry> NormalDepthViewer::renderer_geometry_ptr()
    const {
  return renderer_geometry_ptr_;
}

float NormalDepthViewer::opacity() const { return opacity_; }

bool NormalDepthViewer::LoadMetaData() {
  // Open file storage from yaml
  cv::FileStorage fs;
  if (!OpenYamlFileStorage(metafile_path_, &fs)) return false;

  // Read parameters from yaml
  ReadOptionalValueFromYaml(fs, "min_depth", &min_depth_);
  ReadOptionalValueFromYaml(fs, "max_depth", &max_depth_);
  ReadOptionalValueFromYaml(fs, "opacity", &opacity_);
  ReadOptionalValueFromYaml(fs, "display_images", &display_images_);
  ReadOptionalValueFromYaml(fs, "save_images", &save_images_);
  ReadOptionalValueFromYaml(fs, "save_directory", &save_directory_);
  ReadOptionalValueFromYaml(fs, "save_image_type", &save_image_type_);
  fs.release();

  // Process parameters
  if (save_directory_.is_relative())
    save_directory_ = metafile_path_.parent_path() / save_directory_;
  return true;
}

}  // namespace m3t
