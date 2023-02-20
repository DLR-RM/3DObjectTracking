// SPDX-License-Identifier: MIT
// Copyright (c) 2023 Manuel Stoiber, German Aerospace Center (DLR)

#include <m3t/viewer.h>

namespace m3t {

void Viewer::set_name(const std::string &name) { name_ = name; }

void Viewer::set_metafile_path(const std::filesystem::path &metafile_path) {
  metafile_path_ = metafile_path;
  set_up_ = false;
}

void Viewer::set_display_images(bool display_images) {
  display_images_ = display_images;
}

void Viewer::StartSavingImages(const std::filesystem::path &save_directory,
                               const std::string &save_image_type) {
  save_images_ = true;
  save_directory_ = save_directory;
  save_image_type_ = save_image_type;
}

void Viewer::StopSavingImages() { save_images_ = false; }

const std::string &Viewer::name() const { return name_; }

const std::filesystem::path &Viewer::metafile_path() const {
  return metafile_path_;
}

std::shared_ptr<RendererGeometry> Viewer::renderer_geometry_ptr() const {
  return nullptr;
}

bool Viewer::display_images() const { return display_images_; }

const std::filesystem::path &Viewer::save_directory() const {
  return save_directory_;
}

const std::string &Viewer::save_image_type() const { return save_image_type_; }

bool Viewer::save_images() const { return save_images_; }

bool Viewer::set_up() const { return set_up_; }

Viewer::Viewer(const std::string &name) : name_{name} {}

Viewer::Viewer(const std::string &name,
               const std::filesystem::path &metafile_path)
    : name_{name}, metafile_path_{metafile_path} {}

void Viewer::DisplayAndSaveImage(int save_index, const cv::Mat &image) {
  if (display_images_) cv::imshow(name_, image);
  if (save_images_) {
    std::filesystem::path path{save_directory_ /
                               (name_ + "_image_" + std::to_string(save_index) +
                                "." + save_image_type_)};
    cv::imwrite(path.string(), image);
  }
}

void ColorViewer::set_color_camera_ptr(
    const std::shared_ptr<ColorCamera> &color_camera_ptr) {
  color_camera_ptr_ = color_camera_ptr;
  set_up_ = false;
}

const std::shared_ptr<ColorCamera> &ColorViewer::color_camera_ptr() const {
  return color_camera_ptr_;
}

std::shared_ptr<Camera> ColorViewer::camera_ptr() const {
  return color_camera_ptr_;
}

ColorViewer::ColorViewer(const std::string &name,
                         const std::shared_ptr<ColorCamera> &color_camera_ptr)
    : Viewer{name}, color_camera_ptr_{color_camera_ptr} {}

ColorViewer::ColorViewer(const std::string &name,
                         const std::filesystem::path &metafile_path,
                         const std::shared_ptr<ColorCamera> &color_camera_ptr)
    : Viewer{name, metafile_path}, color_camera_ptr_{color_camera_ptr} {}

void DepthViewer::set_depth_camera_ptr(
    const std::shared_ptr<DepthCamera> &depth_camera_ptr) {
  depth_camera_ptr_ = depth_camera_ptr;
  set_up_ = false;
}

void DepthViewer::set_min_depth(float min_depth) { min_depth_ = min_depth; }

void DepthViewer::set_max_depth(float max_depth) { max_depth_ = max_depth; }

const std::shared_ptr<DepthCamera> &DepthViewer::depth_camera_ptr() const {
  return depth_camera_ptr_;
}

std::shared_ptr<Camera> DepthViewer::camera_ptr() const {
  return depth_camera_ptr_;
}

float DepthViewer::min_depth() const { return min_depth_; }

float DepthViewer::max_depth() const { return max_depth_; }

DepthViewer::DepthViewer(const std::string &name,
                         const std::shared_ptr<DepthCamera> &depth_camera_ptr,
                         float min_depth, float max_depth)
    : Viewer{name},
      depth_camera_ptr_{depth_camera_ptr},
      min_depth_{min_depth},
      max_depth_{max_depth} {}

DepthViewer::DepthViewer(const std::string &name,
                         const std::filesystem::path &metafile_path,
                         const std::shared_ptr<DepthCamera> &depth_camera_ptr)
    : Viewer{name, metafile_path}, depth_camera_ptr_{depth_camera_ptr} {}

}  // namespace m3t
