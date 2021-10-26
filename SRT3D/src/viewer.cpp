// SPDX-License-Identifier: MIT
// Copyright (c) 2021 Manuel Stoiber, German Aerospace Center (DLR)

#include <srt3d/viewer.h>

namespace srt3d {

Viewer::Viewer(const std::string &name, std::shared_ptr<Camera> camera_ptr)
    : name_{name}, camera_ptr_{std::move(camera_ptr)} {}

void Viewer::set_name(const std::string &name) { name_ = name; }

void Viewer::set_camera_ptr(std::shared_ptr<Camera> camera_ptr) {
  camera_ptr_ = std::move(camera_ptr);
  set_up_ = false;
}

void Viewer::set_display_images(bool dispaly_images) {
  display_images_ = dispaly_images;
}

void Viewer::StartSavingImages(const std::filesystem::path &save_directory,
                               const std::string &save_image_type) {
  save_images_ = true;
  save_directory_ = save_directory;
  save_image_type_ = save_image_type;
}

void Viewer::StopSavingImages() { save_images_ = false; }

const std::string &Viewer::name() const { return name_; }

std::shared_ptr<Camera> Viewer::camera_ptr() const { return camera_ptr_; }

std::shared_ptr<RendererGeometry> Viewer::renderer_geometry_ptr() const {
  return nullptr;
}

bool Viewer::display_images() const { return display_images_; }

bool Viewer::save_images() const { return save_images_; }

bool Viewer::set_up() const { return set_up_; }

void Viewer::DisplayAndSaveImage(int save_index, const cv::Mat &image) {
  if (display_images_) cv::imshow(name_, image);
  if (save_images_) {
    std::filesystem::path path{save_directory_ /
                               (name_ + "_image_" + std::to_string(save_index) +
                                "." + save_image_type_)};
    cv::imwrite(path.string(), image);
  }
}

}  // namespace srt3d
