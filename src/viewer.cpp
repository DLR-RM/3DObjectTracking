// SPDX-License-Identifier: MIT
// Copyright (c) 2020 Manuel Stoiber, German Aerospace Center (DLR)

#include <rbgt/viewer.h>

namespace rbgt {

void Viewer::set_display_images(bool dispaly_images) {
  display_images_ = dispaly_images;
}

void Viewer::StartSavingImages(const std::filesystem::path &path) {
  save_images_ = true;
  save_path_ = path;
}

void Viewer::StopSavingImages() { save_images_ = false; }

const std::string &Viewer::name() const { return name_; }

std::shared_ptr<Camera> Viewer::camera_ptr() const { return camera_ptr_; }

const std::filesystem::path &Viewer::save_path() const { return save_path_; }

bool Viewer::display_images() const { return display_images_; }

bool Viewer::save_images() const { return save_images_; }

bool Viewer::initialized() const { return initialized_; }

}  // namespace rbgt
