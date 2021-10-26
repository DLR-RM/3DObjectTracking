// SPDX-License-Identifier: MIT
// Copyright (c) 2021 Manuel Stoiber, German Aerospace Center (DLR)

#include <srt3d/image_viewer.h>

namespace srt3d {

ImageViewer::ImageViewer(const std::string &name,
                         std::shared_ptr<Camera> camera_ptr)
    : Viewer{name, std::move(camera_ptr)} {}

bool ImageViewer::SetUp() {
  set_up_ = false;
  // Check if all required objects are set up
  if (!camera_ptr_->set_up()) {
    std::cerr << "Camera " << camera_ptr_->name() << " was not set up"
              << std::endl;
    return false;
  }
  set_up_ = true;
  return true;
}

bool ImageViewer::UpdateViewer(int save_index) {
  if (!set_up_) {
    std::cerr << "Set up viewer " << name_ << " first" << std::endl;
    return false;
  }
  DisplayAndSaveImage(save_index, camera_ptr_->image());
}

}  // namespace srt3d
