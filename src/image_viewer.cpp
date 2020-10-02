// SPDX-License-Identifier: MIT
// Copyright (c) 2020 Manuel Stoiber, German Aerospace Center (DLR)

#include <rbgt/image_viewer.h>

namespace rbgt {

void ImageViewer::Init(const std::string &name,
                            std::shared_ptr<Camera> camera_ptr) {
  name_ = name;
  camera_ptr_ = std::move(camera_ptr);
  initialized_ = true;
}

void ImageViewer::UpdateViewer(int save_index) {
  if (display_images_) cv::imshow(name_, camera_ptr_->image());
  if (save_images_)
    cv::imwrite(
        save_path_.string() + name_ + "_" + std::to_string(save_index) + ".png",
        camera_ptr_->image());
}

}  // namespace rbgt
