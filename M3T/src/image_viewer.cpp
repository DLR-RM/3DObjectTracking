// SPDX-License-Identifier: MIT
// Copyright (c) 2023 Manuel Stoiber, German Aerospace Center (DLR)

#include <m3t/image_viewer.h>

namespace m3t {

ImageColorViewer::ImageColorViewer(
    const std::string &name,
    const std::shared_ptr<ColorCamera> &color_camera_ptr)
    : ColorViewer{name, color_camera_ptr} {}

ImageColorViewer::ImageColorViewer(
    const std::string &name, const std::filesystem::path &metafile_path,
    const std::shared_ptr<ColorCamera> &color_camera_ptr)
    : ColorViewer{name, metafile_path, color_camera_ptr} {}

bool ImageColorViewer::SetUp() {
  set_up_ = false;
  if (!metafile_path_.empty())
    if (!LoadMetaData()) return false;

  // Check if all required objects are set up
  if (!color_camera_ptr_->set_up()) {
    std::cerr << "Color camera " << color_camera_ptr_->name()
              << " was not set up" << std::endl;
    return false;
  }
  set_up_ = true;
  return true;
}

bool ImageColorViewer::UpdateViewer(int save_index) {
  if (!set_up_) {
    std::cerr << "Set up image color viewer " << name_ << " first" << std::endl;
    return false;
  }
  DisplayAndSaveImage(save_index, color_camera_ptr_->image());
  return true;
}

bool ImageColorViewer::LoadMetaData() {  // Open file storage from yaml
  cv::FileStorage fs;
  if (!OpenYamlFileStorage(metafile_path_, &fs)) return false;

  // Read parameters from yaml
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

ImageDepthViewer::ImageDepthViewer(
    const std::string &name,
    const std::shared_ptr<DepthCamera> &depth_camera_ptr, float min_depth,
    float max_depth)
    : DepthViewer{name, depth_camera_ptr, min_depth, max_depth} {}

ImageDepthViewer::ImageDepthViewer(
    const std::string &name, const std::filesystem::path &metafile_path,
    const std::shared_ptr<DepthCamera> &depth_camera_ptr)
    : DepthViewer{name, metafile_path, depth_camera_ptr} {}

bool ImageDepthViewer::SetUp() {
  set_up_ = false;
  if (!metafile_path_.empty())
    if (!LoadMetaData()) return false;

  // Check if all required objects are set up
  if (!depth_camera_ptr_->set_up()) {
    std::cerr << "Depth camera " << depth_camera_ptr_->name()
              << " was not set up" << std::endl;
    return false;
  }
  set_up_ = true;
  return true;
}

bool ImageDepthViewer::UpdateViewer(int save_index) {
  if (!set_up_) {
    std::cerr << "Set up image depth viewer " << name_ << " first" << std::endl;
    return false;
  }
  DisplayAndSaveImage(save_index, depth_camera_ptr_->NormalizedDepthImage(
                                      min_depth_, max_depth_));
  return true;
}

bool ImageDepthViewer::LoadMetaData() {
  // Open file storage from yaml
  cv::FileStorage fs;
  if (!OpenYamlFileStorage(metafile_path_, &fs)) return false;

  // Read parameters from yaml
  ReadOptionalValueFromYaml(fs, "min_depth", &min_depth_);
  ReadOptionalValueFromYaml(fs, "max_depth", &max_depth_);
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
