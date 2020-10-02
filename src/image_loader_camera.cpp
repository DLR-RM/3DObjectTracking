// SPDX-License-Identifier: MIT
// Copyright (c) 2020 Manuel Stoiber, German Aerospace Center (DLR)

#include <rbgt/image_loader_camera.h>

namespace rbgt {

bool ImageLoaderCamera::Init(const std::string &name,
                             const std::filesystem::path &load_path,
                             const std::string &load_name, int load_index,
                             int max_load_index) {
  initialized_ = false;
  name_ = name;
  load_path_ = load_path;
  load_name_ = load_name;
  load_index_ = load_index;
  max_load_index_ = max_load_index;

  // Open meta data file
  std::ifstream ifs;
  std::string path{load_path_.string() + load_name_ + "_meta_data.txt"};
  ifs.open(path, std::ios::binary);
  if (!ifs.is_open() || ifs.fail()) {
    ifs.close();
    std::cerr << "Could not open file " << path << std::endl;
    return false;
  }

  // Check if data is of type ColorCamera
  std::string camera_type;
  ReadValueFromFile(ifs, &camera_type);
  if (camera_type != std::string{"ColorCamera"}) {
    std::cerr << "Data is not of type ColorCamera" << std::endl;
    return false;
  }

  // Read meta data
  ReadValueFromFile(ifs, &load_name_);  // just to skip those lines
  ReadValueFromFile(ifs, &intrinsics_);
  ReadValueFromFile(ifs, &camera2world_pose_);
  world2camera_pose_ = camera2world_pose_.inverse();
  ifs.close();

  // Update image
  UpdateImage();
  initialized_ = true;
  return true;
}

bool ImageLoaderCamera::set_load_index(int load_index) {
  if (!initialized_) {
    std::cerr << "Initialize image loader camera first" << std::endl;
    return false;
  }
  load_index_ = load_index;
  UpdateImage();
  return true;
}

void ImageLoaderCamera::set_load_image_type(
    const std::string &load_image_type) {
  load_image_type_ = load_image_type;
}

bool ImageLoaderCamera::UpdateImage() {
  if (load_index_ > max_load_index_) return false;
  image_ = cv::imread(load_path_.string() + load_name_ + "_image_" +
                          std::to_string(load_index_) + "." + load_image_type_,
                      cv::IMREAD_UNCHANGED);
  load_index_++;
  SaveImageIfDesired();
  return true;
}

}  // namespace rbgt
