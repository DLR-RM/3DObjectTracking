// SPDX-License-Identifier: MIT
// Copyright (c) 2020 Manuel Stoiber, German Aerospace Center (DLR)

#ifndef OBJECT_TRACKING_INCLUDE_RBGT_IMAGE_LOADER_CAMERA_H_
#define OBJECT_TRACKING_INCLUDE_RBGT_IMAGE_LOADER_CAMERA_H_

#include <rbgt/camera.h>
#include <rbgt/common.h>

#include <filesystem>
#include <fstream>
#include <iostream>
#include <opencv2/opencv.hpp>

namespace rbgt {

// Class that allows to load images saved by a camera
class ImageLoaderCamera : public Camera {
 public:
  // Initialization
  bool Init(const std::string &name, const std::filesystem::path &load_path,
            const std::string &load_name, int load_index, int max_load_index);
  bool set_load_index(int load_index);
  void set_load_image_type(const std::string &load_image_type);

  // Main method
  bool UpdateImage() override;

 private:
  std::filesystem::path load_path_{};
  std::string load_name_{};
  int load_index_{};
  int max_load_index_{};
  std::string load_image_type_{"png"};
};

}  // namespace rbgt

#endif  // OBJECT_TRACKING_INCLUDE_RBGT_IMAGE_LOADER_CAMERA_H_
