// SPDX-License-Identifier: MIT
// Copyright (c) 2020 Manuel Stoiber, German Aerospace Center (DLR)

#ifndef OBJECTTRACKING_INCLUDE_RBGT_DATASET_RBOT_CAMERA_H_
#define OBJECTTRACKING_INCLUDE_RBGT_DATASET_RBOT_CAMERA_H_

#include <rbgt/camera.h>
#include <rbgt/common.h>

#include <filesystem>
#include <fstream>
#include <iostream>
#include <opencv2/opencv.hpp>

namespace rbgt {

// Class that allows to load images from the RBOT dataset
class DatasetRBOTCamera : public Camera {
 public:
  // Initialization
  bool Init(const std::string &name, const std::filesystem::path &dataset_path,
            const std::string &object_name, const std::string &sequence_name,
            int load_index);
  bool set_load_index(int load_index);

  // Main method
  bool UpdateImage() override;

 private:
  std::filesystem::path dataset_path_{};
  std::string object_name_{};
  std::string sequence_name_{};
  int load_index_ = 0;
  int max_load_index_ = 1000;
};

}  // namespace rbgt

#endif  // OBJECTTRACKING_INCLUDE_RBGT_DATASET_RBOT_CAMERA_H_
