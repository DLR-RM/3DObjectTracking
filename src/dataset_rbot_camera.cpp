// SPDX-License-Identifier: MIT
// Copyright (c) 2020 Manuel Stoiber, German Aerospace Center (DLR)

#include <rbgt/dataset_rbot_camera.h>

namespace rbgt {

bool DatasetRBOTCamera::Init(const std::string &name,
                             const std::filesystem::path &dataset_path,
                             const std::string &object_name,
                             const std::string &sequence_name, int load_index) {
  initialized_ = false;
  name_ = name;
  dataset_path_ = dataset_path;
  object_name_ = object_name;
  sequence_name_ = sequence_name;
  load_index_ = load_index;

  // Open camera calibration file
  std::ifstream ifs;
  std::string path{dataset_path_.string() + "camera_calibration.txt"};
  ifs.open(path, std::ios::binary);
  if (!ifs.is_open() || ifs.fail()) {
    ifs.close();
    std::cerr << "Could not open file " << path << std::endl;
    return false;
  }

  // Read and define intrinsics
  std::string parsed;
  std::getline(ifs, parsed);
  std::getline(ifs, parsed, '\t');
  intrinsics_.fu = stof(parsed);
  std::getline(ifs, parsed, '\t');
  intrinsics_.fv = stof(parsed);
  std::getline(ifs, parsed, '\t');
  intrinsics_.ppu = stof(parsed);
  std::getline(ifs, parsed, '\t');
  intrinsics_.ppv = stof(parsed);
  intrinsics_.width = 640;
  intrinsics_.height = 512;
  ifs.close();

  // Update Image
  UpdateImage();
  initialized_ = true;
  return true;
}

bool DatasetRBOTCamera::set_load_index(int load_index) {
  if (!initialized_) {
    std::cerr << "Initialize dataset RBOT camera first" << std::endl;
    return false;
  }
  load_index_ = load_index;
  UpdateImage();
  return true;
}

bool DatasetRBOTCamera::UpdateImage() {
  if (load_index_ > max_load_index_) return false;
  std::filesystem::path image_path{
      dataset_path_ / object_name_ / "frames" /
      (sequence_name_ +
       std::string(4 - std::to_string(load_index_).length(), '0') +
       std::to_string(load_index_) + ".png")};
  image_ = cv::imread(image_path.string(), cv::IMREAD_UNCHANGED);
  load_index_++;
  SaveImageIfDesired();
  return true;
}

}  // namespace rbgt
