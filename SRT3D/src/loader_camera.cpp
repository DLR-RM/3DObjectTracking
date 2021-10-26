// SPDX-License-Identifier: MIT
// Copyright (c) 2021 Manuel Stoiber, German Aerospace Center (DLR)

#include <srt3d/loader_camera.h>

namespace srt3d {

LoaderCamera::LoaderCamera(const std::string &name,
                           const std::filesystem::path &load_directory,
                           const Intrinsics &intrinsics,
                           const std::string &image_name_pre, int load_index,
                           int n_leading_zeros,
                           const std::string &image_name_post,
                           const std::string &load_image_type)
    : Camera{name},
      load_directory_{load_directory},
      image_name_pre_{image_name_pre},
      load_index_{load_index},
      n_leading_zeros_{n_leading_zeros},
      image_name_post_{image_name_post},
      load_image_type_{load_image_type} {
  intrinsics_ = intrinsics;
}

LoaderCamera::LoaderCamera(const std::string &name,
                           const std::filesystem::path &load_directory,
                           const std::string &meta_filename)
    : Camera{name},
      load_directory_{load_directory},
      meta_filename_{meta_filename} {}

bool LoaderCamera::SetUp() {
  set_up_ = false;
  if (!meta_filename_.empty())
    if (!LoadMetaData()) return false;
  SaveMetaDataIfDesired();
  set_up_ = true;
  return UpdateImage();
}

void LoaderCamera::set_load_directory(
    const std::filesystem::path &load_directory) {
  load_directory_ = load_directory;
  set_up_ = false;
}

void LoaderCamera::set_image_name_pre(const std::string &image_name_pre) {
  image_name_pre_ = image_name_pre;
  set_up_ = false;
}

void LoaderCamera::set_load_index(int load_index) {
  load_index_ = load_index;
  set_up_ = false;
}

void LoaderCamera::set_n_leading_zeros(int n_leading_zeros) {
  n_leading_zeros_ = n_leading_zeros;
  set_up_ = false;
}

void LoaderCamera::set_image_name_post(const std::string &image_name_post) {
  image_name_post_ = image_name_post;
  set_up_ = false;
}

void LoaderCamera::set_load_image_type(const std::string &load_image_type) {
  load_image_type_ = load_image_type;
  set_up_ = false;
}

bool LoaderCamera::UpdateImage() {
  if (!set_up_) {
    std::cerr << "Set up camera " << name_ << " first" << std::endl;
    return false;
  }

  int n_zeros =
      std::max(n_leading_zeros_ - int(std::to_string(load_index_).length()), 0);
  std::filesystem::path path{load_directory_ /
                             (image_name_pre_ + std::string(n_zeros, '0') +
                              std::to_string(load_index_) + image_name_post_ +
                              "." + load_image_type_)};
  image_ = cv::imread(path.string(), cv::IMREAD_UNCHANGED);
  if (image_.empty()) {
    std::cerr << "Could not read image from " << path.string() << std::endl;
    return false;
  }

  load_index_++;
  SaveImageIfDesired();
  return true;
}

const std::filesystem::path &LoaderCamera::load_directory() const {
  return load_directory_;
}

const std::string &LoaderCamera::image_name_pre() const {
  return image_name_pre_;
}

int LoaderCamera::load_index() const { return load_index_; }

int LoaderCamera::n_leading_zeros() const { return n_leading_zeros_; }

const std::string &LoaderCamera::image_name_post() const {
  return image_name_post_;
}

const std::string &LoaderCamera::load_image_type() const {
  return load_image_type_;
}

bool LoaderCamera::LoadMetaData() {
  // Open meta data file
  std::filesystem::path path{load_directory_ / meta_filename_};
  std::ifstream ifs{path, std::ios::binary};
  if (!ifs.is_open() || ifs.fail()) {
    ifs.close();
    std::cerr << "Could not open file " << path << std::endl;
    return false;
  }

  // Read meta data
  ReadValueFromFile(ifs, &image_name_pre_);
  ReadValueFromFile(ifs, &load_index_);
  ReadValueFromFile(ifs, &n_leading_zeros_);
  ReadValueFromFile(ifs, &image_name_post_);
  ReadValueFromFile(ifs, &load_image_type_);
  ReadValueFromFile(ifs, &intrinsics_);
  ReadValueFromFile(ifs, &camera2world_pose_);
  world2camera_pose_ = camera2world_pose_.inverse();
  ifs.close();
  return true;
}

}  // namespace srt3d
