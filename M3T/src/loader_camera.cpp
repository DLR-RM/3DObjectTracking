// SPDX-License-Identifier: MIT
// Copyright (c) 2023 Manuel Stoiber, German Aerospace Center (DLR)

#include <m3t/loader_camera.h>

#include <opencv2/core/eigen.hpp>

namespace m3t {

LoaderColorCamera::LoaderColorCamera(
    const std::string &name, const std::filesystem::path &load_directory,
    const Intrinsics &intrinsics, const std::string &image_name_pre,
    int load_index, int n_leading_zeros, const std::string &image_name_post,
    const std::string &load_image_type)
    : ColorCamera{name},
      load_directory_{load_directory},
      image_name_pre_{image_name_pre},
      load_index_{load_index},
      n_leading_zeros_{n_leading_zeros},
      image_name_post_{image_name_post},
      load_image_type_{load_image_type} {
  intrinsics_ = intrinsics;
}

LoaderColorCamera::LoaderColorCamera(const std::string &name,
                                     const std::filesystem::path &metafile_path)
    : ColorCamera{name, metafile_path} {}

bool LoaderColorCamera::SetUp() {
  set_up_ = false;
  if (!metafile_path_.empty())
    if (!LoadMetaData()) return false;
  SaveMetaDataIfDesired();
  set_up_ = true;
  return UpdateImage(true);
}

void LoaderColorCamera::set_load_directory(
    const std::filesystem::path &load_directory) {
  load_directory_ = load_directory;
  set_up_ = false;
}

void LoaderColorCamera::set_intrinsics(const Intrinsics &intrinsics) {
  intrinsics_ = intrinsics;
  set_up_ = false;
}

void LoaderColorCamera::set_image_name_pre(const std::string &image_name_pre) {
  image_name_pre_ = image_name_pre;
  set_up_ = false;
}

void LoaderColorCamera::set_load_index(int load_index) {
  load_index_ = load_index;
  set_up_ = false;
}

void LoaderColorCamera::set_n_leading_zeros(int n_leading_zeros) {
  n_leading_zeros_ = n_leading_zeros;
  set_up_ = false;
}

void LoaderColorCamera::set_image_name_post(
    const std::string &image_name_post) {
  image_name_post_ = image_name_post;
  set_up_ = false;
}

void LoaderColorCamera::set_load_image_type(
    const std::string &load_image_type) {
  load_image_type_ = load_image_type;
  set_up_ = false;
}

bool LoaderColorCamera::UpdateImage(bool synchronized) {
  if (!set_up_) {
    std::cerr << "Set up loader color camera " << name_ << " first"
              << std::endl;
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

const std::filesystem::path &LoaderColorCamera::load_directory() const {
  return load_directory_;
}

const std::string &LoaderColorCamera::image_name_pre() const {
  return image_name_pre_;
}

int LoaderColorCamera::load_index() const { return load_index_; }

int LoaderColorCamera::n_leading_zeros() const { return n_leading_zeros_; }

const std::string &LoaderColorCamera::image_name_post() const {
  return image_name_post_;
}

const std::string &LoaderColorCamera::load_image_type() const {
  return load_image_type_;
}

bool LoaderColorCamera::LoadMetaData() {
  // Open file storage from yaml
  cv::FileStorage fs;
  if (!OpenYamlFileStorage(metafile_path_, &fs)) return false;

  // Read parameters from yaml
  if (!(ReadRequiredValueFromYaml(fs, "load_directory", &load_directory_) &&
        ReadRequiredValueFromYaml(fs, "intrinsics", &intrinsics_))) {
    std::cerr << "Could not read all required body parameters from "
              << metafile_path_ << std::endl;
    return false;
  }
  ReadOptionalValueFromYaml(fs, "camera2world_pose", &camera2world_pose_);
  ReadOptionalValueFromYaml(fs, "save_directory", &save_directory_);
  ReadOptionalValueFromYaml(fs, "save_index", &save_index_);
  ReadOptionalValueFromYaml(fs, "save_image_type", &save_image_type_);
  ReadOptionalValueFromYaml(fs, "save_images", &save_images_);
  ReadOptionalValueFromYaml(fs, "image_name_pre", &image_name_pre_);
  ReadOptionalValueFromYaml(fs, "load_index", &load_index_);
  ReadOptionalValueFromYaml(fs, "n_leading_zeros", &n_leading_zeros_);
  ReadOptionalValueFromYaml(fs, "image_name_post", &image_name_post_);
  ReadOptionalValueFromYaml(fs, "load_image_type", &load_image_type_);
  fs.release();

  // Process parameters
  if (load_directory_.is_relative())
    load_directory_ = metafile_path_.parent_path() / load_directory_;
  if (save_directory_.is_relative())
    save_directory_ = metafile_path_.parent_path() / save_directory_;
  world2camera_pose_ = camera2world_pose_.inverse();
  return true;
}

LoaderDepthCamera::LoaderDepthCamera(
    const std::string &name, const std::filesystem::path &load_directory,
    const Intrinsics &intrinsics, float depth_scale,
    const std::string &image_name_pre, int load_index, int n_leading_zeros,
    const std::string &image_name_post, const std::string &load_image_type)
    : DepthCamera{name},
      load_directory_{load_directory},
      image_name_pre_{image_name_pre},
      load_index_{load_index},
      n_leading_zeros_{n_leading_zeros},
      image_name_post_{image_name_post},
      load_image_type_{load_image_type} {
  intrinsics_ = intrinsics;
  depth_scale_ = depth_scale;
}

LoaderDepthCamera::LoaderDepthCamera(const std::string &name,
                                     const std::filesystem::path &metafile_path)
    : DepthCamera{name, metafile_path} {}

bool LoaderDepthCamera::SetUp() {
  set_up_ = false;
  if (!metafile_path_.empty())
    if (!LoadMetaData()) return false;
  SaveMetaDataIfDesired();
  set_up_ = true;
  return UpdateImage(true);
}

void LoaderDepthCamera::set_load_directory(
    const std::filesystem::path &load_directory) {
  load_directory_ = load_directory;
  set_up_ = false;
}

void LoaderDepthCamera::set_intrinsics(const Intrinsics &intrinsics) {
  intrinsics_ = intrinsics;
  set_up_ = false;
}

void LoaderDepthCamera::set_depth_scale(float depth_scale) {
  depth_scale_ = depth_scale;
  set_up_ = false;
}

void LoaderDepthCamera::set_image_name_pre(const std::string &image_name_pre) {
  image_name_pre_ = image_name_pre;
  set_up_ = false;
}

void LoaderDepthCamera::set_load_index(int load_index) {
  load_index_ = load_index;
  set_up_ = false;
}

void LoaderDepthCamera::set_n_leading_zeros(int n_leading_zeros) {
  n_leading_zeros_ = n_leading_zeros;
  set_up_ = false;
}

void LoaderDepthCamera::set_image_name_post(
    const std::string &image_name_post) {
  image_name_post_ = image_name_post;
  set_up_ = false;
}

void LoaderDepthCamera::set_load_image_type(
    const std::string &load_image_type) {
  load_image_type_ = load_image_type;
  set_up_ = false;
}

bool LoaderDepthCamera::UpdateImage(bool synchronized) {
  if (!set_up_) {
    std::cerr << "Set up loader depth camera " << name_ << " first"
              << std::endl;
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

const std::filesystem::path &LoaderDepthCamera::load_directory() const {
  return load_directory_;
}

const std::string &LoaderDepthCamera::image_name_pre() const {
  return image_name_pre_;
}

int LoaderDepthCamera::load_index() const { return load_index_; }

int LoaderDepthCamera::n_leading_zeros() const { return n_leading_zeros_; }

const std::string &LoaderDepthCamera::image_name_post() const {
  return image_name_post_;
}

const std::string &LoaderDepthCamera::load_image_type() const {
  return load_image_type_;
}

bool LoaderDepthCamera::LoadMetaData() {
  // Open file storage from yaml
  cv::FileStorage fs;
  if (!OpenYamlFileStorage(metafile_path_, &fs)) return false;

  // Read parameters from yaml
  if (!(ReadRequiredValueFromYaml(fs, "load_directory", &load_directory_) &&
        ReadRequiredValueFromYaml(fs, "intrinsics", &intrinsics_) &&
        ReadRequiredValueFromYaml(fs, "depth_scale", &depth_scale_))) {
    std::cerr << "Could not read all required body parameters from "
              << metafile_path_ << std::endl;
    return false;
  }
  ReadOptionalValueFromYaml(fs, "camera2world_pose", &camera2world_pose_);
  ReadOptionalValueFromYaml(fs, "save_directory", &save_directory_);
  ReadOptionalValueFromYaml(fs, "save_index", &save_index_);
  ReadOptionalValueFromYaml(fs, "save_image_type", &save_image_type_);
  ReadOptionalValueFromYaml(fs, "save_images", &save_images_);
  ReadOptionalValueFromYaml(fs, "image_name_pre", &image_name_pre_);
  ReadOptionalValueFromYaml(fs, "load_index", &load_index_);
  ReadOptionalValueFromYaml(fs, "n_leading_zeros", &n_leading_zeros_);
  ReadOptionalValueFromYaml(fs, "image_name_post", &image_name_post_);
  ReadOptionalValueFromYaml(fs, "load_image_type", &load_image_type_);
  fs.release();

  // Process parameters
  if (load_directory_.is_relative())
    load_directory_ = metafile_path_.parent_path() / load_directory_;
  if (save_directory_.is_relative())
    save_directory_ = metafile_path_.parent_path() / save_directory_;
  world2camera_pose_ = camera2world_pose_.inverse();
  return true;
}

}  // namespace m3t
