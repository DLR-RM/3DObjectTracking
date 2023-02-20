// SPDX-License-Identifier: MIT
// Copyright (c) 2023 Manuel Stoiber, German Aerospace Center (DLR)

#include <m3t/camera.h>

#include <opencv2/core/eigen.hpp>

namespace m3t {

void Camera::set_name(const std::string &name) { name_ = name; }

void Camera::set_metafile_path(const std::filesystem::path &metafile_path) {
  metafile_path_ = metafile_path;
  set_up_ = false;
}

void Camera::set_camera2world_pose(const Transform3fA &camera2world_pose) {
  camera2world_pose_ = camera2world_pose;
  world2camera_pose_ = camera2world_pose_.inverse();
}

void Camera::set_world2camera_pose(const Transform3fA &world2camera_pose) {
  world2camera_pose_ = world2camera_pose;
  camera2world_pose_ = world2camera_pose_.inverse();
}

void Camera::StartSavingImages(const std::filesystem::path &save_directory,
                               int save_index,
                               const std::string &save_image_type) {
  save_images_ = true;
  save_directory_ = save_directory;
  save_index_ = save_index;
  save_image_type_ = save_image_type;
  set_up_ = false;
}

void Camera::StopSavingImages() { save_images_ = false; }

const cv::Mat &Camera::image() const { return image_; }

const std::string &Camera::name() const { return name_; }

const std::filesystem::path &Camera::metafile_path() const {
  return metafile_path_;
}

const Intrinsics &Camera::intrinsics() const { return intrinsics_; }

const Transform3fA &Camera::camera2world_pose() const {
  return camera2world_pose_;
}

const Transform3fA &Camera::world2camera_pose() const {
  return world2camera_pose_;
}

const std::filesystem::path &Camera::save_directory() const {
  return save_directory_;
}

int Camera::save_index() const { return save_index_; }

const std::string &Camera::save_image_type() const { return save_image_type_; }

bool Camera::save_images() const { return save_images_; }

bool Camera::set_up() const { return set_up_; }

Camera::Camera(const std::string &name) : name_{name} {}

Camera::Camera(const std::string &name,
               const std::filesystem::path &metafile_path)
    : name_{name}, metafile_path_{metafile_path} {}

void Camera::SaveImageIfDesired() {
  if (save_images_) {
    std::filesystem::path path{save_directory_ / (name_ + "_image_" +
                                                  std::to_string(save_index_) +
                                                  "." + save_image_type_)};
    cv::imwrite(path.string(), image_);
    save_index_++;
  }
}

ColorCamera::ColorCamera(const std::string &name) : Camera{name} {}

ColorCamera::ColorCamera(const std::string &name,
                         const std::filesystem::path &metafile_path)
    : Camera{name, metafile_path} {}

void ColorCamera::SaveMetaDataIfDesired() const {
  if (save_images_) {
    std::filesystem::path path{save_directory_ / (name_ + ".yaml")};
    cv::FileStorage fs{path.string(), cv::FileStorage::WRITE};
    fs << "load_directory"
       << "./";
    WriteValueToYaml(fs, "intrinsics", intrinsics_);
    WriteValueToYaml(fs, "camera2world_pose", camera2world_pose_);
    fs << "image_name_pre" << name_ + "_image_";
    fs << "load_index" << save_index_;
    fs << "n_leading_zeros" << 0;
    fs << "image_name_post" << std::string{""};
    fs << "load_image_type" << save_image_type_;
    fs.release();
  }
}

cv::Mat DepthCamera::NormalizedDepthImage(float min_depth,
                                          float max_depth) const {
  float alpha = 255.0f / ((max_depth - min_depth) / depth_scale_);
  float beta = -(min_depth / depth_scale_) * alpha;
  cv::Mat normalized_image;
  image_.convertTo(normalized_image, CV_8UC1, alpha, beta);
  return normalized_image;
}

float DepthCamera::depth_scale() const { return depth_scale_; }

DepthCamera::DepthCamera(const std::string &name) : Camera{name} {}

DepthCamera::DepthCamera(const std::string &name,
                         const std::filesystem::path &metafile_path)
    : Camera{name, metafile_path} {}

void DepthCamera::SaveMetaDataIfDesired() const {
  if (save_images_) {
    std::filesystem::path path{save_directory_ / (name_ + ".yaml")};
    cv::FileStorage fs{path.string(), cv::FileStorage::WRITE};
    fs << "load_directory"
       << "./";
    WriteValueToYaml(fs, "intrinsics", intrinsics_);
    WriteValueToYaml(fs, "camera2world_pose", camera2world_pose_);
    fs << "depth_scale" << depth_scale_;
    fs << "image_name_pre" << name_ + "_image_";
    fs << "load_index" << save_index_;
    fs << "n_leading_zeros" << 0;
    fs << "image_name_post" << std::string{""};
    fs << "load_image_type" << save_image_type_;
    fs.release();
  }
}

}  // namespace m3t
