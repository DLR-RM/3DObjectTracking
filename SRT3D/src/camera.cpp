// SPDX-License-Identifier: MIT
// Copyright (c) 2021 Manuel Stoiber, German Aerospace Center (DLR)

#include <srt3d/camera.h>

namespace srt3d {

Camera::Camera(const std::string &name) : name_{name} {}

void Camera::set_name(const std::string &name) { name_ = name; }

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

const Intrinsics &Camera::intrinsics() const { return intrinsics_; }

const Transform3fA &Camera::camera2world_pose() const {
  return camera2world_pose_;
}

const Transform3fA &Camera::world2camera_pose() const {
  return world2camera_pose_;
}

bool Camera::save_images() const { return save_images_; }

bool Camera::set_up() const { return set_up_; }

void Camera::SaveMetaDataIfDesired() const {
  if (save_images_) {
    std::filesystem::path path{save_directory_ / (name_ + "_meta.txt")};
    std::ofstream ofs{path};
    WriteValueToFile(ofs, "image_name_pre_", (name_ + "_image_"));
    WriteValueToFile(ofs, "load_index_", save_index_);
    WriteValueToFile(ofs, "n_leading_zeros_", 0);
    WriteValueToFile(ofs, "image_name_post", std::string{""});
    WriteValueToFile(ofs, "load_image_type_", save_image_type_);
    WriteValueToFile(ofs, "intrinsics_", intrinsics_);
    WriteValueToFile(ofs, "camera2world_pose_", camera2world_pose_);
    ofs.flush();
    ofs.close();
  }
}

void Camera::SaveImageIfDesired() {
  if (save_images_) {
    std::filesystem::path path{save_directory_ / (name_ + "_image_" +
                                                  std::to_string(save_index_) +
                                                  "." + save_image_type_)};
    cv::imwrite(path.string(), image_);
    save_index_++;
  }
}

}  // namespace srt3d
