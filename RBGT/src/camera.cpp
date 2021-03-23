// SPDX-License-Identifier: MIT
// Copyright (c) 2020 Manuel Stoiber, German Aerospace Center (DLR)

#include <rbgt/camera.h>

namespace rbgt {

void Camera::set_camera2world_pose(const Transform3fA &camera2world_pose) {
  camera2world_pose_ = camera2world_pose;
  world2camera_pose_ = camera2world_pose_.inverse();
}

void Camera::set_world2camera_pose(const Transform3fA &world2camera_pose) {
  world2camera_pose_ = world2camera_pose;
  camera2world_pose_ = world2camera_pose_.inverse();
}

void Camera::set_save_index(int save_index) { save_index_ = save_index; }

void Camera::set_save_image_type(const std::string &save_image_type) {
  save_image_type_ = save_image_type;
}

void Camera::StartSavingImages(const std::filesystem::path &path) {
  save_images_ = true;
  save_path_ = path;
  SaveMetaData();
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

const std::filesystem::path &Camera::save_path() const { return save_path_; }

int Camera::save_index() const { return save_index_; }

const std::string &Camera::save_image_type() const { return save_image_type_; }

bool Camera::save_images() const { return save_images_; }

bool Camera::initialized() const { return initialized_; }

void Camera::SaveMetaData() const {
  std::ofstream ofs(save_path_.string() + name_ + "_meta_data.txt");
  WriteValueToFile(ofs, "camera_type", std::string("ColorCamera"));
  WriteValueToFile(ofs, "name_", name_);
  WriteValueToFile(ofs, "intrinsics_", intrinsics_);
  WriteValueToFile(ofs, "camera2world_pose_", camera2world_pose_);
  WriteValueToFile(ofs, "start_index", save_index_);
  ofs.flush();
  ofs.close();
}

void Camera::SaveImageIfDesired() {
  if (save_images_) {
    cv::imwrite(save_path_.string() + name_ + "_image_" +
                    std::to_string(save_index_) + "." + save_image_type_,
                image_);
    save_index_++;
  }
}

}  // namespace rbgt
