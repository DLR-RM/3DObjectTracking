// SPDX-License-Identifier: MIT
// Copyright (c) 2020 Manuel Stoiber, German Aerospace Center (DLR)

#ifndef OBJECT_TRACKING_INCLUDE_RBGT_CAMERA_H_
#define OBJECT_TRACKING_INCLUDE_RBGT_CAMERA_H_

#include <rbgt/common.h>

#include <Eigen/Geometry>
#include <filesystem>
#include <fstream>
#include <opencv2/opencv.hpp>

namespace rbgt {

// Abstract class that defines a camera and functionality to save images.
// It is also able to hold a camera pose.
class Camera {
 public:
  // Setters
  void set_camera2world_pose(const Transform3fA &camera2world_pose);
  void set_world2camera_pose(const Transform3fA &camera2world_pose);
  void set_save_index(int save_index);
  void set_save_image_type(const std::string &save_image_type);

  // Main methods
  virtual bool UpdateImage() = 0;
  void StartSavingImages(const std::filesystem::path &path);
  void StopSavingImages();

  // Getters
  const cv::Mat &image() const;
  const std::string &name() const;
  const Intrinsics &intrinsics() const;
  const Transform3fA &camera2world_pose() const;
  const Transform3fA &world2camera_pose() const;
  const std::filesystem::path &save_path() const;
  int save_index() const;
  const std::string &save_image_type() const;
  bool save_images() const;
  bool initialized() const;

 protected:
  // Helper methods
  void SaveMetaData() const;
  void SaveImageIfDesired();

  // Variables and data
  cv::Mat image_;
  std::string name_{};
  Intrinsics intrinsics_{};
  Transform3fA camera2world_pose_{Transform3fA::Identity()};
  Transform3fA world2camera_pose_{Transform3fA::Identity()};
  std::filesystem::path save_path_{};
  int save_index_ = 0;
  std::string save_image_type_{"bmp"};

  // Internal state
  bool save_images_ = false;
  bool initialized_ = false;
};

}  // namespace rbgt

#endif  // OBJECT_TRACKING_INCLUDE_RBGT_CAMERA_H_
