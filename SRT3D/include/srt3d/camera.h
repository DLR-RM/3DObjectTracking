// SPDX-License-Identifier: MIT
// Copyright (c) 2021 Manuel Stoiber, German Aerospace Center (DLR)

#ifndef OBJECT_TRACKING_INCLUDE_SRT3D_CAMERA_H_
#define OBJECT_TRACKING_INCLUDE_SRT3D_CAMERA_H_

#include <srt3d/common.h>

#include <Eigen/Geometry>
#include <filesystem>
#include <fstream>
#include <opencv2/opencv.hpp>

namespace srt3d {

// Abstract class that defines a camera and functionality to save images.
// It is also able to hold a camera pose.
class Camera {
 public:
  // Constructor and setup method
  Camera(const std::string &name);
  virtual bool SetUp() = 0;

  // Setters
  void set_name(const std::string &name);
  void set_camera2world_pose(const Transform3fA &camera2world_pose);
  void set_world2camera_pose(const Transform3fA &world2camera_pose);
  void StartSavingImages(const std::filesystem::path &save_directory,
                         int save_index = 0,
                         const std::string &save_image_type = "png");
  void StopSavingImages();

  // Main methods
  virtual bool UpdateImage() = 0;

  // Getters
  const std::string &name() const;
  const cv::Mat &image() const;
  const Intrinsics &intrinsics() const;
  const Transform3fA &camera2world_pose() const;
  const Transform3fA &world2camera_pose() const;
  bool save_images() const;
  bool set_up() const;

 protected:
  // Helper methods
  void SaveMetaDataIfDesired() const;
  void SaveImageIfDesired();

  // Variables and data
  std::string name_{};
  cv::Mat image_;
  Intrinsics intrinsics_{};
  Transform3fA camera2world_pose_{Transform3fA::Identity()};
  Transform3fA world2camera_pose_{Transform3fA::Identity()};
  std::filesystem::path save_directory_{};
  int save_index_{};
  std::string save_image_type_{};

  // Internal state
  bool save_images_ = false;
  bool set_up_ = false;
};

}  // namespace srt3d

#endif  // OBJECT_TRACKING_INCLUDE_SRT3D_CAMERA_H_
