// SPDX-License-Identifier: MIT
// Copyright (c) 2021 Manuel Stoiber, German Aerospace Center (DLR)

#ifndef OBJECT_TRACKING_INCLUDE_SRT3D_LOADER_CAMERA_H_
#define OBJECT_TRACKING_INCLUDE_SRT3D_LOADER_CAMERA_H_

#include <srt3d/camera.h>
#include <srt3d/common.h>

#include <filesystem>
#include <fstream>
#include <iostream>
#include <opencv2/opencv.hpp>

namespace srt3d {

// Class that allows to load images from a directory.
class LoaderCamera : public Camera {
 public:
  // Constructor and setup method
  LoaderCamera(const std::string &name,
               const std::filesystem::path &load_directory,
               const Intrinsics &intrinsics, const std::string &image_name_pre,
               int load_index = 0, int n_leading_zeros = 0,
               const std::string &image_name_post = "",
               const std::string &load_image_type = "png");
  LoaderCamera(const std::string &name,
               const std::filesystem::path &load_directory,
               const std::string &meta_filename);
  bool SetUp() override;

  // Setters
  void set_load_directory(const std::filesystem::path &load_directory);
  void set_image_name_pre(const std::string &image_name_pre);
  void set_load_index(int load_index);
  void set_n_leading_zeros(int n_leading_zeros);
  void set_image_name_post(const std::string &image_name_post);
  void set_load_image_type(const std::string &load_image_type);

  // Main method
  bool UpdateImage() override;

  // Getters
  const std::filesystem::path &load_directory() const;
  const std::string &image_name_pre() const;
  int load_index() const;
  int n_leading_zeros() const;
  const std::string &image_name_post() const;
  const std::string &load_image_type() const;

 private:
  // Helper method
  bool LoadMetaData();

  // Data
  std::filesystem::path load_directory_{};
  std::string meta_filename_{};
  std::string image_name_pre_{};
  int load_index_{};
  int n_leading_zeros_{};
  std::string image_name_post_{};
  std::string load_image_type_{};
};

}  // namespace srt3d

#endif  // OBJECT_TRACKING_INCLUDE_SRT3D_LOADER_CAMERA_H_
