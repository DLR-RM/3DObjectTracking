// SPDX-License-Identifier: MIT
// Copyright (c) 2023 Manuel Stoiber, German Aerospace Center (DLR)

#ifndef M3T_INCLUDE_M3T_LOADER_CAMERA_H_
#define M3T_INCLUDE_M3T_LOADER_CAMERA_H_

#include <filesystem/filesystem.h>
#include <m3t/camera.h>
#include <m3t/common.h>

#include <fstream>
#include <iostream>
#include <opencv2/opencv.hpp>

namespace m3t {

/**
 * \brief \ref Camera that allows loading color images from a directory.
 * @param load_directory directory from which images are loaded.
 * @param intrinsics intrinsics of the camera that was used to record images.
 * @param image_name_pre text at the beginning of image name before load_index.
 * @param load_index index of the first image that is loaded.
 * @param n_leading_zeros minimum number of digits.
 * @param image_name_post text at the end of image name after load_index.
 * @param load_image_type file format to which images are saved.
 */
class LoaderColorCamera : public ColorCamera {
 public:
  // Constructor and setup method
  LoaderColorCamera(const std::string &name,
                    const std::filesystem::path &load_directory,
                    const Intrinsics &intrinsics,
                    const std::string &image_name_pre = "", int load_index = 0,
                    int n_leading_zeros = 0,
                    const std::string &image_name_post = "",
                    const std::string &load_image_type = "png");
  LoaderColorCamera(const std::string &name,
                    const std::filesystem::path &metafile_path);
  bool SetUp() override;

  // Setters
  void set_load_directory(const std::filesystem::path &load_directory);
  void set_intrinsics(const Intrinsics &intrinsics);
  void set_image_name_pre(const std::string &image_name_pre);
  void set_load_index(int load_index);
  void set_n_leading_zeros(int n_leading_zeros);
  void set_image_name_post(const std::string &image_name_post);
  void set_load_image_type(const std::string &load_image_type);

  // Main method
  bool UpdateImage(bool synchronized) override;

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
  std::string image_name_pre_ = "";
  int load_index_ = 0;
  int n_leading_zeros_ = 0;
  std::string image_name_post_ = "";
  std::string load_image_type_ = "png";
};

/**
 * \brief \ref Camera that allows loading depth images from a directory.
 * @param load_directory directory from which images are loaded.
 * @param intrinsics intrinsics of the camera that was used to record images.
 * @param depth_scale scale with which pixel values have to be multiplied to get
 * depth in meter.
 * @param image_name_pre text at the beginning of image name before load_index.
 * @param load_index index of the first image that is loaded.
 * @param n_leading_zeros minimum number of digits.
 * @param image_name_post text at the end of image name after load_index.
 * @param load_image_type file format to which images are saved.
 */
class LoaderDepthCamera : public DepthCamera {
 public:
  // Constructor and setup method
  LoaderDepthCamera(const std::string &name,
                    const std::filesystem::path &load_directory,
                    const Intrinsics &intrinsics, float depth_scale,
                    const std::string &image_name_pre = "", int load_index = 0,
                    int n_leading_zeros = 0,
                    const std::string &image_name_post = "",
                    const std::string &load_image_type = "png");
  LoaderDepthCamera(const std::string &name,
                    const std::filesystem::path &metafile_path);
  bool SetUp() override;

  // Setters
  void set_load_directory(const std::filesystem::path &load_directory);
  void set_intrinsics(const Intrinsics &intrinsics);
  void set_depth_scale(float depth_scale);
  void set_image_name_pre(const std::string &image_name_pre);
  void set_load_index(int load_index);
  void set_n_leading_zeros(int n_leading_zeros);
  void set_image_name_post(const std::string &image_name_post);
  void set_load_image_type(const std::string &load_image_type);

  // Main method
  bool UpdateImage(bool synchronized) override;

  // Getters
  const std::filesystem::path &load_directory() const;
  const std::string &image_name_pre() const;
  int load_index() const;
  int n_leading_zeros() const;
  const std::string &image_name_post() const;
  const std::string &load_image_type() const;

 private:
  // Helper methods
  bool LoadMetaData();

  // Data
  std::filesystem::path load_directory_{};
  std::string image_name_pre_ = "";
  int load_index_ = 0;
  int n_leading_zeros_ = 0;
  std::string image_name_post_ = "";
  std::string load_image_type_ = "png";
};

}  // namespace m3t

#endif  // M3T_INCLUDE_M3T_LOADER_CAMERA_H_
