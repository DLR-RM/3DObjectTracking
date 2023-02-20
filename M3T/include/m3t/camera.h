// SPDX-License-Identifier: MIT
// Copyright (c) 2023 Manuel Stoiber, German Aerospace Center (DLR)

#ifndef M3T_INCLUDE_M3T_CAMERA_H_
#define M3T_INCLUDE_M3T_CAMERA_H_

#include <filesystem/filesystem.h>
#include <m3t/common.h>

#include <Eigen/Geometry>
#include <fstream>
#include <opencv2/opencv.hpp>

namespace m3t {

/**
 * \brief Abstract class that provides images to other components and contains
 * intrinsics as well as the pose of the camera relative to the world coordinate
 * frame.
 *
 * \details Using the main method `UpdateImage(bool synchronized)`, a new image
 * is obtained. If the flag `synchronized` is true, a corresponding real-world
 * camera waits until a new image arrives. The class also includes functionality
 * to save images using `StartSavingImages()` and `StopSavingImages()`.
 *
 * @param camera2world_pose pose of the camera relative to the world frame.
 * @param save_directory directory to which images are saved.
 * @param save_index index of the first image that is saved.
 * @param save_image_type file format to which images are saved.
 * @param save_images if true, images are saved.
 */
class Camera {
 public:
  // Setup method
  virtual bool SetUp() = 0;

  // Setters
  void set_name(const std::string &name);
  void set_metafile_path(const std::filesystem::path &metafile_path);
  void set_camera2world_pose(const Transform3fA &camera2world_pose);
  void set_world2camera_pose(const Transform3fA &world2camera_pose);
  void StartSavingImages(const std::filesystem::path &save_directory,
                         int save_index = 0,
                         const std::string &save_image_type = "png");
  void StopSavingImages();

  // Main methods
  virtual bool UpdateImage(bool synchronized) = 0;

  // Getters
  const std::string &name() const;
  const std::filesystem::path &metafile_path() const;
  const cv::Mat &image() const;
  const Intrinsics &intrinsics() const;
  const Transform3fA &camera2world_pose() const;
  const Transform3fA &world2camera_pose() const;
  const std::filesystem::path &save_directory() const;
  int save_index() const;
  const std::string &save_image_type() const;
  bool save_images() const;
  bool set_up() const;

 protected:
  // Constructor
  Camera(const std::string &name);
  Camera(const std::string &name, const std::filesystem::path &metafile_path);

  // Helper methods
  void SaveImageIfDesired();

  // Data
  std::string name_{};
  std::filesystem::path metafile_path_{};
  cv::Mat image_;
  Intrinsics intrinsics_{};
  Transform3fA camera2world_pose_{Transform3fA::Identity()};
  Transform3fA world2camera_pose_{Transform3fA::Identity()};
  std::filesystem::path save_directory_{};
  int save_index_ = 0;
  std::string save_image_type_ = "png";

  // Internal state
  bool save_images_ = false;
  bool set_up_ = false;
};

/**
 * \brief Abstract \ref Camera class that defines a color camera.
 */
class ColorCamera : public Camera {
 public:
  // Setup method
  virtual bool SetUp() override = 0;

  // Main methods
  virtual bool UpdateImage(bool synchronized) override = 0;

 protected:
  // Constructor
  ColorCamera(const std::string &name);
  ColorCamera(const std::string &name,
              const std::filesystem::path &metafile_path);

  // Helper methods
  void SaveMetaDataIfDesired() const;
};

/**
 * \brief Abstract \ref Camera class that defines a depth camera that, in
 * addition to intrinsics, specifies the `depth_scale`.
 *
 * \details It also provides functionality to normalize a depth image between
 * `min_depth`, and `max_depth` using the `NormalizedDepthImage()` method.
 */
class DepthCamera : public Camera {
 public:
  // Setup method
  virtual bool SetUp() override = 0;

  // Main methods
  virtual bool UpdateImage(bool synchronized) override = 0;
  cv::Mat NormalizedDepthImage(float min_depth, float max_depth) const;

  // Getters
  float depth_scale() const;

 protected:
  // Constructor
  DepthCamera(const std::string &name);
  DepthCamera(const std::string &name,
              const std::filesystem::path &metafile_path);

  // Helper methods
  void SaveMetaDataIfDesired() const;

  // Data
  float depth_scale_ = 0.001f;
};

}  // namespace m3t

#endif  // M3T_INCLUDE_M3T_CAMERA_H_
