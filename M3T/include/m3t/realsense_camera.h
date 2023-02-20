// SPDX-License-Identifier: MIT
// Copyright (c) 2023 Manuel Stoiber, German Aerospace Center (DLR)

#ifndef M3T_INCLUDE_M3T_REALSENSE_CAMERA_H_
#define M3T_INCLUDE_M3T_REALSENSE_CAMERA_H_

#include <filesystem/filesystem.h>
#include <m3t/camera.h>
#include <m3t/common.h>

#include <chrono>
#include <iostream>
#include <librealsense2/rs.hpp>
#include <mutex>
#include <opencv2/opencv.hpp>

namespace m3t {

/**
 * \brief Singleton class that allows getting data from a single RealSense
 * instance and that is used by \ref RealSenseColorCamera and \ref
 * RealSenseDepthCamera.
 *
 * \details The method `UpdateCapture()` updates the `capture` object if
 * `UpdateCapture()` was already called with the same `id` before. If
 * `UpdateCapture()` was not yet called by the `id`, the same capture is used.
 * If the capture is updated, all memory values except for the `id` that called
 * the function are reset. All methods that are required to operate multiple
 * \ref Camera objects are thread-safe.
 */
class RealSense {
 public:
  /// Singleton instance getter
  static RealSense &GetInstance();
  RealSense(const RealSense &) = delete;
  RealSense &operator=(const RealSense &) = delete;
  ~RealSense();

  // Configuration and setup
  void UseColorCamera();
  void UseDepthCamera();
  int RegisterID();
  bool UnregisterID(int id);
  bool SetUp();

  // Main methods
  bool UpdateCapture(int id, bool synchronized);

  // Getters
  bool use_color_camera() const;
  bool use_depth_camera() const;
  const rs2::frameset &frameset() const;
  const rs2::pipeline_profile &profile() const;
  const Transform3fA *color2depth_pose() const;
  const Transform3fA *depth2color_pose() const;

 private:
  RealSense() = default;

  // Private data
  rs2::config config_;
  rs2::pipeline pipe_;
  std::map<int, bool> update_capture_ids_{};
  int next_id_ = 0;

  // Public data
  rs2::pipeline_profile profile_;
  rs2::frameset frameset_;
  Transform3fA color2depth_pose_{Transform3fA::Identity()};
  Transform3fA depth2color_pose_{Transform3fA::Identity()};

  // Internal state variables
  std::mutex mutex_;
  bool use_color_camera_ = false;
  bool use_depth_camera_ = false;
  bool initial_set_up_ = false;
};

/**
 * \brief \ref Camera that allows getting color images from a \ref RealSense
 * camera.
 *
 * @param use_depth_as_world_frame specifies the depth camera frame as world
 * frame and automatically defines `camera2world_pose` as `color2depth_pose`.
 */
class RealSenseColorCamera : public ColorCamera {
 public:
  // Constructors, destructor, and setup method
  RealSenseColorCamera(const std::string &name,
                       bool use_depth_as_world_frame = false);
  RealSenseColorCamera(const std::string &name,
                       const std::filesystem::path &metafile_path);
  RealSenseColorCamera(const RealSenseColorCamera &) = delete;
  RealSenseColorCamera &operator=(const RealSenseColorCamera &) = delete;
  ~RealSenseColorCamera();
  bool SetUp() override;

  // Setters
  void set_use_depth_as_world_frame(bool use_depth_as_world_frame);

  // Main method
  bool UpdateImage(bool synchronized) override;

  // Getters
  bool use_depth_as_world_frame() const;
  const Transform3fA *color2depth_pose() const;
  const Transform3fA *depth2color_pose() const;

 private:
  // Helper methods
  bool LoadMetaData();
  void GetIntrinsics();

  // Data
  RealSense &realsense_;
  int realsense_id_{};
  bool use_depth_as_world_frame_ = false;
  bool initial_set_up_ = false;
};

/**
 * \brief \ref Camera that allows getting depth images from a \ref RealSense
 * camera.
 *
 * @param use_color_as_world_frame specifies the color camera frame as world
 * frame and automatically defines `camera2world_pose` as `depth2color_pose`.
 */
class RealSenseDepthCamera : public DepthCamera {
 public:
  // Constructors, destructor, and setup method
  RealSenseDepthCamera(const std::string &name,
                       bool use_color_as_world_frame = true);
  RealSenseDepthCamera(const std::string &name,
                       const std::filesystem::path &metafile_path);
  RealSenseDepthCamera(const RealSenseDepthCamera &) = delete;
  RealSenseDepthCamera &operator=(const RealSenseDepthCamera &) = delete;
  ~RealSenseDepthCamera();
  bool SetUp() override;

  // Setters
  void set_use_color_as_world_frame(bool use_color_as_world_frame);

  // Main method
  bool UpdateImage(bool synchronized) override;

  // Getters
  bool use_color_as_world_frame() const;
  const Transform3fA *color2depth_pose() const;
  const Transform3fA *depth2color_pose() const;

 private:
  // Helper methods
  bool LoadMetaData();
  void GetIntrinsics();

  // Data
  RealSense &realsense_;
  int realsense_id_{};
  bool use_color_as_world_frame_ = true;
  bool initial_set_up_ = false;
};

}  // namespace m3t

#endif  // M3T_INCLUDE_M3T_REALSENSE_CAMERA_H_
