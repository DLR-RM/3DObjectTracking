// SPDX-License-Identifier: MIT
// Copyright (c) 2023 Manuel Stoiber and Anne Elisabeth Reichert,
// German Aerospace Center (DLR)

#ifndef M3T_INCLUDE_M3T_MANUAL_DETECTOR_H_
#define M3T_INCLUDE_M3T_MANUAL_DETECTOR_H_

#include <m3t/camera.h>
#include <m3t/common.h>
#include <m3t/detector.h>
#include <m3t/optimizer.h>

#include <filesystem/filesystem.h>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <fstream>
#include <iostream>
#include <opencv2/opencv.hpp>
#include <string>
#include <vector>

namespace m3t {

/**
 * \brief Class that shows an image on which the user is able to specify points
 * and that is used by the \ref ManualDetector class.
 *
 * \details It is able to load a detector image to visualize the points the user
 * should select. Calling the method `DetectPoints()` starts the selection
 * process and stores the image coordinates in `detected_points`.
 *
 * @param image image that is shown.
 * @param window_name name of the window in which the image is shown.
 * @param detector_image_path optional path to a helper image in which the
 * position of `reference_points` can be marked.
 */
class PointDetector {
 public:
  PointDetector(const cv::Mat& image, const std::string& window_name,
                const std::filesystem::path& detector_image_path);

  // Setters
  void set_image(const cv::Mat& image);
  void set_window_name(const std::string& window_name);
  void set_detector_image_path(
      const std::filesystem::path& detector_image_path);

  // Main methods
  void static MouseClickCallback(int event, int x, int y, int flags,
                                 void* param);
  bool DetectPoints();

  // Getters
  const cv::Mat& image() const;
  const std::string& window_name() const;
  const std::filesystem::path& detector_image_path() const;
  const std::vector<cv::Point2f>& detected_points() const;

 private:
  cv::Mat image_{};
  cv::Mat viewer_image_{};
  std::string window_name_{};
  std::filesystem::path detector_image_path_{};
  std::vector<cv::Point2f> detected_points_{};
};

/**
 * \brief \ref Detector that allows a user to select 4 points in the image and
 * that, based on those points, infers the pose of the root_link and updates all
 * \ref Link and \ref Body objects referenced by the \ref Optimizer object.
 *
 * \details To select points, the \ref PointDetector class is employed. Using
 * the main method `DetectPoses()`, the detection process is started and poses
 * of referenced \ref Link and \ref Body objects are set.
 *
 * @param optimizer_ptr referenced \ref Optimizer object for which referenced
 * body and link poses are set.
 * @param color_camera_ptr referenced \ref ColorCamera object that is used.
 * @param reference_points 4 3D model points on the body's surface that are
 * given in the body reference frame.
 * @param detector_image_path optional path to a helper image in which the
 * position of `reference_points` can be marked.
 */
class ManualDetector : public m3t::Detector {
 private:
  // Maximum reasonable distance
  static constexpr float kMaxDistance = 1000.0f;

 public:
  // Constructor and setup method
  ManualDetector(const std::string& name,
                 const std::shared_ptr<Optimizer>& optimizer_ptr,
                 const std::shared_ptr<ColorCamera>& color_camera_ptr,
                 const std::vector<cv::Point3f>& reference_points,
                 const std::filesystem::path& detector_image_path = {},
                 bool reset_joint_poses = true);
  ManualDetector(const std::string& name,
                 const std::filesystem::path& metafile_path,
                 const std::shared_ptr<Optimizer>& optimizer_ptr,
                 const std::shared_ptr<ColorCamera>& color_camera_ptr);
  bool SetUp() override;

  // Setters
  void set_optimizer_ptr(const std::shared_ptr<Optimizer>& optimizer_ptr);
  void set_color_camera_ptr(
      const std::shared_ptr<ColorCamera>& color_camera_ptr);
  void set_reference_points(const std::vector<cv::Point3f>& reference_points);
  void set_detector_image_path(
      const std::filesystem::path& detector_image_path);

  // Main methods
  bool DetectPoses(const std::set<std::string>& names,
                   std::set<std::string>* detected_names = nullptr) override;

  // Getters
  const std::shared_ptr<Optimizer>& optimizer_ptr() const;
  const std::shared_ptr<ColorCamera>& color_camera_ptr() const;
  const std::vector<cv::Point3f>& reference_points() const;
  const std::filesystem::path& detector_image_path() const;
  std::vector<std::shared_ptr<Optimizer>> optimizer_ptrs() const override;
  std::shared_ptr<Camera> camera_ptr() const override;

 private:
  // Helper methods
  cv::Mat GetCameraMatrixFromIntrinsics() const;
  bool LoadMetaData();

  // Data
  std::shared_ptr<Optimizer> optimizer_ptr_{};
  std::shared_ptr<ColorCamera> color_camera_ptr_{};
  std::vector<cv::Point3f> reference_points_{};
  std::filesystem::path detector_image_path_{};
};

}  // namespace m3t

#endif  // M3T_INCLUDE_M3T_MANUAL_DETECTOR_H_
