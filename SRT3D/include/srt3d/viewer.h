// SPDX-License-Identifier: MIT
// Copyright (c) 2021 Manuel Stoiber, German Aerospace Center (DLR)

#ifndef OBJECT_TRACKING_INCLUDE_SRT3D_VIEWER_H_
#define OBJECT_TRACKING_INCLUDE_SRT3D_VIEWER_H_

#include <srt3d/camera.h>
#include <srt3d/common.h>
#include <srt3d/renderer_geometry.h>

#include <filesystem>
#include <memory>
#include <string>

namespace srt3d {

// Abstract class that defines a viewer and functionality to view and save the
// current tracking state
class Viewer {
 public:
  // Constructor and setup method
  Viewer(const std::string &name, std::shared_ptr<Camera> camera_ptr);
  virtual bool SetUp() = 0;

  // Setters
  void set_name(const std::string &name);
  void set_camera_ptr(std::shared_ptr<Camera> camera_ptr);
  void set_display_images(bool dispaly_images);
  void StartSavingImages(const std::filesystem::path &save_directory,
                         const std::string &save_image_type = "png");
  void StopSavingImages();

  // Main methods
  virtual bool UpdateViewer(int save_index) = 0;

  // Getters
  const std::string &name() const;
  std::shared_ptr<Camera> camera_ptr() const;
  virtual std::shared_ptr<RendererGeometry> renderer_geometry_ptr() const;
  bool display_images() const;
  bool save_images() const;
  bool set_up() const;

 protected:
  // Helper methods
  void DisplayAndSaveImage(int save_index, const cv::Mat &image);

  // Variables
  std::string name_{};
  std::shared_ptr<Camera> camera_ptr_ = nullptr;
  std::filesystem::path save_directory_{};
  std::string save_image_type_{};
  bool display_images_ = true;
  bool save_images_ = false;
  bool set_up_ = false;
};

}  // namespace srt3d

#endif  // OBJECT_TRACKING_INCLUDE_SRT3D_VIEWER_H_
