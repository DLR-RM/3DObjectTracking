// SPDX-License-Identifier: MIT
// Copyright (c) 2020 Manuel Stoiber, German Aerospace Center (DLR)

#ifndef OBJECT_TRACKING_INCLUDE_RBGT_VIEWER_H_
#define OBJECT_TRACKING_INCLUDE_RBGT_VIEWER_H_

#include <rbgt/camera.h>
#include <rbgt/common.h>

#include <filesystem>
#include <memory>
#include <string>

namespace rbgt {

// Abstract class that defines a viewer and functionality to view and save the
// current tracking state
class Viewer {
 public:
  // Setter methods
  void set_display_images(bool dispaly_images);

  // Main methods
  virtual void UpdateViewer(int save_index) = 0;
  void StartSavingImages(const std::filesystem::path &path);
  void StopSavingImages();

  // Getters
  const std::string &name() const;
  std::shared_ptr<Camera> camera_ptr() const;
  const std::filesystem::path &save_path() const;
  bool display_images() const;
  bool save_images() const;
  bool initialized() const;

 protected:
  // Variables
  std::string name_{};
  std::shared_ptr<Camera> camera_ptr_ = nullptr;
  std::filesystem::path save_path_{};
  bool display_images_ = true;
  bool save_images_ = false;
  bool initialized_ = false;
};

}  // namespace rbgt

#endif  // OBJECT_TRACKING_INCLUDE_RBGT_VIEWER_H_
