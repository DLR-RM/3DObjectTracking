// SPDX-License-Identifier: MIT
// Copyright (c) 2021 Manuel Stoiber, German Aerospace Center (DLR)

#include <srt3d/azure_kinect_camera.h>
#include <srt3d/image_viewer.h>
#include <srt3d/tracker.h>

#include <filesystem>
#include <memory>
#include <string>

int main() {
  // Change accordingly
  const std::filesystem::path sequence_directory{"/your/directory/to/save/images/"};

  auto camera_ptr{std::make_shared<srt3d::AzureKinectCamera>("azure_kinect")};
  camera_ptr->StartSavingImages(sequence_directory);

  auto viewer_ptr{std::make_shared<srt3d::ImageViewer>("viewer", camera_ptr)};

  auto tracker_ptr{std::make_shared<srt3d::Tracker>("tracker")};
  tracker_ptr->AddViewer(viewer_ptr);
  tracker_ptr->SetUpTracker();
  tracker_ptr->StartTracker(false);
  return 0;
}
