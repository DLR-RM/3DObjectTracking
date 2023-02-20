// SPDX-License-Identifier: MIT
// Copyright (c) 2023 Manuel Stoiber, German Aerospace Center (DLR)

#include <filesystem/filesystem.h>
#include <m3t/azure_kinect_camera.h>
#include <m3t/image_viewer.h>
#include <m3t/tracker.h>

#include <memory>
#include <string>

int main(int argc, char *argv[]) {
  if (argc != 2) {
    std::cerr << "Not enough arguments: Provide sequence directory";
    return -1;
  }
  const std::filesystem::path sequence_directory{argv[1]};

  auto color_camera_ptr{
      std::make_shared<m3t::AzureKinectColorCamera>("color_camera")};
  auto depth_camera_ptr{
      std::make_shared<m3t::AzureKinectDepthCamera>("depth_camera")};
  color_camera_ptr->StartSavingImages(sequence_directory);
  depth_camera_ptr->StartSavingImages(sequence_directory);

  auto color_viewer_ptr{std::make_shared<m3t::ImageColorViewer>(
      "color_viewer", color_camera_ptr)};
  auto depth_viewer_ptr{std::make_shared<m3t::ImageDepthViewer>(
      "depth_viewer", depth_camera_ptr, 0.1f, 2.0f)};

  auto tracker_ptr{std::make_shared<m3t::Tracker>("tracker")};
  tracker_ptr->AddViewer(color_viewer_ptr);
  tracker_ptr->AddViewer(depth_viewer_ptr);
  if (!tracker_ptr->SetUp()) return -1;
  if (!tracker_ptr->RunTrackerProcess(false, false)) return -1;
  return 0;
}
