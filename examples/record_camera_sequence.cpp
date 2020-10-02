// SPDX-License-Identifier: MIT
// Copyright (c) 2020 Manuel Stoiber, German Aerospace Center (DLR)

#include <rbgt/azure_kinect_camera.h>
#include <rbgt/image_viewer.h>
#include <rbgt/tracker.h>

#include <memory>
#include <string>

int main() {
  // Change accordingly
  const std::string save_path{"/your/path/to/save/camera/images/"};

  auto camera_ptr{std::make_shared<rbgt::AzureKinectCamera>()};
  camera_ptr->Init("azure kinect");
  camera_ptr->set_save_image_type("png");
  camera_ptr->StartSavingImages(save_path);

  auto viewer_ptr{std::make_shared<rbgt::ImageViewer>()};
  viewer_ptr->Init("viewer", camera_ptr);

  auto tracker_ptr{std::make_shared<rbgt::Tracker>()};
  tracker_ptr->AddViewer(viewer_ptr);
  tracker_ptr->StartTracker(false);
  return 0;
}
