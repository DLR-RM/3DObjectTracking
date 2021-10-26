// SPDX-License-Identifier: MIT
// Copyright (c) 2021 Manuel Stoiber, German Aerospace Center (DLR)

#ifndef OBJECT_TRACKING_INCLUDE_SRT3D_AZURE_KINECT_CAMERA_H_
#define OBJECT_TRACKING_INCLUDE_SRT3D_AZURE_KINECT_CAMERA_H_

#include <srt3d/camera.h>
#include <srt3d/common.h>

#include <chrono>
#include <filesystem>
#include <iostream>
#include <k4a/k4a.hpp>
#include <opencv2/opencv.hpp>

namespace srt3d {

// Class that allows to get color images from an AzureKinect camera
class AzureKinectCamera : public Camera {
 public:
  // Constructor, destructor, and setup method
  AzureKinectCamera(const std::string &name, float image_scale = 1.05f);
  ~AzureKinectCamera();
  bool SetUp() override;

  // Setters
  void set_image_scale(float image_scale);

  // Main method
  bool UpdateImage() override;

  // Getters
  float image_scale() const;

 private:
  // Helper methods
  bool StartAzureKinect();
  void GetIntrinsicsAndDistortionMap();

  // Data
  k4a::device device_;
  k4a::capture capture_;
  k4a_device_configuration_t config_;
  float image_scale_ = 1.05f;
  cv::Mat distortion_map_;
  bool initial_set_up_ = false;
};

}  // namespace srt3d

#endif  // OBJECT_TRACKING_INCLUDE_SRT3D_AZURE_KINECT_CAMERA_H_
