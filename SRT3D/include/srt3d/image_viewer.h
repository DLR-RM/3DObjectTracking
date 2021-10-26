// SPDX-License-Identifier: MIT
// Copyright (c) 2021 Manuel Stoiber, German Aerospace Center (DLR)

#ifndef OBJECT_TRACKING_INCLUDE_SRT3D_IMAGE_VIEWER_H_
#define OBJECT_TRACKING_INCLUDE_SRT3D_IMAGE_VIEWER_H_

#include <srt3d/camera.h>
#include <srt3d/common.h>
#include <srt3d/viewer.h>

#include <memory>
#include <opencv2/opencv.hpp>
#include <string>

namespace srt3d {

// Viewer that displays a color image
class ImageViewer : public Viewer {
 public:
  ImageViewer(const std::string &name, std::shared_ptr<Camera> camera_ptr);
  bool SetUp() override;

  bool UpdateViewer(int save_index) override;
};

}  // namespace srt3d

#endif  // OBJECT_TRACKING_INCLUDE_SRT3D_IMAGE_VIEWER_H_
