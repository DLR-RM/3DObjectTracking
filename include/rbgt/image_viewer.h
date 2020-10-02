// SPDX-License-Identifier: MIT
// Copyright (c) 2020 Manuel Stoiber, German Aerospace Center (DLR)

#ifndef OBJECT_TRACKING_INCLUDE_RBGT_IMAGE_VIEWER_H_
#define OBJECT_TRACKING_INCLUDE_RBGT_IMAGE_VIEWER_H_

#include <rbgt/camera.h>
#include <rbgt/common.h>
#include <rbgt/viewer.h>

#include <memory>
#include <opencv2/opencv.hpp>
#include <string>

namespace rbgt {

// Viewer that displays a color image
class ImageViewer : public Viewer {
 public:
  void Init(const std::string &name, std::shared_ptr<Camera> camera_ptr);
  ImageViewer() = default;

  void UpdateViewer(int save_index) override;
};

}  // namespace rbgt

#endif  // OBJECT_TRACKING_INCLUDE_RBGT_IMAGE_VIEWER_H_
