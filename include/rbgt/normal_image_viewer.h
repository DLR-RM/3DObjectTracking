// SPDX-License-Identifier: MIT
// Copyright (c) 2020 Manuel Stoiber, German Aerospace Center (DLR)

#ifndef OBJECT_TRACKING_INCLUDE_RBGT_NORMAL_IMAGE_VIEWER_H_
#define OBJECT_TRACKING_INCLUDE_RBGT_NORMAL_IMAGE_VIEWER_H_

#include <rbgt/camera.h>
#include <rbgt/common.h>
#include <rbgt/normal_image_renderer.h>
#include <rbgt/renderer_geometry.h>
#include <rbgt/viewer.h>

#include <memory>
#include <opencv2/opencv.hpp>
#include <string>

namespace rbgt {

// Viewer that renders all bodies into a normal image that is displayed on top
// of a color camera image
class NormalImageViewer : public Viewer {
 public:
  bool Init(const std::string &name,
            std::shared_ptr<RendererGeometry> renderer_geometry,
            std::shared_ptr<Camera> camera_ptr);
  bool set_opacity(float opacity);

  void UpdateViewer(int save_index) override;

 private:
  void CalculateAlphaBlend(const cv::Mat &camera_image,
                           const cv::Mat &renderer_image,
                           cv::Mat *viewer_image) const;

  NormalImageRenderer renderer_;
  std::shared_ptr<RendererGeometry> renderer_geometry_ptr_ = nullptr;
  float opacity_ = 0.5f;
};

}  // namespace rbgt

#endif  // OBJECT_TRACKING_INCLUDE_RBGT_NORMAL_IMAGE_VIEWER_H_
