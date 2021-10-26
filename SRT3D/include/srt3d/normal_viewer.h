// SPDX-License-Identifier: MIT
// Copyright (c) 2021 Manuel Stoiber, German Aerospace Center (DLR)

#ifndef OBJECT_TRACKING_INCLUDE_SRT3D_NORMAL_VIEWER_H_
#define OBJECT_TRACKING_INCLUDE_SRT3D_NORMAL_VIEWER_H_

#include <srt3d/camera.h>
#include <srt3d/common.h>
#include <srt3d/normal_renderer.h>
#include <srt3d/renderer_geometry.h>
#include <srt3d/viewer.h>

#include <memory>
#include <opencv2/opencv.hpp>
#include <string>

namespace srt3d {

// Viewer that renders all bodies into a normal image that is displayed on top
// of a color camera image
class NormalViewer : public Viewer {
 public:
  NormalViewer(const std::string &name, std::shared_ptr<Camera> camera_ptr,
               std::shared_ptr<RendererGeometry> renderer_geometry_ptr,
               float opacity = 0.5f);
  bool SetUp() override;

  // Setters
  void set_renderer_geometry_ptr(
      std::shared_ptr<RendererGeometry> renderer_geometry_ptr);
  void set_opacity(float opacity);

  // Main methods
  bool UpdateViewer(int save_index) override;

  // Getters
  std::shared_ptr<RendererGeometry> renderer_geometry_ptr() const override;
  float opacity() const;

 private:
  void CalculateAlphaBlend(const cv::Mat &camera_image,
                           const cv::Mat &renderer_image,
                           cv::Mat *viewer_image) const;

  std::shared_ptr<RendererGeometry> renderer_geometry_ptr_ = nullptr;
  NormalRenderer renderer_;
  float opacity_ = 0.5f;
};

}  // namespace srt3d

#endif  // OBJECT_TRACKING_INCLUDE_SRT3D_NORMAL_VIEWER_H_
