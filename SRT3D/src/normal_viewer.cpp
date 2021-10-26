// SPDX-License-Identifier: MIT
// Copyright (c) 2021 Manuel Stoiber, German Aerospace Center (DLR)

#include <srt3d/normal_viewer.h>

namespace srt3d {

NormalViewer::NormalViewer(
    const std::string &name, std::shared_ptr<Camera> camera_ptr,
    std::shared_ptr<RendererGeometry> renderer_geometry_ptr, float opacity)
    : Viewer{name, camera_ptr},
      renderer_geometry_ptr_{renderer_geometry_ptr},
      opacity_{opacity},
      renderer_{"renderer", std::move(renderer_geometry_ptr),
                std::move(camera_ptr)} {}

bool NormalViewer::SetUp() {
  set_up_ = false;

  // Check if all required objects are set up
  if (!camera_ptr_->set_up()) {
    std::cerr << "Camera " << camera_ptr_->name() << " was not set up"
              << std::endl;
    return false;
  }
  if (!renderer_geometry_ptr_->set_up()) {
    std::cerr << "Renderer geometry " << renderer_geometry_ptr_->name()
              << " was not set up" << std::endl;
    return false;
  }

  if (!renderer_.SetUp()) return false;
  set_up_ = true;
  return true;
}

void NormalViewer::set_renderer_geometry_ptr(
    std::shared_ptr<RendererGeometry> renderer_geometry_ptr) {
  renderer_geometry_ptr_ = std::move(renderer_geometry_ptr);
  set_up_ = false;
}

void NormalViewer::set_opacity(float opacity) { opacity_ = opacity; }

bool NormalViewer::UpdateViewer(int save_index) {
  if (!set_up_) {
    std::cerr << "Set up viewer " << name_ << " first" << std::endl;
    return false;
  }

  // Calculate viewer image
  cv::Mat viewer_image{camera_ptr_->image().size(), CV_8UC3};
  renderer_.StartRendering();
  renderer_.FetchNormalImage();
  CalculateAlphaBlend(camera_ptr_->image(), renderer_.normal_image(),
                      &viewer_image);

  // Display and save images
  DisplayAndSaveImage(save_index, viewer_image);
}

std::shared_ptr<RendererGeometry> NormalViewer::renderer_geometry_ptr() const {
  return renderer_geometry_ptr_;
}

float NormalViewer::opacity() const { return opacity_; }

void NormalViewer::CalculateAlphaBlend(const cv::Mat &camera_image,
                                       const cv::Mat &renderer_image,
                                       cv::Mat *viewer_image) const {
  // Declare variables
  int v, u;
  const cv::Vec3b *ptr_camera_image;
  const cv::Vec4b *ptr_renderer_image;
  cv::Vec3b *ptr_viewer_image;
  const uchar *val_camera_image;
  const uchar *val_renderer_image;
  uchar *val_viewer_image;
  float alpha, alpha_inv;
  float alpha_scale = opacity_ / 255.0f;

  // Iterate over all pixels
  for (v = 0; v < camera_image.rows; ++v) {
    ptr_camera_image = camera_image.ptr<cv::Vec3b>(v);
    ptr_renderer_image = renderer_image.ptr<cv::Vec4b>(v);
    ptr_viewer_image = viewer_image->ptr<cv::Vec3b>(v);
    for (u = 0; u < camera_image.cols; ++u) {
      val_camera_image = ptr_camera_image[u].val;
      val_renderer_image = ptr_renderer_image[u].val;
      val_viewer_image = ptr_viewer_image[u].val;

      // Blend images
      alpha = float(val_renderer_image[3]) * alpha_scale;
      alpha_inv = 1.0f - alpha;
      val_viewer_image[0] =
          char(val_camera_image[0] * alpha_inv + val_renderer_image[0] * alpha);
      val_viewer_image[1] =
          char(val_camera_image[1] * alpha_inv + val_renderer_image[1] * alpha);
      val_viewer_image[2] =
          char(val_camera_image[2] * alpha_inv + val_renderer_image[2] * alpha);
    }
  }
}

}  // namespace srt3d
