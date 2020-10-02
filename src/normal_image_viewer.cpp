// SPDX-License-Identifier: MIT
// Copyright (c) 2020 Manuel Stoiber, German Aerospace Center (DLR)

#include <rbgt/normal_image_viewer.h>

namespace rbgt {

bool NormalImageViewer::Init(
    const std::string &name,
    std::shared_ptr<RendererGeometry> renderer_geometry_ptr,
    std::shared_ptr<Camera> camera_ptr) {
  name_ = name;
  renderer_geometry_ptr_ = std::move(renderer_geometry_ptr);
  camera_ptr_ = std::move(camera_ptr);

  if (!renderer_.InitFromCamera("renderer", renderer_geometry_ptr_,
                                *camera_ptr_))
    return false;

  initialized_ = true;
  return true;
}

bool NormalImageViewer::set_opacity(float opacity) {
  if (opacity < 0.0f || opacity > 1.0f) return false;
  opacity_ = opacity;
  return true;
}

void NormalImageViewer::UpdateViewer(int save_index) {
  // Calculate viewer image
  cv::Mat viewer_image{camera_ptr_->image().size(), CV_8UC3};
  renderer_.StartRendering();
  renderer_.FetchNormalImage();
  CalculateAlphaBlend(camera_ptr_->image(), renderer_.normal_image(),
                      &viewer_image);

  // Display and save images
  if (display_images_) cv::imshow(name_, viewer_image);
  if (save_images_)
    cv::imwrite(
        save_path_.string() + name_ + "_" + std::to_string(save_index) + ".png",
        viewer_image);
}

void NormalImageViewer::CalculateAlphaBlend(const cv::Mat &camera_image,
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

}  // namespace rbgt
