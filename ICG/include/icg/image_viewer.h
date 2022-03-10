// SPDX-License-Identifier: MIT
// Copyright (c) 2022 Manuel Stoiber, German Aerospace Center (DLR)

#ifndef ICG_INCLUDE_ICG_IMAGE_VIEWER_H_
#define ICG_INCLUDE_ICG_IMAGE_VIEWER_H_

#include <icg/camera.h>
#include <icg/common.h>
#include <icg/viewer.h>

#include <memory>
#include <opencv2/opencv.hpp>
#include <string>

namespace icg {

/**
 * \brief \ref Viewer that displays color images from a \ref ColorCamera.
 */
class ImageColorViewer : public ColorViewer {
 public:
  ImageColorViewer(const std::string &name,
                   const std::shared_ptr<ColorCamera> &color_camera_ptr);
  ImageColorViewer(const std::string &name,
                   const std::filesystem::path &metafile_path,
                   const std::shared_ptr<ColorCamera> &color_camera_ptr);
  bool SetUp() override;

  bool UpdateViewer(int save_index) override;

 private:
  // Helper method
  bool LoadMetaData();
};

/**
 * \brief \ref Viewer that displays depth images from a \ref DepthCamera that
 * are normalized between a defined minimum and maximum depth value.
 */
class ImageDepthViewer : public DepthViewer {
 public:
  ImageDepthViewer(const std::string &name,
                   const std::shared_ptr<DepthCamera> &depth_camera_ptr,
                   float min_depth = 0.0f, float max_depth = 1.0f);
  ImageDepthViewer(const std::string &name,
                   const std::filesystem::path &metafile_path,
                   const std::shared_ptr<DepthCamera> &depth_camera_ptr);
  bool SetUp() override;

  bool UpdateViewer(int save_index) override;

 private:
  // Helper method
  bool LoadMetaData();
};

}  // namespace icg

#endif  // ICG_INCLUDE_ICG_IMAGE_VIEWER_H_
