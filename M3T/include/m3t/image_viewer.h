// SPDX-License-Identifier: MIT
// Copyright (c) 2023 Manuel Stoiber, German Aerospace Center (DLR)

#ifndef M3T_INCLUDE_M3T_IMAGE_VIEWER_H_
#define M3T_INCLUDE_M3T_IMAGE_VIEWER_H_

#include <m3t/camera.h>
#include <m3t/common.h>
#include <m3t/viewer.h>

#include <memory>
#include <opencv2/opencv.hpp>
#include <string>

namespace m3t {

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

}  // namespace m3t

#endif  // M3T_INCLUDE_M3T_IMAGE_VIEWER_H_
