// SPDX-License-Identifier: MIT
// Copyright (c) 2023 Manuel Stoiber, German Aerospace Center (DLR)

#ifndef M3T_INCLUDE_M3T_VIEWER_H_
#define M3T_INCLUDE_M3T_VIEWER_H_

#include <filesystem/filesystem.h>
#include <m3t/camera.h>
#include <m3t/common.h>
#include <m3t/renderer_geometry.h>

#include <memory>
#include <string>

namespace m3t {

/**
 * \brief Abstract class that defines a viewer and functionality to view and
 * save images.
 *
 * \details Using the main method `UpdateViewer()`, views are updated. The class
 * also includes functionality to save images using `StartSavingImages()` and
 * `StopSavingImages()`.
 *
 * @param display_images if true images are displayed.
 * @param save_directory directory to which images are saved.
 * @param save_image_type file format to which images are saved.
 * @param save_images if true, images are saved.
 */
class Viewer {
 public:
  // Setup method
  virtual bool SetUp() = 0;

  // Setters
  void set_name(const std::string &name);
  void set_metafile_path(const std::filesystem::path &metafile_path);
  void set_display_images(bool display_images);
  void StartSavingImages(const std::filesystem::path &save_directory,
                         const std::string &save_image_type = "png");
  void StopSavingImages();

  // Main methods
  virtual bool UpdateViewer(int save_index) = 0;

  // Getters
  const std::string &name() const;
  const std::filesystem::path &metafile_path() const;
  virtual std::shared_ptr<Camera> camera_ptr() const = 0;
  virtual std::shared_ptr<RendererGeometry> renderer_geometry_ptr() const;
  bool display_images() const;
  const std::filesystem::path &save_directory() const;
  const std::string &save_image_type() const;
  bool save_images() const;
  bool set_up() const;

 protected:
  // Constructor
  Viewer(const std::string &name);
  Viewer(const std::string &name, const std::filesystem::path &metafile_path);

  // Helper methods
  void DisplayAndSaveImage(int save_index, const cv::Mat &image);

  // Variables
  std::string name_{};
  std::filesystem::path metafile_path_{};
  std::filesystem::path save_directory_{};
  std::string save_image_type_ = "png";
  bool display_images_ = true;
  bool save_images_ = false;
  bool set_up_ = false;
};

/**
 * \brief Abstract \ref Viewer class that defines a color viewer.
 *
 * @param color_camera_ptr referenced \ref ColorCamera from which images are
 * taken.
 */
class ColorViewer : public Viewer {
 public:
  // Setup method
  virtual bool SetUp() override = 0;

  // Setters
  void set_color_camera_ptr(
      const std::shared_ptr<ColorCamera> &color_camera_ptr);

  // Main methods
  virtual bool UpdateViewer(int save_index) override = 0;

  // Getters
  const std::shared_ptr<ColorCamera> &color_camera_ptr() const;
  std::shared_ptr<Camera> camera_ptr() const override;

 protected:
  // Constructor
  ColorViewer(const std::string &name,
              const std::shared_ptr<ColorCamera> &color_camera_ptr);
  ColorViewer(const std::string &name,
              const std::filesystem::path &metafile_path,
              const std::shared_ptr<ColorCamera> &color_camera_ptr);

  // Data
  std::shared_ptr<ColorCamera> color_camera_ptr_ = nullptr;
};

/**
 * \brief Abstract \ref Viewer class that defines a depth viewer that normalizes
 * images between a set minimum and maximum depth value.
 *
 * @param depth_camera_ptr referenced \ref DepthCamera from which images are
 * taken.
 * @param min_depth minimum depth for the normalization of depth images.
 * @param max_depth maximum depth for the normalization of depth images.
 */
class DepthViewer : public Viewer {
 public:
  // Setup method
  virtual bool SetUp() override = 0;

  // Setters
  void set_depth_camera_ptr(
      const std::shared_ptr<DepthCamera> &depth_camera_ptr);
  void set_min_depth(float min_depth);
  void set_max_depth(float max_depth);

  // Main methods
  virtual bool UpdateViewer(int save_index) override = 0;

  // Getters
  const std::shared_ptr<DepthCamera> &depth_camera_ptr() const;
  std::shared_ptr<Camera> camera_ptr() const override;
  float min_depth() const;
  float max_depth() const;

 protected:
  // Constructor
  DepthViewer(const std::string &name,
              const std::shared_ptr<DepthCamera> &depth_camera_ptr,
              float min_depth, float max_depth);
  DepthViewer(const std::string &name,
              const std::filesystem::path &metafile_path,
              const std::shared_ptr<DepthCamera> &depth_camera_ptr);

  // Data
  std::shared_ptr<DepthCamera> depth_camera_ptr_ = nullptr;
  float min_depth_ = 0.0f;
  float max_depth_ = 1.0f;
};

}  // namespace m3t

#endif  // M3T_INCLUDE_M3T_VIEWER_H_
