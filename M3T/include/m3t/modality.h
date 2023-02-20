// SPDX-License-Identifier: MIT
// Copyright (c) 2023 Manuel Stoiber, German Aerospace Center (DLR)

#ifndef M3T_INCLUDE_M3T_MODALITY_H_
#define M3T_INCLUDE_M3T_MODALITY_H_

#include <filesystem/filesystem.h>
#include <m3t/body.h>
#include <m3t/camera.h>
#include <m3t/color_histograms.h>
#include <m3t/common.h>
#include <m3t/model.h>

#include <filesystem/filesystem.h>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <iostream>
#include <memory>
#include <opencv2/opencv.hpp>
#include <string>
#include <vector>

namespace m3t {

/**
 * \brief Abstract class that considers information from a \ref Camera to
 * calculate the gradient vector and Hessian matrix that are used by the \ref
 * Optimizer to update the \ref Body pose.
 *
 * \details It provides interfaces for `StartModality()`,
 * `CalculateCorrespondences()`, `VisualizeCorrespondences()`,
 * `CalculateGradientAndHessian()`, `VisualizeOptimization()`,
 * `CalculateResults()`, and `VisualizeResults()` that are used by the \ref
 * Tracker object to coordinate tracking. The getter functions
 * `imshow_correspondence()`, `imshow_optimization()`, and `imshow_result()`
 * indicate if `VisualizeCorrespondences()`, `VisualizeOptimization()`, and
 * `VisualizeResults()` show images. The class also includes functionality to
 * save visualizations using `StartSavingVisualizations()` and
 * `StopSavingVisualizations()`. In addition, it provides interfaces to access
 * referenced \ref Camera, \ref Model, \ref ColorHistograms, and \ref Renderer
 * objects.
 *
 * @param body_ptr referenced \ref Body object that is considered by the
 * modality.
 * @param visualize_pose_result print the `body2world_pose` during
 * `VisualizeResults()`.
 * @param visualize_gradient_optimization print the gradient vector during
 * `VisualizeOptimization()`.
 * @param visualize_hessian_optimization print the hessian matrix during
 * `VisualizeOptimization()`.
 * @param display_visualization if true visualization images are displayed.
 * @param save_directory directory to which visualization images are saved.
 * @param save_image_type file format to which visualization images are saved.
 * @param save_visualization if true, visualization images are saved.
 */
class Modality {
 public:
  // Setup methods
  virtual bool SetUp() = 0;

  // Setters data
  void set_name(const std::string &name);
  void set_metafile_path(const std::filesystem::path &metafile_path);
  void set_body_ptr(const std::shared_ptr<Body> &body_ptr);

  // Setter methods visualization
  void set_visualize_pose_result(bool visualize_pose_result);
  void set_visualize_gradient_optimization(
      bool visualize_gradient_optimization);
  void set_visualize_hessian_optimization(bool visualize_hessian_optimization);

  // Setters for general visualization settings
  void set_display_visualization(bool display_visualization);
  void StartSavingVisualizations(const std::filesystem::path &save_directory,
                                 const std::string &save_image_type = "png");
  void StopSavingVisualizations();

  // Main methods
  virtual bool StartModality(int iteration, int corr_iteration) = 0;
  virtual bool CalculateCorrespondences(int iteration, int corr_iteration) = 0;
  virtual bool VisualizeCorrespondences(int save_idx) = 0;
  virtual bool CalculateGradientAndHessian(int iteration, int corr_iteration,
                                           int opt_iteration) = 0;
  virtual bool VisualizeOptimization(int save_idx) = 0;
  virtual bool CalculateResults(int iteration) = 0;
  virtual bool VisualizeResults(int save_idx) = 0;

  // Getters gradient and hessian
  const Eigen::Matrix<float, 6, 1> &gradient() const;
  const Eigen::Matrix<float, 6, 6> &hessian() const;

  // Getters data
  const std::string &name() const;
  const std::filesystem::path &metafile_path() const;
  const std::shared_ptr<Body> &body_ptr() const;
  virtual std::shared_ptr<Model> model_ptr() const;
  virtual std::vector<std::shared_ptr<Camera>> camera_ptrs() const = 0;
  virtual std::vector<std::shared_ptr<Renderer>> start_modality_renderer_ptrs()
      const;
  virtual std::vector<std::shared_ptr<Renderer>> correspondence_renderer_ptrs()
      const;
  virtual std::vector<std::shared_ptr<Renderer>> results_renderer_ptrs() const;
  virtual std::shared_ptr<ColorHistograms> color_histograms_ptr() const;
  bool set_up() const;

  // Getters visuatlization state
  bool imshow_correspondence() const;
  bool imshow_optimization() const;
  bool imshow_result() const;

  // Getters methods visualization
  bool visualize_pose_result() const;
  bool visualize_gradient_optimization() const;
  bool visualize_hessian_optimization() const;

  // Getters for general visualization settings
  bool display_visualization() const;
  const std::filesystem::path &save_directory() const;
  const std::string &save_image_type() const;

 protected:
  // Constructor
  Modality(const std::string &name, const std::shared_ptr<Body> &body_ptr);
  Modality(const std::string &name, const std::filesystem::path &metafile_path,
           const std::shared_ptr<Body> &body_ptr);

  // Helper methods for visualization
  void VisualizePose();
  void VisualizeGradient();
  void VisualizeHessian();

  // Internal data
  std::string name_;
  std::filesystem::path metafile_path_{};
  Eigen::Matrix<float, 6, 1> gradient_;
  Eigen::Matrix<float, 6, 6> hessian_;
  std::shared_ptr<Body> body_ptr_ = nullptr;

  // Parameters to turn on individual visualizations
  bool visualize_pose_result_ = false;
  bool visualize_gradient_optimization_ = false;
  bool visualize_hessian_optimization_ = false;

  // Parameters for general visualization settings
  bool display_visualization_ = true;
  bool save_visualizations_ = false;
  std::filesystem::path save_directory_{};
  std::string save_image_type_ = "png";

  // Visualization and state variables
  bool imshow_correspondence_ = false;
  bool imshow_optimization_ = false;
  bool imshow_result_ = false;
  bool set_up_ = false;
};

}  // namespace m3t

#endif  // M3T_INCLUDE_M3T_MODALITY_H_
