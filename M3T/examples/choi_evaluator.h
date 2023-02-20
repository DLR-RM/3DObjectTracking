// SPDX-License-Identifier: MIT
// Copyright (c) 2023 Manuel Stoiber, German Aerospace Center (DLR)

#ifndef M3T_EXAMPLES_CHOI_EVALUATOR_H_
#define M3T_EXAMPLES_CHOI_EVALUATOR_H_

#include <m3t/body.h>
#include <m3t/common.h>
#include <m3t/depth_modality.h>
#include <m3t/depth_model.h>
#include <m3t/link.h>
#include <m3t/loader_camera.h>
#include <m3t/normal_viewer.h>
#include <m3t/region_modality.h>
#include <m3t/region_model.h>
#include <m3t/renderer_geometry.h>
#include <m3t/texture_modality.h>
#include <m3t/tracker.h>

#include <filesystem/filesystem.h>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <array>
#include <fstream>
#include <iostream>
#include <iterator>
#include <memory>
#include <numeric>
#include <random>
#include <string>
#include <vector>

/**
 * \brief Class that holds a tracker to evaluate it on the Choi dataset.
 * \details It includes functionality to calculate mean vertex errors, compute
 * AUC (Area Under Curve) scores, measure execution times, and save results to a
 * file.
 */
class ChoiEvaluator {
 private:
  static constexpr m3t::Intrinsics kChoiIntrinsics{525.0f, 525.0f, 319.0f,
                                                   239.0f, 640,    480};

 public:
  struct ExecutionTimes {
    float complete_cycle = 0.0f;
    float calculate_correspondences = 0.0f;
    float calculate_gradient_and_hessian = 0.0f;
    float calculate_optimization = 0.0f;
    float calculate_results = 0.0f;
  };

  struct Result {
    int frame_index = 0;
    float error_x = 0.0f;
    float error_y = 0.0f;
    float error_z = 0.0f;
    float error_alpha = 0.0f;
    float error_beta = 0.0f;
    float error_gamma = 0.0f;
    ExecutionTimes execution_times;
  };

  struct SequenceResult {
    std::vector<Result> frame_results{};
    float rms_error_x = 0.0f;
    float rms_error_y = 0.0f;
    float rms_error_z = 0.0f;
    float rms_error_alpha = 0.0f;
    float rms_error_beta = 0.0f;
    float rms_error_gamma = 0.0f;
    float mean_translation_error = 0.0f;
    float mean_rotation_error = 0.0f;
    ExecutionTimes mean_execution_times;
  };

  // Constructors and setup method
  ChoiEvaluator(const std::string &name,
                const std::filesystem::path &dataset_directory,
                const std::filesystem::path &external_directory,
                const std::vector<std::string> &body_names);
  bool SetUp();

  // Setters for evaluation
  void set_run_sequentially(bool run_sequentially);
  void set_visualize_tracking(bool visualize_tracking);
  void set_visualize_frame_results(bool visualize_frame_results);

  // Setters for tracker configuration
  void set_use_region_modality(bool use_region_modality);
  void set_use_depth_modality(bool use_depth_modality);
  void set_use_texture_modality(bool use_texture_modality);
  void set_measure_occlusions_region(bool measure_occlusions_region);
  void set_measure_occlusions_depth(bool measure_occlusions_depth);
  void set_measure_occlusions_texture(bool measure_occlusions_texture);

  // Setters for object setters
  void set_tracker_setter(
      const std::function<void(std::shared_ptr<m3t::Tracker>)> &tracker_setter);
  void set_optimizer_setter(
      const std::function<void(std::shared_ptr<m3t::Optimizer>)>
          &optimizer_setter);
  void set_region_modality_setter(
      const std::function<void(std::shared_ptr<m3t::RegionModality>)>
          &region_modality_setter);
  void set_region_model_setter(
      const std::function<void(std::shared_ptr<m3t::RegionModel>)>
          &region_model_setter);
  void set_depth_modality_setter(
      const std::function<void(std::shared_ptr<m3t::DepthModality>)>
          &depth_modality_setter);
  void set_depth_model_setter(
      const std::function<void(std::shared_ptr<m3t::DepthModel>)>
          &depth_model_setter);
  void set_texture_modality_setter(
      const std::function<void(std::shared_ptr<m3t::TextureModality>)>
          &texture_modality_setter);

  // Main methods
  bool Evaluate();
  void SaveResults(std::filesystem::path path) const;

  // Getters
  float mean_translation_error() const;
  float mean_rotation_error() const;
  float execution_time() const;
  std::map<std::string, SequenceResult> final_results() const;

 private:
  // Helper methods to run evaluation
  bool EvaluateBody(
      const std::string &body_name,
      const std::shared_ptr<m3t::RendererGeometry> &renderer_geometry_ptr,
      SequenceResult *sequence_result) const;
  bool SetUpTracker(
      const std::string &body_name,
      const std::shared_ptr<m3t::RendererGeometry> &renderer_geometry_ptr,
      std::shared_ptr<m3t::Tracker> *tracker_ptr) const;
  void ExecuteMeasuredTrackingCycle(
      const std::shared_ptr<m3t::Tracker> &tracker_ptr, int iteration,
      ExecutionTimes *execution_times) const;

  // Helper methods for the calculation of results
  SequenceResult CalculateAverageFinalResult() const;
  static void CalculateAverageSequenceResult(SequenceResult *result);
  void CalculatePoseResults(const m3t::Transform3fA &body2world_pose,
                            const m3t::Transform3fA &gt_body2world_pose,
                            Result *result) const;
  static void VisualizeFrameResult(const Result &result,
                                   const std::string &title);
  static void VisualizeSequenceResult(const SequenceResult &result,
                                      const std::string &title);

  // General helper methods
  bool LoadBodies();
  bool GenerateModels();
  bool GetGTPosesChoiDataset(
      const std::string &body_name,
      std::vector<m3t::Transform3fA> *gt_body2world_poses) const;
  static float ElapsedTime(
      const std::chrono::high_resolution_clock::time_point &begin_time);

  // Internal data objects
  std::map<std::string, std::shared_ptr<m3t::Body>> body2body_ptr_map_;
  std::map<std::string, std::shared_ptr<m3t::RegionModel>>
      body2region_model_ptr_map_;
  std::map<std::string, std::shared_ptr<m3t::DepthModel>>
      body2depth_model_ptr_map_;
  std::map<std::string, SequenceResult> final_results_{};

  // Parameters for Choi dataset
  std::filesystem::path dataset_directory_;
  std::filesystem::path external_directory_;
  std::vector<std::string> body_names_;

  // Parameters for tracker configuration
  bool use_region_modality_;
  bool use_depth_modality_;
  bool use_texture_modality_ = true;
  bool measure_occlusions_region_ = true;
  bool measure_occlusions_depth_ = true;
  bool measure_occlusions_texture_ = true;

  // Setters for object setters
  std::function<void(std::shared_ptr<m3t::Tracker>)> tracker_setter_{
      [](auto) {}};
  std::function<void(std::shared_ptr<m3t::Optimizer>)> optimizer_setter_{
      [](auto) {}};
  std::function<void(std::shared_ptr<m3t::RegionModality>)>
      region_modality_setter_{[](auto) {}};
  std::function<void(std::shared_ptr<m3t::RegionModel>)> region_model_setter_{
      [](auto) {}};
  std::function<void(std::shared_ptr<m3t::DepthModality>)>
      depth_modality_setter_{[](auto) {}};
  std::function<void(std::shared_ptr<m3t::DepthModel>)> depth_model_setter_{
      [](auto) {}};
  std::function<void(std::shared_ptr<m3t::TextureModality>)>
      texture_modality_setter_{[](auto) {}};

  // Parameters for evaluation
  std::string name_{};
  bool run_sequentially_ = true;
  bool visualize_tracking_ = false;
  bool visualize_frame_results_ = false;
  bool set_up_ = false;
};

#endif  // M3T_EXAMPLES_CHOI_EVALUATOR_H_
