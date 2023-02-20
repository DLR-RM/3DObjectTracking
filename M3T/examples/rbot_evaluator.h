// SPDX-License-Identifier: MIT
// Copyright (c) 2023 Manuel Stoiber, German Aerospace Center (DLR)

#ifndef M3T_EXAMPLES_RBOT_EVALUATOR_H_
#define M3T_EXAMPLES_RBOT_EVALUATOR_H_

#include <m3t/basic_depth_renderer.h>
#include <m3t/body.h>
#include <m3t/common.h>
#include <m3t/link.h>
#include <m3t/loader_camera.h>
#include <m3t/modality.h>
#include <m3t/normal_viewer.h>
#include <m3t/optimizer.h>
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
#include <memory>
#include <string>
#include <vector>

/**
 * \brief Class that holds a tracker to evaluate it on the RBOT dataset.
 * \details It includes functionality to reset bodies, calculate rotational and
 * translational errors, compute tracking success, measure execution times, and
 * save results to a file
 */
class RBOTEvaluator {
 private:
  static constexpr int kNFrames_ = 1000;
  static constexpr m3t::Intrinsics kRBOTIntrinsics{
      650.048f, 647.183f, 324.328f - 0.5f, 257.323f - 0.5f, 640, 512};
  const std::string kOcclusionBodyName{"squirrel_small"};

  struct RunConfiguration {
    std::string sequence_name{};
    bool occlusions;
    std::string body_name{};
  };

  struct ExecutionTimes {
    float complete_cycle = 0.0f;
    float calculate_correspondences = 0.0f;
    float calculate_gradient_and_hessian = 0.0f;
    float calculate_optimization = 0.0f;
    float calculate_results = 0.0f;
  };

  struct Result {
    int frame_index = 0;
    float translation_error = 0.0f;
    float rotation_error = 0.0f;
    float tracking_success = 0.0f;
    ExecutionTimes execution_times;
  };

 public:
  // Constructors and setup method
  RBOTEvaluator(const std::string &name,
                const std::filesystem::path &dataset_directory,
                const std::filesystem::path &external_directory,
                const std::vector<std::string> &body_names,
                const std::vector<std::string> &sequence_names,
                const std::vector<bool> &sequence_occlusions);
  bool SetUp();

  // Setters for evaluation
  void set_translation_error_threshold(float translation_error_threshold);
  void set_rotation_error_threshold(float rotation_error_threshold);
  void set_run_sequentially(bool run_sequentially);
  void set_visualize_all_results(bool visualize_all_results);
  void SaveResults(std::filesystem::path save_directory);
  void DoNotSaveResults();

  // Setters for tracker configuration
  void set_use_region_modality(bool use_region_modality);
  void set_use_texture_modality(bool use_texture_modality);

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
  void set_texture_modality_setter(
      const std::function<void(std::shared_ptr<m3t::TextureModality>)>
          &texture_modality_setter);
  void set_depth_renderer_setter(
      const std::function<void(std::shared_ptr<m3t::FocusedBasicDepthRenderer>)>
          &depth_renderer_setter);

  // Main methods
  bool Evaluate();

  // Getters
  float tracking_success() const;
  float execution_time() const;

 private:
  // Helper methods to run evaluation
  void EvaluateRunConfiguration(
      const RunConfiguration &run_configuration,
      const std::shared_ptr<m3t::RendererGeometry> &renderer_geometry_ptr,
      Result *average_result) const;
  void SetUpTracker(
      const RunConfiguration &run_configuration,
      const std::shared_ptr<m3t::RendererGeometry> &renderer_geometry_ptr,
      std::shared_ptr<m3t::Tracker> *tracker_ptr) const;
  void ResetBody(const std::shared_ptr<m3t::Tracker> &tracker_ptr,
                 int i_frame) const;
  void ResetOcclusionBody(const std::shared_ptr<m3t::Tracker> &tracker_ptr,
                          int i_frame) const;
  void ExecuteMeasuredTrackingCycle(
      const std::shared_ptr<m3t::Tracker> &tracker_ptr, int iteration,
      ExecutionTimes *execution_times) const;

  // Helper methods for the calculation of results
  void CalculatePoseResults(const m3t::Transform3fA &body2world_pose,
                            const m3t::Transform3fA &body2world_pose_gt,
                            Result *result) const;
  static void CalculateAverageResult(const std::vector<Result> &results,
                                     Result *average_result);
  static void VisualizeFrameResult(const Result &result,
                                   const std::string &title);
  static void VisualizeFinalResult(const Result &result,
                                   const std::string &title);
  void SaveFinalResult(const Result &result, const std::string &title) const;

  // General helper methods
  bool LoadBodies();
  bool LoadSingleBody(const std::string &body_name,
                      const std::filesystem::path &directory);
  bool GenerateModels();
  bool GenerateSingleModel(const std::string &body_name,
                           const std::filesystem::path &directory);
  static bool ReadPosesRBOTDataset(const std::filesystem::path &path,
                                   std::vector<m3t::Transform3fA> *poses);
  static float ElapsedTime(
      const std::chrono::high_resolution_clock::time_point &begin_time);

  // Internal data objects
  std::map<std::string, std::shared_ptr<m3t::Body>> body2body_ptr_map_;
  std::map<std::string, std::shared_ptr<m3t::RegionModel>> body2model_ptr_map_;
  std::vector<m3t::Transform3fA> poses_gt_first_;
  std::vector<m3t::Transform3fA> poses_gt_second_;
  std::vector<RunConfiguration> run_configurations_;
  std::vector<Result> results_;
  Result final_result_;

  // Parameters for RBOT dataset
  std::filesystem::path dataset_directory_;
  std::filesystem::path external_directory_;
  std::vector<std::string> body_names_;
  std::vector<std::string> sequence_names_;
  std::vector<bool> sequence_occlusions_;

  // Parameters for tracker configuration
  bool use_region_modality_;
  bool use_texture_modality_;

  // Setters for object setters
  std::function<void(std::shared_ptr<m3t::Tracker>)> tracker_setter_{
      [](auto) {}};
  std::function<void(std::shared_ptr<m3t::Optimizer>)> optimizer_setter_{
      [](auto) {}};
  std::function<void(std::shared_ptr<m3t::RegionModality>)>
      region_modality_setter_{[](auto) {}};
  std::function<void(std::shared_ptr<m3t::RegionModel>)> region_model_setter_{
      [](auto) {}};
  std::function<void(std::shared_ptr<m3t::TextureModality>)>
      texture_modality_setter_{[](auto) {}};
  std::function<void(std::shared_ptr<m3t::FocusedBasicDepthRenderer>)>
      depth_renderer_setter_{[](auto) {}};

  // Parameters for evaluator
  std::string name_{};
  float translation_error_threshold_ = 0.05f;
  float rotation_error_threshold_ = 5.0f * m3t::kPi / 180.0f;
  bool run_sequentially_ = true;
  bool visualize_all_results_ = true;
  bool save_results_ = false;
  std::filesystem::path save_directory_;
  bool set_up_ = false;
};

#endif  // M3T_EXAMPLES_RBOT_EVALUATOR_H_
