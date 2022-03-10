// SPDX-License-Identifier: MIT
// Copyright (c) 2022 Manuel Stoiber, German Aerospace Center (DLR)

#ifndef ICG_EXAMPLES_RBOT_EVALUATOR_H_
#define ICG_EXAMPLES_RBOT_EVALUATOR_H_

#include <icg/basic_depth_renderer.h>
#include <icg/body.h>
#include <icg/common.h>
#include <icg/loader_camera.h>
#include <icg/modality.h>
#include <icg/normal_viewer.h>
#include <icg/optimizer.h>
#include <icg/region_modality.h>
#include <icg/region_model.h>
#include <icg/renderer_geometry.h>
#include <icg/tracker.h>

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
  static constexpr icg::Intrinsics kRBOTIntrinsics{
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
  void set_visualize_all_results(bool visualize_all_results);
  void SaveResults(std::filesystem::path save_directory);
  void DoNotSaveResults();

  // Setters for object setters
  void set_tracker_setter(
      const std::function<void(std::shared_ptr<icg::Tracker>)> &tracker_setter);
  void set_optimizer_setter(
      const std::function<void(std::shared_ptr<icg::Optimizer>)>
          &optimizer_setter);
  void set_region_modality_setter(
      const std::function<void(std::shared_ptr<icg::RegionModality>)>
          &region_modality_setter);
  void set_region_model_setter(
      const std::function<void(std::shared_ptr<icg::RegionModel>)>
          &region_model_setter);
  void set_depth_renderer_setter(
      const std::function<void(std::shared_ptr<icg::FocusedBasicDepthRenderer>)>
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
      const std::shared_ptr<icg::RendererGeometry> &renderer_geometry_ptr,
      Result *average_result) const;
  void SetUpTracker(
      const RunConfiguration &run_configuration,
      const std::shared_ptr<icg::RendererGeometry> &renderer_geometry_ptr,
      std::shared_ptr<icg::Tracker> *tracker_ptr) const;
  void ResetBody(const std::shared_ptr<icg::Tracker> &tracker_ptr,
                 int i_frame) const;
  void ResetOcclusionBody(const std::shared_ptr<icg::Tracker> &tracker_ptr,
                          int i_frame) const;
  void ExecuteMeasuredTrackingCycle(
      const std::shared_ptr<icg::Tracker> &tracker_ptr, int iteration,
      ExecutionTimes *execution_times) const;

  // Helper methods for the calculation of results
  void CalculatePoseResults(const icg::Transform3fA &body2world_pose,
                            const icg::Transform3fA &body2world_pose_gt,
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
                                   std::vector<icg::Transform3fA> *poses);
  static float ElapsedTime(
      const std::chrono::high_resolution_clock::time_point &begin_time);

  // Internal data objects
  std::map<std::string, std::shared_ptr<icg::Body>> body2body_ptr_map_;
  std::map<std::string, std::shared_ptr<icg::RegionModel>> body2model_ptr_map_;
  std::vector<icg::Transform3fA> poses_gt_first_;
  std::vector<icg::Transform3fA> poses_gt_second_;
  std::vector<RunConfiguration> run_configurations_;
  std::vector<Result> results_;
  Result final_result_;

  // Parameters for RBOT dataset
  std::filesystem::path dataset_directory_;
  std::filesystem::path external_directory_;
  std::vector<std::string> body_names_;
  std::vector<std::string> sequence_names_;
  std::vector<bool> sequence_occlusions_;

  // Setters for object setters
  std::function<void(std::shared_ptr<icg::Tracker>)> tracker_setter_{
      [](auto) {}};
  std::function<void(std::shared_ptr<icg::Optimizer>)> optimizer_setter_{
      [](auto) {}};
  std::function<void(std::shared_ptr<icg::RegionModality>)>
      region_modality_setter_{[](auto) {}};
  std::function<void(std::shared_ptr<icg::RegionModel>)> region_model_setter_{
      [](auto) {}};
  std::function<void(std::shared_ptr<icg::FocusedBasicDepthRenderer>)>
      depth_renderer_setter_{[](auto) {}};

  // Parameters for evaluator
  std::string name_{};
  float translation_error_threshold_ = 0.05f;
  float rotation_error_threshold_ = 5.0f * icg::kPi / 180.0f;
  bool visualize_all_results_ = true;
  bool save_results_ = false;
  std::filesystem::path save_directory_;
  bool set_up_ = false;
};

#endif  // ICG_EXAMPLES_RBOT_EVALUATOR_H_
