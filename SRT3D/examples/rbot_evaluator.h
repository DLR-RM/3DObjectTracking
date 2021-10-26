// SPDX-License-Identifier: MIT
// Copyright (c) 2021 Manuel Stoiber, German Aerospace Center (DLR)

#ifndef OBJECTTRACKING_EXAMPLES_RBOT_EVALUATOR_H_
#define OBJECTTRACKING_EXAMPLES_RBOT_EVALUATOR_H_

#include <srt3d/body.h>
#include <srt3d/common.h>
#include <srt3d/loader_camera.h>
#include <srt3d/normal_viewer.h>
#include <srt3d/occlusion_renderer.h>
#include <srt3d/region_modality.h>
#include <srt3d/renderer_geometry.h>
#include <srt3d/tracker.h>

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <array>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <memory>
#include <string>
#include <vector>

// Class that holds a tracker to evaluate it on the RBOT dataset. It
// includes functionality to reset bodies, calculate rotational and
// translational errors, measure execution times, and save results to a file
class RBOTEvaluator {
 private:
  static constexpr int kNFrames_ = 1000;
  static constexpr srt3d::Intrinsics kRBOTIntrinsics{
      650.048f, 647.183f, 324.328f - 0.5f, 257.323f - 0.5f, 640, 512};

  using RunConfiguration = struct RunConfiguration {
    std::string sequence_name{};
    bool occlusions;
    std::string body_name{};
  };

  using DataExecutionTimes = struct DataExecutionTimes {
    float complete_cycle = 0.0f;
    float calculate_before_camera_update = 0.0f;
    float start_occlusion_rendering = 0.0f;
    float calculate_correspondences = 0.0f;
    float calculate_pose_update = 0.0f;
  };

  using DataResult = struct DataResult {
    int frame_index = 0;
    float translation_error = 0.0f;
    float rotation_error = 0.0f;
    float tracking_success = 0.0f;
    DataExecutionTimes execution_times;
  };

 public:
  // Constructors and setup method
  RBOTEvaluator(const std::string &name,
                const std::filesystem::path &dataset_directory,
                const std::vector<std::string> &body_names,
                const std::vector<std::string> &sequence_names,
                const std::vector<bool> &sequence_occlusions);
  bool SetUp();

  // Setters
  void set_translation_error_threshold(float translation_error_threshold);
  void set_rotation_error_threshold(float rotation_error_threshold);
  void set_visualize_all_results(bool visualize_all_results);
  void SaveResults(std::filesystem::path save_directory);
  void DoNotSaveResults();

  // Setters for object setters
  void set_tracker_setter(
      const std::function<void(std::shared_ptr<srt3d::Tracker>)>
          &tracker_setter);
  void set_region_modality_setter(
      const std::function<void(std::shared_ptr<srt3d::RegionModality>)>
          &region_modality_setter);
  void set_model_setter(
      const std::function<void(std::shared_ptr<srt3d::Model>)> &model_setter);
  void set_occlusion_renderer_setter(
      const std::function<void(std::shared_ptr<srt3d::OcclusionRenderer>)>
          &occlusion_renderer_setter);

  // Main methods
  bool Evaluate();

  // Getters
  float tracking_success();

 private:
  // Helper methods to run evaluation
  void EvaluateRunConfiguration(
      RunConfiguration run_configuration,
      std::shared_ptr<srt3d::RendererGeometry> renderer_geometry_ptr,
      DataResult *average_result);
  void SetUpTracker(
      const RunConfiguration &run_configuration,
      std::shared_ptr<srt3d::RendererGeometry> renderer_geometry_ptr,
      std::shared_ptr<srt3d::Tracker> tracker_ptr);
  void ResetBody(std::shared_ptr<srt3d::Tracker> tracker_ptr, int i_frame);
  void ResetOcclusionBody(std::shared_ptr<srt3d::Tracker> tracker_ptr,
                          int i_frame);
  void ExecuteMeasuredTrackingCycle(std::shared_ptr<srt3d::Tracker> tracker_ptr,
                                    int iteration,
                                    DataExecutionTimes *execution_times);

  // Helper methods for the calculation of results
  void CalculatePoseResults(const srt3d::Transform3fA &body2world_pose,
                            const srt3d::Transform3fA &body2world_pose_gt,
                            DataResult *result) const;
  static void CalculateAverageResult(const std::vector<DataResult> &results,
                                     DataResult *average_result);
  static void VisualizeFrameResult(const DataResult &result,
                                   const std::string &title);
  static void VisualizeFinalResult(const DataResult &result,
                                   const std::string &title);
  void SaveFinalResult(const DataResult &result,
                       const std::string &title) const;

  // General helper methods
  bool GenerateModels();
  bool GenerateSingleModel(const std::string &body_name,
                           const std::filesystem::path &directory);
  static bool ReadPosesRBOTDataset(const std::filesystem::path &path,
                                   std::vector<srt3d::Transform3fA> *poses);
  static float ElapsedTime(
      const std::chrono::high_resolution_clock::time_point &begin_time);

  // Internal data objects
  std::vector<srt3d::Transform3fA> poses_gt_first_;
  std::vector<srt3d::Transform3fA> poses_gt_second_;
  std::vector<RunConfiguration> run_configurations_;
  std::vector<DataResult> results_;
  DataResult final_result_;

  // Parameters for RBOT dataset
  std::filesystem::path dataset_directory_;
  std::vector<std::string> body_names_;
  std::vector<std::string> sequence_names_;
  std::vector<bool> sequence_occlusions_;

  // Setters for object setters
  std::function<void(std::shared_ptr<srt3d::Tracker>)> tracker_setter_{
      [](auto) {}};
  std::function<void(std::shared_ptr<srt3d::RegionModality>)>
      region_modality_setter_{[](auto) {}};
  std::function<void(std::shared_ptr<srt3d::Model>)> model_setter_{[](auto) {}};
  std::function<void(std::shared_ptr<srt3d::OcclusionRenderer>)>
      occlusion_renderer_setter_{[](auto) {}};

  // Parameters for evaluator
  std::string name_{};
  float translation_error_threshold_ = 0.05f;
  float rotation_error_threshold_ = 5.0f * srt3d::kPi / 180.0f;
  bool visualize_all_results_ = true;
  bool save_results_ = false;
  std::filesystem::path save_directory_;
  bool set_up_ = false;
};

#endif  // OBJECTTRACKING_EXAMPLES_RBOT_EVALUATOR_H_
