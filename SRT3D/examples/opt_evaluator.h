// SPDX-License-Identifier: MIT
// Copyright (c) 2021 Manuel Stoiber, German Aerospace Center (DLR)

#ifndef OBJECTTRACKING_EXAMPLES_OPT_EVALUATOR_H_
#define OBJECTTRACKING_EXAMPLES_OPT_EVALUATOR_H_

#include <srt3d/body.h>
#include <srt3d/common.h>
#include <srt3d/loader_camera.h>
#include <srt3d/normal_viewer.h>
#include <srt3d/region_modality.h>
#include <srt3d/renderer_geometry.h>
#include <srt3d/tracker.h>

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <array>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <iterator>
#include <memory>
#include <numeric>
#include <random>
#include <string>
#include <vector>

// Class that holds a tracker to evaluate it on the OPT dataset. It includes
// functionality to calculate the AUC score, measure execution times, and save
// results to a file
class OPTEvaluator {
 private:
  static constexpr size_t kNCurveValues = 100;
  static constexpr float kThresholdMax = 0.2f;
  static constexpr srt3d::Intrinsics kOPTIntrinsics{
      1060.197f, 1060.273f, 964.809f, 560.952f, 1920, 1080};
  const std::map<std::string, srt3d::Transform3fA> kBody2Geometry2BodyPoseMap{
      {"soda",
       srt3d::Transform3fA{Eigen::Translation3f{0.0006f, -0.0004f, -0.0549f}}},
      {"chest",
       srt3d::Transform3fA{Eigen::Translation3f{-0.0002f, -0.0009f, -0.0377f}}},
      {"ironman",
       srt3d::Transform3fA{Eigen::Translation3f{0.0023f, 0.0005f, -0.0506f}}},
      {"house",
       srt3d::Transform3fA{Eigen::Translation3f{-0.0008f, -0.0059f, -0.0271f}}},
      {"bike",
       srt3d::Transform3fA{Eigen::Translation3f{-0.0018f, 0.0001f, -0.0267f}}},
      {"jet",
       srt3d::Transform3fA{Eigen::Translation3f{-0.0004f, 0.0001f, -0.0117f}}},
  };
  const std::map<std::string, float> kBody2PrecomputedDiametersMap{
      {"soda", 0.121923f}, {"chest", 0.122367f}, {"ironman", 0.101993f},
      {"house", 0.12937f}, {"bike", 0.120494f},  {"jet", 0.112384f}};

  using RunConfiguration = struct RunConfiguration {
    std::string body_name{};
    std::string body_orientation{};
    std::string motion_patterns{};
    std::string sequence_name{};
  };

  using DataExecutionTimes = struct DataExecutionTimes {
    float complete_cycle = 0.0f;
    float calculate_before_camera_update = 0.0f;
    float calculate_correspondences = 0.0f;
    float calculate_pose_update = 0.0f;
  };

  using DataResult = struct DataResult {
    int frame_index = 0;
    std::array<float, kNCurveValues> curve_values;
    float area_under_curve = 0.0f;
    DataExecutionTimes execution_times;
  };

 public:
  // Constructors and setup method
  OPTEvaluator(const std::string &name,
               const std::filesystem::path &dataset_directory,
               const std::vector<std::string> &body_names,
               const std::vector<std::string> &body_orientations,
               const std::vector<std::string> &motion_patterns);
  bool SetUp();

  // Setters
  void set_visualize_tracking(bool visualize_tracking);
  void set_visualize_frame_results(bool visualize_frame_results);
  void set_calculate_diameters(bool calculate_diameters);
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

  // Main methods
  bool Evaluate();

  // Getters
  float area_under_curve();

 private:
  // Helper methods to run evaluation
  void EvaluateRunConfiguration(
      RunConfiguration run_configuration,
      std::shared_ptr<srt3d::RendererGeometry> renderer_geometry_ptr,
      std::vector<DataResult> *results);
  void SetUpTracker(
      const RunConfiguration &run_configuration,
      std::shared_ptr<srt3d::RendererGeometry> renderer_geometry_ptr,
      std::shared_ptr<srt3d::Tracker> tracker_ptr);
  void ExecuteMeasuredTrackingCycle(std::shared_ptr<srt3d::Tracker> tracker_ptr,
                                    int iteration,
                                    DataExecutionTimes *execution_times);

  // Helper methods for the calculation of results
  void CalculatePoseResults(const std::string &body_name,
                            const srt3d::Transform3fA &body2world_pose,
                            const srt3d::Transform3fA &gt_body2world_pose,
                            DataResult *result) const;
  static void CalculateAverageResult(
      const std::vector<std::vector<DataResult>> &results,
      DataResult *average_result);
  static void VisualizeResult(const DataResult &result,
                              const std::string &title);
  void SaveResults(const std::vector<DataResult> &results,
                   const std::string &title) const;

  // General helper methods
  bool GenerateModels();
  bool LoadVertices();
  void CalculateDiameters();
  bool GetGTPosesOPTDataset(
      const RunConfiguration &run_configuration,
      std::vector<srt3d::Transform3fA> *gt_body2world_poses) const;
  static float ElapsedTime(
      const std::chrono::high_resolution_clock::time_point &begin_time);

  // Internal data objects
  std::array<float, kNCurveValues> thresholds_;
  std::map<std::string, float> body2diameter_map_;
  std::map<std::string, std::vector<Eigen::Vector3f>> body2vertice_map_;
  std::vector<RunConfiguration> run_configurations_;
  std::vector<std::vector<DataResult>> results_;
  DataResult final_result_;

  // Parameters for RBOT dataset
  std::filesystem::path dataset_directory_;
  std::vector<std::string> body_names_;
  std::vector<std::string> body_orientations_;
  std::vector<std::string> motion_patterns_;

  // Setters for object setters
  std::function<void(std::shared_ptr<srt3d::Tracker>)> tracker_setter_{
      [](auto) {}};
  std::function<void(std::shared_ptr<srt3d::RegionModality>)>
      region_modality_setter_{[](auto) {}};
  std::function<void(std::shared_ptr<srt3d::Model>)> model_setter_{[](auto) {}};

  // Parameters for evaluation
  std::string name_{};
  bool visualize_tracking_ = false;
  bool visualize_frame_results_ = false;
  bool save_results_ = false;
  bool calculate_diameters_ = false;
  std::filesystem::path save_directory_;
  bool set_up_ = false;
};

#endif  // OBJECTTRACKING_EXAMPLES_OPT_EVALUATOR_H_
