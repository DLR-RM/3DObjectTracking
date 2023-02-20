// SPDX-License-Identifier: MIT
// Copyright (c) 2023 Manuel Stoiber, German Aerospace Center (DLR)

#ifndef M3T_EXAMPLES_OPT_EVALUATOR_H_
#define M3T_EXAMPLES_OPT_EVALUATOR_H_

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
 * \brief Class that holds a tracker to evaluate it on the OPT dataset.
 * \details It includes functionality to calculate mean vertex errors, compute
 * AUC (Area Under Curve) scores, measure execution times, and save results to a
 * file.
 */
class OPTEvaluator {
 private:
  static constexpr size_t kNCurveValues = 100;
  static constexpr float kThresholdMax = 0.2f;
  static constexpr m3t::Intrinsics kOPTIntrinsics{
      1060.197f, 1060.273f, 964.809f, 560.952f, 1920, 1080};
  static constexpr std::array<float, 16> kDepth2Color_Pose{
      0.9999788893f,  -0.0052817802f, 0.0037846718f,  -0.0525133559,
      0.0052971168f,  0.9999777534f,  -0.0040537989f, 0.0006022050f,
      -0.0037631764f, 0.0040737612f,  0.9999846214f,  -0.0003262078f,
      0.0f,           0.0f,           0.0f,           1.0f};
  const std::map<std::string, m3t::Transform3fA> kBody2Geometry2BodyPoseMap{
      {"soda",
       m3t::Transform3fA{Eigen::Translation3f{0.0006f, -0.0004f, -0.0549f}}},
      {"chest",
       m3t::Transform3fA{Eigen::Translation3f{-0.0002f, -0.0009f, -0.0377f}}},
      {"ironman",
       m3t::Transform3fA{Eigen::Translation3f{0.0023f, 0.0005f, -0.0506f}}},
      {"house",
       m3t::Transform3fA{Eigen::Translation3f{-0.0008f, -0.0059f, -0.0271f}}},
      {"bike",
       m3t::Transform3fA{Eigen::Translation3f{-0.0018f, 0.0001f, -0.0267f}}},
      {"jet",
       m3t::Transform3fA{Eigen::Translation3f{-0.0004f, 0.0001f, -0.0117f}}},
  };
  const std::map<std::string, float> kBody2PrecomputedDiametersMap{
      {"soda", 0.121923f}, {"chest", 0.122367f}, {"ironman", 0.101993f},
      {"house", 0.12937f}, {"bike", 0.120494f},  {"jet", 0.112384f}};

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
    std::array<float, kNCurveValues> curve_values;
    float area_under_curve = 0.0f;
    ExecutionTimes execution_times;
  };

 private:
  struct RunConfiguration {
    std::string body_name{};
    std::string body_orientation{};
    std::string motion_pattern{};
    std::string sequence_name{};
  };

  struct SequenceResult {
    std::string body_name{};
    std::string body_orientation{};
    std::string motion_pattern{};
    std::vector<Result> frame_results{};
    Result average_result{};
  };

 public:
  // Constructors and setup method
  OPTEvaluator(const std::string &name,
               const std::filesystem::path &dataset_directory,
               const std::filesystem::path &external_directory,
               const std::vector<std::string> &body_names,
               const std::vector<std::string> &body_orientations,
               const std::vector<std::string> &motion_patterns);
  bool SetUp();

  // Setters for evaluation
  void set_run_sequentially(bool run_sequentially);
  void set_use_random_seed(bool use_random_seed);
  void set_n_vertices_evaluation(int n_vertices_evaluation);
  void set_calculate_diameters(bool calculate_diameters);
  void set_visualize_tracking(bool visualize_tracking);
  void set_visualize_frame_results(bool visualize_frame_results);

  // Setters for tracker configuration
  void set_use_region_modality(bool use_region_modality);
  void set_use_depth_modality(bool use_depth_modality);
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
  float area_under_curve() const;
  float execution_time() const;
  std::map<std::string, Result> final_results() const;

 private:
  // Helper methods to run evaluation
  bool EvaluateRunConfiguration(
      const RunConfiguration &run_configuration,
      const std::shared_ptr<m3t::RendererGeometry> &renderer_geometry_ptr,
      SequenceResult *sequence_result) const;
  bool SetUpTracker(
      const RunConfiguration &run_configuration,
      const std::shared_ptr<m3t::RendererGeometry> &renderer_geometry_ptr,
      std::shared_ptr<m3t::Tracker> *tracker_ptr) const;
  void ExecuteMeasuredTrackingCycle(
      const std::shared_ptr<m3t::Tracker> &tracker_ptr, int iteration,
      ExecutionTimes *execution_times) const;

  // Helper methods for the calculation of results
  Result CalculateAverageBodyResult(
      const std::vector<std::string> &body_names) const;
  static Result CalculateAverageResult(const std::vector<Result> &results);
  static Result SumResults(const std::vector<Result> &results);
  static Result DivideResult(const Result &result, size_t n);
  void CalculatePoseResults(const std::string &body_name,
                            const m3t::Transform3fA &body2world_pose,
                            const m3t::Transform3fA &gt_body2world_pose,
                            Result *result) const;
  static void VisualizeResult(const Result &result, const std::string &title);

  // General helper methods
  void CreateRunConfigurations();
  bool LoadBodies();
  bool GenerateModels();
  void GenderateReducedVertices();
  void CalculateDiameters();
  bool GetGTPosesOPTDataset(
      const RunConfiguration &run_configuration,
      std::vector<m3t::Transform3fA> *gt_body2world_poses) const;
  static float ElapsedTime(
      const std::chrono::high_resolution_clock::time_point &begin_time);

  // Internal data objects
  std::array<float, kNCurveValues> thresholds_;
  std::map<std::string, std::shared_ptr<m3t::Body>> body2body_ptr_map_;
  std::map<std::string, std::shared_ptr<m3t::RegionModel>>
      body2region_model_ptr_map_;
  std::map<std::string, std::shared_ptr<m3t::DepthModel>>
      body2depth_model_ptr_map_;
  std::map<std::string, std::vector<Eigen::Vector3f>>
      body2reduced_vertice_map_{};
  std::map<std::string, float> body2diameter_map_;
  std::vector<RunConfiguration> run_configurations_;
  std::vector<SequenceResult> results_{};
  std::map<std::string, Result> final_results_{};

  // Parameters for OPT dataset
  std::filesystem::path dataset_directory_;
  std::filesystem::path external_directory_;
  std::vector<std::string> body_names_;
  std::vector<std::string> body_orientations_;
  std::vector<std::string> motion_patterns_;

  // Parameters for tracker configuration
  bool use_region_modality_;
  bool use_depth_modality_;
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
  std::function<void(std::shared_ptr<m3t::DepthModality>)>
      depth_modality_setter_{[](auto) {}};
  std::function<void(std::shared_ptr<m3t::DepthModel>)> depth_model_setter_{
      [](auto) {}};
  std::function<void(std::shared_ptr<m3t::TextureModality>)>
      texture_modality_setter_{[](auto) {}};

  // Parameters for evaluation
  std::string name_{};
  bool run_sequentially_ = true;
  bool use_random_seed_ = false;
  int n_vertices_evaluation_ = -1;
  bool calculate_diameters_ = false;
  bool visualize_tracking_ = false;
  bool visualize_frame_results_ = false;
  bool set_up_ = false;
};

#endif  // M3T_EXAMPLES_OPT_EVALUATOR_H_
