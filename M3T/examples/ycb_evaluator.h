// SPDX-License-Identifier: MIT
// Copyright (c) 2023 Manuel Stoiber, German Aerospace Center (DLR)

#ifndef M3T_EXAMPLES_YCB_EVALUATOR_H_
#define M3T_EXAMPLES_YCB_EVALUATOR_H_

#include <m3t/basic_depth_renderer.h>
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
#include <omp.h>

#include <filesystem/filesystem.h>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <array>
#include <fstream>
#include <iostream>
#include <iterator>
#include <memory>
#include <nanoflann/KDTreeVectorOfVectorAdaptor.hpp>
#include <nanoflann/nanoflann.hpp>
#include <numeric>
#include <random>
#include <string>
#include <vector>

/**
 * \brief Class that holds a tracker to evaluate it on the YCB-video dataset.
 * \details It includes functionality to calculate mean vertex errors, compute
 * symmetric and non-symmetric AUC (Area Under Curve) scores, measure execution
 * times, and save results to a file.
 */
class YCBEvaluator {
 private:
  static constexpr size_t kNCurveValues = 100;
  static constexpr float kThresholdMax = 0.1f;
  static constexpr m3t::Intrinsics kYCBIntrinsics{
      1066.778f, 1067.487f, 312.9869f, 241.3109f, 640, 480};

 public:
  struct ExecutionTimes {
    float start_modalities = 0.0f;
    float calculate_correspondences = 0.0f;
    float calculate_correspondences_depth = 0.0f;
    float calculate_correspondences_region = 0.0f;
    float calculate_correspondences_texture = 0.0f;
    float calculate_gradient_and_hessian = 0.0f;
    float calculate_optimization = 0.0f;
    float calculate_results = 0.0f;
    float complete_cycle = 0.0f;
  };

  struct Result {
    int frame_index = 0;
    float add_auc = 0.0f;
    float adds_auc = 0.0f;
    std::array<float, kNCurveValues> add_curve{};
    std::array<float, kNCurveValues> adds_curve{};
    ExecutionTimes execution_times{};
  };

 private:
  typedef KDTreeVectorOfVectorsAdaptor<std::vector<Eigen::Vector3f>, float>
      KDTreeVector3f;

  struct RunConfiguration {
    int sequence_id{};
    std::string sequence_name{};
    std::vector<std::string> evaluated_body_names{};
    std::vector<std::string> tracked_body_names{};
  };

  struct SequenceResult {
    std::string sequence_name{};
    std::string body_name{};
    std::vector<Result> frame_results{};
    Result average_result{};
  };

 public:
  // Constructors and setup method
  YCBEvaluator(const std::string &name,
               const std::filesystem::path &dataset_directory,
               const std::filesystem::path &external_directory,
               const std::vector<int> &sequence_ids,
               const std::vector<std::string> &evaluated_body_names,
               const std::vector<std::string> &multi_region_body_names = {});
  bool SetUp();

  // Setters for evaluation
  void set_detector_folder(const std::string &detector_folder);
  void set_evaluate_refinement(bool evaluate_refinement);
  void set_use_detector_initialization(bool use_detector_initialization);
  void set_use_matlab_gt_poses(bool use_matlab_gt_poses);
  void set_run_sequentially(bool run_sequentially);
  void set_use_random_seed(bool use_random_seed);
  void set_n_vertices_evaluation(int n_vertices_evaluation);
  void set_visualize_tracking(bool visualize_tracking);
  void set_visualize_frame_results(bool visualize_frame_results);
  void StartSavingImages(const std::filesystem::path &save_directory);
  void StopSavingImages();

  // Setters for tracker configuration
  void set_use_multi_region(bool use_multi_region);
  void set_use_region_modality(bool use_region_modality);
  void set_use_depth_modality(bool use_depth_modality);
  void set_use_texture_modality(bool use_texture_modality);
  void set_measure_occlusions_region(bool measure_occlusions_region);
  void set_measure_occlusions_depth(bool measure_occlusions_depth);
  void set_measure_occlusions_texture(bool measure_occlusions_texture);
  void set_model_occlusions_region(bool model_occlusions_region);
  void set_model_occlusions_depth(bool model_occlusions_depth);
  void set_model_occlusions_texture(bool model_occlusions_texture);

  // Setters for object setters
  void set_tracker_setter(
      const std::function<void(std::shared_ptr<m3t::Tracker>)> &tracker_setter);
  void set_refiner_setter(
      const std::function<void(std::shared_ptr<m3t::Refiner>)> &refiner_setter);
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
  float add_auc() const;
  float adds_auc() const;
  float execution_time() const;
  std::map<std::string, Result> final_results() const;

 private:
  // Helper methods to run evaluation
  bool EvaluateRunConfiguration(
      const RunConfiguration &run_configuration,
      const std::shared_ptr<m3t::RendererGeometry> &renderer_geometry_ptr,
      std::vector<SequenceResult> *sequence_results) const;
  bool SetUpTracker(
      const RunConfiguration &run_configuration,
      const std::shared_ptr<m3t::RendererGeometry> &renderer_geometry_ptr,
      std::shared_ptr<m3t::Tracker> *tracker_ptr) const;
  void ResetBodies(const std::shared_ptr<m3t::Tracker> &tracker_ptr,
                   const std::vector<std::string> &body_names,
                   const std::map<std::string, std::vector<m3t::Transform3fA>>
                       &body2world_poses,
                   int idx) const;
  void ExecuteMeasuredRefinementCycle(
      const std::shared_ptr<m3t::Tracker> &tracker_ptr, int load_index,
      int iteration, ExecutionTimes *execution_times) const;
  void ExecuteMeasuredTrackingCycle(
      const std::shared_ptr<m3t::Tracker> &tracker_ptr, int load_index,
      int iteration, ExecutionTimes *execution_times) const;
  void UpdateCameras(const std::shared_ptr<m3t::Tracker> &tracker_ptr,
                     int load_index) const;

  // Helper methods for the calculation of results
  Result CalculateAverageBodyResult(
      const std::vector<std::string> &body_names) const;
  Result CalculateAverageSequenceResult(
      const std::vector<std::string> &sequence_names) const;
  static Result CalculateAverageResult(const std::vector<Result> &results);
  static Result SumResults(const std::vector<Result> &results);
  static Result DivideResult(const Result &result, size_t n);
  void CalculatePoseResults(const std::shared_ptr<m3t::Tracker> tracker_ptr,
                            const std::string &body_name,
                            const m3t::Transform3fA &gt_body2world_pose,
                            Result *result) const;
  bool LoadGTPoses(const std::string &sequence_name,
                   const std::string &body_name,
                   std::vector<m3t::Transform3fA> *gt_body2world_poses) const;
  bool LoadMatlabGTPoses(
      const std::string &sequence_name, const std::string &body_name,
      std::vector<m3t::Transform3fA> *gt_body2world_poses) const;
  bool LoadDetectorPoses(
      const std::string &sequence_name, const std::string &body_name,
      std::vector<m3t::Transform3fA> *detector_body2world_poses,
      std::vector<bool> *body_detected) const;
  static void VisualizeResult(const Result &result, const std::string &title);

  // Set up methods
  bool CreateRunConfigurations();
  void AssembleTrackedBodyNames();
  bool LoadBodies();
  bool GenerateModels();
  bool LoadKeyframes();
  void LoadNFrames();
  void LoadPoseBegin();
  void GenderateReducedVertices();
  void GenerateKDTrees();

  // General helper methods
  bool BodyExistsInSequence(const std::string &sequence_name,
                            const std::string &body_name) const;
  bool SequenceBodyNames(const std::string &sequence_name,
                         std::vector<std::string> *sequence_body_names) const;
  int NFramesInSequence(const std::string &sequence_name) const;
  std::string SequenceIDToName(int sequence_id) const;
  static float ElapsedTime(
      const std::chrono::high_resolution_clock::time_point &begin_time);

  // Internal data objects
  std::array<float, kNCurveValues> thresholds_{};
  std::map<std::string, std::shared_ptr<m3t::Body>> body2body_ptr_map_{};
  std::map<std::string, std::vector<std::shared_ptr<m3t::Body>>>
      body2sub_body_ptrs_map_{};
  std::map<std::string, std::vector<std::shared_ptr<m3t::RegionModel>>>
      body2region_model_ptrs_map_{};
  std::map<std::string, std::shared_ptr<m3t::DepthModel>>
      body2depth_model_ptr_map_{};
  std::map<std::string, std::vector<Eigen::Vector3f>>
      body2reduced_vertice_map_{};
  std::map<std::string, std::unique_ptr<KDTreeVector3f>> body2kdtree_ptr_map_{};
  std::map<std::string, std::vector<int>> sequence2keyframes_map_{};
  std::map<std::string, int> sequence2nframes_map_{};
  std::map<std::string, std::map<std::string, int>> body2sequence2pose_begin_{};
  std::vector<RunConfiguration> run_configurations_{};
  std::vector<SequenceResult> results_{};
  std::map<std::string, Result> final_results_{};

  // Parameters for RBOT dataset
  std::filesystem::path dataset_directory_{};
  std::filesystem::path external_directory_{};
  std::string detector_folder_{};
  std::vector<std::string> evaluated_body_names_{};
  std::vector<std::string> tracked_body_names_{};
  std::vector<std::string> multi_region_body_names_{};
  std::vector<int> sequence_ids_{};

  // Parameters for tracker configuration
  bool use_multi_region_ = false;
  bool use_region_modality_ = true;
  bool use_depth_modality_ = true;
  bool use_texture_modality_ = true;
  bool measure_occlusions_region_ = true;
  bool measure_occlusions_depth_ = true;
  bool measure_occlusions_texture_ = true;
  bool model_occlusions_region_ = false;
  bool model_occlusions_depth_ = false;
  bool model_occlusions_texture_ = false;

  // Setters for object setters
  std::function<void(std::shared_ptr<m3t::Tracker>)> tracker_setter_{
      [](auto) {}};
  std::function<void(std::shared_ptr<m3t::Refiner>)> refiner_setter_{
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
  bool evaluate_refinement_ = false;
  bool use_matlab_gt_poses_ = true;
  bool use_detector_initialization_ = false;
  bool run_sequentially_ = true;
  bool use_random_seed_ = false;
  int n_vertices_evaluation_ = -1;
  bool visualize_tracking_ = false;
  bool visualize_frame_results_ = false;
  bool save_images_ = false;
  std::filesystem::path save_directory_{};
  bool set_up_ = false;
};

#endif  // M3T_EXAMPLES_YCB_EVALUATOR_H_
