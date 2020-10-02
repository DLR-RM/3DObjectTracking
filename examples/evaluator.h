// SPDX-License-Identifier: MIT
// Copyright (c) 2020 Manuel Stoiber, German Aerospace Center (DLR)

#ifndef OBJECTTRACKING_EXAMPLES_EVALUATOR_H_
#define OBJECTTRACKING_EXAMPLES_EVALUATOR_H_

#include <rbgt/body.h>
#include <rbgt/common.h>
#include <rbgt/dataset_rbot_camera.h>
#include <rbgt/normal_image_viewer.h>
#include <rbgt/occlusion_mask_renderer.h>
#include <rbgt/region_modality.h>
#include <rbgt/renderer_geometry.h>
#include <rbgt/tracker.h>

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <memory>
#include <string>
#include <vector>

// Class that holds a tracker to evaluate it on the RBOT dataset. It
// includes functionality to reset bodies, calculate rotational and
// translational errors, measure execution times, and save results to a file
class Evaluator {
 private:
  static constexpr int kNFrames_ = 1001;

  using DataExecutionTimes = struct DataExecutionTimes {
    float complete_cycle = 0.0f;
    float calculate_before_camera_update = 0.0f;
    float start_occlusion_mask_rendering = 0.0f;
    float calculate_correspondences = 0.0f;
    float calculate_pose_update = 0.0f;
  };

  using DataResult = struct DataResult {
    int frame_index = 0;
    float translation_error = 0.0f;
    float rotation_error = 0.0f;
    bool tracking_loss = false;
    DataExecutionTimes execution_times;
  };

 public:
  // Constructor and setter methods
  bool Init(const std::filesystem::path &dataset_path,
            const std::vector<std::string> &body_names,
            const std::string &sequence_name, bool use_occlusions);
  void set_translation_error_threshold(float translation_error_threshold);
  void set_rotation_error_threshold(float rotation_error_threshold);
  void set_visualize_all_results(bool visualize_all_results);

  // Setters for model parameters
  void set_sphere_radius(float sphere_radius);
  void set_n_divides(int n_divides);
  void set_n_points(int n_points);

  // Main methods
  bool Evaluate();
  void SaveResults(const std::filesystem::path &path);

  // Getters
  std::shared_ptr<rbgt::Tracker> tracker_ptr() const;
  std::shared_ptr<rbgt::RegionModality> region_modality_ptr() const;
  std::shared_ptr<rbgt::RegionModality> occlusion_region_modality_ptr() const;
  std::shared_ptr<rbgt::OcclusionMaskRenderer> occlusion_mask_renderer_ptr()
      const;
  std::shared_ptr<rbgt::NormalImageViewer> viewer_ptr() const;

 private:
  // Helper Methods for setup
  void InitTracker();
  void InitBodies(const std::string &body_name);
  void ResetBody(int i_frame);
  void ResetOcclusionBody(int i_frame);

  void ExecuteMeasuredTrackingCycle(int iteration,
                                    DataExecutionTimes *execution_times);
  void CalculatePoseResults(const rbgt::Transform3fA &body2world_pose,
                            const rbgt::Transform3fA &body2world_pose_gt,
                            DataResult *result) const;
  static void VisualizeResult(const DataResult &result,
                              const std::string &body_name);
  static void VisualizeAverageResults(
      const std::vector<std::vector<DataResult>> &results,
      const std::string &title);

  // General helper methods
  static bool ReadPosesRBOTDataset(const std::filesystem::path &path,
                                   std::vector<rbgt::Transform3fA> *poses);
  static float ElapsedTime(
      const std::chrono::high_resolution_clock::time_point &begin_time);

  // Internal data objects
  std::vector<rbgt::Transform3fA> poses_first_;
  std::vector<rbgt::Transform3fA> poses_second_;
  std::vector<std::vector<DataResult>> results_;

  // Pointers for main objects
  std::shared_ptr<rbgt::Tracker> tracker_ptr_ = nullptr;
  std::shared_ptr<rbgt::RegionModality> region_modality_ptr_ = nullptr;
  std::shared_ptr<rbgt::RegionModality> occlusion_region_modality_ptr_ =
      nullptr;
  std::shared_ptr<rbgt::OcclusionMaskRenderer> occlusion_mask_renderer_ptr_ =
      nullptr;
  std::shared_ptr<rbgt::NormalImageViewer> viewer_ptr_ = nullptr;

  // Pointers for other objects
  std::shared_ptr<rbgt::Body> body_ptr_ = nullptr;
  std::shared_ptr<rbgt::Body> occlusion_body_ptr_ = nullptr;
  std::shared_ptr<rbgt::Model> model_ptr_ = nullptr;
  std::shared_ptr<rbgt::Model> occlusion_model_ptr_ = nullptr;
  std::shared_ptr<rbgt::RendererGeometry> renderer_geometry_ptr_ = nullptr;
  std::shared_ptr<rbgt::DatasetRBOTCamera> camera_ptr_ = nullptr;

  // Parameters for RBOT dataset
  std::filesystem::path dataset_path_;
  std::vector<std::string> body_names_;
  std::string sequence_name_;
  bool use_occlusions_;

  // Parameters for model
  float sphere_radius_ = 0.8f;
  int n_divides_ = 4;
  int n_points_ = 200;

  // Parameters for evaluation
  float translation_error_threshold_ = 0.05f;
  float rotation_error_threshold_ = 5.0f * rbgt::kPi / 180.0f;
  bool visualize_all_results_ = true;
  bool initialized_ = false;
};

#endif  // OBJECTTRACKING_EXAMPLES_EVALUATOR_H_
