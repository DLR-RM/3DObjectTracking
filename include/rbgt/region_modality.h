// SPDX-License-Identifier: MIT
// Copyright (c) 2020 Manuel Stoiber, German Aerospace Center (DLR)

#ifndef OBJECT_TRACKING_INCLUDE_RBGT_REGION_MODALITY_H_
#define OBJECT_TRACKING_INCLUDE_RBGT_REGION_MODALITY_H_

#include <rbgt/body.h>
#include <rbgt/camera.h>
#include <rbgt/common.h>
#include <rbgt/model.h>
#include <rbgt/occlusion_mask_renderer.h>

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <filesystem>
#include <iostream>
#include <memory>
#include <opencv2/opencv.hpp>
#include <string>
#include <unsupported/Eigen/MatrixFunctions>
#include <vector>

namespace rbgt {

// Class that implements all functionality for a region modality
// including correspondence search, calculation of the gradient vector and
// hessian matrix, pose optimization, calculation of color histograms, and
// visualization. Also the modality is able to consider occlusions using an
// occlusion mask.
class RegionModality {
 private:
  // Data for correspondence line calculated during CalculateCorrespondences
  using DataLine = struct DataLine {
    Eigen::Vector3f center_f_body;
    Eigen::Vector3f center_f_camera;
    float center_u = 0.0f;
    float center_v = 0.0f;
    float normal_u = 0.0f;
    float normal_v = 0.0f;
    float delta_r = 0.0f;
    float normal_component_to_scale = 0.0f;
    float continuous_distance = 0.0f;
    std::vector<float> distribution;
    float mean = 0.0f;
    float standard_deviation = 0.0f;
    float variance = 0.0f;
  };

 public:
  // Constructor and initialization method
  RegionModality();
  bool Init(const std::string &name, std::shared_ptr<Body> body_ptr,
            std::shared_ptr<Model> model_ptr,
            std::shared_ptr<Camera> camera_ptr);

  // Setters for general distribution
  void set_n_points(int n_points);
  void set_function_slope(float function_slope);
  void set_function_length(int function_length);
  void set_distribution_length(int distribution_length);
  void set_scales(const std::vector<int> &scales);
  void set_probability_threshold(float probability_threshold);
  void set_min_continuous_distance(float min_continuous_distance);
  void set_use_linear_function(bool use_linear_function);
  void set_use_const_variance(bool use_const_variance);

  // Setters for histogram calculation
  bool set_n_histogram_bins(int n_histogram_bins);
  void set_learning_rate_f(float learning_rate_f);
  void set_learning_rate_b(float learning_rate_b);
  void set_unconsidered_line_length(float unconsidered_line_length);
  void set_considered_line_length(float considered_line_length);

  // Setters for optimization
  void set_tikhonov_parameter_rotation(float tikhonov_parameter_rotation);
  void set_tikhonov_parameter_translation(float tikhonov_parameter_translation);

  // Setters for occlusion handling
  void UseOcclusionHandling(
      std::shared_ptr<OcclusionMaskRenderer> occlusion_mask_renderer_ptr);
  void DoNotUseOcclusionHandling();

  // Setters for general visualization settings
  void set_display_visualization(bool display_visualization);
  void StartSavingVisualizations(const std::filesystem::path &save_path);
  void StopSavingVisualizations();

  // Setters to turn on individual visualizations
  void set_visualize_lines_correspondence(bool visualize_lines_correspondence);
  void set_visualize_points_occlusion_mask_correspondence(
      bool visualize_points_occlusion_mask_correspondence);
  void set_visualize_points_pose_update(bool visualize_points_pose_update);
  void set_visualize_points_histogram_image_pose_update(
      bool visualize_points_histogram_image_pose_update);
  void set_visualize_points_result(bool visualize_points_result);
  void set_visualize_points_histogram_image_result(
      bool visualize_points_result);

  // Main methods
  bool StartModality();
  bool CalculateBeforeCameraUpdate();
  bool CalculateCorrespondences(int corr_iteration);
  bool VisualizeCorrespondences(int save_idx);
  bool CalculatePoseUpdate();
  bool VisualizePoseUpdate(int save_idx);
  bool VisualizeResults(int save_idx);

  // Getters data
  const std::string &name() const;
  std::shared_ptr<Body> body_ptr() const;
  std::shared_ptr<Model> model_ptr() const;
  std::shared_ptr<Camera> camera_ptr() const;
  std::shared_ptr<OcclusionMaskRenderer> occlusion_mask_renderer_ptr() const;

  // Getters visualization and state
  bool imshow_correspondence() const;
  bool imshow_pose_update() const;
  bool imshow_result() const;
  bool initialized() const;

 private:
  // Helper methods for precalculation of internal data
  void PrecalculateFunctionLookup();
  void PrecalculateDistributionVariables();
  void PrecalculateHistogramBinVariables();
  void SetImshowCorrespondence();
  void SetImshowPoseUpdate();
  void SetImshowResult();

  // Helper methods for precalculation of referenced data and changing data
  void PrecalculateBodyVariables();
  void PrecalculateCameraVariables();
  void PrecalculatePoseVariables();
  void PrecalculateScaleDependentVariables(int corr_iteration);

  // Helper methods for histogram calculation
  void AddLinePixelColorsToTempHistograms();
  void AddPixelColorToHistogram(const cv::Vec3b &pixel_color,
                                std::vector<float> *enlarged_histogram) const;
  bool CalculateHistogram(float learning_rate,
                          const std::vector<float> &temp_histogram,
                          std::vector<float> *histogram);

  // Helper methods for CalculateCorrespondences
  void CalculateBasicLineData(const Model::PointData &data_point,
                              DataLine *data_line) const;
  bool IsLineValid(float u, float v, float continuous_distance) const;
  bool CalculateSegmentProbabilities(
      float center_u, float center_v, float normal_u, float normal_v,
      std::vector<float> *segment_probabilities_f,
      std::vector<float> *segment_probabilities_b,
      float *normal_component_to_scale, float *delta_r) const;
  void MultiplyPixelColorProbability(const cv::Vec3b &pixel_color,
                                     float *probability_f,
                                     float *probability_b) const;
  void CalculateDistribution(const std::vector<float> &segment_probabilities_f,
                             const std::vector<float> &segment_probabilities_b,
                             std::vector<float> *distribution) const;
  void CalculateDistributionMoments(const std::vector<float> &distribution,
                                    float *mean, float *standard_deviation,
                                    float *variance) const;

  // Helper methods for visualization
  void ShowAndSaveImage(const std::string &title, int save_index,
                        const cv::Mat &image) const;
  void VisualizePointsCameraImage(const std::string &title,
                                  int save_index) const;
  void VisualizePointsHistogramImage(const std::string &title,
                                     int save_index) const;
  void VisualizePointsOcclusionMask(const std::string &title,
                                    int save_index) const;
  void VisualizeLines(const std::string &title, int save_index) const;
  void DrawPoints(const cv::Vec3b &color_point, cv::Mat *image) const;
  void DrawLines(const cv::Vec3b &color_line,
                 const cv::Vec3b &color_high_probability, cv::Mat *image) const;
  void DrawProbabilityImage(const cv::Vec3b &color_b,
                            cv::Mat *probability_image) const;
  void UpdateLineCentersWithCurrentPose();

  // Mathematical helper methods
  static float MinAbsValueWithSignOfValue1(float value_1, float abs_value_2);

  // Internal data objects
  std::string name_;
  std::vector<float> temp_histogram_f_;
  std::vector<float> temp_histogram_b_;
  std::vector<float> histogram_f_;
  std::vector<float> histogram_b_;
  std::vector<DataLine> data_lines_;

  // Pointers to referenced objects
  std::shared_ptr<Body> body_ptr_ = nullptr;
  std::shared_ptr<Model> model_ptr_ = nullptr;
  std::shared_ptr<Camera> camera_ptr_ = nullptr;
  std::shared_ptr<OcclusionMaskRenderer> occlusion_mask_renderer_ptr_ = nullptr;

  // Parameters for general distribution
  int n_points_ = 200;
  float function_slope_ = 1.3f;
  int function_length_ = 10;
  int distribution_length_ = 11;
  std::vector<int> scales_ = {5, 2, 1};
  float probability_threshold_ = 0.01f;
  float min_continuous_distance_ = 8.0f;
  bool use_linear_function_ = false;
  bool use_const_variance_ = false;

  // Parameters for histogram calculation
  int n_histogram_bins_ = 32;
  int histogram_bitshift_ = 3;
  float learning_rate_f_ = 0.1f;
  float learning_rate_b_ = 0.2f;
  float unconsidered_line_length_ = 2;
  float considered_line_length_ = 10;

  // Parameters for optimization
  float tikhonov_parameter_rotation_ = 5000.0f;
  float tikhonov_parameter_translation_ = 500000.0f;
  Eigen::Matrix<float, 6, 6> tikhonov_matrix_;

  // Parameters for occlusion handling
  bool use_occlusion_handling_ = false;

  // Parameters for general visualization settings
  bool display_visualization_ = true;
  bool save_visualizations_ = false;
  std::filesystem::path save_path_;

  // Parameters to turn on individual visualizations
  bool visualize_lines_correspondence_ = false;
  bool visualize_points_occlusion_mask_correspondence_ = false;
  bool visualize_points_pose_update_ = false;
  bool visualize_points_histogram_image_pose_update_ = false;
  bool visualize_points_result_ = false;
  bool visualize_points_histogram_image_result_ = false;

  // State variables (internal data)
  bool imshow_correspondence_ = false;
  bool imshow_pose_update_ = false;
  bool imshow_result_ = false;
  bool initialized_ = false;

  // Precalculated variables for smoothed step function lookup (internal data)
  std::vector<float> function_lookup_f_;
  std::vector<float> function_lookup_b_;

  // Precalculated variables for distributions (internal data)
  int line_length_in_segments_{};
  float distribution_length_minus_1_half_{};
  float distribution_length_plus_1_half_{};
  float max_abs_dloglikelihood_ddelta_cs_{};

  // Precalculated variables for histogram calculation (internal data)
  int n_histogram_bins_squared_{};
  int n_histogram_bins_cubed_{};

  // Precalculated variables for body (referenced data)
  uchar encoded_occlusion_mask_id_ = 0;

  // Precalculated variables for camera (referenced data)
  float fu_{};
  float fv_{};
  float ppu_{};
  float ppv_{};
  int image_width_minus_1_{};
  int image_height_minus_1_{};
  int image_width_minus_2_{};
  int image_height_minus_2_{};

  // Precalculated variables for poses (continuously changing)
  Transform3fA body2camera_pose_;
  Eigen::Matrix3f body2camera_rotation_;
  Eigen::Matrix<float, 2, 3> body2camera_rotation_xy_;

  // Precalculate variables depending on scale (continuously changing)
  int scale_{};
  float fscale_{};
  int line_length_{};
  int line_length_minus_1_{};
  float line_length_minus_1_half_{};
  float line_length_half_minus_1_{};
};

}  // namespace rbgt

#endif  // OBJECT_TRACKING_INCLUDE_RBGT_REGION_MODALITY_H_
