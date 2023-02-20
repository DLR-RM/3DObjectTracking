// SPDX-License-Identifier: MIT
// Copyright (c) 2023 Manuel Stoiber, German Aerospace Center (DLR)

#ifndef M3T_INCLUDE_M3T_REGION_MODALITY_H_
#define M3T_INCLUDE_M3T_REGION_MODALITY_H_

#include <filesystem/filesystem.h>
#include <m3t/body.h>
#include <m3t/camera.h>
#include <m3t/color_histograms.h>
#include <m3t/common.h>
#include <m3t/modality.h>
#include <m3t/region_model.h>
#include <m3t/renderer.h>
#include <m3t/silhouette_renderer.h>

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <iostream>
#include <memory>
#include <opencv2/opencv.hpp>
#include <string>
#include <vector>

namespace m3t {

/**
 * \brief \ref Modality class that implements a region-based modality, which
 * uses information from a \ref ColorCamera and \ref RegionModel to calculate
 * the gradient vector and Hessian matrix that are used by a \ref Link and \ref
 * Optimizer to update the \ref Body pose.
 *
 * \details The modality is able to measure occlusions using images from a
 * referenced \ref DepthCamera that is close to the referenced \ref ColorCamera
 * and model occlusions using renderings from a \ref FocusedDepthRenderer
 * object. Also, it is able to validate contour points using a \ref
 * FocusedSilhouetteRenderer, where it checks if regions from both sides have
 * correct properties along the line. In addition, it is able to use shared \ref
 * ColorHistograms.
 *
 * @param color_camera_ptr referenced \ref ColorCamera from which images are
 * taken.
 * @param region_model_ptr referenced \ref RegionModel that provides views from
 * a *Sparse Viewpoint Model*.
 * @param depth_camera_ptr referenced \ref DepthCamera that is used to for
 * measured occlusion handling.
 * @param depth_renderer_ptr referenced \ref FocusedDepthRenderer that is used
 * for modeled occlusion handling.
 * @param silhouette_renderer_ptr referenced \ref FocusedSilhouetteRenderer that
 * is used for region checking. It has to be configured with the IDType REGION.
 * @param color_histograms_ptr referenced \ref ColorHistograms that can be
 * shared with other modalities.
 * @param n_lines_max number of correspondence lines
 * @param use_adaptive_coverage flag that specifies if the number of lines is
 * dynamically changed depending on the contour length.
 * @param reference_contour_length reference value that is considered if
 * `use_adaptive_coverage = true`. The number of lines is adapted depending on
 * the ratio between the current contour length and this value. If the contour
 * length is bigger or equal to this value the number of lines is set to
 * `n_lines_max`. If `reference_contour_length <= 0.0` the `max_contour_length`
 * value from the \ref RegionModel is used.
 * @param min_continuous_distance minimum distance in segments for which the
 * foreground is not interrupted by the background and vice versa.
 * @param function_length total length in segments and number of discrete values
 * considered by smoothed step functions.
 * @param distribution_length total length in segments and number of discrete
 * values considered for the probability density function.
 * @param function_amplitude amplitude parameter that models global uncertainty
 * in smoothed step functions.
 * @param function_slope slope parameter that models local uncertainty in
 * smoothed step functions.
 * @param learning_rate factor that specifies how far the optimization proceeds
 * in the direction of a gradient calculated from finite differences.
 * @param n_global_iterations number of global optimization iterations conducted
 * before switching to local optimization.
 * @param scales scale for each iteration. If fewer scales than
 * `n_corr_iterations` are given, the last provided scale is used.
 * @param standard_deviations user-defined standard deviation for each
 * iteration in pixels. If fewer values than `n_corr_iterations` are given, the
 * last provided standard deviation is used.
 * @param n_histogram_bins number of bins that is used to discretize each
 * dimension of the RGB color space. Has to be 2, 4, 8, 16, 32, or 64. Can only
 * be set if no shared \ref ColorHistograms are used.
 * @param learning_rate_f learning rate that is used in the update of the
 * foreground histogram. Can only be set if no shared \ref ColorHistograms are
 * used.
 * @param learning_rate_b learning rate that is used in the update of the
 * background histogram. Can only be set if no shared \ref ColorHistograms are
 * used.
 * @param unconsidered_line_length distance along the line from the center that
 * is not considered in the update of color histograms.
 * @param max_considered_line_length maximum length along the line that is
 * considered in the update of color_histograms.
 * @param measured_depth_offset_radius radius in meter that specifies the depth
 * offset provided by the \ref RegionModel and that is used to measure
 * occlusions.
 * @param measured_occlusion_radius radius in meter that defines the area in
 * which depth measurements from a \ref DepthCamera are considered for occlusion
 * handling.
 * @param measured_occlusion_threshold defines how much smaller the minimum
 * value from the \ref DepthCamera is allowed to be compared to the value that
 * is expected from the current estimate.
 * @param modeled_depth_offset_radius radius in meter that specifies the depth
 * offset provided by the \ref RegionModel and that is used to model occlusions.
 * @param modeled_occlusion_radius radius in meter that defines the area in
 * which depth measurements from a \ref FocusedDepthRenderer are considered for
 * occlusion handling.
 * @param modeled_occlusion_threshold defines how much smaller the minimum value
 * from the \ref FocusedDepthRenderer is allowed to be compared to the value
 * that is expected from the current estimate.
 * @param n_unoccluded_iterations number of iterations, after `StartModality()`
 * is called, in which occlusion handling is turned off.
 * @param min_n_unoccluded_lines minimum number of lines that have to be valid.
 * Otherwise, occlusion handling is turned off.
 * @param visualize_lines_correspondence visualize correspondence lines during
 * `VisualizeCorrespondences()`.
 * @param visualize_points_correspondence visualize correspondence line centers
 * during `VisualizeCorrespondences()`.
 * @param visualize_points_depth_image_correspondence visualize correspondence
 * line centers on the depth image, which is used for occlusion handling, during
 * `VisualizeCorrespondences()`.
 * @param visualize_points_depth_rendering_correspondence  visualize
 * correspondence line centers on the depth rendering, which is used for
 * occlusion handling, during `VisualizeCorrespondences()`.
 * @param visualize_points_silhouette_rendering_correspondence visualize
 * correspondence line centers on the silhouette rendering, which is used for
 * dynamic region checking, during `VisualizeCorrespondences()`.
 * @param visualize_points_optimization visualize correspondence line centers
 * during `VisualizeOptimization()`.
 * @param visualize_points_histogram_image_optimization visualize correspondence
 * line centers on a histogram image, which shows pixel-wise posteriors for each
 * pixel, during `VisualizeOptimization()`.
 * @param visualize_points_result visualize correspondence line centers during
 * `VisualizeResults()`.
 * @param visualize_points_histogram_image_result visualize correspondence line
 * centers on histograms images, which show pixel-wise posteriors for each
 * pixel, during `VisualizeResults()`.
 * @param visualization_min_depth minimum depth for normalization of depth
 * images in visualization.
 * @param visualization_max_depth maximum depth for normalization of depth
 * images in visualization.
 */
class RegionModality : public Modality {
 private:
  static constexpr int kMaxNOcclusionStrides = 5;
  static constexpr int kNRegionStride = 5;
  static constexpr float kRegionOffset = 2.0f;

  // Data for correspondence line calculated during CalculateCorrespondences
  struct DataLine {
    Eigen::Vector3f center_f_body{};
    Eigen::Vector3f center_f_camera{};
    float center_u = 0.0f;
    float center_v = 0.0f;
    float normal_u = 0.0f;
    float normal_v = 0.0f;
    float measured_depth_offset = 0.0f;
    float modeled_depth_offset = 0.0f;
    float continuous_distance = 0.0f;
    float delta_r = 0.0f;
    float normal_component_to_scale = 0.0f;
    std::vector<float> distribution{};
    float mean = 0.0f;
    float measured_variance = 0.0f;
  };

 public:
  // Constructors and setup methods
  RegionModality(const std::string &name, const std::shared_ptr<Body> &body_ptr,
                 const std::shared_ptr<ColorCamera> &color_camera_ptr,
                 const std::shared_ptr<RegionModel> &region_model_ptr);
  RegionModality(const std::string &name,
                 const std::filesystem::path &metafile_path,
                 const std::shared_ptr<Body> &body_ptr,
                 const std::shared_ptr<ColorCamera> &color_camera_ptr,
                 const std::shared_ptr<RegionModel> &region_model_ptr);
  bool SetUp() override;

  // Setters referenced data
  void set_color_camera_ptr(
      const std::shared_ptr<ColorCamera> &color_camera_ptr);
  void set_region_model_ptr(
      const std::shared_ptr<RegionModel> &region_model_ptr);

  // Setters for general distribution
  void set_n_lines_max(int n_lines_max);
  void set_use_adaptive_coverage(bool use_adaptive_coverage);
  void set_reference_contour_length(float reference_contour_length);
  void set_min_continuous_distance(float min_continuous_distance);
  void set_function_length(int function_length);
  void set_distribution_length(int distribution_length);
  void set_function_amplitude(float function_amplitude);
  void set_function_slope(float function_slope);
  void set_learning_rate(float learning_rate);
  void set_n_global_iterations(int n_global_iterations);
  void set_scales(const std::vector<int> &scales);
  void set_standard_deviations(const std::vector<float> &standard_deviations);

  // Setters for histogram calculation
  void UseSharedColorHistograms(
      const std::shared_ptr<ColorHistograms> &color_histograms_ptr);
  void DoNotUseSharedColorHistograms();
  bool set_n_histogram_bins(int n_histogram_bins);
  bool set_learning_rate_f(float learning_rate_f);
  bool set_learning_rate_b(float learning_rate_b);
  void set_unconsidered_line_length(float unconsidered_line_length);
  void set_max_considered_line_length(float max_considered_line_length);

  // Setters for occlusion handling and line validation
  void UseRegionChecking(const std::shared_ptr<FocusedSilhouetteRenderer>
                             &silhouette_renderer_ptr);
  void DoNotUseRegionChecking();
  void MeasureOcclusions(const std::shared_ptr<DepthCamera> &depth_camera_ptr);
  void DoNotMeasureOcclusions();
  void ModelOcclusions(
      const std::shared_ptr<FocusedDepthRenderer> &depth_renderer_ptr);
  void DoNotModelOcclusions();
  void set_measured_depth_offset_radius(float measured_depth_offset_radius);
  void set_measured_occlusion_radius(float measured_occlusion_radius);
  void set_measured_occlusion_threshold(float measured_occlusion_threshold);
  void set_modeled_depth_offset_radius(float modeled_depth_offset_radius);
  void set_modeled_occlusion_radius(float modeled_occlusion_radius);
  void set_modeled_occlusion_threshold(float modeled_occlusion_threshold);
  void set_n_unoccluded_iterations(int n_unoccluded_iterations);
  void set_min_n_unoccluded_lines(int min_n_unoccluded_lines);

  // Setters to turn on individual visualizations
  void set_visualize_lines_correspondence(bool visualize_lines_correspondence);
  void set_visualize_points_correspondence(
      bool visualize_points_correspondence);
  void set_visualize_points_depth_image_correspondence(
      bool visualize_points_depth_image_correspondence);
  void set_visualize_points_depth_rendering_correspondence(
      bool visualize_points_depth_rendering_correspondence);
  void set_visualize_points_silhouette_rendering_correspondence(
      bool visualize_points_silhouette_rendering_correspondence);
  void set_visualize_points_optimization(bool visualize_points_optimization);
  void set_visualize_points_histogram_image_optimization(
      bool visualize_points_histogram_image_optimization);
  void set_visualize_points_result(bool visualize_points_result);
  void set_visualize_points_histogram_image_result(
      bool visualize_points_histogram_image_result);

  // Setters for visualization parameters
  void set_visualization_min_depth(float visualization_min_depth);
  void set_visualization_max_depth(float visualization_max_depth);

  // Main methods
  bool StartModality(int iteration, int corr_iteration) override;
  bool CalculateCorrespondences(int iteration, int corr_iteration) override;
  bool VisualizeCorrespondences(int save_idx) override;
  bool CalculateGradientAndHessian(int iteration, int corr_iteration,
                                   int opt_iteration) override;
  bool VisualizeOptimization(int save_idx) override;
  bool CalculateResults(int iteration) override;
  bool VisualizeResults(int save_idx) override;

  // Getters data
  const std::shared_ptr<ColorCamera> &color_camera_ptr() const;
  const std::shared_ptr<DepthCamera> &depth_camera_ptr() const;
  const std::shared_ptr<RegionModel> &region_model_ptr() const;
  const std::shared_ptr<FocusedDepthRenderer> &depth_renderer_ptr() const;
  const std::shared_ptr<FocusedSilhouetteRenderer> &silhouette_renderer_ptr()
      const;
  std::shared_ptr<Model> model_ptr() const override;
  std::vector<std::shared_ptr<Camera>> camera_ptrs() const override;
  std::vector<std::shared_ptr<Renderer>> start_modality_renderer_ptrs()
      const override;
  std::vector<std::shared_ptr<Renderer>> correspondence_renderer_ptrs()
      const override;
  std::vector<std::shared_ptr<Renderer>> results_renderer_ptrs() const override;
  std::shared_ptr<ColorHistograms> color_histograms_ptr() const override;

  // Getters for general distribution
  int n_lines_max() const;
  bool use_adaptive_coverage() const;
  float reference_contour_length() const;
  float min_continuous_distance() const;
  int function_length() const;
  int distribution_length() const;
  float function_amplitude() const;
  float function_slope() const;
  float learning_rate() const;
  int n_global_iterations() const;
  const std::vector<int> &scales() const;
  const std::vector<float> &standard_deviations() const;

  // Getters for histogram calculation
  bool use_shared_color_histograms() const;
  int n_histogram_bins() const;
  float learning_rate_f() const;
  float learning_rate_b() const;
  float unconsidered_line_length() const;
  float max_considered_line_length() const;

  // Getters for occlusion handling and line validation
  bool use_region_checking() const;
  bool measure_occlusions() const;
  float measured_depth_offset_radius() const;
  float measured_occlusion_radius() const;
  float measured_occlusion_threshold() const;
  bool model_occlusions() const;
  float modeled_depth_offset_radius() const;
  float modeled_occlusion_radius() const;
  float modeled_occlusion_threshold() const;
  int n_unoccluded_iterations() const;
  int min_n_unoccluded_lines() const;

  // Getters to turn on individual visualizations
  bool visualize_lines_correspondence() const;
  bool visualize_points_correspondence() const;
  bool visualize_points_depth_image_correspondence() const;
  bool visualize_points_depth_rendering_correspondence() const;
  bool visualize_points_silhouette_rendering_correspondence() const;
  bool visualize_points_optimization() const;
  bool visualize_points_histogram_image_optimization() const;
  bool visualize_points_result() const;
  bool visualize_points_histogram_image_result() const;

  // Getters for visualization parameters
  float visualization_min_depth() const;
  float visualization_max_depth() const;

 private:
  // Helper method for setup
  bool LoadMetaData();

  // Helper methods for precalculation of internal data
  void PrecalculateFunctionLookup();
  void PrecalculateDistributionVariables();
  bool SetUpInternalColorHistograms();
  void SetImshowVariables();

  // Helper methods for precalculation of referenced data and changing data
  void PrecalculateCameraVariables();
  bool PrecalculateModelVariables();
  void PrecalculateRendererVariables();
  void PrecalculatePoseVariables();
  void PrecalculateIterationDependentVariables(int corr_iteration);

  // Helper methods for histogram calculation
  void AddLinePixelColorsToTempHistograms(bool handle_occlusions);
  void DynamicRegionDistance(float center_u, float center_v, float normal_u,
                             float normal_v, float *dynamic_foreground_distance,
                             float *dynamic_background_distance) const;

  // Helper methods for CalculateCorrespondences
  void CalculateBasicLineData(const RegionModel::DataPoint &data_point,
                              DataLine *data_line) const;
  bool IsLineValid(const DataLine &data_line, bool use_region_checking,
                   bool measure_occlusions, bool model_occlusions) const;
  bool IsDynamicLineRegionSufficient(float center_u, float center_v,
                                     float normal_u, float normal_v) const;
  bool IsLineUnoccludedMeasured(const Eigen::Vector3f &center_f_body,
                                float depth_offset) const;
  bool IsLineUnoccludedModeled(float center_u, float center_v, float depth,
                               float depth_offset) const;
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
                                    float *mean, float *variance) const;

  // Helper methods for visualization
  void ShowAndSaveImage(const std::string &title, int save_index,
                        const cv::Mat &image) const;
  void VisualizePointsColorImage(const std::string &title,
                                 int save_index) const;
  void VisualizePointsDepthImage(const std::string &title,
                                 int save_index) const;
  void VisualizePointsDepthRendering(const std::string &title,
                                     int save_index) const;
  void VisualizePointsSilhouetteRendering(const std::string &title,
                                          int save_index) const;
  void VisualizePointsHistogramImage(const std::string &title,
                                     int save_index) const;
  void VisualizeLines(const std::string &title, int save_index) const;
  void DrawPoints(const cv::Vec3b &color_point, cv::Mat *image) const;
  void DrawDepthPoints(const cv::Vec3b &color_point, cv::Mat *image) const;
  void DrawFocusedPoints(const std::shared_ptr<FocusedRenderer> &renderer_ptr,
                         const cv::Vec3b &color_point, cv::Mat *image) const;
  void DrawLines(const cv::Vec3b &color_line,
                 const cv::Vec3b &color_high_probability, cv::Mat *image) const;
  void DrawProbabilityImage(const cv::Vec3b &color_b,
                            cv::Mat *probability_image) const;

  // Other helper methods
  static float MinAbsValueWithSignOfValue1(float value_1, float abs_value_2);
  bool IsSetup() const;

  // Internal data objects
  std::vector<DataLine> data_lines_;

  // Pointers to referenced objects
  std::shared_ptr<ColorCamera> color_camera_ptr_ = nullptr;
  std::shared_ptr<DepthCamera> depth_camera_ptr_ = nullptr;
  std::shared_ptr<RegionModel> region_model_ptr_ = nullptr;
  std::shared_ptr<FocusedDepthRenderer> depth_renderer_ptr_ = nullptr;
  std::shared_ptr<ColorHistograms> color_histograms_ptr_ = nullptr;
  std::shared_ptr<FocusedSilhouetteRenderer> silhouette_renderer_ptr_ = nullptr;

  // Parameters for general distribution
  int n_lines_max_ = 200;
  bool use_adaptive_coverage_ = false;
  float reference_contour_length_ = 0.0f;
  float min_continuous_distance_ = 3.0f;
  int function_length_ = 8;
  int distribution_length_ = 12;
  float function_amplitude_ = 0.43f;
  float function_slope_ = 0.5f;
  float learning_rate_ = 1.3f;
  int n_global_iterations_ = 1;
  std::vector<int> scales_{6, 4, 2, 1};
  std::vector<float> standard_deviations_{15.0f, 5.0f, 3.5f, 1.5f};

  // Parameters for histogram calculation
  bool use_shared_color_histograms_ = false;
  int n_histogram_bins_ = 16;
  float learning_rate_f_ = 0.2f;
  float learning_rate_b_ = 0.2f;
  float unconsidered_line_length_ = 0.5f;
  float max_considered_line_length_ = 20.0f;

  // Parameters for occlusion handling and line validation
  bool use_region_checking_ = false;
  bool measure_occlusions_ = false;
  float measured_depth_offset_radius_ = 0.01f;
  float measured_occlusion_radius_ = 0.01f;
  float measured_occlusion_threshold_ = 0.03f;
  bool model_occlusions_ = false;
  float modeled_depth_offset_radius_ = 0.01f;
  float modeled_occlusion_radius_ = 0.01f;
  float modeled_occlusion_threshold_ = 0.03f;
  int n_unoccluded_iterations_ = 10;
  int min_n_unoccluded_lines_ = 0;

  // Parameters to turn on individual visualizations
  bool visualize_lines_correspondence_ = false;
  bool visualize_points_correspondence_ = false;
  bool visualize_points_depth_image_correspondence_ = false;
  bool visualize_points_depth_rendering_correspondence_ = false;
  bool visualize_points_silhouette_rendering_correspondence_ = false;
  bool visualize_points_optimization_ = false;
  bool visualize_points_histogram_image_optimization_ = false;
  bool visualize_points_result_ = false;
  bool visualize_points_histogram_image_result_ = false;

  // Parameters for visualizations
  float visualization_min_depth_ = 0.0f;
  float visualization_max_depth_ = 1.0f;

  // Precalculated variables for smoothed step function lookup (internal
  // data)
  std::vector<float> function_lookup_f_;
  std::vector<float> function_lookup_b_;

  // Precalculated variables for distributions (internal data)
  int line_length_in_segments_{};
  float distribution_length_minus_1_half_{};
  float distribution_length_plus_1_half_{};
  float min_expected_variance_{};

  // Precalculated variables for camera (referenced data)
  float fu_{};
  float fv_{};
  float ppu_{};
  float ppv_{};
  int image_width_minus_1_{};
  int image_height_minus_1_{};
  int image_width_minus_2_{};
  int image_height_minus_2_{};
  float depth_fu_{};
  float depth_fv_{};
  float depth_ppu_{};
  float depth_ppv_{};
  float depth_scale_{};
  int depth_image_width_minus_1_{};
  int depth_image_height_minus_1_{};

  // Precalculated variables for model (referenced data)
  int measured_depth_offset_id_{};
  int modeled_depth_offset_id_{};

  // Precalculated variables for renderer (referenced data)
  int depth_image_size_minus_1_{};
  float fsilhouette_image_size_{};

  // Precalculated variables for poses (continuously changing)
  Transform3fA body2camera_pose_;
  Transform3fA body2depth_camera_pose_;
  Eigen::Matrix3f body2camera_rotation_;
  Eigen::Matrix<float, 2, 3> body2camera_rotation_xy_;

  // Precalculate variables depending on iteration (continuously changing)
  int scale_{};
  float fscale_{};
  int line_length_{};
  int line_length_minus_1_{};
  float line_length_minus_1_half_{};
  float line_length_half_minus_1_{};
  float variance_{};

  // State variables
  int first_iteration_ = 0;
};

}  // namespace m3t

#endif  // M3T_INCLUDE_M3T_REGION_MODALITY_H_
