// SPDX-License-Identifier: MIT
// Copyright (c) 2023 Manuel Stoiber, German Aerospace Center (DLR)

#ifndef M3T_INCLUDE_M3T_DEPTH_MODALITY_H_
#define M3T_INCLUDE_M3T_DEPTH_MODALITY_H_

#include <filesystem/filesystem.h>
#include <m3t/body.h>
#include <m3t/camera.h>
#include <m3t/common.h>
#include <m3t/depth_model.h>
#include <m3t/modality.h>
#include <m3t/renderer.h>

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <iostream>
#include <memory>
#include <opencv2/opencv.hpp>
#include <string>
#include <vector>

namespace m3t {

/**
 * \brief \ref Modality class that implements a depth-based modality, which uses
 * information from a \ref DepthCamera and \ref DepthModel to calculate the
 * gradient vector and Hessian matrix that are used by a \ref Link and \ref
 * Optimizer to update the \ref Body pose.
 *
 * \details The modality is able to measure occlusions using the depth image or
 * model occlusions using renderings from a \ref FocusedDepthRenderer object. In
 * addition, it is able to validate surface points using a \ref
 * FocusedSilhouetteRenderer, where it checks if points lie on the bodies's
 * silhouette.
 *
 * @param depth_camera_ptr referenced \ref DepthCamera from which depth images
 * are taken.
 * @param depth_model_ptr referenced \ref DepthModel that provides views from a
 * *Sparse Viewpoint Model*.
 * @param depth_renderer_ptr referenced \ref FocusedDepthRenderer that is used
 * for modeled occlusion handling.
 * @param silhouette_renderer_ptr referenced \ref FocusedSilhouetteRenderer that
 * is used for silhouette checking. It has to be configured with the IDType
 * BODY.
 * @param n_points_max number of correspondence points.
 * @param use_adaptive_coverage flag that specifies if the number of points is
 * dynamically changed depending on the surface area.
 * @param use_depth_scaling flag that specifies if parameters such as the
 * `condiered_distances`, `depth_offset_radius`, `occlusion_radius`, or
 * `occlusion_threshold` are scaled with respect to the depth value of the
 * surface point. Unscaled reference values are defined at a distance of 1m.
 * @param reference_surface_area reference value that is considered if
 * `use_adaptive_coverage = true`. The number of points is adapted depending on
 * the ratio between the current surface area and this value. If the surface
 * area is bigger or equal to this value the number of points is set to
 * `n_points_max`. If `reference_surface_area <= 0.0` the `max_surface_area`
 * value from the \ref DepthModel is used.
 * @param stride_length distance in meter between points that are sampled during
 * the correspondence search.
 * @param considered_distances considered distance for each iteration. The
 * distance defines the area in meter in which points are sampled during the
 * correspondence search. If fewer values than `n_corr_iterations` are given,
 * the last provided distance is used.
 * @param standard_deviations user-defined standard deviation for each
 * iteration in meter. If fewer values than `n_corr_iterations` are given, the
 * last provided standard deviation is used.
 * @param measure_occlusions defines if occlusions are measured using the depth
 * image.
 * @param measured_depth_offset_radius radius in meter that specifies the depth
 * offset provided by the \ref DepthModel and that is used to measure
 * occlusions.
 * @param measured_occlusion_radius radius in meter that defines the area in
 * which depth measurements from a \ref DepthCamera are considered for occlusion
 * handling.
 * @param measured_occlusion_threshold defines how much smaller the
 * minimum value from the \ref DepthCamera is allowed to be compared to the
 * value that is expected from the current estimate.
 * @param modeled_depth_offset_radius radius in meter that specifies the depth
 * offset provided by the \ref DepthModel and that is used to model occlusions.
 * @param modeled_occlusion_radius radius in meter that defines the area in
 * which depth measurements from a \ref FocusedDepthRenderer are considered for
 * occlusion handling.
 * @param modeled_occlusion_threshold defines how much smaller the minimum value
 * from the \ref FocusedDepthRenderer is allowed to be compared to the value
 * that is expected from the current estimate.
 * @param n_unoccluded_iterations number of iterations, after `StartModality()`
 * is called, in which occlusion handling is turned off.
 * @param min_n_unoccluded_points minimum number of points that have to be
 * valid. Otherwise, occlusion handling is turned off.
 * @param visualize_correspondences_correspondence visualize correspondence
 * points during `VisualizeCorrespondences()`.
 * @param visualize_points_correspondence visualize model points during
 * `VisualizeCorrespondences()`.
 * @param visualize_points_depth_rendering_correspondence  visualize model
 * points on the depth rendering, which is used for occlusion handling, during
 * `VisualizeCorrespondences()`.
 * @param visualize_points_silhouette_rendering_correspondence visualize model
 * points on the silhouette rendering, which is used for silhouette checking,
 * during `VisualizeCorrespondences()`.
 * @param visualize_points_optimization visualize model points during
 * `VisualizeOptimization()`.
 * @param visualize_points_result visualize model points during
 * `VisualizeResults()`.
 * @param visualization_min_depth minimum depth for normalization of depth
 * images in visualization.
 * @param visualization_max_depth maximum depth for normalization of depth
 * images in visualization.
 */
class DepthModality : public Modality {
 private:
 private:
  static constexpr int kMaxNOcclusionStrides = 5;

  // Data for correspondence point calculated during `CalculateCorrespondences`
  struct DataPoint {
    Eigen::Vector3f center_f_body{};
    Eigen::Vector3f center_f_camera{};
    Eigen::Vector3f normal_f_body{};
    float center_u = 0.0f;
    float center_v = 0.0f;
    float depth = 0.0f;
    float measured_depth_offset = 0.0f;
    float modeled_depth_offset = 0.0f;
    Eigen::Vector3f correspondence_center_f_camera{};
  };

 public:
  // Constructors and setup methods
  DepthModality(const std::string &name, const std::shared_ptr<Body> &body_ptr,
                const std::shared_ptr<DepthCamera> &depth_camera_ptr,
                const std::shared_ptr<DepthModel> &depth_model_ptr);
  DepthModality(const std::string &name,
                const std::filesystem::path &metafile_path,
                const std::shared_ptr<Body> &body_ptr,
                const std::shared_ptr<DepthCamera> &depth_camera_ptr,
                const std::shared_ptr<DepthModel> &depth_model_ptr);
  bool SetUp() override;

  // Setters data
  void set_depth_camera_ptr(
      const std::shared_ptr<DepthCamera> &depth_camera_ptr);
  void set_depth_model_ptr(const std::shared_ptr<DepthModel> &depth_model_ptr);

  // Setters for general distribution
  void set_n_points_max(int n_points_max);
  void set_use_adaptive_coverage(bool use_adaptive_coverage);
  void set_use_depth_scaling(bool use_depth_scaling);
  void set_reference_surface_area(float reference_surface_area);
  void set_stride_length(float stride_length);
  void set_considered_distances(const std::vector<float> &considered_distances);
  void set_standard_deviations(const std::vector<float> &standard_deviations);

  // Setters for occlusion handling and point validation
  void UseSilhouetteChecking(const std::shared_ptr<FocusedSilhouetteRenderer>
                                 &silhouette_renderer_ptr);
  void DoNotUseSilhouetteChecking();
  void MeasureOcclusions();
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
  void set_min_n_unoccluded_points(int min_n_unoccluded_points);

  // Setters to turn on individual visualizations
  void set_visualize_correspondences_correspondence(
      bool visualize_correspondences_correspondence);
  void set_visualize_points_correspondence(
      bool visualize_points_correspondence);
  void set_visualize_points_depth_rendering_correspondence(
      bool visualize_points_depth_rendering_correspondence);
  void set_visualize_points_silhouette_rendering_correspondence(
      bool visualize_points_silhouette_rendering_correspondence);
  void set_visualize_points_optimization(bool visualize_points_optimization);
  void set_visualize_points_result(bool visualize_points_result);

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
  const std::shared_ptr<DepthCamera> &depth_camera_ptr() const;
  const std::shared_ptr<DepthModel> &depth_model_ptr() const;
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

  // Getters for general distribution
  int n_points_max() const;
  bool use_adaptive_coverage() const;
  bool use_depth_scaling() const;
  float reference_surface_area() const;
  float stride_length() const;
  const std::vector<float> &considered_distances() const;
  const std::vector<float> &standard_deviations() const;

  // Getters for occlusion handling and point validation
  bool use_silhouette_checking() const;
  bool measure_occlusions() const;
  float measured_depth_offset_radius() const;
  float measured_occlusion_radius() const;
  float measured_occlusion_threshold() const;
  bool model_occlusions() const;
  float modeled_depth_offset_radius() const;
  float modeled_occlusion_radius() const;
  float modeled_occlusion_threshold() const;
  int n_unoccluded_iterations() const;
  int min_n_unoccluded_points() const;

  // Getters to turn on individual visualizations
  bool visualize_correspondences_correspondence() const;
  bool visualize_points_correspondence() const;
  bool visualize_points_depth_rendering_correspondence() const;
  bool visualize_points_silhouette_rendering_correspondence() const;
  bool visualize_points_optimization() const;
  bool visualize_points_result() const;

  // Getters for visualization parameters
  float visualization_min_depth() const;
  float visualization_max_depth() const;

 private:
  // Helper method for setup
  bool LoadMetaData();

  // Helper methods for precalculation of internal data
  void SetImshowVariables();

  // Helper methods for precalculation of referenced data and changing data
  void PrecalculateCameraVariables();
  void PrecalculateRendererVariables();
  void PrecalculatePoseVariables();
  void PrecalculateIterationDependentVariables(int corr_iteration);

  // Helper methods for CalculateCorrespondences
  void CalculateBasicPointData(const DepthModel::DataPoint &data_model_point,
                               DataPoint *data_point) const;
  bool IsPointValid(const DataPoint &data_point, bool use_silhouette_checking,
                    bool measure_occlusions, bool model_occlusions) const;
  bool IsPointOnValidSilhouette(const DataPoint &data_point) const;
  bool IsPointUnoccludedMeasured(const DataPoint &data_point) const;
  bool IsPointUnoccludedModeled(const DataPoint &data_point) const;
  bool FindCorrespondence(
      const DataPoint &data_point,
      Eigen::Vector3f *correspondence_center_f_camera) const;

  // Helper methods for visualization
  void ShowAndSaveImage(const std::string &title, int save_index,
                        const cv::Mat &image) const;
  void VisualizePointsDepthImage(const std::string &title,
                                 int save_index) const;
  void VisualizePointsDepthRendering(const std::string &title,
                                     int save_index) const;
  void VisualizePointsSilhouetteRendering(const std::string &title,
                                          int save_index) const;
  void VisualizeCorrespondences(const std::string &title, int save_index) const;
  void DrawPoints(const cv::Vec3b &color_point, cv::Mat *image) const;
  void DrawFocusedPoints(const std::shared_ptr<FocusedRenderer> &renderer_ptr,
                         const cv::Vec3b &color_point, cv::Mat *image) const;
  void DrawCorrespondences(const cv::Vec3b &color_point,
                           const cv::Vec3b &color_correspondence_point,
                           const cv::Vec3b &color_correspondence,
                           cv::Mat *image) const;

  // Other helper methods
  bool IsSetup() const;

  // Internal data objects
  std::vector<DataPoint> data_points_;

  // Pointers to referenced objects
  std::shared_ptr<DepthCamera> depth_camera_ptr_ = nullptr;
  std::shared_ptr<DepthModel> depth_model_ptr_ = nullptr;
  std::shared_ptr<FocusedDepthRenderer> depth_renderer_ptr_ = nullptr;
  std::shared_ptr<FocusedSilhouetteRenderer> silhouette_renderer_ptr_ = nullptr;

  // Parameters for general distribution
  int n_points_max_ = 200;
  bool use_adaptive_coverage_ = false;
  bool use_depth_scaling_ = false;
  float reference_surface_area_ = 0.0f;
  float stride_length_ = 0.005f;
  std::vector<float> considered_distances_{0.05f, 0.02f, 0.01f};
  std::vector<float> standard_deviations_{0.05f, 0.03f, 0.02f};

  // Parameters for occlusion handling and point validation
  bool use_silhouette_checking_ = false;
  bool measure_occlusions_ = false;
  float measured_depth_offset_radius_ = 0.01f;
  float measured_occlusion_radius_ = 0.01f;
  float measured_occlusion_threshold_ = 0.03f;
  bool model_occlusions_ = false;
  float modeled_depth_offset_radius_ = 0.01f;
  float modeled_occlusion_radius_ = 0.01f;
  float modeled_occlusion_threshold_ = 0.03f;
  int n_unoccluded_iterations_ = 10;
  int min_n_unoccluded_points_ = 0;

  // Parameters to turn on individual visualizations
  bool visualize_correspondences_correspondence_ = false;
  bool visualize_points_correspondence_ = false;
  bool visualize_points_depth_rendering_correspondence_ = false;
  bool visualize_points_silhouette_rendering_correspondence_ = false;
  bool visualize_points_optimization_ = false;
  bool visualize_points_result_ = false;

  // Parameters for visualizations
  float visualization_min_depth_ = 0.0f;
  float visualization_max_depth_ = 1.0f;

  // Precalculated variables for camera (referenced data)
  float fu_{};
  float fv_{};
  float ppu_{};
  float ppv_{};
  int image_width_minus_1_{};
  int image_height_minus_1_{};
  float depth_scale_{};

  // Precalculated variables for renderer (referenced data)
  int depth_image_size_minus_1_{};

  // Precalculated variables for poses (continuously changing)
  Transform3fA body2camera_pose_{};
  Transform3fA camera2body_pose_{};
  Eigen::Matrix3f body2camera_rotation_{};

  // Precalculate variables depending on iteration (continuously changing)
  float considered_distance_{};
  float standard_deviation_{};
  int max_n_strides_{};

  // State variables
  int first_iteration_ = 0;
};

}  // namespace m3t

#endif  // M3T_INCLUDE_M3T_DEPTH_MODALITY_H_
