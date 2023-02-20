// SPDX-License-Identifier: MIT
// Copyright (c) 2023 Manuel Stoiber, German Aerospace Center (DLR)

#include <m3t/region_modality.h>

namespace m3t {

RegionModality::RegionModality(
    const std::string &name, const std::shared_ptr<Body> &body_ptr,
    const std::shared_ptr<ColorCamera> &color_camera_ptr,
    const std::shared_ptr<RegionModel> &region_model_ptr)
    : Modality{name, body_ptr},
      color_camera_ptr_{color_camera_ptr},
      region_model_ptr_{region_model_ptr} {}

RegionModality::RegionModality(
    const std::string &name, const std::filesystem::path &metafile_path,
    const std::shared_ptr<Body> &body_ptr,
    const std::shared_ptr<ColorCamera> &color_camera_ptr,
    const std::shared_ptr<RegionModel> &region_model_ptr)
    : Modality{name, metafile_path, body_ptr},
      color_camera_ptr_{color_camera_ptr},
      region_model_ptr_{region_model_ptr} {}

bool RegionModality::SetUp() {
  set_up_ = false;
  if (!metafile_path_.empty())
    if (!LoadMetaData()) return false;

  // Check if all required objects are set up
  if (!body_ptr_->set_up()) {
    std::cerr << "Body " << body_ptr_->name() << " was not set up" << std::endl;
    return false;
  }
  if (!region_model_ptr_->set_up()) {
    std::cerr << "Region model " << region_model_ptr_->name()
              << " was not set up" << std::endl;
    return false;
  }
  if (!color_camera_ptr_->set_up()) {
    std::cerr << "Color camera " << color_camera_ptr_->name()
              << " was not set up" << std::endl;
    return false;
  }
  if (measure_occlusions_ && !depth_camera_ptr_->set_up()) {
    std::cerr << "Depth camera " << depth_camera_ptr_->name()
              << " was not set up" << std::endl;
    return false;
  }
  if (model_occlusions_ && !depth_renderer_ptr_->set_up()) {
    std::cerr << "Focused depth renderer " << depth_renderer_ptr_->name()
              << " was not set up" << std::endl;
    return false;
  }
  if (use_shared_color_histograms_ && !color_histograms_ptr_->set_up()) {
    std::cerr << "Color histogram " << color_histograms_ptr_->name()
              << " was not set up" << std::endl;
    return false;
  }
  if (use_region_checking_ && !silhouette_renderer_ptr_->set_up()) {
    std::cerr << "Focused silhouette renderer "
              << silhouette_renderer_ptr_->name() << " was not set up"
              << std::endl;
    return false;
  }

  // Check if all required objects are correctly configured
  if (model_occlusions_ &&
      !depth_renderer_ptr_->IsBodyReferenced(body_ptr_->name())) {
    std::cerr << "Focused depth renderer " << depth_renderer_ptr_->name()
              << " does not reference body " << body_ptr_->name() << std::endl;
    return false;
  }
  if (use_region_checking_ &&
      silhouette_renderer_ptr_->id_type() != IDType::REGION) {
    std::cerr << "Focused silhouette renderer "
              << silhouette_renderer_ptr_->name()
              << " does not use id_type REGION" << std::endl;
  }
  if (use_region_checking_ &&
      !silhouette_renderer_ptr_->IsBodyReferenced(body_ptr_->name())) {
    std::cerr << "Focused silhouette renderer "
              << silhouette_renderer_ptr_->name() << " does not reference body "
              << body_ptr_->name() << std::endl;
    return false;
  }

  PrecalculateFunctionLookup();
  PrecalculateDistributionVariables();
  if (!use_shared_color_histograms_)
    if (!SetUpInternalColorHistograms()) return false;
  SetImshowVariables();
  PrecalculateCameraVariables();
  if (!PrecalculateModelVariables()) return false;
  PrecalculateRendererVariables();

  set_up_ = true;
  return true;
}

void RegionModality::set_color_camera_ptr(
    const std::shared_ptr<ColorCamera> &color_camera_ptr) {
  color_camera_ptr_ = color_camera_ptr;
  set_up_ = false;
}

void RegionModality::set_region_model_ptr(
    const std::shared_ptr<RegionModel> &region_model_ptr) {
  region_model_ptr_ = region_model_ptr;
  set_up_ = false;
}

void RegionModality::set_n_lines_max(int n_lines_max) {
  n_lines_max_ = n_lines_max;
}

void RegionModality::set_use_adaptive_coverage(bool use_adaptive_coverage) {
  use_adaptive_coverage_ = use_adaptive_coverage;
}

void RegionModality::set_reference_contour_length(
    float reference_contour_length) {
  reference_contour_length_ = reference_contour_length;
}

void RegionModality::set_min_continuous_distance(
    float min_continuous_distance) {
  min_continuous_distance_ = min_continuous_distance;
}

void RegionModality::set_function_length(int function_length) {
  function_length_ = function_length;
  set_up_ = false;
}

void RegionModality::set_distribution_length(int distribution_length) {
  distribution_length_ = distribution_length;
  set_up_ = false;
}

void RegionModality::set_function_amplitude(float function_amplitude) {
  function_amplitude_ = function_amplitude;
  set_up_ = false;
}

void RegionModality::set_function_slope(float function_slope) {
  function_slope_ = function_slope;
  set_up_ = false;
}

void RegionModality::set_learning_rate(float learning_rate) {
  learning_rate_ = learning_rate;
}

void RegionModality::set_n_global_iterations(int n_global_iterations) {
  n_global_iterations_ = n_global_iterations;
}

void RegionModality::set_scales(const std::vector<int> &scales) {
  scales_ = scales;
}

void RegionModality::set_standard_deviations(
    const std::vector<float> &standard_deviations) {
  standard_deviations_ = standard_deviations;
}

void RegionModality::UseSharedColorHistograms(
    const std::shared_ptr<ColorHistograms> &color_histograms_ptr) {
  color_histograms_ptr_ = color_histograms_ptr;
  use_shared_color_histograms_ = true;
  set_up_ = false;
}

void RegionModality::DoNotUseSharedColorHistograms() {
  color_histograms_ptr_ = nullptr;
  use_shared_color_histograms_ = false;
  set_up_ = false;
}

bool RegionModality::set_n_histogram_bins(int n_histogram_bins) {
  if (use_shared_color_histograms_) {
    std::cerr << "Not able to set n_histogram_bins. "
              << "Directly set parameter in shared color histogram "
              << color_histograms_ptr_->name() << std::endl;
    return false;
  } else {
    n_histogram_bins_ = n_histogram_bins;
    set_up_ = false;
    return true;
  }
}

bool RegionModality::set_learning_rate_f(float learning_rate_f) {
  if (use_shared_color_histograms_) {
    std::cerr << "Not able to set learning_rate_f. "
              << "Directly set parameter in shared color histogram "
              << color_histograms_ptr_->name() << std::endl;
    return false;
  } else {
    learning_rate_f_ = learning_rate_f;
    set_up_ = false;
    return true;
  }
}

bool RegionModality::set_learning_rate_b(float learning_rate_b) {
  if (use_shared_color_histograms_) {
    std::cerr << "Not able to set learning_rate_b. "
              << "Directly set parameter in shared color histogram "
              << color_histograms_ptr_->name() << std::endl;
    return false;
  } else {
    learning_rate_b_ = learning_rate_b;
    set_up_ = false;
    return true;
  }
}

void RegionModality::set_unconsidered_line_length(
    float unconsidered_line_length) {
  unconsidered_line_length_ = unconsidered_line_length;
}

void RegionModality::set_max_considered_line_length(
    float max_considered_line_length) {
  max_considered_line_length_ = max_considered_line_length;
}

void RegionModality::UseRegionChecking(
    const std::shared_ptr<FocusedSilhouetteRenderer> &silhouette_renderer_ptr) {
  silhouette_renderer_ptr_ = silhouette_renderer_ptr;
  use_region_checking_ = true;
  set_up_ = false;
}

void RegionModality::DoNotUseRegionChecking() {
  silhouette_renderer_ptr_ = nullptr;
  use_region_checking_ = false;
  set_up_ = false;
}

void RegionModality::MeasureOcclusions(
    const std::shared_ptr<DepthCamera> &depth_camera_ptr) {
  depth_camera_ptr_ = depth_camera_ptr;
  measure_occlusions_ = true;
  set_up_ = false;
}

void RegionModality::DoNotMeasureOcclusions() {
  depth_camera_ptr_ = nullptr;
  measure_occlusions_ = false;
  set_up_ = false;
}

void RegionModality::ModelOcclusions(
    const std::shared_ptr<FocusedDepthRenderer> &depth_renderer_ptr) {
  depth_renderer_ptr_ = depth_renderer_ptr;
  model_occlusions_ = true;
  set_up_ = false;
}

void RegionModality::DoNotModelOcclusions() {
  depth_renderer_ptr_ = nullptr;
  model_occlusions_ = false;
  set_up_ = false;
}

void RegionModality::set_measured_depth_offset_radius(
    float measured_depth_offset_radius) {
  measured_depth_offset_radius_ = measured_depth_offset_radius;
}

void RegionModality::set_measured_occlusion_radius(
    float measured_occlusion_radius) {
  measured_occlusion_radius_ = measured_occlusion_radius;
}

void RegionModality::set_measured_occlusion_threshold(
    float measured_occlusion_threshold) {
  measured_occlusion_threshold_ = measured_occlusion_threshold;
}

void RegionModality::set_modeled_depth_offset_radius(
    float modeled_depth_offset_radius) {
  modeled_depth_offset_radius_ = modeled_depth_offset_radius;
}

void RegionModality::set_modeled_occlusion_radius(
    float modeled_occlusion_radius) {
  modeled_occlusion_radius_ = modeled_occlusion_radius;
}

void RegionModality::set_modeled_occlusion_threshold(
    float modeled_occlusion_threshold) {
  modeled_occlusion_threshold_ = modeled_occlusion_threshold;
}

void RegionModality::set_n_unoccluded_iterations(int n_unoccluded_iterations) {
  n_unoccluded_iterations_ = n_unoccluded_iterations;
}

void RegionModality::set_min_n_unoccluded_lines(int min_n_unoccluded_lines) {
  min_n_unoccluded_lines_ = min_n_unoccluded_lines;
}

void RegionModality::set_visualize_lines_correspondence(
    bool visualize_lines_correspondence) {
  visualize_lines_correspondence_ = visualize_lines_correspondence;
  SetImshowVariables();
}

void RegionModality::set_visualize_points_correspondence(
    bool visualize_points_correspondence) {
  visualize_points_correspondence_ = visualize_points_correspondence;
  SetImshowVariables();
}

void RegionModality::set_visualize_points_depth_image_correspondence(
    bool visualize_points_depth_image_correspondence) {
  visualize_points_depth_image_correspondence_ =
      visualize_points_depth_image_correspondence;
  SetImshowVariables();
}

void RegionModality::set_visualize_points_depth_rendering_correspondence(
    bool visualize_points_depth_rendering_correspondence) {
  visualize_points_depth_rendering_correspondence_ =
      visualize_points_depth_rendering_correspondence;
  SetImshowVariables();
}

void RegionModality::set_visualize_points_silhouette_rendering_correspondence(
    bool visualize_points_silhouette_rendering_correspondence) {
  visualize_points_silhouette_rendering_correspondence_ =
      visualize_points_silhouette_rendering_correspondence;
  SetImshowVariables();
}

void RegionModality::set_visualize_points_optimization(
    bool visualize_points_optimization) {
  visualize_points_optimization_ = visualize_points_optimization;
  SetImshowVariables();
}

void RegionModality::set_visualize_points_histogram_image_optimization(
    bool visualize_points_histogram_image_optimization) {
  visualize_points_histogram_image_optimization_ =
      visualize_points_histogram_image_optimization;
  SetImshowVariables();
}

void RegionModality::set_visualize_points_result(bool visualize_points_result) {
  visualize_points_result_ = visualize_points_result;
  SetImshowVariables();
}

void RegionModality::set_visualize_points_histogram_image_result(
    bool visualize_points_histogram_image_result) {
  visualize_points_histogram_image_result_ =
      visualize_points_histogram_image_result;
  SetImshowVariables();
}

void RegionModality::set_visualization_min_depth(
    float visualization_min_depth) {
  visualization_min_depth_ = visualization_min_depth;
}

void RegionModality::set_visualization_max_depth(
    float visualization_max_depth) {
  visualization_max_depth_ = visualization_max_depth;
}

bool RegionModality::StartModality(int iteration, int corr_iteration) {
  if (!IsSetup()) return false;

  first_iteration_ = iteration;
  PrecalculatePoseVariables();

  // Initialize histograms
  bool handle_occlusions = n_unoccluded_iterations_ == 0;
  if (!use_shared_color_histograms_) color_histograms_ptr_->ClearMemory();
  AddLinePixelColorsToTempHistograms(handle_occlusions);
  if (!use_shared_color_histograms_)
    color_histograms_ptr_->InitializeHistograms();
  return true;
}

bool RegionModality::CalculateCorrespondences(int iteration,
                                              int corr_iteration) {
  if (!IsSetup()) return false;

  PrecalculatePoseVariables();
  PrecalculateIterationDependentVariables(corr_iteration);

  // Check if body is visible and fetch images from renderers
  bool body_visible_depth;
  if (model_occlusions_) {
    body_visible_depth = depth_renderer_ptr_->IsBodyVisible(body_ptr_->name());
    if (body_visible_depth) depth_renderer_ptr_->FetchDepthImage();
  }
  bool body_visible_silhouette;
  if (use_region_checking_) {
    body_visible_silhouette =
        silhouette_renderer_ptr_->IsBodyVisible(body_ptr_->name());
    if (body_visible_silhouette)
      silhouette_renderer_ptr_->FetchSilhouetteImage();
  }

  // Search closest template view
  const RegionModel::View *view;
  region_model_ptr_->GetClosestView(body2camera_pose_, &view);
  auto &data_model_points{view->data_points};

  // Scale number of lines with contour_length ratio
  int n_lines = n_lines_max_;
  if (use_adaptive_coverage_) {
    if (reference_contour_length_ > 0.0f)
      n_lines = n_lines_max_ * std::min(1.0f, view->contour_length /
                                                  reference_contour_length_);
    else
      n_lines = n_lines_max_ * view->contour_length /
                region_model_ptr_->max_contour_length();
  }
  if (n_lines > data_model_points.size()) {
    std::cerr << "Number of model points too small: "
              << data_model_points.size() << " < " << n_lines << std::endl;
    n_lines = data_model_points.size();
  }

  // Differentiate cases with and without occlusion handling
  std::vector<float> segment_probabilities_f(line_length_in_segments_);
  std::vector<float> segment_probabilities_b(line_length_in_segments_);
  for (int j = 0; j < 2; ++j) {
    data_lines_.clear();
    bool handle_occlusions =
        j == 0 && (iteration - first_iteration_) >= n_unoccluded_iterations_;

    // Iterate over n_lines
    for (int i = 0; i < n_lines; ++i) {
      DataLine data_line;
      CalculateBasicLineData(data_model_points[i], &data_line);
      if (!IsLineValid(
              data_line, use_region_checking_ && body_visible_silhouette,
              handle_occlusions && measure_occlusions_,
              handle_occlusions && model_occlusions_ && body_visible_depth))
        continue;
      if (!CalculateSegmentProbabilities(
              data_line.center_u, data_line.center_v, data_line.normal_u,
              data_line.normal_v, &segment_probabilities_f,
              &segment_probabilities_b, &data_line.normal_component_to_scale,
              &data_line.delta_r))
        continue;
      CalculateDistribution(segment_probabilities_f, segment_probabilities_b,
                            &data_line.distribution);

      CalculateDistributionMoments(data_line.distribution, &data_line.mean,
                                   &data_line.measured_variance);
      data_lines_.push_back(std::move(data_line));
    }
    if (data_lines_.size() >= min_n_unoccluded_lines_) break;
  }
  return true;
}

bool RegionModality::VisualizeCorrespondences(int save_idx) {
  if (!IsSetup()) return false;

  if (visualize_lines_correspondence_)
    VisualizeLines("lines_correspondence", save_idx);
  if (visualize_points_correspondence_)
    VisualizePointsColorImage("color_image_correspondence", save_idx);
  if (visualize_points_depth_image_correspondence_ && measure_occlusions_)
    VisualizePointsDepthImage("depth_image_correspondence", save_idx);
  if (visualize_points_depth_rendering_correspondence_ && model_occlusions_)
    VisualizePointsDepthRendering("depth_rendering_correspondence", save_idx);
  if (visualize_points_silhouette_rendering_correspondence_ &&
      use_region_checking_)
    VisualizePointsSilhouetteRendering("silhouette_rendering_correspondence",
                                       save_idx);
  return true;
}

bool RegionModality::CalculateGradientAndHessian(int iteration,
                                                 int corr_iteration,
                                                 int opt_iteration) {
  if (!IsSetup()) return false;

  PrecalculatePoseVariables();
  gradient_.setZero();
  hessian_.setZero();

  // Iterate over correspondence lines
  for (auto &data_line : data_lines_) {
    // Calculate point coordinates in camera frame
    data_line.center_f_camera = body2camera_pose_ * data_line.center_f_body;
    float x = data_line.center_f_camera(0);
    float y = data_line.center_f_camera(1);
    float z = data_line.center_f_camera(2);

    // Calculate delta_cs
    float fu_z = fu_ / z;
    float fv_z = fv_ / z;
    float xfu_z = x * fu_z;
    float yfv_z = y * fv_z;
    float delta_cs = (data_line.normal_u * (xfu_z + ppu_ - data_line.center_u) +
                      data_line.normal_v * (yfv_z + ppv_ - data_line.center_v) -
                      data_line.delta_r) *
                     data_line.normal_component_to_scale;

    // Calculate first derivative of loglikelihood with respect to delta_cs
    float dloglikelihood_ddelta_cs;
    if (opt_iteration < n_global_iterations_) {
      dloglikelihood_ddelta_cs =
          (data_line.mean - delta_cs) / data_line.measured_variance;
    } else {
      // Calculate distribution indexes
      // Note: (distribution_length - 1) / 2 + 1 = (distribution_length + 1) / 2
      int dist_idx_upper = int(delta_cs + distribution_length_plus_1_half_);
      int dist_idx_lower = dist_idx_upper - 1;
      if (dist_idx_upper <= 0 || dist_idx_upper >= distribution_length_)
        continue;

      dloglikelihood_ddelta_cs =
          (std::log(data_line.distribution[dist_idx_upper]) -
           std::log(data_line.distribution[dist_idx_lower])) *
          learning_rate_ / data_line.measured_variance;
    }

    // Calculate first order derivative of delta_cs with respect to theta
    Eigen::RowVector3f ddelta_cs_dcenter{
        data_line.normal_component_to_scale * data_line.normal_u * fu_z,
        data_line.normal_component_to_scale * data_line.normal_v * fv_z,
        data_line.normal_component_to_scale *
            (-data_line.normal_u * xfu_z - data_line.normal_v * yfv_z) / z};
    Eigen::RowVector3f ddelta_cs_dtranslation{ddelta_cs_dcenter *
                                              body2camera_rotation_};
    Eigen::Matrix<float, 1, 6> ddelta_cs_dtheta;
    ddelta_cs_dtheta << data_line.center_f_body.transpose().cross(
        ddelta_cs_dtranslation),
        ddelta_cs_dtranslation;

    // Calculate weight
    float weight = min_expected_variance_ /
                   (data_line.normal_component_to_scale *
                    data_line.normal_component_to_scale * variance_);

    // Calculate gradient and hessian
    gradient_ +=
        (weight * dloglikelihood_ddelta_cs) * ddelta_cs_dtheta.transpose();
    hessian_.triangularView<Eigen::Lower>() -=
        (weight / data_line.measured_variance) * ddelta_cs_dtheta.transpose() *
        ddelta_cs_dtheta;
  }
  hessian_ = hessian_.selfadjointView<Eigen::Lower>();
  return true;
}

bool RegionModality::VisualizeOptimization(int save_idx) {
  if (!IsSetup()) return false;

  if (visualize_points_optimization_)
    VisualizePointsColorImage("color_image_optimization", save_idx);
  if (visualize_points_histogram_image_optimization_)
    VisualizePointsHistogramImage("histogram_image_optimization", save_idx);
  if (visualize_gradient_optimization_) VisualizeGradient();
  if (visualize_hessian_optimization_) VisualizeHessian();
  return true;
}

bool RegionModality::CalculateResults(int iteration) {
  if (!IsSetup()) return false;

  // Calculate histograms
  if (!use_shared_color_histograms_) color_histograms_ptr_->ClearMemory();
  PrecalculatePoseVariables();
  bool handle_occlusions =
      (iteration - first_iteration_) >= n_unoccluded_iterations_;
  AddLinePixelColorsToTempHistograms(handle_occlusions);
  if (!use_shared_color_histograms_) color_histograms_ptr_->UpdateHistograms();
  return true;
}

bool RegionModality::VisualizeResults(int save_idx) {
  if (!IsSetup()) return false;

  if (visualize_points_result_)
    VisualizePointsColorImage("color_image_result", save_idx);
  if (visualize_points_histogram_image_result_)
    VisualizePointsHistogramImage("histogram_image_result", save_idx);
  if (visualize_pose_result_) VisualizePose();
  return true;
}

const std::shared_ptr<ColorCamera> &RegionModality::color_camera_ptr() const {
  return color_camera_ptr_;
}

const std::shared_ptr<DepthCamera> &RegionModality::depth_camera_ptr() const {
  return depth_camera_ptr_;
}

const std::shared_ptr<RegionModel> &RegionModality::region_model_ptr() const {
  return region_model_ptr_;
}

const std::shared_ptr<FocusedDepthRenderer>
    &RegionModality::depth_renderer_ptr() const {
  return depth_renderer_ptr_;
}

const std::shared_ptr<FocusedSilhouetteRenderer>
    &RegionModality::silhouette_renderer_ptr() const {
  return silhouette_renderer_ptr_;
}

std::shared_ptr<Model> RegionModality::model_ptr() const {
  return region_model_ptr_;
}

std::vector<std::shared_ptr<Camera>> RegionModality::camera_ptrs() const {
  return {color_camera_ptr_, depth_camera_ptr_};
}

std::vector<std::shared_ptr<Renderer>>
RegionModality::start_modality_renderer_ptrs() const {
  return {depth_renderer_ptr_, silhouette_renderer_ptr_};
}

std::vector<std::shared_ptr<Renderer>>
RegionModality::correspondence_renderer_ptrs() const {
  return {depth_renderer_ptr_, silhouette_renderer_ptr_};
}

std::vector<std::shared_ptr<Renderer>> RegionModality::results_renderer_ptrs()
    const {
  return {depth_renderer_ptr_, silhouette_renderer_ptr_};
}

std::shared_ptr<ColorHistograms> RegionModality::color_histograms_ptr() const {
  if (use_shared_color_histograms_)
    return color_histograms_ptr_;
  else
    return nullptr;
}

int RegionModality::n_lines_max() const { return n_lines_max_; }

bool RegionModality::use_adaptive_coverage() const {
  return use_adaptive_coverage_;
}

float RegionModality::reference_contour_length() const {
  return reference_contour_length_;
}

float RegionModality::min_continuous_distance() const {
  return min_continuous_distance_;
}

int RegionModality::function_length() const { return function_length_; }

int RegionModality::distribution_length() const { return distribution_length_; }

float RegionModality::function_amplitude() const { return function_amplitude_; }

float RegionModality::function_slope() const { return function_slope_; }

float RegionModality::learning_rate() const { return learning_rate_; }

int RegionModality::n_global_iterations() const { return n_global_iterations_; }

const std::vector<int> &RegionModality::scales() const { return scales_; }

const std::vector<float> &RegionModality::standard_deviations() const {
  return standard_deviations_;
}

bool RegionModality::use_shared_color_histograms() const {
  return use_shared_color_histograms_;
}

int RegionModality::n_histogram_bins() const {
  if (use_shared_color_histograms_) {
    return color_histograms_ptr_->n_bins();
  } else {
    return n_histogram_bins_;
  }
}

float RegionModality::learning_rate_f() const {
  if (use_shared_color_histograms_) {
    return color_histograms_ptr_->learning_rate_f();
  } else {
    return learning_rate_f_;
  }
}

float RegionModality::learning_rate_b() const {
  if (use_shared_color_histograms_) {
    return color_histograms_ptr_->learning_rate_b();
  } else {
    return learning_rate_b_;
  }
}

float RegionModality::unconsidered_line_length() const {
  return unconsidered_line_length_;
}

float RegionModality::max_considered_line_length() const {
  return max_considered_line_length_;
}

bool RegionModality::use_region_checking() const {
  return use_region_checking_;
}

bool RegionModality::measure_occlusions() const { return measure_occlusions_; }

float RegionModality::measured_depth_offset_radius() const {
  return measured_depth_offset_radius_;
}

float RegionModality::measured_occlusion_radius() const {
  return measured_occlusion_radius_;
}

float RegionModality::measured_occlusion_threshold() const {
  return measured_occlusion_threshold_;
}

bool RegionModality::model_occlusions() const { return model_occlusions_; }

float RegionModality::modeled_depth_offset_radius() const {
  return modeled_depth_offset_radius_;
}

float RegionModality::modeled_occlusion_radius() const {
  return modeled_occlusion_radius_;
}

float RegionModality::modeled_occlusion_threshold() const {
  return modeled_occlusion_threshold_;
}

int RegionModality::n_unoccluded_iterations() const {
  return n_unoccluded_iterations_;
}

int RegionModality::min_n_unoccluded_lines() const {
  return min_n_unoccluded_lines_;
}

bool RegionModality::visualize_lines_correspondence() const {
  return visualize_lines_correspondence_;
}

bool RegionModality::visualize_points_correspondence() const {
  return visualize_points_correspondence_;
}

bool RegionModality::visualize_points_depth_image_correspondence() const {
  return visualize_points_depth_image_correspondence_;
}

bool RegionModality::visualize_points_depth_rendering_correspondence() const {
  return visualize_points_depth_rendering_correspondence_;
}

bool RegionModality::visualize_points_silhouette_rendering_correspondence()
    const {
  return visualize_points_silhouette_rendering_correspondence_;
}

bool RegionModality::visualize_points_optimization() const {
  return visualize_points_optimization_;
}

bool RegionModality::visualize_points_histogram_image_optimization() const {
  return visualize_points_histogram_image_optimization_;
}

bool RegionModality::visualize_points_result() const {
  return visualize_points_result_;
}

bool RegionModality::visualize_points_histogram_image_result() const {
  return visualize_points_histogram_image_result_;
}

float RegionModality::visualization_min_depth() const {
  return visualization_min_depth_;
}

float RegionModality::visualization_max_depth() const {
  return visualization_max_depth_;
}

void RegionModality::SetImshowVariables() {
  imshow_correspondence_ =
      display_visualization_ &&
      (visualize_lines_correspondence_ || visualize_points_correspondence_ ||
       (visualize_points_depth_image_correspondence_ && measure_occlusions_) ||
       (visualize_points_depth_rendering_correspondence_ &&
        model_occlusions_) ||
       (visualize_points_silhouette_rendering_correspondence_ &&
        use_region_checking_));
  imshow_optimization_ = display_visualization_ &&
                         (visualize_points_optimization_ ||
                          visualize_points_histogram_image_optimization_);
  imshow_result_ =
      display_visualization_ &&
      (visualize_points_result_ || visualize_points_histogram_image_result_);
}

bool RegionModality::LoadMetaData() {
  // Open file storage from yaml
  cv::FileStorage fs;
  if (!OpenYamlFileStorage(metafile_path_, &fs)) return false;

  // Read parameters from yaml file for general distribution
  ReadOptionalValueFromYaml(fs, "n_lines_max", &n_lines_max_);
  ReadOptionalValueFromYaml(fs, "use_adaptive_coverage",
                            &use_adaptive_coverage_);
  ReadOptionalValueFromYaml(fs, "reference_contour_length",
                            &reference_contour_length_);
  ReadOptionalValueFromYaml(fs, "min_continuous_distance",
                            &min_continuous_distance_);
  ReadOptionalValueFromYaml(fs, "function_length", &function_length_);
  ReadOptionalValueFromYaml(fs, "distribution_length", &distribution_length_);
  ReadOptionalValueFromYaml(fs, "function_amplitude", &function_amplitude_);
  ReadOptionalValueFromYaml(fs, "function_slope", &function_slope_);
  ReadOptionalValueFromYaml(fs, "learning_rate", &learning_rate_);
  ReadOptionalValueFromYaml(fs, "n_global_iterations", &n_global_iterations_);
  ReadOptionalValueFromYaml(fs, "scales", &scales_);
  ReadOptionalValueFromYaml(fs, "standard_deviations", &standard_deviations_);

  // Read parameters from yaml file for histogram calculation
  ReadOptionalValueFromYaml(fs, "n_histogram_bins", &n_histogram_bins_);
  ReadOptionalValueFromYaml(fs, "learning_rate_f", &learning_rate_f_);
  ReadOptionalValueFromYaml(fs, "learning_rate_b", &learning_rate_b_);
  ReadOptionalValueFromYaml(fs, "unconsidered_line_length",
                            &unconsidered_line_length_);
  ReadOptionalValueFromYaml(fs, "max_considered_line_length",
                            &max_considered_line_length_);

  // Read parameters from yaml file for occlusion handling
  ReadOptionalValueFromYaml(fs, "measured_depth_offset_radius",
                            &measured_depth_offset_radius_);
  ReadOptionalValueFromYaml(fs, "measured_occlusion_radius",
                            &measured_occlusion_radius_);
  ReadOptionalValueFromYaml(fs, "measured_occlusion_threshold",
                            &measured_occlusion_threshold_);
  ReadOptionalValueFromYaml(fs, "modeled_depth_offset_radius",
                            &modeled_depth_offset_radius_);
  ReadOptionalValueFromYaml(fs, "modeled_occlusion_radius",
                            &modeled_occlusion_radius_);
  ReadOptionalValueFromYaml(fs, "modeled_occlusion_threshold",
                            &modeled_occlusion_threshold_);
  ReadOptionalValueFromYaml(fs, "n_unoccluded_iterations",
                            &n_unoccluded_iterations_);
  ReadOptionalValueFromYaml(fs, "min_n_unoccluded_lines",
                            &min_n_unoccluded_lines_);

  // Read parameters from yaml for visualization
  ReadOptionalValueFromYaml(fs, "visualize_pose_result",
                            &visualize_pose_result_);
  ReadOptionalValueFromYaml(fs, "visualize_gradient_optimization",
                            &visualize_gradient_optimization_);
  ReadOptionalValueFromYaml(fs, "visualize_hessian_optimization",
                            &visualize_hessian_optimization_);
  ReadOptionalValueFromYaml(fs, "visualize_lines_correspondence",
                            &visualize_lines_correspondence_);
  ReadOptionalValueFromYaml(fs, "visualize_points_correspondence",
                            &visualize_points_correspondence_);
  ReadOptionalValueFromYaml(fs, "visualize_points_depth_image_correspondence",
                            &visualize_points_depth_image_correspondence_);
  ReadOptionalValueFromYaml(fs,
                            "visualize_points_depth_rendering_correspondence",
                            &visualize_points_depth_rendering_correspondence_);
  ReadOptionalValueFromYaml(
      fs, "visualize_points_silhouette_rendering_correspondence",
      &visualize_points_silhouette_rendering_correspondence_);
  ReadOptionalValueFromYaml(fs, "visualize_points_optimization",
                            &visualize_points_optimization_);
  ReadOptionalValueFromYaml(fs, "visualize_points_histogram_image_optimization",
                            &visualize_points_histogram_image_optimization_);
  ReadOptionalValueFromYaml(fs, "visualize_points_result",
                            &visualize_points_result_);
  ReadOptionalValueFromYaml(fs, "visualize_points_histogram_image_result",
                            &visualize_points_histogram_image_result_);
  ReadOptionalValueFromYaml(fs, "visualization_min_depth",
                            &visualization_min_depth_);
  ReadOptionalValueFromYaml(fs, "visualization_max_depth",
                            &visualization_max_depth_);
  ReadOptionalValueFromYaml(fs, "display_visualization",
                            &display_visualization_);
  ReadOptionalValueFromYaml(fs, "save_visualizations", &save_visualizations_);
  ReadOptionalValueFromYaml(fs, "save_directory", &save_directory_);
  ReadOptionalValueFromYaml(fs, "save_image_type", &save_image_type_);

  // Process parameters
  if (save_directory_.is_relative())
    save_directory_ = metafile_path_.parent_path() / save_directory_;
  return true;
}

void RegionModality::PrecalculateFunctionLookup() {
  function_lookup_f_.resize(function_length_);
  function_lookup_b_.resize(function_length_);
  for (int i = 0; i < function_length_; ++i) {
    float x = float(i) - float(function_length_ - 1) / 2.0f;
    if (function_slope_ == 0.0f)
      function_lookup_f_[i] =
          0.5f - function_amplitude_ * ((0.0f < x) - (x < 0.0f));
    else
      function_lookup_f_[i] =
          0.5f - function_amplitude_ * std::tanh(x / (2.0f * function_slope_));
    function_lookup_b_[i] = 1.0f - function_lookup_f_[i];
  }
}

void RegionModality::PrecalculateDistributionVariables() {
  line_length_in_segments_ = function_length_ + distribution_length_ - 1;
  distribution_length_minus_1_half_ =
      (float(distribution_length_) - 1.0f) / 2.0f;
  distribution_length_plus_1_half_ =
      (float(distribution_length_) + 1.0f) / 2.0f;
  float min_expected_variance_laplace =
      1.0f / (2.0f * powf(std::atanhf(2.0f * function_amplitude_), 2.0f));
  float min_expected_variance_gaussian = function_slope_;
  min_expected_variance_ =
      std::max(min_expected_variance_laplace, min_expected_variance_gaussian);
}

bool RegionModality::SetUpInternalColorHistograms() {
  color_histograms_ptr_ = std::make_shared<ColorHistograms>(
      name_ + "_histograms", n_histogram_bins_, learning_rate_f_,
      learning_rate_b_);
  return color_histograms_ptr_->SetUp();
}

void RegionModality::PrecalculateCameraVariables() {
  fu_ = color_camera_ptr_->intrinsics().fu;
  fv_ = color_camera_ptr_->intrinsics().fv;
  ppu_ = color_camera_ptr_->intrinsics().ppu;
  ppv_ = color_camera_ptr_->intrinsics().ppv;
  image_width_minus_1_ = color_camera_ptr_->intrinsics().width - 1;
  image_height_minus_1_ = color_camera_ptr_->intrinsics().height - 1;
  image_width_minus_2_ = color_camera_ptr_->intrinsics().width - 2;
  image_height_minus_2_ = color_camera_ptr_->intrinsics().height - 2;
  if (measure_occlusions_) {
    depth_fu_ = depth_camera_ptr_->intrinsics().fu;
    depth_fv_ = depth_camera_ptr_->intrinsics().fv;
    depth_ppu_ = depth_camera_ptr_->intrinsics().ppu;
    depth_ppv_ = depth_camera_ptr_->intrinsics().ppv;
    depth_scale_ = depth_camera_ptr_->depth_scale();
    depth_image_width_minus_1_ = depth_camera_ptr_->intrinsics().width - 1;
    depth_image_height_minus_1_ = depth_camera_ptr_->intrinsics().height - 1;
  }
}

bool RegionModality::PrecalculateModelVariables() {
  float stride = region_model_ptr_->stride_depth_offset();
  float max_radius = region_model_ptr_->max_radius_depth_offset();
  if (measure_occlusions_) {
    if (measured_depth_offset_radius_ > max_radius) {
      std::cerr << "Measured depth offset radius too large: "
                << measured_depth_offset_radius_ << " > " << max_radius
                << std::endl;
      return false;
    }
    measured_depth_offset_id_ =
        int(measured_depth_offset_radius_ / stride + 0.5f);
  }
  if (model_occlusions_) {
    if (modeled_depth_offset_radius_ > max_radius) {
      std::cerr << "Modeled depth offset radius too large: "
                << modeled_depth_offset_radius_ << " > " << max_radius
                << std::endl;
      return false;
    }
    modeled_depth_offset_id_ =
        int(modeled_depth_offset_radius_ / stride + 0.5f);
  }
  return true;
}

void RegionModality::PrecalculateRendererVariables() {
  if (model_occlusions_) {
    depth_image_size_minus_1_ = depth_renderer_ptr_->image_size() - 1;
  }
  if (use_region_checking_) {
    fsilhouette_image_size_ = float(silhouette_renderer_ptr_->image_size());
  }
}

void RegionModality::PrecalculatePoseVariables() {
  body2camera_pose_ =
      color_camera_ptr_->world2camera_pose() * body_ptr_->body2world_pose();
  if (measure_occlusions_) {
    body2depth_camera_pose_ =
        depth_camera_ptr_->world2camera_pose() * body_ptr_->body2world_pose();
  }
  body2camera_rotation_ = body2camera_pose_.rotation().matrix();
  body2camera_rotation_xy_ = body2camera_rotation_.topRows<2>();
}

void RegionModality::PrecalculateIterationDependentVariables(
    int corr_iteration) {
  scale_ = LastValidValue(scales_, corr_iteration);
  fscale_ = float(scale_);
  line_length_ = line_length_in_segments_ * scale_;
  line_length_minus_1_ = line_length_ - 1;
  line_length_minus_1_half_ = float(line_length_ - 1) * 0.5f;
  line_length_half_minus_1_ = float(line_length_) * 0.5f - 1.0f;

  float standard_deviation =
      LastValidValue(standard_deviations_, corr_iteration);
  variance_ = powf(standard_deviation, 2.0f);
}

void RegionModality::AddLinePixelColorsToTempHistograms(
    bool handle_occlusions) {
  const cv::Mat &image{color_camera_ptr_->image()};
  const RegionModel::View *view;
  region_model_ptr_->GetClosestView(body2camera_pose_, &view);

  // Check if body is visible and fetch images from renderers
  bool body_visible_depth;
  if (handle_occlusions && model_occlusions_) {
    body_visible_depth = depth_renderer_ptr_->IsBodyVisible(body_ptr_->name());
    if (body_visible_depth) depth_renderer_ptr_->FetchDepthImage();
  }
  bool body_visible_silhouette;
  if (use_region_checking_) {
    body_visible_silhouette =
        silhouette_renderer_ptr_->IsBodyVisible(body_ptr_->name());
    if (body_visible_silhouette)
      silhouette_renderer_ptr_->FetchSilhouetteImage();
  }

  // Scale number of lines with contour_length ratio
  int n_lines = n_lines_max_;
  if (use_adaptive_coverage_) {
    if (reference_contour_length_ > 0.0f)
      n_lines = n_lines_max_ * std::min(1.0f, view->contour_length /
                                                  reference_contour_length_);
    else
      n_lines = n_lines_max_ * view->contour_length /
                region_model_ptr_->max_contour_length();
  }
  if (n_lines > view->data_points.size()) {
    std::cerr << "Number of model points too small: "
              << view->data_points.size() << " < " << n_lines << std::endl;
    n_lines = view->data_points.size();
  }

  // Iterate over n_lines
  for (int i = 0; i < n_lines; ++i) {
    const auto &data_point{view->data_points[i]};

    // Calculate center in image coordinates
    Eigen::Vector3f center_f_camera{body2camera_pose_ *
                                    data_point.center_f_body};
    if (center_f_camera(2) <= 0.0f) continue;
    float center_u = center_f_camera(0) * fu_ / center_f_camera(2) + ppu_;
    float center_v = center_f_camera(1) * fv_ / center_f_camera(2) + ppv_;
    int i_center_u = int(center_u + 0.5f);
    int i_center_v = int(center_v + 0.5f);
    if (i_center_u < 0.0f || i_center_u > image_width_minus_1_ ||
        i_center_v < 0 || i_center_v > image_height_minus_1_)
      continue;

    // Handle occlusions
    if (handle_occlusions) {
      if (model_occlusions_ && body_visible_depth &&
          !IsLineUnoccludedModeled(
              center_u, center_v, center_f_camera(2),
              data_point.depth_offsets[modeled_depth_offset_id_]))
        continue;
      if (measure_occlusions_ &&
          !IsLineUnoccludedMeasured(
              data_point.center_f_body,
              data_point.depth_offsets[measured_depth_offset_id_]))
        continue;
    }

    // Region checking
    float length_f = max_considered_line_length_;
    float length_b = max_considered_line_length_;
    if (use_region_checking_ && body_visible_silhouette) {
      Eigen::Vector2f normal_f_camera{
          (body2camera_rotation_xy_ * data_point.normal_f_body).normalized()};
      DynamicRegionDistance(center_u, center_v, normal_f_camera(0),
                            normal_f_camera(1), &length_f, &length_b);
    }

    // Consider foreground and background distance
    float l_f = data_point.foreground_distance * fu_ / center_f_camera(2);
    float l_b = data_point.background_distance * fu_ / center_f_camera(2);
    length_f = std::fmin(length_f, l_f - 2.0f * unconsidered_line_length_);
    length_b = std::fmin(length_b, l_b - 2.0f * unconsidered_line_length_);

    // Define steps and projected considered line lengths
    Eigen::Vector2f normal{
        (body2camera_rotation_xy_ * data_point.normal_f_body).normalized()};
    float u_step, v_step;
    int projected_length_f, projected_length_b;
    float abs_normal_u = std::fabs(normal(0));
    float abs_normal_v = std::fabs(normal(1));
    if (abs_normal_u > abs_normal_v) {
      u_step = sgnf(normal(0));
      v_step = normal(1) / abs_normal_u;
      projected_length_f = int(length_f * abs_normal_u + 0.5f);
      projected_length_b = int(length_b * abs_normal_u + 0.5f);
    } else {
      u_step = normal(0) / abs_normal_v;
      v_step = sgnf(normal(1));
      projected_length_f = int(length_f * abs_normal_v + 0.5f);
      projected_length_b = int(length_b * abs_normal_v + 0.5f);
    }

    // Iterate over foreground pixels
    float u = center_u - normal(0) * unconsidered_line_length_ + 0.5f;
    float v = center_v - normal(1) * unconsidered_line_length_ + 0.5f;
    int i_u, i_v;
    for (int i = 0; i < projected_length_f; ++i) {
      i_u = int(u);
      i_v = int(v);
      if (i_u < 0 || i_u > image_width_minus_1_ || i_v < 0 ||
          i_v > image_height_minus_1_)
        break;
      color_histograms_ptr_->AddForegroundColor(image.at<cv::Vec3b>(i_v, i_u));
      u -= u_step;
      v -= v_step;
    }

    // Iterate over background pixels
    u = center_u + normal(0) * unconsidered_line_length_ + 0.5f;
    v = center_v + normal(1) * unconsidered_line_length_ + 0.5f;
    for (int i = 0; i < projected_length_b; ++i) {
      i_u = int(u);
      i_v = int(v);
      if (i_u < 0 || i_u > image_width_minus_1_ || i_v < 0 ||
          i_v > image_height_minus_1_)
        break;
      color_histograms_ptr_->AddBackgroundColor(image.at<cv::Vec3b>(i_v, i_u));
      u += u_step;
      v += v_step;
    }
  }
}

void RegionModality::DynamicRegionDistance(
    float center_u, float center_v, float normal_u, float normal_v,
    float *dynamic_foreground_distance,
    float *dynamic_background_distance) const {
  const cv::Mat &silhouette_image{
      silhouette_renderer_ptr_->focused_silhouette_image()};
  uchar region_id = body_ptr_->region_id();

  // Precalculate variables in pixel coordinates of focused image
  float stride = max_considered_line_length_ / float(kNRegionStride);
  float focused_stride = stride * silhouette_renderer_ptr_->scale();
  float focused_stride_u = focused_stride * normal_u;
  float focused_stride_v = focused_stride * normal_v;

  // Calculate index for first iteration
  float delta_start = kRegionOffset / silhouette_renderer_ptr_->scale() -
                      unconsidered_line_length_;
  int i_start = std::max(int(delta_start / stride + 1.0f), 0);

  // Calculate offset based on start index and unconsidered line length
  float offset = unconsidered_line_length_ + float(i_start) * stride;
  float focused_offset = offset * silhouette_renderer_ptr_->scale();
  float focused_offset_u = focused_offset * normal_u;
  float focused_offset_v = focused_offset * normal_v;

  // Calculate start points
  float focused_center_u =
      0.5f + (center_u - silhouette_renderer_ptr_->corner_u()) *
                 silhouette_renderer_ptr_->scale();
  float focused_center_v =
      0.5f + (center_v - silhouette_renderer_ptr_->corner_v()) *
                 silhouette_renderer_ptr_->scale();

  // Calculate dynamic foregrund distance
  float u = focused_center_u - focused_offset_u;
  float v = focused_center_v - focused_offset_v;
  for (int i = i_start; i <= kNRegionStride; ++i) {
    if (u >= fsilhouette_image_size_ || u < 0.0f ||
        v >= fsilhouette_image_size_ || v < 0.0f) {
      *dynamic_foreground_distance = stride * float(i);
      break;
    }
    if (silhouette_image.at<uchar>(int(v), int(u)) != region_id) {
      if (i == i_start)
        *dynamic_foreground_distance = 0.0f;
      else
        *dynamic_foreground_distance = stride * float(i);
      break;
    }
    u -= focused_stride_u;
    v -= focused_stride_v;
  }

  // Check background region
  u = focused_center_u + focused_offset_u;
  v = focused_center_v + focused_offset_v;
  for (int i = i_start; i <= kNRegionStride; ++i) {
    if (u >= fsilhouette_image_size_ || u < 0.0f ||
        v >= fsilhouette_image_size_ || v < 0.0f) {
      *dynamic_background_distance = max_considered_line_length_;
      break;
    }
    if (silhouette_image.at<uchar>(int(v), int(u)) == region_id) {
      if (i == i_start)
        *dynamic_background_distance = 0.0f;
      else
        *dynamic_foreground_distance = stride * float(i);
      break;
    }
    u += focused_stride_u;
    v += focused_stride_v;
  }
}

void RegionModality::CalculateBasicLineData(
    const RegionModel::DataPoint &data_point, DataLine *data_line) const {
  Eigen::Vector3f center_f_camera{body2camera_pose_ * data_point.center_f_body};
  Eigen::Vector2f normal_f_camera{
      (body2camera_rotation_xy_ * data_point.normal_f_body).normalized()};

  data_line->center_f_body = data_point.center_f_body;
  data_line->center_f_camera = center_f_camera;
  data_line->center_u = center_f_camera(0) * fu_ / center_f_camera(2) + ppu_;
  data_line->center_v = center_f_camera(1) * fv_ / center_f_camera(2) + ppv_;
  data_line->normal_u = normal_f_camera(0);
  data_line->normal_v = normal_f_camera(1);
  data_line->measured_depth_offset =
      data_point.depth_offsets[measured_depth_offset_id_];
  data_line->modeled_depth_offset =
      data_point.depth_offsets[modeled_depth_offset_id_];
  data_line->continuous_distance =
      std::min(data_point.background_distance, data_point.foreground_distance) *
      fu_ / (center_f_camera(2) * fscale_);
}

bool RegionModality::IsLineValid(const DataLine &data_line,
                                 bool use_region_checking,
                                 bool measure_occlusions,
                                 bool model_occlusions) const {
  // Check if continuous distance is long enough
  if (data_line.continuous_distance < min_continuous_distance_) return false;

  // Check if point is in front of image
  if (data_line.center_f_camera(2) <= 0.0f) return false;

  // Check if image coordinate is on image
  int i_center_u = int(data_line.center_u + 0.5f);
  int i_center_v = int(data_line.center_v + 0.5f);
  if (i_center_u < 0 || i_center_u > image_width_minus_1_ || i_center_v < 0 ||
      i_center_v > image_height_minus_1_)
    return false;

  // Check dynamic region size
  if (use_region_checking) {
    if (!IsDynamicLineRegionSufficient(data_line.center_u, data_line.center_v,
                                       data_line.normal_u, data_line.normal_v))
      return false;
  }

  // Check measured occlusions
  if (measure_occlusions) {
    if (!IsLineUnoccludedMeasured(data_line.center_f_body,
                                  data_line.measured_depth_offset))
      return false;
  }

  // Check modeled occlusions
  if (model_occlusions) {
    if (!IsLineUnoccludedModeled(data_line.center_u, data_line.center_v,
                                 data_line.center_f_camera(2),
                                 data_line.modeled_depth_offset))
      return false;
  }
  return true;
}

bool RegionModality::IsDynamicLineRegionSufficient(float center_u,
                                                   float center_v,
                                                   float normal_u,
                                                   float normal_v) const {
  const cv::Mat &silhouette_image{
      silhouette_renderer_ptr_->focused_silhouette_image()};
  uchar region_id = body_ptr_->region_id();

  // Precalculate variables in pixel coordinates of focused image
  float focused_min_continuous_distance =
      min_continuous_distance_ * fscale_ * silhouette_renderer_ptr_->scale();
  float focused_stride = std::max(
      (focused_min_continuous_distance - kRegionOffset) / float(kNRegionStride),
      0.0f);
  float stride_u = focused_stride * normal_u;
  float stride_v = focused_stride * normal_v;
  float offset_u = kRegionOffset * normal_u;
  float offset_v = kRegionOffset * normal_v;

  // Calculate start points
  float focused_center_u =
      0.5f + (center_u - silhouette_renderer_ptr_->corner_u()) *
                 silhouette_renderer_ptr_->scale();
  float focused_center_v =
      0.5f + (center_v - silhouette_renderer_ptr_->corner_v()) *
                 silhouette_renderer_ptr_->scale();

  // Check foreground region
  float u = focused_center_u - offset_u;
  float v = focused_center_v - offset_v;
  for (int i = 0; i <= kNRegionStride; ++i) {
    if (silhouette_image.at<uchar>(int(v), int(u)) != region_id) return false;
    u -= stride_u;
    v -= stride_v;
  }

  // Check background region
  u = focused_center_u + offset_u;
  v = focused_center_v + offset_v;
  for (int i = 0; i <= kNRegionStride; ++i) {
    if (u >= fsilhouette_image_size_ || u < 0.0f ||
        v >= fsilhouette_image_size_ || v < 0.0f)
      break;
    if (silhouette_image.at<uchar>(int(v), int(u)) == region_id) return false;
    u += stride_u;
    v += stride_v;
  }
  return true;
}

bool RegionModality::IsLineUnoccludedMeasured(
    const Eigen::Vector3f &center_f_body, float depth_offset) const {
  // Calculate center on depth image
  Eigen::Vector3f center_f_depth_camera{body2depth_camera_pose_ *
                                        center_f_body};
  float center_u =
      center_f_depth_camera(0) * depth_fu_ / center_f_depth_camera(2) +
      depth_ppu_;
  float center_v =
      center_f_depth_camera(1) * depth_fv_ / center_f_depth_camera(2) +
      depth_ppv_;

  // Precalculate variables in pixel coordinates
  float meter_to_pixel = depth_fu_ / center_f_depth_camera(2);
  float diameter = 2.0f * measured_occlusion_radius_ * meter_to_pixel;
  int stride = int(diameter / kMaxNOcclusionStrides + 1.0f);
  int n_strides = int(diameter / stride + 0.5f);
  int rounded_diameter = n_strides * stride;
  float rounded_radius = 0.5f * float(rounded_diameter);

  // Calculate limits for iteration
  int u_min = int(center_u - rounded_radius + 0.5f);
  int v_min = int(center_v - rounded_radius + 0.5f);
  int u_max = u_min + rounded_diameter;
  int v_max = v_min + rounded_diameter;
  u_min = std::max(u_min, 0);
  v_min = std::max(v_min, 0);
  u_max = std::min(u_max, depth_image_width_minus_1_);
  v_max = std::min(v_max, depth_image_height_minus_1_);

  // Iterate over pixels to check if depth value is big enough
  ushort depth;
  ushort min_depth = ushort((center_f_depth_camera(2) - depth_offset -
                             measured_occlusion_threshold_) /
                            depth_scale_);
  int u, v;
  const cv::Mat &image{depth_camera_ptr_->image()};
  const ushort *ptr_image;
  for (v = v_min; v <= v_max; v += stride) {
    ptr_image = image.ptr<ushort>(v);
    for (u = u_min; u <= u_max; u += stride) {
      depth = ptr_image[u];
      if (depth > 0 && depth < min_depth) return false;
    }
  }
  return true;
}

bool RegionModality::IsLineUnoccludedModeled(float center_u, float center_v,
                                             float depth,
                                             float depth_offset) const {
  // Precalculate variables in pixel coordinates of focused image
  float meter_to_pixel = (fu_ / depth) * depth_renderer_ptr_->scale();
  float diameter = 2.0f * modeled_occlusion_radius_ * meter_to_pixel;
  int stride = int(diameter / kMaxNOcclusionStrides + 1.0f);
  int n_strides = int(diameter / stride + 0.5f);
  int rounded_diameter = n_strides * stride;
  float rounded_radius = 0.5f * float(rounded_diameter);

  // Calculate limits for iteration in focused image
  float focused_center_u = (center_u - depth_renderer_ptr_->corner_u()) *
                           depth_renderer_ptr_->scale();
  float focused_center_v = (center_v - depth_renderer_ptr_->corner_v()) *
                           depth_renderer_ptr_->scale();
  int u_min = int(focused_center_u - rounded_radius + 0.5f);
  int v_min = int(focused_center_v - rounded_radius + 0.5f);
  int u_max = u_min + rounded_diameter;
  int v_max = v_min + rounded_diameter;
  u_min = std::max(u_min, 0);
  v_min = std::max(v_min, 0);
  u_max = std::min(u_max, depth_image_size_minus_1_);
  v_max = std::min(v_max, depth_image_size_minus_1_);

  // Find minimum depth value
  ushort min_depth_value = std::numeric_limits<ushort>::max();
  int u, v;
  const cv::Mat &image{depth_renderer_ptr_->focused_depth_image()};
  const ushort *ptr_image;
  for (v = v_min; v <= v_max; v += stride) {
    ptr_image = image.ptr<ushort>(v);
    for (u = u_min; u <= u_max; u += stride) {
      min_depth_value = std::min(min_depth_value, ptr_image[u]);
    }
  }

  float min_depth = depth_renderer_ptr_->Depth(min_depth_value);
  float min_allowed_depth = depth - depth_offset - modeled_occlusion_threshold_;
  return min_depth > min_allowed_depth;
}

bool RegionModality::CalculateSegmentProbabilities(
    float center_u, float center_v, float normal_u, float normal_v,
    std::vector<float> *segment_probabilities_f,
    std::vector<float> *segment_probabilities_b,
    float *normal_component_to_scale, float *delta_r) const {
  const cv::Mat &image{color_camera_ptr_->image()};

  // Select case if line is more horizontal or vertical
  if (std::fabs(normal_v) < std::fabs(normal_u)) {
    // Calculate step and starting position
    float v_step = normal_v / normal_u;
    // Notice: u = int(center_u - (line_length / 2 - 0.5) + 0.5)
    int u = int(center_u - line_length_half_minus_1_);
    int u_end = u + line_length_minus_1_;
    float v_f = center_v + v_step * (float(u) - center_u) + 0.5f;
    float v_f_end = v_f + v_step * float(line_length_minus_1_);

    // Check if line is on image (margin of 1 for rounding errors of v_f_end)
    if (u < 0 || u_end > image_width_minus_1_ || int(v_f) < 0 ||
        int(v_f) > image_height_minus_1_ || int(v_f_end) < 1 ||
        int(v_f_end) > image_height_minus_2_) {
      return false;
    }

    // Iterate over all pixels of line and calculate probabilities
    if (normal_u > 0) {
      float *segment_probability_f = segment_probabilities_f->data();
      float *segment_probability_b = segment_probabilities_b->data();
      *segment_probability_f = 1.0f;
      *segment_probability_b = 1.0f;
      int segment_idx = 0;
      for (; u <= u_end; ++u, v_f += v_step, segment_idx++) {
        if (segment_idx == scale_) {
          *(++segment_probability_f) = 1.0f;
          *(++segment_probability_b) = 1.0f;
          segment_idx = 0;
        }
        MultiplyPixelColorProbability(image.at<cv::Vec3b>(int(v_f), u),
                                      segment_probability_f,
                                      segment_probability_b);
      }
    } else {
      float *segment_probability_f = &segment_probabilities_f->back();
      float *segment_probability_b = &segment_probabilities_b->back();
      *segment_probability_f = 1.0f;
      *segment_probability_b = 1.0f;
      int segment_idx = 0;
      for (; u <= u_end; ++u, v_f += v_step, ++segment_idx) {
        if (segment_idx == scale_) {
          *(--segment_probability_f) = 1.0f;
          *(--segment_probability_b) = 1.0f;
          segment_idx = 0;
        }
        MultiplyPixelColorProbability(image.at<cv::Vec3b>(int(v_f), u),
                                      segment_probability_f,
                                      segment_probability_b);
      }
    }

    // define dominant normal component and calculate delta_r
    *normal_component_to_scale = std::fabs(normal_u) / fscale_;
    *delta_r = (std::round(center_u - line_length_minus_1_half_) +
                line_length_minus_1_half_ - center_u) /
               normal_u;
  } else {
    // Calculate step and starting position
    float u_step = normal_u / normal_v;
    // Notice: v = int(center_v - (line_length / 2 - 0.5) + 0.5)
    int v = int(center_v - line_length_half_minus_1_);
    int v_end = v + line_length_minus_1_;
    float u_f = center_u + u_step * (float(v) - center_v) + 0.5f;
    float u_f_end = u_f + u_step * float(line_length_minus_1_);

    // Check if line is on image (margin of 1 for rounding errors of u_f_end)
    if (v < 0 || v_end > image_height_minus_1_ || int(u_f) < 0 ||
        int(u_f) > image_width_minus_1_ || int(u_f_end) < 1 ||
        int(u_f_end) > image_width_minus_2_) {
      return false;
    }

    // Iterate over all pixels of line and calculate probabilities
    if (normal_v > 0) {
      float *segment_probability_f = segment_probabilities_f->data();
      float *segment_probability_b = segment_probabilities_b->data();
      *segment_probability_f = 1.0f;
      *segment_probability_b = 1.0f;
      int segment_idx = 0;
      for (; v <= v_end; ++v, u_f += u_step, ++segment_idx) {
        if (segment_idx == scale_) {
          *(++segment_probability_f) = 1.0f;
          *(++segment_probability_b) = 1.0f;
          segment_idx = 0;
        }
        MultiplyPixelColorProbability(image.at<cv::Vec3b>(v, int(u_f)),
                                      segment_probability_f,
                                      segment_probability_b);
      }
    } else {
      float *segment_probability_f = &segment_probabilities_f->back();
      float *segment_probability_b = &segment_probabilities_b->back();
      *segment_probability_f = 1.0f;
      *segment_probability_b = 1.0f;
      int segment_idx = 0;
      for (; v <= v_end; ++v, u_f += u_step, ++segment_idx) {
        if (segment_idx == scale_) {
          *(--segment_probability_f) = 1.0f;
          *(--segment_probability_b) = 1.0f;
          segment_idx = 0;
        }
        MultiplyPixelColorProbability(image.at<cv::Vec3b>(v, int(u_f)),
                                      segment_probability_f,
                                      segment_probability_b);
      }
    }

    // define normal component and calculate delta_r
    *normal_component_to_scale = std::fabs(normal_v) / fscale_;
    *delta_r = (std::round(center_v - line_length_minus_1_half_) +
                line_length_minus_1_half_ - center_v) /
               normal_v;
  }

  // Normalize segment probabilities
  if (scale_ > 1) {
    auto segment_probability_f = begin(*segment_probabilities_f);
    auto segment_probability_b = begin(*segment_probabilities_b);
    for (; segment_probability_f != end(*segment_probabilities_f);
         ++segment_probability_f, ++segment_probability_b) {
      if (*segment_probability_f || *segment_probability_b) {
        float sum = *segment_probability_f;
        sum += *segment_probability_b;
        *segment_probability_f /= sum;
        *segment_probability_b /= sum;
      } else {
        *segment_probability_f = 0.5f;
        *segment_probability_b = 0.5f;
      }
    }
  }
  return true;
}

void RegionModality::MultiplyPixelColorProbability(const cv::Vec3b &pixel_color,
                                                   float *probability_f,
                                                   float *probability_b) const {
  // Retrieve pixel color probability values
  float pixel_color_probability_f;
  float pixel_color_probability_b;
  color_histograms_ptr_->GetProbabilities(
      pixel_color, &pixel_color_probability_f, &pixel_color_probability_b);

  // Normalize pixel color probability values
  if (pixel_color_probability_f || pixel_color_probability_b) {
    float sum = pixel_color_probability_f;
    sum += pixel_color_probability_b;
    pixel_color_probability_f /= sum;
    pixel_color_probability_b /= sum;
  } else {
    pixel_color_probability_f = 0.5f;
    pixel_color_probability_b = 0.5f;
  }

  // Multiply pixel color probability values
  *probability_f *= pixel_color_probability_f;
  *probability_b *= pixel_color_probability_b;
}

void RegionModality::CalculateDistribution(
    const std::vector<float> &segment_probabilities_f,
    const std::vector<float> &segment_probabilities_b,
    std::vector<float> *distribution) const {
  std::vector<float>::const_iterator segment_probabilities_f_it;
  std::vector<float>::const_iterator segment_probabilities_b_it;
  std::vector<float>::const_iterator function_lookup_f_it;
  std::vector<float>::const_iterator function_lookup_b_it;
  distribution->resize(distribution_length_);
  float distribution_area = 0.0f;

  // Loop over entire distribution and start values of segment probabilities
  auto segment_probabilities_f_it_start = begin(segment_probabilities_f);
  auto segment_probabilities_b_it_start = begin(segment_probabilities_b);
  for (auto distribution_it = begin(*distribution);
       distribution_it != end(*distribution);
       ++distribution_it, ++segment_probabilities_f_it_start,
            ++segment_probabilities_b_it_start) {
    *distribution_it = 1.0f;
    // Loop over values of segment probabilities and corresponding lookup values
    segment_probabilities_f_it = segment_probabilities_f_it_start;
    segment_probabilities_b_it = segment_probabilities_b_it_start;
    function_lookup_f_it = begin(function_lookup_f_);
    function_lookup_b_it = begin(function_lookup_b_);
    for (; function_lookup_f_it != end(function_lookup_f_);
         ++function_lookup_f_it, ++function_lookup_b_it,
         ++segment_probabilities_f_it, ++segment_probabilities_b_it) {
      *distribution_it *= *segment_probabilities_f_it * *function_lookup_f_it +
                          *segment_probabilities_b_it * *function_lookup_b_it;
    }
    distribution_area += *distribution_it;
  }

  // Normalize distribution
  for (auto &probability_distribution : *distribution) {
    probability_distribution /= distribution_area;
  }
}

void RegionModality::CalculateDistributionMoments(
    const std::vector<float> &distribution, float *mean,
    float *variance) const {
  // Calculate mean from the beginning of the distribution
  float mean_from_begin = 0.0f;
  for (int i = 0; i < distribution_length_; ++i) {
    mean_from_begin += float(i) * distribution[i];
  }

  // Calculate variance
  float distribution_variance = 0.0f;
  for (int i = 0; i < distribution_length_; ++i) {
    distribution_variance +=
        powf(float(i) - mean_from_begin, 2.0f) * distribution[i];
  }

  // Calculate moments
  *mean = mean_from_begin - distribution_length_minus_1_half_;
  *variance = std::max(distribution_variance, min_expected_variance_);
}

void RegionModality::ShowAndSaveImage(const std::string &title, int save_index,
                                      const cv::Mat &image) const {
  if (display_visualization_) cv::imshow(title, image);
  if (save_visualizations_) {
    std::filesystem::path path{
        save_directory_ /
        (title + "_" + std::to_string(save_index) + "." + save_image_type_)};
    cv::imwrite(path.string(), image);
  }
}

void RegionModality::VisualizePointsColorImage(const std::string &title,
                                               int save_index) const {
  cv::Mat visualization_image;
  color_camera_ptr_->image().copyTo(visualization_image);
  DrawPoints(cv::Vec3b{24, 184, 234}, &visualization_image);
  ShowAndSaveImage(name_ + "_" + title, save_index, visualization_image);
}

void RegionModality::VisualizePointsDepthImage(const std::string &title,
                                               int save_index) const {
  cv::Mat visualization_image;
  cv::cvtColor(depth_camera_ptr_->NormalizedDepthImage(
                   visualization_min_depth_, visualization_max_depth_),
               visualization_image, cv::COLOR_GRAY2BGR);
  DrawDepthPoints(cv::Vec3b{24, 184, 234}, &visualization_image);
  ShowAndSaveImage(name_ + "_" + title, save_index, visualization_image);
}

void RegionModality::VisualizePointsDepthRendering(const std::string &title,
                                                   int save_index) const {
  cv::Mat visualization_image;
  depth_renderer_ptr_->FetchDepthImage();
  cv::cvtColor(depth_renderer_ptr_->NormalizedFocusedDepthImage(
                   visualization_min_depth_, visualization_max_depth_),
               visualization_image, cv::COLOR_GRAY2BGR);
  DrawFocusedPoints(depth_renderer_ptr_, cv::Vec3b{24, 184, 234},
                    &visualization_image);
  ShowAndSaveImage(name_ + "_" + title, save_index, visualization_image);
}

void RegionModality::VisualizePointsSilhouetteRendering(
    const std::string &title, int save_index) const {
  cv::Mat visualization_image;
  silhouette_renderer_ptr_->FetchSilhouetteImage();
  cv::cvtColor(silhouette_renderer_ptr_->focused_silhouette_image(),
               visualization_image, cv::COLOR_GRAY2BGR);
  DrawFocusedPoints(silhouette_renderer_ptr_, cv::Vec3b{24, 184, 234},
                    &visualization_image);
  ShowAndSaveImage(name_ + "_" + title, save_index, visualization_image);
}

void RegionModality::VisualizePointsHistogramImage(const std::string &title,
                                                   int save_index) const {
  cv::Mat visualization_image(color_camera_ptr_->image().size(), CV_8UC3);
  DrawProbabilityImage(cv::Vec3b{255, 255, 255}, &visualization_image);
  DrawPoints(cv::Vec3b{24, 184, 234}, &visualization_image);
  ShowAndSaveImage(name_ + "_" + title, save_index, visualization_image);
}

void RegionModality::VisualizeLines(const std::string &title,
                                    int save_index) const {
  cv::Mat visualization_image(color_camera_ptr_->image().size(), CV_8UC3);
  DrawProbabilityImage(cv::Vec3b{255, 255, 255}, &visualization_image);
  DrawLines(cv::Vec3b{24, 184, 234}, cv::Vec3b{61, 63, 179},
            &visualization_image);
  ShowAndSaveImage(name_ + "_" + title, save_index, visualization_image);
}

void RegionModality::DrawPoints(const cv::Vec3b &color_point,
                                cv::Mat *image) const {
  for (const auto &data_line : data_lines_) {
    DrawPointInImage(body2camera_pose_ * data_line.center_f_body, color_point,
                     color_camera_ptr_->intrinsics(), image);
  }
}

void RegionModality::DrawDepthPoints(const cv::Vec3b &color_point,
                                     cv::Mat *image) const {
  for (const auto &data_line : data_lines_) {
    DrawPointInImage(body2depth_camera_pose_ * data_line.center_f_body,
                     color_point, depth_camera_ptr_->intrinsics(), image);
  }
}

void RegionModality::DrawFocusedPoints(
    const std::shared_ptr<FocusedRenderer> &renderer_ptr,
    const cv::Vec3b &color_point, cv::Mat *image) const {
  for (const auto &data_line : data_lines_) {
    DrawFocusedPointInImage(body2camera_pose_ * data_line.center_f_body,
                            color_point, renderer_ptr->intrinsics(),
                            renderer_ptr->corner_u(), renderer_ptr->corner_v(),
                            renderer_ptr->scale(), image);
  }
}

void RegionModality::DrawLines(const cv::Vec3b &color_line,
                               const cv::Vec3b &color_high_probability,
                               cv::Mat *image) const {
  float scale_minus_1_half_ = (fscale_ - 1.0f) / 2.0f;
  float x, u, v, u_step, v_step;
  for (const auto &data_line : data_lines_) {
    if (std::fabs(data_line.normal_u) > std::fabs(data_line.normal_v)) {
      u_step = sgnf(data_line.normal_u);
      v_step = data_line.normal_v / std::fabs(data_line.normal_u);
    } else {
      u_step = data_line.normal_u / std::fabs(data_line.normal_v);
      v_step = sgnf(data_line.normal_v);
    }

    x = -fscale_ * distribution_length_minus_1_half_ - scale_minus_1_half_;
    u = data_line.center_u + u_step * x + 0.5f;
    v = data_line.center_v + v_step * x + 0.5f;
    for (int i = 0; i < distribution_length_; ++i) {
      for (int j = 0; j < scale_; ++j) {
        float color_ratio = std::min(3 * data_line.distribution[i], 1.0f);
        image->at<cv::Vec3b>(int(v), int(u)) =
            color_ratio * color_high_probability +
            (1.0f - color_ratio) * color_line;
        u += u_step;
        v += v_step;
      }
    }
  }
}

void RegionModality::DrawProbabilityImage(const cv::Vec3b &color_b,
                                          cv::Mat *probability_image) const {
  const cv::Mat &color_image{color_camera_ptr_->image()};
  float pixel_probability_f, pixel_probability_b;
  const cv::Vec3b *color_image_value;
  cv::Vec3b *probability_image_value;
  for (int v = 0; v < color_image.rows; ++v) {
    color_image_value = color_image.ptr<cv::Vec3b>(v);
    probability_image_value = probability_image->ptr<cv::Vec3b>(v);
    for (int u = 0; u < color_image.cols; ++u) {
      pixel_probability_f = 1.0f;
      pixel_probability_b = 1.0f;
      MultiplyPixelColorProbability(color_image_value[u], &pixel_probability_f,
                                    &pixel_probability_b);
      probability_image_value[u] = color_b * pixel_probability_b;
    }
  }
}

float RegionModality::MinAbsValueWithSignOfValue1(float value_1,
                                                  float abs_value_2) {
  if (std::abs(value_1) < abs_value_2)
    return value_1;
  else
    return sgnf(value_1) * abs_value_2;
}

bool RegionModality::IsSetup() const {
  if (!set_up_) {
    std::cerr << "Set up region modality " << name_ << " first" << std::endl;
    return false;
  }
  return true;
}

}  // namespace m3t
