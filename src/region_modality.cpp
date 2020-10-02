// SPDX-License-Identifier: MIT
// Copyright (c) 2020 Manuel Stoiber, German Aerospace Center (DLR)

#include <rbgt/region_modality.h>

namespace rbgt {

RegionModality::RegionModality() {
  tikhonov_matrix_.setZero();
  tikhonov_matrix_.diagonal().head<3>().array() = tikhonov_parameter_rotation_;
  tikhonov_matrix_.diagonal().tail<3>().array() =
      tikhonov_parameter_translation_;
}

bool RegionModality::Init(const std::string &name,
                          std::shared_ptr<Body> body_ptr,
                          std::shared_ptr<Model> model_ptr,
                          std::shared_ptr<Camera> camera_ptr) {
  initialized_ = false;
  name_ = name;
  body_ptr_ = std::move(body_ptr);
  model_ptr_ = std::move(model_ptr);
  camera_ptr_ = std::move(camera_ptr);

  PrecalculateFunctionLookup();
  PrecalculateDistributionVariables();
  PrecalculateHistogramBinVariables();

  initialized_ = true;
  return true;
}

void RegionModality::set_n_points(int n_points) { n_points_ = n_points; }

void RegionModality::set_function_slope(float function_slope) {
  function_slope_ = function_slope;
  PrecalculateFunctionLookup();
  PrecalculateDistributionVariables();
}

void RegionModality::set_function_length(int function_length) {
  function_length_ = function_length;
  PrecalculateFunctionLookup();
  PrecalculateDistributionVariables();
}

void RegionModality::set_distribution_length(int distribution_length) {
  distribution_length_ = distribution_length;
  PrecalculateDistributionVariables();
}

void RegionModality::set_scales(const std::vector<int> &scales) {
  scales_ = scales;
}

void RegionModality::set_probability_threshold(float probability_threshold) {
  probability_threshold_ = probability_threshold;
}

void RegionModality::set_min_continuous_distance(
    float min_continuous_distance) {
  min_continuous_distance_ = min_continuous_distance;
}

void RegionModality::set_use_linear_function(bool use_linear_function) {
  use_linear_function_ = use_linear_function;
  PrecalculateFunctionLookup();
}

void RegionModality::set_use_const_variance(bool use_const_variance) {
  use_const_variance_ = use_const_variance;
}

bool RegionModality::set_n_histogram_bins(int n_histogram_bins) {
  switch (n_histogram_bins) {
    case 2:
      histogram_bitshift_ = 7;
      break;
    case 4:
      histogram_bitshift_ = 6;
      break;
    case 8:
      histogram_bitshift_ = 5;
      break;
    case 16:
      histogram_bitshift_ = 4;
      break;
    case 32:
      histogram_bitshift_ = 3;
      break;
    case 64:
      histogram_bitshift_ = 2;
      break;
    default:
      std::cerr << "n_histogram_bins = " << n_histogram_bins << " not valid. "
                << "Has to be of value 2, 4, 8, 16, 32, or 64" << std::endl;
      return false;
  }
  n_histogram_bins_ = n_histogram_bins;
  PrecalculateHistogramBinVariables();
  return true;
}

void RegionModality::set_learning_rate_f(float learning_rate_f) {
  learning_rate_f_ = learning_rate_f;
}

void RegionModality::set_learning_rate_b(float learning_rate_b) {
  learning_rate_b_ = learning_rate_b;
}

void RegionModality::set_unconsidered_line_length(
    float unconsidered_line_length) {
  unconsidered_line_length_ = unconsidered_line_length;
}

void RegionModality::set_considered_line_length(float considered_line_length) {
  considered_line_length_ = considered_line_length;
}

void RegionModality::set_tikhonov_parameter_rotation(
    float tikhonov_parameter_rotation) {
  tikhonov_parameter_rotation_ = tikhonov_parameter_rotation;
  tikhonov_matrix_.diagonal().head<3>().array() = tikhonov_parameter_rotation_;
}

void RegionModality::set_tikhonov_parameter_translation(
    float tikhonov_parameter_translation) {
  tikhonov_parameter_translation_ = tikhonov_parameter_translation;
  tikhonov_matrix_.diagonal().tail<3>().array() =
      tikhonov_parameter_translation_;
}

void RegionModality::UseOcclusionHandling(
    std::shared_ptr<OcclusionMaskRenderer> occlusion_mask_renderer_ptr) {
  occlusion_mask_renderer_ptr_ = std::move(occlusion_mask_renderer_ptr);
  use_occlusion_handling_ = true;
  SetImshowCorrespondence();
}

void RegionModality::DoNotUseOcclusionHandling() {
  occlusion_mask_renderer_ptr_ = nullptr;
  use_occlusion_handling_ = false;
  SetImshowCorrespondence();
}

void RegionModality::set_display_visualization(bool display_visualization) {
  display_visualization_ = display_visualization;
}

void RegionModality::StartSavingVisualizations(
    const std::filesystem::path &save_path) {
  save_visualizations_ = true;
  save_path_ = save_path;
}

void RegionModality::StopSavingVisualizations() {
  save_visualizations_ = false;
}

void RegionModality::set_visualize_lines_correspondence(
    bool visualize_lines_correspondence) {
  visualize_lines_correspondence_ = visualize_lines_correspondence;
  SetImshowCorrespondence();
}

void RegionModality::set_visualize_points_occlusion_mask_correspondence(
    bool visualize_points_occlusion_mask_correspondence) {
  visualize_points_occlusion_mask_correspondence_ =
      visualize_points_occlusion_mask_correspondence;
  SetImshowCorrespondence();
}

void RegionModality::set_visualize_points_pose_update(
    bool visualize_points_pose_update) {
  visualize_points_pose_update_ = visualize_points_pose_update;
  SetImshowPoseUpdate();
}

void RegionModality::set_visualize_points_histogram_image_pose_update(
    bool visualize_points_histogram_image_pose_update) {
  visualize_points_histogram_image_pose_update_ =
      visualize_points_histogram_image_pose_update;
  SetImshowPoseUpdate();
}

void RegionModality::set_visualize_points_result(bool visualize_points_result) {
  visualize_points_result_ = visualize_points_result;
  SetImshowResult();
}

void RegionModality::set_visualize_points_histogram_image_result(
    bool visualize_points_histogram_image_result) {
  visualize_points_histogram_image_result_ =
      visualize_points_histogram_image_result;
  SetImshowResult();
}

bool RegionModality::StartModality() {
  PrecalculateBodyVariables();
  PrecalculateCameraVariables();
  PrecalculatePoseVariables();

  // Initialize histograms
  AddLinePixelColorsToTempHistograms();
  if (CalculateHistogram(1.0f, temp_histogram_f_, &histogram_f_) &&
      CalculateHistogram(1.0f, temp_histogram_b_, &histogram_b_)) {
    return true;
  } else {
    std::cerr << "Histograms could not be initialised for modality " << name_
              << std::endl;
    return false;
  }
}

bool RegionModality::CalculateBeforeCameraUpdate() {
  PrecalculatePoseVariables();
  AddLinePixelColorsToTempHistograms();
  CalculateHistogram(learning_rate_f_, temp_histogram_f_, &histogram_f_);
  CalculateHistogram(learning_rate_b_, temp_histogram_b_, &histogram_b_);
  return true;
}

bool RegionModality::CalculateCorrespondences(int corr_iteration) {
  PrecalculatePoseVariables();
  PrecalculateScaleDependentVariables(corr_iteration);
  if (use_occlusion_handling_)
    occlusion_mask_renderer_ptr_->FetchOcclusionMask();

  // Search closest template view
  const Model::TemplateView *template_view;
  model_ptr_->GetClosestTemplateView(body2camera_pose_, &template_view);

  // Iterate over n_points
  std::vector<float> segment_probabilities_f(line_length_in_segments_);
  std::vector<float> segment_probabilities_b(line_length_in_segments_);
  data_lines_.clear();
  for (auto data_point = template_view->data_points.begin();
       data_point != template_view->data_points.begin() + n_points_;
       ++data_point) {
    DataLine data_line;
    CalculateBasicLineData(*data_point, &data_line);
    if (!IsLineValid(data_line.center_u, data_line.center_v,
                     data_line.continuous_distance))
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
                                 &data_line.standard_deviation,
                                 &data_line.variance);
    data_lines_.push_back(std::move(data_line));
  }
  return true;
}

bool RegionModality::VisualizeCorrespondences(int save_idx) {
  if (visualize_lines_correspondence_)
    VisualizeLines("lines_correspondence", save_idx);
  if (visualize_points_occlusion_mask_correspondence_ &&
      use_occlusion_handling_)
    VisualizePointsOcclusionMask("occlusion_mask_correspondence", save_idx);
  return true;
}

bool RegionModality::CalculatePoseUpdate() {
  PrecalculatePoseVariables();
  Eigen::Matrix<float, 6, 1> gradient;
  Eigen::Matrix<float, 6, 6> hessian;
  gradient.setZero();
  hessian.setZero();

  // Iterate over correspondence lines
  for (auto &data_line : data_lines_) {
    // Calculate point coordinates in camera frame
    data_line.center_f_camera = body2camera_pose_ * data_line.center_f_body;
    float x = data_line.center_f_camera[0];
    float y = data_line.center_f_camera[1];
    float z = data_line.center_f_camera[2];

    // Calculate delta_cs
    float fu_z = fu_ / z;
    float fv_z = fv_ / z;
    float xfu_z = x * fu_z;
    float yfv_z = y * fv_z;
    float delta_cs = (data_line.normal_u * (xfu_z + ppu_ - data_line.center_u) +
                      data_line.normal_v * (yfv_z + ppv_ - data_line.center_v) -
                      data_line.delta_r) *
                     data_line.normal_component_to_scale;

    // Calculate distribution indexes and delta
    // Notice: (distribution_length - 1) / 2 + 1 = (distribution_length + 1) / 2
    int dist_idx_upper = int(delta_cs + distribution_length_plus_1_half_);
    int dist_idx_lower = dist_idx_upper - 1;
    float delta_dist_idx_upper =
        float(dist_idx_upper) - (distribution_length_minus_1_half_ + delta_cs);
    float delta_dist_idx_lower = 1.0f - delta_dist_idx_upper;

    // Check if distritbution indexes are within range of distribution
    if (dist_idx_lower < 0 || dist_idx_upper >= distribution_length_) continue;

    // Calculate first derivative of loglikelihood with respect to delta_cs
    float dloglikelihood_ddelta_cs;
    if (data_line.distribution[dist_idx_upper] > probability_threshold_ &&
        data_line.distribution[dist_idx_lower] > probability_threshold_) {
      float delta_distribution_values = data_line.distribution[dist_idx_upper] -
                                        data_line.distribution[dist_idx_lower];
      float interpolated_distribution_value =
          data_line.distribution[dist_idx_lower] * delta_dist_idx_upper +
          data_line.distribution[dist_idx_upper] * delta_dist_idx_lower;
      dloglikelihood_ddelta_cs = MinAbsValueWithSignOfValue1(
          (delta_distribution_values / interpolated_distribution_value),
          max_abs_dloglikelihood_ddelta_cs_);
    } else {
      dloglikelihood_ddelta_cs =
          (data_line.mean - delta_cs) / data_line.variance;
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

    // Calculate gradient and hessian
    gradient += dloglikelihood_ddelta_cs * ddelta_cs_dtheta.transpose();
    ddelta_cs_dtheta /= data_line.standard_deviation;
    hessian.triangularView<Eigen::Lower>() -=
        ddelta_cs_dtheta.transpose() * ddelta_cs_dtheta;
  }
  hessian = hessian.selfadjointView<Eigen::Lower>();

  // Optimize and update pose
  Eigen::FullPivLU<Eigen::Matrix<float, 6, 6>> lu{tikhonov_matrix_ - hessian};
  if (lu.isInvertible()) {
    Eigen::Matrix<float, 6, 1> theta{lu.solve(gradient)};
    Transform3fA pose_variation{Transform3fA::Identity()};
    pose_variation.rotate(Vector2Skewsymmetric(theta.head<3>()).exp());
    pose_variation.translate(theta.tail<3>());
    body_ptr_->set_body2world_pose(body_ptr_->body2world_pose() *
                                   pose_variation);
  }
  return true;
}

bool RegionModality::VisualizePoseUpdate(int save_idx) {
  if (visualize_points_pose_update_) {
    UpdateLineCentersWithCurrentPose();
    VisualizePointsCameraImage("camera_image_pose_update", save_idx);
  }
  if (visualize_points_histogram_image_pose_update_) {
    UpdateLineCentersWithCurrentPose();
    VisualizePointsHistogramImage("histogram_image_pose_update", save_idx);
  }
  return true;
}

bool RegionModality::VisualizeResults(int save_idx) {
  if (visualize_points_result_) {
    UpdateLineCentersWithCurrentPose();
    VisualizePointsCameraImage("camera_image_result", save_idx);
  }
  if (visualize_points_histogram_image_result_) {
    UpdateLineCentersWithCurrentPose();
    VisualizePointsHistogramImage("histogram_image_result", save_idx);
  }
  return true;
}

const std::string &RegionModality::name() const { return name_; }

std::shared_ptr<Body> RegionModality::body_ptr() const { return body_ptr_; }

std::shared_ptr<Model> RegionModality::model_ptr() const { return model_ptr_; }

std::shared_ptr<Camera> RegionModality::camera_ptr() const {
  return camera_ptr_;
}

std::shared_ptr<OcclusionMaskRenderer>
RegionModality::occlusion_mask_renderer_ptr() const {
  return occlusion_mask_renderer_ptr_;
}

bool RegionModality::imshow_correspondence() const {
  return imshow_correspondence_;
}

bool RegionModality::imshow_pose_update() const { return imshow_pose_update_; }

bool RegionModality::imshow_result() const { return imshow_result_; }

bool RegionModality::initialized() const { return initialized_; }

void RegionModality::PrecalculateFunctionLookup() {
  function_lookup_f_.resize(function_length_);
  function_lookup_b_.resize(function_length_);
  for (int i = 0; i < function_length_; ++i) {
    float x = float(i) - float(function_length_ - 1) / 2.0f;
    if (use_linear_function_)
      function_lookup_f_[i] = 0.5f - x / float(function_length_ - 1);
    else
      function_lookup_f_[i] =
          0.5f - 0.5f * std::tanh(x / (2.0f * function_slope_));
    function_lookup_b_[i] = 1.0f - function_lookup_f_[i];
  }
}

void RegionModality::PrecalculateDistributionVariables() {
  line_length_in_segments_ = function_length_ + distribution_length_ - 1;
  distribution_length_minus_1_half_ =
      (float(distribution_length_) - 1.0f) / 2.0f;
  distribution_length_plus_1_half_ =
      (float(distribution_length_) + 1.0f) / 2.0f;
  max_abs_dloglikelihood_ddelta_cs_ =
      float(distribution_length_ - 1) / function_slope_;
}

void RegionModality::PrecalculateHistogramBinVariables() {
  n_histogram_bins_squared_ = pow(n_histogram_bins_, 2);
  n_histogram_bins_cubed_ = pow(n_histogram_bins_, 3);
  temp_histogram_f_.resize(n_histogram_bins_cubed_);
  temp_histogram_b_.resize(n_histogram_bins_cubed_);
  histogram_f_.resize(n_histogram_bins_cubed_);
  histogram_b_.resize(n_histogram_bins_cubed_);
}

void RegionModality::SetImshowCorrespondence() {
  imshow_correspondence_ = visualize_lines_correspondence_ ||
                           (visualize_points_occlusion_mask_correspondence_ &&
                            use_occlusion_handling_);
}

void RegionModality::SetImshowPoseUpdate() {
  imshow_pose_update_ = visualize_points_pose_update_ ||
                        visualize_points_histogram_image_pose_update_;
}

void RegionModality::SetImshowResult() {
  imshow_result_ =
      visualize_points_result_ || visualize_points_histogram_image_result_;
}

void RegionModality::PrecalculateBodyVariables() {
  if (use_occlusion_handling_)
    encoded_occlusion_mask_id_ =
        (uchar(1) << unsigned(body_ptr_->occlusion_mask_id()));
}

void RegionModality::PrecalculateCameraVariables() {
  fu_ = camera_ptr_->intrinsics().fu;
  fv_ = camera_ptr_->intrinsics().fv;
  ppu_ = camera_ptr_->intrinsics().ppu;
  ppv_ = camera_ptr_->intrinsics().ppv;
  image_width_minus_1_ = camera_ptr_->image().cols - 1;
  image_height_minus_1_ = camera_ptr_->image().rows - 1;
}

void RegionModality::PrecalculatePoseVariables() {
  body2camera_pose_ =
      camera_ptr_->world2camera_pose() * body_ptr_->body2world_pose();
  body2camera_rotation_ = body2camera_pose_.rotation().matrix();
  body2camera_rotation_xy_ = body2camera_rotation_.topRows<2>();
}

void RegionModality::PrecalculateScaleDependentVariables(int corr_iteration) {
  if (corr_iteration < int(scales_.size()))
    scale_ = scales_[corr_iteration];
  else
    scale_ = 1;
  fscale_ = float(scale_);
  line_length_ = line_length_in_segments_ * scale_;
  line_length_minus_1_ = line_length_ - 1;
  line_length_minus_1_half_ = float(line_length_ - 1) * 0.5f;
  line_length_half_minus_1_ = float(line_length_) * 0.5f - 1.0f;
}

void RegionModality::AddLinePixelColorsToTempHistograms() {
  const cv::Mat &image{camera_ptr_->image()};
  const Model::TemplateView *template_view;
  model_ptr_->GetClosestTemplateView(body2camera_pose_, &template_view);

  // Iterate over all points
  std::fill(begin(temp_histogram_f_), end(temp_histogram_f_), 0.0f);
  std::fill(begin(temp_histogram_b_), end(temp_histogram_b_), 0.0f);
  for (auto data_point = template_view->data_points.begin();
       data_point != template_view->data_points.begin() + n_points_;
       ++data_point) {
    // Project point data in camera frame
    Eigen::Vector3f center_f_camera{body2camera_pose_ *
                                    data_point->center_f_body};
    Eigen::Vector2f center{
        center_f_camera[0] * fu_ / center_f_camera[2] + ppu_,
        center_f_camera[1] * fv_ / center_f_camera[2] + ppv_};
    Eigen::Vector2f normal{
        (body2camera_rotation_xy_ * data_point->normal_f_body).normalized()};
    float foreground_distance =
        data_point->foreground_distance * fu_ / center_f_camera[2];
    float background_distance =
        data_point->background_distance * fu_ / center_f_camera[2];

    // Iterate over foreground pixels
    float u = center[0] - normal[0] * unconsidered_line_length_ + 0.5f;
    float v = center[1] - normal[1] * unconsidered_line_length_ + 0.5f;
    int n_iteration =
        int(std::fmin(foreground_distance - unconsidered_line_length_,
                      considered_line_length_) +
            0.5f);
    for (int i = 0; i < n_iteration; ++i) {
      if (int(u) < 0 || int(u) > image_width_minus_1_ || int(v) < 0 ||
          int(v) > image_height_minus_1_)
        break;
      AddPixelColorToHistogram(image.at<cv::Vec3b>(int(v), int(u)),
                               &temp_histogram_f_);
      u -= normal[0];
      v -= normal[1];
    }

    // Iterate over background pixels
    u = center[0] + normal[0] * unconsidered_line_length_ + 0.5f;
    v = center[1] + normal[1] * unconsidered_line_length_ + 0.5f;
    n_iteration = int(std::fmin(background_distance - unconsidered_line_length_,
                                considered_line_length_) +
                      0.5f);
    for (int i = 0; i < n_iteration; ++i) {
      if (int(u) < 0 || int(u) > image_width_minus_1_ || int(v) < 0 ||
          int(v) > image_height_minus_1_)
        break;
      AddPixelColorToHistogram(image.at<cv::Vec3b>(int(v), int(u)),
                               &temp_histogram_b_);
      u += normal[0];
      v += normal[1];
    }
  }
}

void RegionModality::AddPixelColorToHistogram(
    const cv::Vec3b &pixel_color,
    std::vector<float> *enlarged_histogram) const {
  (*enlarged_histogram)[(pixel_color[0] >> histogram_bitshift_) *
                            n_histogram_bins_squared_ +
                        (pixel_color[1] >> histogram_bitshift_) *
                            n_histogram_bins_ +
                        (pixel_color[2] >> histogram_bitshift_)] += 1.0f;
}

bool RegionModality::CalculateHistogram(
    float learning_rate, const std::vector<float> &temp_histogram,
    std::vector<float> *histogram) {
  // Calculate sum for normalization
  float sum = 0.0f;
#ifndef _DEBUG
#pragma omp simd
#endif
  for (int i = 0; i < n_histogram_bins_cubed_; i++) {
    sum += temp_histogram[i];
  }
  if (!sum) return false;

  // Calculate histogram
  float complement_learning_rate = 1.0f - learning_rate;
  float learning_rate_divide_sum = learning_rate / sum;
#ifndef _DEBUG
#pragma omp simd
#endif
  for (int i = 0; i < n_histogram_bins_cubed_; i++) {
    (*histogram)[i] *= complement_learning_rate;
    (*histogram)[i] += temp_histogram[i] * learning_rate_divide_sum;
  }
  return true;
}

void RegionModality::CalculateBasicLineData(const Model::PointData &data_point,
                                            DataLine *data_line) const {
  Eigen::Vector3f center_f_camera{body2camera_pose_ * data_point.center_f_body};
  Eigen::Vector2f normal_f_camera{
      (body2camera_rotation_xy_ * data_point.normal_f_body).normalized()};

  data_line->center_f_body = data_point.center_f_body;
  data_line->center_f_camera = center_f_camera;
  data_line->center_u = center_f_camera[0] * fu_ / center_f_camera[2] + ppu_;
  data_line->center_v = center_f_camera[1] * fv_ / center_f_camera[2] + ppv_;
  data_line->normal_u = normal_f_camera[0];
  data_line->normal_v = normal_f_camera[1];
  data_line->continuous_distance =
      std::min(data_point.background_distance, data_point.foreground_distance) *
      fu_ / (center_f_camera[2] * fscale_);
}

bool RegionModality::IsLineValid(float u, float v,
                                 float continuous_distance) const {
  // Check if continuous distance is long enough
  if (continuous_distance < min_continuous_distance_) return false;

  // Check if image coordinate is on image
  int i_u = int(u + 0.5f);
  int i_v = int(v + 0.5f);
  if (i_u < 0 || i_u > image_width_minus_1_ || i_v < 0 ||
      i_v > image_height_minus_1_)
    return false;

  // Check if line center is on mask
  if (use_occlusion_handling_) {
    return occlusion_mask_renderer_ptr_->occlusion_mask().at<uchar>(i_v, i_u) &
           encoded_occlusion_mask_id_;
  }
  return true;
}

bool RegionModality::CalculateSegmentProbabilities(
    float center_u, float center_v, float normal_u, float normal_v,
    std::vector<float> *segment_probabilities_f,
    std::vector<float> *segment_probabilities_b,
    float *normal_component_to_scale, float *delta_r) const {
  const cv::Mat &image{camera_ptr_->image()};

  // Select case if line is more horizontal or vertical
  if (std::fabs(normal_v) < std::fabs(normal_u)) {
    // Calculate step and starting position
    float v_step = normal_v / normal_u;
    // Notice: u = int(center_u - (line_length / 2 - 0.5) + 0.5)
    int u = int(center_u - line_length_half_minus_1_);
    int u_end = u + line_length_minus_1_;
    float v_f = center_v + v_step * (float(u) - center_u) + 0.5f;
    float v_f_end = v_f + v_step * float(line_length_minus_1_);

    // Check if line is on image
    if (u < 0 || u_end > image_width_minus_1_ || int(v_f) < 0 ||
        int(v_f) > image_height_minus_1_ || int(v_f_end) < 0 ||
        int(v_f_end) > image_height_minus_1_) {
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

    // Check if line is on image
    if (v < 0 || v_end > image_height_minus_1_ || int(u_f) < 0 ||
        int(u_f) > image_width_minus_1_ || int(u_f_end) < 0 ||
        int(u_f_end) > image_width_minus_1_) {
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
    auto segment_probability_f = segment_probabilities_f->begin();
    auto segment_probability_b = segment_probabilities_b->begin();
    for (; segment_probability_f != segment_probabilities_f->end();
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
  // Retrive pixel color probability values
  int idx = (pixel_color[0] >> histogram_bitshift_) * n_histogram_bins_squared_;
  idx += (pixel_color[1] >> histogram_bitshift_) * n_histogram_bins_;
  idx += pixel_color[2] >> histogram_bitshift_;
  float pixel_color_probability_f = histogram_f_[idx];
  float pixel_color_probability_b = histogram_b_[idx];

  // Normalize pixel color probabilitiy values
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
  auto segment_probabilities_f_it_start = segment_probabilities_f.begin();
  auto segment_probabilities_b_it_start = segment_probabilities_b.begin();
  for (auto distribution_it = distribution->begin();
       distribution_it != distribution->end();
       ++distribution_it, ++segment_probabilities_f_it_start,
            ++segment_probabilities_b_it_start) {
    *distribution_it = 1.0f;
    // Loop over values of segment probabilities and corresponding lookup values
    segment_probabilities_f_it = segment_probabilities_f_it_start;
    segment_probabilities_b_it = segment_probabilities_b_it_start;
    function_lookup_f_it = function_lookup_f_.begin();
    function_lookup_b_it = function_lookup_b_.begin();
    for (; function_lookup_f_it != function_lookup_f_.end();
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
    float *standard_deviation, float *variance) const {
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
  if (use_const_variance_)
    *variance = function_slope_;
  else
    *variance = std::max(distribution_variance, function_slope_);
  *standard_deviation = std::sqrt(*variance);
}

void RegionModality::ShowAndSaveImage(const std::string &title, int save_index,
                                      const cv::Mat &image) const {
  if (display_visualization_) cv::imshow(title, image);
  if (save_visualizations_)
    cv::imwrite(
        save_path_.string() + title + "_" + std::to_string(save_index) + ".png",
        image);
}

void RegionModality::VisualizePointsCameraImage(const std::string &title,
                                                int save_index) const {
  cv::Mat visualization_image;
  camera_ptr_->image().copyTo(visualization_image);
  DrawPoints(cv::Vec3b{24, 184, 234}, &visualization_image);
  ShowAndSaveImage(name_ + "_" + title, save_index, visualization_image);
}

void RegionModality::VisualizePointsHistogramImage(const std::string &title,
                                                   int save_index) const {
  cv::Mat visualization_image(camera_ptr_->image().size(), CV_8UC3);
  DrawProbabilityImage(cv::Vec3b{255, 255, 255}, &visualization_image);
  DrawPoints(cv::Vec3b{24, 184, 234}, &visualization_image);
  ShowAndSaveImage(name_ + "_" + title, save_index, visualization_image);
}

void RegionModality::VisualizePointsOcclusionMask(const std::string &title,
                                                  int save_index) const {
  cv::Mat visualization_image;
  occlusion_mask_renderer_ptr_->FetchOcclusionMask();
  cv::cvtColor(occlusion_mask_renderer_ptr_->occlusion_mask(),
               visualization_image, cv::COLOR_GRAY2BGR);
  cv::addWeighted(visualization_image, 0.4, camera_ptr_->image(), 0.6, 10,
                  visualization_image);
  DrawPoints(cv::Vec3b{24, 184, 234}, &visualization_image);
  ShowAndSaveImage(name_ + "_" + title, save_index, visualization_image);
}

void RegionModality::VisualizeLines(const std::string &title,
                                    int save_index) const {
  cv::Mat visualization_image(camera_ptr_->image().size(), CV_8UC3);
  DrawProbabilityImage(cv::Vec3b{255, 255, 255}, &visualization_image);
  DrawLines(cv::Vec3b{24, 184, 234}, cv::Vec3b{61, 63, 179},
            &visualization_image);
  ShowAndSaveImage(name_ + "_" + title, save_index, visualization_image);
}

void RegionModality::DrawPoints(const cv::Vec3b &color_point,
                                cv::Mat *image) const {
  for (const auto &data_line : data_lines_) {
    DrawPointInImage(data_line.center_f_camera, color_point,
                     camera_ptr_->intrinsics(), image);
  }
}

void RegionModality::DrawLines(const cv::Vec3b &color_line,
                               const cv::Vec3b &color_high_probability,
                               cv::Mat *image) const {
  float scale_minus_1_half_ = (fscale_ - 1.0f) / 2.0f;
  int u, v;
  for (const auto &data_line : data_lines_) {
    for (int i = 0; i < distribution_length_; ++i) {
      for (int j = 0; j < scale_; ++j) {
        if (std::fabs(data_line.normal_u) > std::fabs(data_line.normal_v)) {
          u = int(
              data_line.center_u +
              float(sgn(data_line.normal_u)) *
                  (fscale_ * (float(i) - distribution_length_minus_1_half_) +
                   float(j) - scale_minus_1_half_) +
              0.5f);
          v = int(data_line.center_v +
                  (float(u) - data_line.center_u) *
                      (data_line.normal_v / data_line.normal_u) +
                  0.5f);
        } else {
          v = int(
              data_line.center_v +
              float(sgn(data_line.normal_v)) *
                  (fscale_ * (float(i) - distribution_length_minus_1_half_) +
                   float(j) - scale_minus_1_half_) +
              0.5f);
          u = int(data_line.center_u +
                  (float(v) - data_line.center_v) *
                      (data_line.normal_u / data_line.normal_v) +
                  0.5f);
        }
        float color_ratio = std::min(3 * data_line.distribution[i], 1.0f);
        image->at<cv::Vec3b>(v, u) = color_ratio * color_high_probability +
                                     (1.0f - color_ratio) * color_line;
      }
    }
  }
}

void RegionModality::DrawProbabilityImage(const cv::Vec3b &color_b,
                                          cv::Mat *probability_image) const {
  const cv::Mat &color_image{camera_ptr_->image()};
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

void RegionModality::UpdateLineCentersWithCurrentPose() {
  Transform3fA body2camera_pose{camera_ptr_->world2camera_pose() *
                                body_ptr_->body2world_pose()};
  for (auto &data_line : data_lines_) {
    data_line.center_f_camera = body2camera_pose * data_line.center_f_body;
  }
}

float RegionModality::MinAbsValueWithSignOfValue1(float value_1,
                                                  float abs_value_2) {
  if (std::abs(value_1) < abs_value_2)
    return value_1;
  else
    return sgnf(value_1) * abs_value_2;
}

}  // namespace rbgt
