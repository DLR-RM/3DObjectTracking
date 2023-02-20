// SPDX-License-Identifier: MIT
// Copyright (c) 2023 Manuel Stoiber and Mariam Elsayed,
// German Aerospace Center (DLR)

#include <m3t/texture_modality.h>

namespace m3t {

TextureModality::TextureModality(
    const std::string &name, const std::shared_ptr<Body> &body_ptr,
    const std::shared_ptr<ColorCamera> &color_camera_ptr,
    const std::shared_ptr<FocusedSilhouetteRenderer> &silhouette_renderer_ptr)
    : Modality{name, body_ptr},
      color_camera_ptr_{color_camera_ptr},
      silhouette_renderer_ptr_{silhouette_renderer_ptr} {}

TextureModality::TextureModality(
    const std::string &name, const std::filesystem::path &metafile_path,
    const std::shared_ptr<Body> &body_ptr,
    const std::shared_ptr<ColorCamera> &color_camera_ptr,
    const std::shared_ptr<FocusedSilhouetteRenderer> &silhouette_renderer_ptr)
    : Modality{name, metafile_path, body_ptr},
      color_camera_ptr_{color_camera_ptr},
      silhouette_renderer_ptr_{silhouette_renderer_ptr} {}

bool TextureModality::SetUp() {
  set_up_ = false;
  if (!metafile_path_.empty())
    if (!LoadMetaData()) return false;

  // Check if all required objects are set up
  if (!body_ptr_->set_up()) {
    std::cerr << "Body " << body_ptr_->name() << " was not set up" << std::endl;
    return false;
  }
  if (!color_camera_ptr_->set_up()) {
    std::cerr << "Color camera " << color_camera_ptr_->name()
              << " was not set up" << std::endl;
    return false;
  }
  if (!silhouette_renderer_ptr_->set_up()) {
    std::cerr << "Focused silhouette renderer "
              << silhouette_renderer_ptr_->name() << " was not set up"
              << std::endl;
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

  // Check if all required objects are correctly configured
  if (silhouette_renderer_ptr_->id_type() != IDType::BODY) {
    std::cerr << "Focused silhouette renderer "
              << silhouette_renderer_ptr_->name()
              << " does not use id_type BODY" << std::endl;
  }

  SetUpFeatureDetectorAndMatcher();
  PrecalculateCameraVariables();
  PrecalculateRendererVariables();
  SetImshowVariables();

  set_up_ = true;
  return true;
}

void TextureModality::set_color_camera_ptr(
    const std::shared_ptr<ColorCamera> &color_camera_ptr) {
  color_camera_ptr_ = color_camera_ptr;
  set_up_ = false;
}

void TextureModality::set_silhouette_renderer_ptr(
    const std::shared_ptr<FocusedSilhouetteRenderer> &silhouette_renderer_ptr) {
  silhouette_renderer_ptr_ = silhouette_renderer_ptr;
  set_up_ = false;
}

void TextureModality::set_descriptor_type(
    TextureModality::DescriptorType descriptor_type) {
  descriptor_type_ = descriptor_type;
  set_up_ = false;
}

void TextureModality::set_focused_image_size(int focused_image_size) {
  focused_image_size_ = focused_image_size;
}

void TextureModality::set_tukey_norm_constant(float tukey_norm_constant) {
  tukey_norm_constant_ = tukey_norm_constant;
}

void TextureModality::set_descriptor_distance_threshold(
    float descriptor_distance_threshold) {
  descriptor_distance_threshold_ = descriptor_distance_threshold;
}

void TextureModality::set_standard_deviations(
    const std::vector<float> &standard_deviations) {
  standard_deviations_ = standard_deviations;
}

void TextureModality::set_max_keyframe_rotation_difference(
    float max_keyframe_rotation_difference) {
  max_keyframe_rotation_difference_ = max_keyframe_rotation_difference;
}

void TextureModality::set_max_keyframe_age(int max_keyframe_age) {
  max_keyframe_age_ = max_keyframe_age;
}

void TextureModality::set_n_keyframes(int n_keyframes) {
  n_keyframes_ = n_keyframes;
}

void TextureModality::set_orb_n_features(int orb_n_features) {
  orb_n_features_ = orb_n_features;
  set_up_ = false;
}

void TextureModality::set_orb_scale_factor(float orb_scale_factor) {
  orb_scale_factor_ = orb_scale_factor;
  set_up_ = false;
}

void TextureModality::set_orb_n_levels(int orb_n_levels) {
  orb_n_levels_ = orb_n_levels;
  set_up_ = false;
}

void TextureModality::set_brisk_threshold(int brisk_threshold) {
  brisk_threshold_ = brisk_threshold;
  set_up_ = false;
}

void TextureModality::set_brisk_octave(int brisk_octave) {
  brisk_octave_ = brisk_octave;
  set_up_ = false;
}

void TextureModality::set_brisk_pattern_scale(float brisk_pattern_scale) {
  brisk_pattern_scale_ = brisk_pattern_scale;
  set_up_ = false;
}

void TextureModality::set_daisy_radius(float daisy_radius) {
  daisy_radius_ = daisy_radius;
  set_up_ = false;
}

void TextureModality::set_daisy_q_radius(int daisy_q_radius) {
  daisy_q_radius_ = daisy_q_radius;
  set_up_ = false;
}

void TextureModality::set_daisy_q_theta(int daisy_q_theta) {
  daisy_q_theta_ = daisy_q_theta;
  set_up_ = false;
}

void TextureModality::set_daisy_q_hist(int daisy_q_hist) {
  daisy_q_hist_ = daisy_q_hist;
  set_up_ = false;
}

void TextureModality::set_freak_orientation_normalized(
    bool freak_orientation_normalized) {
  freak_orientation_normalized_ = freak_orientation_normalized;
  set_up_ = false;
}

void TextureModality::set_freak_scale_normalized(bool freak_scale_normalized) {
  freak_scale_normalized_ = freak_scale_normalized;
  set_up_ = false;
}

void TextureModality::set_freak_pattern_scale(float freak_pattern_scale) {
  freak_pattern_scale_ = freak_pattern_scale;
  set_up_ = false;
}

void TextureModality::set_freak_n_octaves(int freak_n_octaves) {
  freak_n_octaves_ = freak_n_octaves;
  set_up_ = false;
}

void TextureModality::set_sift_n_features(int sift_n_features) {
  sift_n_features_ = sift_n_features;
  set_up_ = false;
}

void TextureModality::set_sift_n_octave_layers(int sift_n_octave_layers) {
  sift_n_octave_layers_ = sift_n_octave_layers;
  set_up_ = false;
}

void TextureModality::set_sift_contrast_threshold(
    double sift_contrast_threshold) {
  sift_contrast_threshold_ = sift_contrast_threshold;
  set_up_ = false;
}

void TextureModality::set_sift_edge_threshold(double sift_edge_threshold) {
  sift_edge_threshold_ = sift_edge_threshold;
  set_up_ = false;
}

void TextureModality::set_sift_sigma(double sift_sigma) {
  sift_sigma_ = sift_sigma;
  set_up_ = false;
}

void TextureModality::MeasureOcclusions(
    const std::shared_ptr<DepthCamera> &depth_camera_ptr) {
  depth_camera_ptr_ = depth_camera_ptr;
  measure_occlusions_ = true;
  set_up_ = false;
}

void TextureModality::DoNotMeasureOcclusions() {
  measure_occlusions_ = false;
  set_up_ = false;
}

void TextureModality::ModelOcclusions(
    const std::shared_ptr<FocusedDepthRenderer> &depth_renderer_ptr) {
  depth_renderer_ptr_ = depth_renderer_ptr;
  model_occlusions_ = true;
  set_up_ = false;
}

void TextureModality::DoNotModelOcclusions() {
  depth_renderer_ptr_ = nullptr;
  model_occlusions_ = false;
  set_up_ = false;
}

void TextureModality::set_measured_occlusion_radius(
    float measured_occlusion_radius) {
  measured_occlusion_radius_ = measured_occlusion_radius;
}

void TextureModality::set_measured_occlusion_threshold(
    float measured_occlusion_threshold) {
  measured_occlusion_threshold_ = measured_occlusion_threshold;
}

void TextureModality::set_modeled_occlusion_radius(
    float modeled_occlusion_radius) {
  modeled_occlusion_radius_ = modeled_occlusion_radius;
}

void TextureModality::set_modeled_occlusion_threshold(
    float modeled_occlusion_threshold) {
  modeled_occlusion_threshold_ = modeled_occlusion_threshold;
}

void TextureModality::set_visualize_correspondences_correspondence(
    bool visualize_correspondences_correspondence) {
  visualize_correspondences_correspondence_ =
      visualize_correspondences_correspondence;
  SetImshowVariables();
}

void TextureModality::set_visualize_corresopndences_optimization(
    bool visualize_correspondence_optimization) {
  visualize_correspondences_optimization_ =
      visualize_correspondence_optimization;
  SetImshowVariables();
}

void TextureModality::set_visualize_points_result(
    bool visualize_points_result) {
  visualize_points_result_ = visualize_points_result;
  SetImshowVariables();
}

void TextureModality::set_visualize_points_depth_image_result(
    bool visualize_points_depth_image_result) {
  visualize_points_depth_image_result_ = visualize_points_depth_image_result;
  SetImshowVariables();
}

void TextureModality ::set_visualize_points_silhouette_rendering_result(
    bool visualize_points_silhouete_rendering_result) {
  visualize_points_silhouette_rendering_result_ =
      visualize_points_silhouete_rendering_result;
  SetImshowVariables();
}

void TextureModality ::set_visualize_points_depth_rendering_result(
    bool visualize_points_depth_rendering_result) {
  visualize_points_depth_rendering_result_ =
      visualize_points_depth_rendering_result;
  SetImshowVariables();
}

void TextureModality::set_visualization_min_depth(
    float visualization_min_depth) {
  visualization_min_depth_ = visualization_min_depth;
}

void TextureModality::set_visualization_max_depth(
    float visualization_max_depth) {
  visualization_max_depth_ = visualization_max_depth;
}

bool TextureModality::StartModality(int iteration, int corr_iteration) {
  if (!IsSetup()) return false;

  PrecalculatePoseVariables();
  DetectAndComputeCorrKeypoints();
  ComputeKeyframeData();
  return true;
}

bool TextureModality::CalculateCorrespondences(int iteration,
                                               int corr_iteration) {
  if (!IsSetup()) return false;

  PrecalculatePoseVariables();
  PrecalculateIterationDependentVariables(corr_iteration);

  // Calculate matches and reconstruct data points
  if (corr_iteration == 0) {
    // Match descriptors
    DetectAndComputeCorrKeypoints();
    std::vector<std::vector<std::vector<cv::DMatch>>> knn_matches_keyframes;
#ifdef USE_CUDA
    if (descriptor_type_ == DescriptorType::ORB_CUDA) {
      for (const auto &descriptors_keyframe_cuda :
           descriptors_keyframes_cuda_) {
        std::vector<std::vector<cv::DMatch>> knn_matches;
        if (!descriptors_keyframe_cuda.empty() && !descriptors_cuda_.empty())
          descriptor_matcher_cuda_->knnMatch(descriptors_keyframe_cuda,
                                             descriptors_cuda_, knn_matches, 2);
        knn_matches_keyframes.push_back(std::move(knn_matches));
      }
    } else
#endif
    {
      for (const auto &descriptors_keyframe : descriptors_keyframes_) {
        std::vector<std::vector<cv::DMatch>> knn_matches;
        if (!descriptors_keyframe.empty() && !descriptors_.empty())
          descriptor_matcher_->knnMatch(descriptors_keyframe, descriptors_,
                                        knn_matches, 2);
        knn_matches_keyframes.push_back(std::move(knn_matches));
      }
    }

    // Compute data points
    data_points_.clear();
    for (size_t i = 0; i < knn_matches_keyframes.size(); ++i) {
      const auto &knn_matches{knn_matches_keyframes[i]};
      const auto &points_keyframe{points_keyframes_[i]};
      for (const auto &knn_match : knn_matches) {
        if (knn_match.size() < 2) continue;
        if (knn_match[0].distance / knn_match[1].distance >=
            descriptor_distance_threshold_)
          continue;

        DataPoint data_point;
        data_point.correspondence_center =
            Eigen::Vector2f{keypoints_[knn_match[0].trainIdx].pt.x,
                            keypoints_[knn_match[0].trainIdx].pt.y};
        data_point.center_f_body = points_keyframe[knn_match[0].queryIdx];
        data_points_.push_back(std::move(data_point));
      }
    }
  }

  // Compute iteration dependent data points
  for (auto &data_point : data_points_) {
    data_point.center_f_camera = body2camera_pose_ * data_point.center_f_body;
    const auto &center_f_camera{data_point.center_f_camera};
    data_point.center(0) = center_f_camera(0) * fu_ / center_f_camera(2) + ppu_;
    data_point.center(1) = center_f_camera(1) * fv_ / center_f_camera(2) + ppv_;
  }
  return true;
}

bool TextureModality::VisualizeCorrespondences(int save_idx) {
  if (!IsSetup()) return false;

  if (visualize_correspondences_correspondence_)
    VisualizeCorrespondences("correspondences_correspondence", save_idx);
  return true;
}

bool TextureModality::CalculateGradientAndHessian(int iteration,
                                                  int corr_iteration,
                                                  int opt_iteration) {
  if (!IsSetup()) return false;

  PrecalculatePoseVariables();
  gradient_.setZero();
  hessian_.setZero();

  // iterate over points
  for (auto &data_point : data_points_) {
    // Calculate center in camera frame and image
    data_point.center_f_camera = body2camera_pose_ * data_point.center_f_body;
    const auto &center_f_camera{data_point.center_f_camera};
    const auto &center_f_body{data_point.center_f_body};
    data_point.center(0) = center_f_camera(0) * fu_ / center_f_camera(2) + ppu_;
    data_point.center(1) = center_f_camera(1) * fv_ / center_f_camera(2) + ppv_;
    float x = center_f_camera(0);
    float y = center_f_camera(1);
    float z = center_f_camera(2);
    float z2 = z * z;

    // Calculate error
    Eigen::Vector2f diff{data_point.center - data_point.correspondence_center};
    float squared_error = diff.squaredNorm();
    float error = sqrtf(squared_error);

    // Calculate weight with Tukey norm
    float weight = 1.0f / variance_;
    if (error > std::numeric_limits<float>::min())
      weight = (TukeyNorm(error) / squared_error) / variance_;

    // Calculate derivatives
    Eigen::Matrix<float, 2, 3> dx_dX;
    dx_dX << fu_ / z, 0.0f, -x * fu_ / z2, 0.0f, fv_ / z, -y * fv_ / z2;
    Eigen::Matrix<float, 2, 3> dx_dtranslation{dx_dX * body2camera_rotation_};
    Eigen::Matrix<float, 2, 6> dx_dtheta;
    dx_dtheta << -dx_dtranslation * Vector2Skewsymmetric(center_f_body),
        dx_dtranslation;

    // Calculate gradient and hessian
    gradient_ -= (weight * diff.transpose()) * dx_dtheta;
    hessian_.triangularView<Eigen::Lower>() -=
        (weight * dx_dtheta.transpose()) * dx_dtheta;
  }
  hessian_ = hessian_.selfadjointView<Eigen::Lower>();
  return true;
}

bool TextureModality::VisualizeOptimization(int save_idx) {
  if (!IsSetup()) return false;

  if (visualize_correspondences_optimization_)
    VisualizePointsColorImage("correspondences_optimization", save_idx);
  if (visualize_gradient_optimization_) VisualizeGradient();
  if (visualize_hessian_optimization_) VisualizeHessian();
  return true;
}

bool TextureModality::CalculateResults(int iteration) {
  if (!IsSetup()) return false;

  // Calculate rotation difference
  Eigen::Vector3f orientation{
      body2camera_pose_.rotation().inverse() *
      body2camera_pose_.translation().matrix().normalized()};
  float rotation_difference =
      acos(orientation.transpose() * orientation_last_keyframe_);
  keyframe_age_++;

  // Compute new data if difference is above threshold
  if (rotation_difference > max_keyframe_rotation_difference_ ||
      keyframe_age_ > max_keyframe_age_)
    ComputeKeyframeData();
  return true;
}

bool TextureModality::VisualizeResults(int save_idx) {
  if (!IsSetup()) return false;

  if (keyframe_age_ == 0) {
    if (visualize_points_result_)
      VisualizePointsColorImage("color_image_result", save_idx);
    if (visualize_points_depth_image_result_ && measure_occlusions_)
      VisualizePointsDepthImage("depth_image_result", save_idx);
    if (visualize_points_depth_rendering_result_ && model_occlusions_)
      VisualizePointsDepthRendering("depth_rendering_result", save_idx);
    if (visualize_points_silhouette_rendering_result_)
      VisualizePointsSilhouetteRendering("silhouette_rendering_result",
                                         save_idx);
  }
  if (visualize_pose_result_) VisualizePose();
  return true;
}

const std::shared_ptr<ColorCamera> &TextureModality::color_camera_ptr() const {
  return color_camera_ptr_;
}

const std::shared_ptr<DepthCamera> &TextureModality::depth_camera_ptr() const {
  return depth_camera_ptr_;
}

const std::shared_ptr<FocusedSilhouetteRenderer>
    &TextureModality::silhouette_renderer_ptr() const {
  return silhouette_renderer_ptr_;
}

const std::shared_ptr<FocusedDepthRenderer>
    &TextureModality::depth_renderer_ptr() const {
  return depth_renderer_ptr_;
}

std::shared_ptr<Model> TextureModality::model_ptr() const { return {}; }

std::vector<std::shared_ptr<Camera>> TextureModality::camera_ptrs() const {
  return {color_camera_ptr_};
}

std::vector<std::shared_ptr<Renderer>>
TextureModality::start_modality_renderer_ptrs() const {
  return {silhouette_renderer_ptr_, depth_renderer_ptr_};
}

std::vector<std::shared_ptr<Renderer>> TextureModality::results_renderer_ptrs()
    const {
  return {silhouette_renderer_ptr_, depth_renderer_ptr_};
}

TextureModality::DescriptorType TextureModality::descriptor_type() const {
  return descriptor_type_;
}

int TextureModality::focused_image_size() const { return focused_image_size_; }

float TextureModality::descriptor_distance_threshold() const {
  return descriptor_distance_threshold_;
}

float TextureModality::tukey_norm_constant() const {
  return tukey_norm_constant_;
}

const std::vector<float> &TextureModality::standard_deviations() const {
  return standard_deviations_;
}

float TextureModality::max_keyframe_rotation_difference() const {
  return max_keyframe_rotation_difference_;
}

int TextureModality::max_keyframe_age() const { return max_keyframe_age_; }

int TextureModality::n_keyframes() const { return n_keyframes_; }

int TextureModality::orb_n_features() const { return orb_n_features_; }

float TextureModality::orb_scale_factor() const { return orb_scale_factor_; }

int TextureModality::orb_n_levels() const { return orb_n_levels_; }

int TextureModality::brisk_threshold() const { return brisk_threshold_; }

int TextureModality::brisk_octave() const { return brisk_octave_; }

float TextureModality::brisk_pattern_scale() const {
  return brisk_pattern_scale_;
}

float TextureModality::daisy_radius() const { return daisy_radius_; }

int TextureModality::daisy_q_radius() const { return daisy_q_radius_; }

int TextureModality::daisy_q_theta() const { return daisy_q_theta_; }

int TextureModality::daisy_q_hist() const { return daisy_q_hist_; }

bool TextureModality::freak_orientation_normalized() const {
  return freak_orientation_normalized_;
}

bool TextureModality::freak_scale_normalized() const {
  return freak_scale_normalized_;
}

float TextureModality::freak_pattern_scale() const {
  return freak_pattern_scale_;
}

int TextureModality::freak_n_octaves() const { return freak_n_octaves_; }

int TextureModality::sift_n_features() const { return sift_n_features_; }

int TextureModality::sift_n_octave_layers() const {
  return sift_n_octave_layers_;
}

double TextureModality::sift_contrast_threshold() const {
  return sift_contrast_threshold_;
}

double TextureModality::sift_edge_threshold() const {
  return sift_edge_threshold_;
}

double TextureModality::sift_sigma() const { return sift_sigma_; }

bool TextureModality::measure_occlusions() const { return measure_occlusions_; }

float TextureModality::measured_occlusion_radius() const {
  return measured_occlusion_radius_;
}

float TextureModality::measured_occlusion_threshold() const {
  return measured_occlusion_threshold_;
}

bool TextureModality::model_occlusions() const { return model_occlusions_; }

float TextureModality::modeled_occlusion_radius() const {
  return modeled_occlusion_radius_;
}

float TextureModality::modeled_occlusion_threshold() const {
  return modeled_occlusion_threshold_;
}

bool TextureModality::visualize_correspondences_correspondence() const {
  return visualize_correspondences_correspondence_;
}

bool TextureModality::visualize_points_depth_image_result() const {
  return visualize_points_depth_image_result_;
}

bool TextureModality::visualize_points_silhouette_rendering_result() const {
  return visualize_points_silhouette_rendering_result_;
}

bool TextureModality::visualize_points_depth_rendering_result() const {
  return visualize_points_depth_rendering_result_;
}

bool TextureModality::visualize_correspondences_optimization() const {
  return visualize_correspondences_optimization_;
}

bool TextureModality::visualize_points_result() const {
  return visualize_points_result_;
}

float TextureModality::visualization_min_depth() const {
  return visualization_min_depth_;
}

float TextureModality::visualization_max_depth() const {
  return visualization_max_depth_;
}

bool TextureModality::LoadMetaData() {
  // Open file storage from yaml
  cv::FileStorage fs;
  if (!OpenYamlFileStorage(metafile_path_, &fs)) return false;

  // Read parameters from yaml file for general settings
  ReadOptionalValueFromYaml(fs, "descriptor_type", &descriptor_type_);
  ReadOptionalValueFromYaml(fs, "focused_image_size", &focused_image_size_);
  ReadOptionalValueFromYaml(fs, "descriptor_distance_threshold",
                            &descriptor_distance_threshold_);
  ReadOptionalValueFromYaml(fs, "tukey_norm_constant", &tukey_norm_constant_);
  ReadOptionalValueFromYaml(fs, "standard_deviations", &standard_deviations_);
  ReadOptionalValueFromYaml(fs, "max_keyframe_rotation_difference",
                            &max_keyframe_rotation_difference_);
  ReadOptionalValueFromYaml(fs, "max_keyframe_age", &max_keyframe_age_);
  ReadOptionalValueFromYaml(fs, "n_keyframes", &n_keyframes_);

  // Read parameters from yaml file for feature detection
  ReadOptionalValueFromYaml(fs, "orb_n_features", &orb_n_features_);
  ReadOptionalValueFromYaml(fs, "orb_scale_factor", &orb_scale_factor_);
  ReadOptionalValueFromYaml(fs, "orb_n_levels", &orb_n_levels_);
  ReadOptionalValueFromYaml(fs, "brisk_threshold", &brisk_threshold_);
  ReadOptionalValueFromYaml(fs, "brisk_octave", &brisk_octave_);
  ReadOptionalValueFromYaml(fs, "brisk_pattern_scale", &brisk_pattern_scale_);
  ReadOptionalValueFromYaml(fs, "daisy_radius", &daisy_radius_);
  ReadOptionalValueFromYaml(fs, "daisy_q_radius", &daisy_q_radius_);
  ReadOptionalValueFromYaml(fs, "daisy_q_theta", &daisy_q_theta_);
  ReadOptionalValueFromYaml(fs, "daisy_q_hist", &daisy_q_hist_);
  ReadOptionalValueFromYaml(fs, "freak_orientation_normalized",
                            &freak_orientation_normalized_);
  ReadOptionalValueFromYaml(fs, "freak_n_octaves", &freak_n_octaves_);
  ReadOptionalValueFromYaml(fs, "sift_n_features", &sift_n_features_);
  ReadOptionalValueFromYaml(fs, "sift_n_octave_layers", &sift_n_octave_layers_);
  ReadOptionalValueFromYaml(fs, "sift_contrast_threshold",
                            &sift_contrast_threshold_);
  ReadOptionalValueFromYaml(fs, "sift_edge_threshold", &sift_edge_threshold_);
  ReadOptionalValueFromYaml(fs, "sift_sigma", &sift_sigma_);

  // Read parameters from yaml file for occlusion handling
  ReadOptionalValueFromYaml(fs, "measured_occlusion_radius",
                            &measured_occlusion_radius_);
  ReadOptionalValueFromYaml(fs, "measured_occlusion_threshold",
                            &measured_occlusion_threshold_);
  ReadOptionalValueFromYaml(fs, "modeled_occlusion_radius",
                            &modeled_occlusion_radius_);
  ReadOptionalValueFromYaml(fs, "modeled_occlusion_threshold",
                            &modeled_occlusion_threshold_);

  // Read parameters from yaml for visualization
  ReadOptionalValueFromYaml(fs, "visualize_pose_result",
                            &visualize_pose_result_);
  ReadOptionalValueFromYaml(fs, "visualize_gradient_optimization",
                            &visualize_gradient_optimization_);
  ReadOptionalValueFromYaml(fs, "visualize_hessian_optimization",
                            &visualize_hessian_optimization_);
  ReadOptionalValueFromYaml(fs, "visualize_correspondences_correspondence",
                            &visualize_correspondences_correspondence_);
  ReadOptionalValueFromYaml(fs, "visualize_correspondences_optimization",
                            &visualize_correspondences_optimization_);
  ReadOptionalValueFromYaml(fs, "visualize_points_result",
                            &visualize_points_result_);
  ReadOptionalValueFromYaml(fs, "visualize_points_depth_image_result",
                            &visualize_points_depth_image_result_);
  ReadOptionalValueFromYaml(fs, "visualize_points_silhouette_rendering_result",
                            &visualize_points_depth_rendering_result_);
  ReadOptionalValueFromYaml(fs, "visualize_points_silhouette_rendering_result",
                            &visualize_points_depth_rendering_result_);
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

void TextureModality::SetUpFeatureDetectorAndMatcher() {
  // Set up feature detector
  switch (descriptor_type_) {
    case DescriptorType::BRISK:
      feature_detector_ = cv::BRISK::create(brisk_threshold_, brisk_octave_,
                                            brisk_pattern_scale_);
      feature_descriptor_ = cv::BRISK::create(brisk_threshold_, brisk_octave_,
                                              brisk_pattern_scale_);
      break;
    case DescriptorType::DAISY:
      feature_detector_ =
          cv::ORB::create(orb_n_features_, orb_scale_factor_, orb_n_levels_);
      feature_descriptor_ = cv::xfeatures2d::DAISY::create(
          daisy_radius_, daisy_q_radius_, daisy_q_theta_, daisy_q_hist_);
      break;
    case DescriptorType::FREAK:
      feature_detector_ =
          cv::ORB::create(orb_n_features_, orb_scale_factor_, orb_n_levels_);
      feature_descriptor_ = cv::xfeatures2d::FREAK::create(
          freak_orientation_normalized_, freak_scale_normalized_,
          freak_pattern_scale_, freak_n_octaves_);
      break;
    case DescriptorType::SIFT:
      // Look in both namespaces for SIFT
      using namespace cv;
      using namespace cv::xfeatures2d;
      feature_detector_ = SIFT::create(sift_n_features_, sift_n_octave_layers_,
                                       sift_contrast_threshold_,
                                       sift_edge_threshold_, sift_sigma_);
      feature_descriptor_ = SIFT::create(
          sift_n_features_, sift_n_octave_layers_, sift_contrast_threshold_,
          sift_edge_threshold_, sift_sigma_);
      break;
#ifdef USE_CUDA
    case DescriptorType::ORB_CUDA:
      feature_detector_orb_cuda_ = cv::cuda::ORB::create(
          orb_n_features_, orb_scale_factor_, orb_n_levels_);
#endif
    case DescriptorType::ORB:
    default:
      feature_detector_ =
          cv::ORB::create(orb_n_features_, orb_scale_factor_, orb_n_levels_);
      feature_descriptor_ =
          cv::ORB::create(orb_n_features_, orb_scale_factor_, orb_n_levels_);
      break;
  }

    // Set up feature matcher
#ifdef USE_CUDA
  if (descriptor_type_ == DescriptorType::ORB_CUDA)
    descriptor_matcher_cuda_ =
        cv::cuda::DescriptorMatcher::createBFMatcher(cv::NORM_HAMMING);
  else
#endif
  {
    if (descriptor_type_ == DescriptorType::DAISY ||
        descriptor_type_ == DescriptorType::SIFT)
      descriptor_matcher_ = cv::BFMatcher::create(cv::NORM_L2);
    else
      descriptor_matcher_ = cv::BFMatcher::create(cv::NORM_HAMMING);
  }
}

void TextureModality::SetImshowVariables() {
  imshow_correspondence_ =
      display_visualization_ && visualize_correspondences_correspondence_;
  imshow_optimization_ =
      display_visualization_ && visualize_correspondences_optimization_;
  imshow_result_ =
      display_visualization_ &&
      (visualize_points_result_ ||
       (visualize_points_depth_image_result_ && measure_occlusions_) ||
       (visualize_points_silhouette_rendering_result_) ||
       (visualize_points_depth_rendering_result_ && model_occlusions_));
}

void TextureModality::PrecalculateCameraVariables() {
  fu_ = color_camera_ptr_->intrinsics().fu;
  fv_ = color_camera_ptr_->intrinsics().fv;
  ppu_ = color_camera_ptr_->intrinsics().ppu;
  ppv_ = color_camera_ptr_->intrinsics().ppv;
  image_width_minus_1_ = color_camera_ptr_->intrinsics().width - 1;
  image_height_minus_1_ = color_camera_ptr_->intrinsics().height - 1;
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

void TextureModality::PrecalculateRendererVariables() {
  silhouette_image_size_minus_1_ = silhouette_renderer_ptr_->image_size() - 1;
  if (model_occlusions_) {
    depth_image_size_minus_1_ = depth_renderer_ptr_->image_size() - 1;
  }
}

void TextureModality::PrecalculatePoseVariables() {
  body2camera_pose_ =
      color_camera_ptr_->world2camera_pose() * body_ptr_->body2world_pose();
  camera2body_pose_ = body2camera_pose_.inverse();
  body2camera_rotation_ = body2camera_pose_.rotation().matrix();
  if (measure_occlusions_) {
    body2depth_camera_pose_ =
        depth_camera_ptr_->world2camera_pose() * body_ptr_->body2world_pose();
  }
}

void TextureModality::PrecalculateIterationDependentVariables(
    int corr_iteration) {
  float standard_deviation =
      LastValidValue(standard_deviations_, corr_iteration);
  variance_ = powf(standard_deviation, 2.0f);
}

void TextureModality::DetectAndComputeCorrKeypoints() {
  keypoints_.clear();

  // Compute focused image
  cv::Mat focused_image;
  cv::Rect region_of_interest;
  float scale;
  if (!CalculateScaleAndRegionOfInterest(&region_of_interest, &scale)) return;
  cv::cvtColor(color_camera_ptr_->image(), focused_image, cv::COLOR_BGR2GRAY);
  cv::resize(focused_image(region_of_interest), focused_image, cv::Size(),
             scale, scale);

  // Detect features and compute descriptors
#ifdef USE_CUDA
  if (descriptor_type_ == DescriptorType::ORB_CUDA) {
    cv::cuda::GpuMat focused_image_cuda{focused_image};
    feature_detector_orb_cuda_->detectAndCompute(
        focused_image_cuda, cv::noArray(), keypoints_, descriptors_cuda_);
  } else
#endif
  {
    feature_detector_->detect(focused_image, keypoints_);
    feature_descriptor_->compute(focused_image, keypoints_, descriptors_);
  }

  // Add focus offset to keypoints
  for (auto &corr_keypoint : keypoints_) {
    corr_keypoint.pt.x = region_of_interest.x + corr_keypoint.pt.x / scale;
    corr_keypoint.pt.y = region_of_interest.y + corr_keypoint.pt.y / scale;
  }
}

bool TextureModality::CalculateScaleAndRegionOfInterest(
    cv::Rect *region_of_interest, float *scale) const {
  // Project sphere into image
  float r = 0.5f * body_ptr_->maximum_body_diameter();
  auto translation{body2camera_pose_.translation()};
  float x = translation(0);
  float y = translation(1);
  float z = translation(2);
  if (z < r * 1.5f) return false;
  float abs_x = std::abs(x);
  float abs_y = std::abs(y);
  float x2 = x * x;
  float y2 = y * y;
  float z2 = z * z;
  float r2 = r * r;
  float rz = r * z;
  float z2_r2 = z2 - r2;
  float z3_zr2 = z2_r2 * z;
  float r_u = fu_ * (abs_x * r2 + rz * sqrtf(z2_r2 + x2)) / z3_zr2;
  float r_v = fv_ * (abs_y * r2 + rz * sqrtf(z2_r2 + y2)) / z3_zr2;
  float center_u = x * fu_ / z + ppu_;
  float center_v = y * fv_ / z + ppv_;

  // Calculate region of interest
  int u_min = int(center_u - r_u - kRegionOfInterestMargin + 0.5f);
  int u_max = int(center_u + r_u + kRegionOfInterestMargin + 0.5f);
  int v_min = int(center_v - r_v - kRegionOfInterestMargin + 0.5f);
  int v_max = int(center_v + r_v + kRegionOfInterestMargin + 0.5f);
  u_min = std::max(u_min, 0);
  u_max = std::min(u_max, image_width_minus_1_);
  v_min = std::max(v_min, 0);
  v_max = std::min(v_max, image_height_minus_1_);
  if (u_min >= u_max || v_min >= v_max) return false;
  region_of_interest->x = u_min;
  region_of_interest->y = v_min;
  region_of_interest->width = u_max - u_min;
  region_of_interest->height = v_max - v_min;

  // Calculate scale
  *scale = float(focused_image_size_) / std::max(2.0f * r_u, 2.0f * r_v);
  return true;
}

void TextureModality::ComputeKeyframeData() {
  if (points_keyframes_.size() >= n_keyframes_) {
    points_keyframes_.pop_front();
#ifdef USE_CUDA
    if (descriptor_type_ == DescriptorType::ORB_CUDA)
      descriptors_keyframes_cuda_.pop_front();
    else
#endif
      descriptors_keyframes_.pop_front();
  }

  // Fetch depth and silhouette images
  if (!silhouette_renderer_ptr_->IsBodyVisible(body_ptr_->name())) return;
  silhouette_renderer_ptr_->FetchDepthImage();
  silhouette_renderer_ptr_->FetchSilhouetteImage();
  bool body_visible_depth;
  if (model_occlusions_) {
    body_visible_depth = depth_renderer_ptr_->IsBodyVisible(body_ptr_->name());
    if (body_visible_depth) depth_renderer_ptr_->FetchDepthImage();
  }

  // Reconstruct points
  std::vector<size_t> indexes;
  std::vector<Eigen::Vector3f> points_keyframe;
  for (size_t i = 0; i < keypoints_.size(); ++i) {
    Eigen::Vector3f point;
    if (!Reconstruct3DPoint(keypoints_[i].pt, &point)) continue;
    if (!IsPointValid(point, measure_occlusions_,
                      model_occlusions_ && body_visible_depth))
      continue;
    points_keyframe.push_back(std::move(point));
    indexes.push_back(i);
  }
  points_keyframes_.push_back(std::move(points_keyframe));

  // Copy descriptors
#ifdef USE_CUDA
  if (descriptor_type_ == DescriptorType::ORB_CUDA) {
    cv::cuda::GpuMat descriptors_keyframe_cuda(
        indexes.size(), descriptors_cuda_.cols, descriptors_cuda_.type());
    for (size_t i = 0; i < indexes.size(); ++i)
      descriptors_cuda_.row(indexes[i])
          .copyTo(descriptors_keyframe_cuda.row(i));
    descriptors_keyframes_cuda_.push_back(std::move(descriptors_keyframe_cuda));
  } else
#endif
  {
    cv::Mat descriptors_keyframe(indexes.size(), descriptors_.cols,
                                 descriptors_.type());
    for (size_t i = 0; i < indexes.size(); ++i)
      descriptors_.row(indexes[i]).copyTo(descriptors_keyframe.row(i));
    descriptors_keyframes_.push_back(std::move(descriptors_keyframe));
  }

  // Store orientation
  orientation_last_keyframe_ =
      body2camera_pose_.rotation().inverse() *
      body2camera_pose_.translation().matrix().normalized();
  keyframe_age_ = 0;
}

bool TextureModality::Reconstruct3DPoint(const cv::Point2f &center,
                                         Eigen::Vector3f *center_f_body) const {
  const cv::Mat &silhouette_image{
      silhouette_renderer_ptr_->focused_silhouette_image()};
  const cv::Mat &depth_image{silhouette_renderer_ptr_->focused_depth_image()};
  uchar body_id = body_ptr_->body_id();

  // Compute coordinates for silhouette renderer
  int u_silhouette = int((center.x - silhouette_renderer_ptr_->corner_u()) *
                             silhouette_renderer_ptr_->scale() +
                         0.5f);
  int v_silhouette = int((center.y - silhouette_renderer_ptr_->corner_v()) *
                             silhouette_renderer_ptr_->scale() +
                         0.5f);
  if (u_silhouette < 0 || u_silhouette > silhouette_image_size_minus_1_ ||
      v_silhouette < 0 || v_silhouette > silhouette_image_size_minus_1_)
    return false;

  // Check if point is on body
  if (silhouette_image.at<uchar>(v_silhouette, u_silhouette) != body_id)
    return false;

  // Calculate 3D model point
  ushort depth_image_value = depth_image.at<ushort>(v_silhouette, u_silhouette);
  float depth = silhouette_renderer_ptr_->Depth(depth_image_value);
  Eigen::Vector3f center_f_camera{depth * (center.x - ppu_) / fu_,
                                  depth * (center.y - ppv_) / fv_, depth};
  *center_f_body = camera2body_pose_ * center_f_camera;
  return true;
}

bool TextureModality::IsPointValid(const Eigen::Vector3f &center_f_body,
                                   bool measure_occlusions,
                                   bool model_occlusions) {
  if (measure_occlusions)
    if (!IsPointUnoccludedMeasured(center_f_body)) return false;
  if (model_occlusions)
    if (!IsPointUnoccludedModeled(center_f_body)) return false;
  return true;
}

bool TextureModality::IsPointUnoccludedMeasured(
    const Eigen::Vector3f &center_f_body) const {
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
  ushort min_depth =
      ushort((center_f_depth_camera(2) - measured_occlusion_threshold_) /
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

bool TextureModality::IsPointUnoccludedModeled(
    const Eigen::Vector3f &center_f_body) const {
  Eigen::Vector3f center_f_camera{body2camera_pose_ * center_f_body};

  // Precalculate variables in pixel coordinates of focused image
  float meter_to_pixel =
      (fu_ / center_f_camera(2)) * depth_renderer_ptr_->scale();
  float diameter = 2.0f * modeled_occlusion_radius_ * meter_to_pixel;
  int stride = int(diameter / kMaxNOcclusionStrides + 1.0f);
  int n_strides = int(diameter / stride + 0.5f);
  int rounded_diameter = n_strides * stride;
  float rounded_radius = 0.5f * float(rounded_diameter);

  // Calculate limits for iteration in focused image
  float center_u = center_f_camera(0) * fu_ / center_f_camera(2) + ppu_;
  float center_v = center_f_camera(1) * fv_ / center_f_camera(2) + ppv_;
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
  float min_allowed_depth = center_f_camera(2) - modeled_occlusion_threshold_;
  return min_depth > min_allowed_depth;
}

void TextureModality::ShowAndSaveImage(const std::string &title, int save_index,
                                       const cv::Mat &image) const {
  if (display_visualization_) cv::imshow(title, image);
  if (save_visualizations_) {
    std::filesystem::path path{
        save_directory_ /
        (title + "_" + std::to_string(save_index) + "." + save_image_type_)};
    cv::imwrite(path.string(), image);
  }
}

void TextureModality::VisualizePointsColorImage(const std::string &title,
                                                int save_index) const {
  cv::Mat visualization_image;
  color_camera_ptr_->image().copyTo(visualization_image);
  DrawPoints(cv::Vec3b{24, 184, 234}, &visualization_image);
  ShowAndSaveImage(name_ + "_" + title, save_index, visualization_image);
}

void TextureModality::VisualizePointsDepthImage(const std::string &title,
                                                int save_index) const {
  cv::Mat visualization_image;
  cv::cvtColor(depth_camera_ptr_->NormalizedDepthImage(
                   visualization_min_depth_, visualization_max_depth_),
               visualization_image, cv::COLOR_GRAY2BGR);
  DrawDepthPoints(cv::Vec3b{187, 117, 0}, &visualization_image);
  ShowAndSaveImage(name_ + "_" + title, save_index, visualization_image);
}

void TextureModality::VisualizePointsDepthRendering(const std::string &title,
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

void TextureModality::VisualizePointsSilhouetteRendering(
    const std::string &title, int save_index) const {
  cv::Mat visualization_image;
  silhouette_renderer_ptr_->FetchDepthImage();
  cv::cvtColor(silhouette_renderer_ptr_->focused_silhouette_image(),
               visualization_image, cv::COLOR_GRAY2BGR);
  DrawFocusedPoints(silhouette_renderer_ptr_, cv::Vec3b{24, 184, 234},
                    &visualization_image);
  ShowAndSaveImage(name_ + "_" + title, save_index, visualization_image);
}

void TextureModality::VisualizeCorrespondences(const std::string &title,
                                               int save_index) const {
  cv::Mat visualization_image;
  color_camera_ptr_->image().copyTo(visualization_image);
  DrawCorrespondences(cv::Vec3b{187, 117, 0}, cv::Vec3b{61, 63, 179},
                      cv::Vec3b{24, 184, 234}, &visualization_image);
  ShowAndSaveImage(name_ + "_" + title, save_index, visualization_image);
}

void TextureModality::DrawPoints(const cv::Vec3b &color_point,
                                 cv::Mat *image) const {
  for (const auto &point : points_keyframes_.back()) {
    DrawPointInImage(body2camera_pose_ * point, color_point,
                     color_camera_ptr_->intrinsics(), image);
  }
}

void TextureModality::DrawDepthPoints(const cv::Vec3b &color_point,
                                      cv::Mat *image) const {
  for (const auto &point : points_keyframes_.back()) {
    DrawPointInImage(body2depth_camera_pose_ * point, color_point,
                     depth_camera_ptr_->intrinsics(), image);
  }
}

void TextureModality::DrawFocusedPoints(
    const std::shared_ptr<FocusedRenderer> &renderer_ptr,
    const cv::Vec3b &color_point, cv::Mat *image) const {
  for (const auto &point : points_keyframes_.back()) {
    DrawFocusedPointInImage(body2camera_pose_ * point, color_point,
                            renderer_ptr->intrinsics(),
                            renderer_ptr->corner_u(), renderer_ptr->corner_v(),
                            renderer_ptr->scale(), image);
  }
}

void TextureModality::DrawCorrespondences(
    const cv::Vec3b &color_point, const cv::Vec3b &color_correspondence_point,
    const cv::Vec3b &color_correspondence, cv::Mat *image) const {
  for (const auto &data_point : data_points_) {
    cv::Point2i center{int(data_point.center(0)), int(data_point.center(1))};
    cv::Point2i correspondence_center{int(data_point.correspondence_center(0)),
                                      int(data_point.correspondence_center(1))};
    cv::line(*image, center, correspondence_center, color_correspondence);
    cv::circle(*image, center, 1, color_point, cv::FILLED);
    cv::circle(*image, correspondence_center, 1, color_correspondence_point,
               cv::FILLED);
  }
}

float TextureModality::TukeyNorm(float error) {
  if (std::abs(error) <= tukey_norm_constant_)
    return powf(tukey_norm_constant_, 2.0f) / 6.0f *
           (1.0f - powf(1.0f - powf(error / tukey_norm_constant_, 2.0f), 3.0f));
  else
    return powf(tukey_norm_constant_, 2.0f) / 6.0f;
}

bool TextureModality::IsSetup() const {
  if (!set_up_) {
    std::cerr << "Set up texture modality " << name_ << " first" << std::endl;
    return false;
  }
  return true;
}

}  // namespace m3t
