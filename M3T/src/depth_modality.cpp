// SPDX-License-Identifier: MIT
// Copyright (c) 2023 Manuel Stoiber, German Aerospace Center (DLR)

#include <m3t/depth_modality.h>

namespace m3t {

DepthModality::DepthModality(
    const std::string &name, const std::shared_ptr<Body> &body_ptr,
    const std::shared_ptr<DepthCamera> &depth_camera_ptr,
    const std::shared_ptr<DepthModel> &depth_model_ptr)
    : Modality{name, body_ptr},
      depth_camera_ptr_{depth_camera_ptr},
      depth_model_ptr_{depth_model_ptr} {}

DepthModality::DepthModality(
    const std::string &name, const std::filesystem::path &metafile_path,
    const std::shared_ptr<Body> &body_ptr,
    const std::shared_ptr<DepthCamera> &depth_camera_ptr,
    const std::shared_ptr<DepthModel> &depth_model_ptr)
    : Modality{name, metafile_path, body_ptr},
      depth_camera_ptr_{depth_camera_ptr},
      depth_model_ptr_{depth_model_ptr} {}

bool DepthModality::SetUp() {
  set_up_ = false;
  if (!metafile_path_.empty())
    if (!LoadMetaData()) return false;

  // Check if all required objects are set up
  if (!body_ptr_->set_up()) {
    std::cerr << "Body " << body_ptr_->name() << " was not set up" << std::endl;
    return false;
  }
  if (!depth_model_ptr_->set_up()) {
    std::cerr << "Depth model " << depth_model_ptr_->name() << " was not set up"
              << std::endl;
    return false;
  }
  if (!depth_camera_ptr_->set_up()) {
    std::cerr << "Depth camera " << depth_camera_ptr_->name()
              << " was not set up" << std::endl;
    return false;
  }
  if (model_occlusions_ && !depth_renderer_ptr_->set_up()) {
    std::cerr << "Focused depth renderer " << depth_renderer_ptr_->name()
              << " was not set up" << std::endl;
    return false;
  }
  if (use_silhouette_checking_ && !silhouette_renderer_ptr_->set_up()) {
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
  if (use_silhouette_checking_ &&
      silhouette_renderer_ptr_->id_type() != IDType::BODY) {
    std::cerr << "Focused silhouette renderer "
              << silhouette_renderer_ptr_->name()
              << " does not use id_type BODY" << std::endl;
  }
  if (use_silhouette_checking_ &&
      !silhouette_renderer_ptr_->IsBodyReferenced(body_ptr_->name())) {
    std::cerr << "Focused silhouette renderer "
              << silhouette_renderer_ptr_->name() << " does not reference body "
              << body_ptr_->name() << std::endl;
    return false;
  }

  PrecalculateCameraVariables();
  PrecalculateRendererVariables();
  SetImshowVariables();

  set_up_ = true;
  return true;
}

void DepthModality::set_depth_camera_ptr(
    const std::shared_ptr<DepthCamera> &depth_camera_ptr) {
  depth_camera_ptr_ = depth_camera_ptr;
  set_up_ = false;
}

void DepthModality::set_depth_model_ptr(
    const std::shared_ptr<DepthModel> &depth_model_ptr) {
  depth_model_ptr_ = depth_model_ptr;
  set_up_ = false;
}

void DepthModality::set_n_points_max(int n_points_max) {
  n_points_max_ = n_points_max;
}

void DepthModality::set_use_adaptive_coverage(bool use_adaptive_coverage) {
  use_adaptive_coverage_ = use_adaptive_coverage;
}

void DepthModality::set_use_depth_scaling(bool use_depth_scaling) {
  use_depth_scaling_ = use_depth_scaling;
}

void DepthModality::set_reference_surface_area(float reference_surface_area) {
  reference_surface_area_ = reference_surface_area;
}

void DepthModality::set_stride_length(float stride_length) {
  stride_length_ = stride_length;
}

void DepthModality::set_considered_distances(
    const std::vector<float> &considered_distances) {
  considered_distances_ = considered_distances;
}

void DepthModality::set_standard_deviations(
    const std::vector<float> &standard_deviations) {
  standard_deviations_ = standard_deviations;
}

void DepthModality::UseSilhouetteChecking(
    const std::shared_ptr<FocusedSilhouetteRenderer> &silhouette_renderer_ptr) {
  silhouette_renderer_ptr_ = silhouette_renderer_ptr;
  use_silhouette_checking_ = true;
  set_up_ = false;
}

void DepthModality::DoNotUseSilhouetteChecking() {
  silhouette_renderer_ptr_ = nullptr;
  use_silhouette_checking_ = false;
  set_up_ = false;
}

void DepthModality::MeasureOcclusions() {
  measure_occlusions_ = true;
  set_up_ = false;
}

void DepthModality::DoNotMeasureOcclusions() {
  measure_occlusions_ = false;
  set_up_ = false;
}

void DepthModality::ModelOcclusions(
    const std::shared_ptr<FocusedDepthRenderer> &depth_renderer_ptr) {
  depth_renderer_ptr_ = depth_renderer_ptr;
  model_occlusions_ = true;
  set_up_ = false;
}

void DepthModality::DoNotModelOcclusions() {
  depth_renderer_ptr_ = nullptr;
  model_occlusions_ = false;
  set_up_ = false;
}

void DepthModality::set_measured_depth_offset_radius(
    float measured_depth_offset_radius) {
  measured_depth_offset_radius_ = measured_depth_offset_radius;
}

void DepthModality::set_measured_occlusion_radius(
    float measured_occlusion_radius) {
  measured_occlusion_radius_ = measured_occlusion_radius;
}

void DepthModality::set_measured_occlusion_threshold(
    float measured_occlusion_threshold) {
  measured_occlusion_threshold_ = measured_occlusion_threshold;
}

void DepthModality::set_modeled_depth_offset_radius(
    float modeled_depth_offset_radius) {
  modeled_depth_offset_radius_ = modeled_depth_offset_radius;
}

void DepthModality::set_modeled_occlusion_radius(
    float modeled_occlusion_radius) {
  modeled_occlusion_radius_ = modeled_occlusion_radius;
}

void DepthModality::set_modeled_occlusion_threshold(
    float modeled_occlusion_threshold) {
  modeled_occlusion_threshold_ = modeled_occlusion_threshold;
}

void DepthModality::set_n_unoccluded_iterations(int n_unoccluded_iterations) {
  n_unoccluded_iterations_ = n_unoccluded_iterations;
}

void DepthModality::set_min_n_unoccluded_points(int min_n_unoccluded_points) {
  min_n_unoccluded_points_ = min_n_unoccluded_points;
}

void DepthModality::set_visualize_correspondences_correspondence(
    bool visualize_correspondences_correspondence) {
  visualize_correspondences_correspondence_ =
      visualize_correspondences_correspondence;
  SetImshowVariables();
}

void DepthModality::set_visualize_points_correspondence(
    bool visualize_points_correspondence) {
  visualize_points_correspondence_ = visualize_points_correspondence;
  SetImshowVariables();
}

void DepthModality::set_visualize_points_depth_rendering_correspondence(
    bool visualize_points_depth_rendering_correspondence) {
  visualize_points_depth_rendering_correspondence_ =
      visualize_points_depth_rendering_correspondence;
  SetImshowVariables();
}

void DepthModality::set_visualize_points_silhouette_rendering_correspondence(
    bool visualize_points_silhouette_rendering_correspondence) {
  visualize_points_silhouette_rendering_correspondence_ =
      visualize_points_silhouette_rendering_correspondence;
  SetImshowVariables();
}

void DepthModality::set_visualize_points_optimization(
    bool visualize_points_optimization) {
  visualize_points_optimization_ = visualize_points_optimization;
  SetImshowVariables();
}

void DepthModality::set_visualize_points_result(bool visualize_points_result) {
  visualize_points_result_ = visualize_points_result;
  SetImshowVariables();
}

void DepthModality::set_visualization_min_depth(float visualization_min_depth) {
  visualization_min_depth_ = visualization_min_depth;
}

void DepthModality::set_visualization_max_depth(float visualization_max_depth) {
  visualization_max_depth_ = visualization_max_depth;
}

bool DepthModality::StartModality(int iteration, int corr_iteration) {
  return IsSetup();
}

bool DepthModality::CalculateCorrespondences(int iteration,
                                             int corr_iteration) {
  if (!IsSetup()) return false;

  PrecalculatePoseVariables();
  PrecalculateIterationDependentVariables(corr_iteration);

  // Check if body is visible and fetch image from renderer
  bool body_visible_depth;
  if (model_occlusions_) {
    body_visible_depth = depth_renderer_ptr_->IsBodyVisible(body_ptr_->name());
    if (body_visible_depth) depth_renderer_ptr_->FetchDepthImage();
  }
  bool body_visible_silhouette;
  if (use_silhouette_checking_) {
    body_visible_silhouette =
        silhouette_renderer_ptr_->IsBodyVisible(body_ptr_->name());
    if (body_visible_silhouette)
      silhouette_renderer_ptr_->FetchSilhouetteImage();
  }

  // Search closest template view
  const DepthModel::View *view;
  depth_model_ptr_->GetClosestView(body2camera_pose_, &view);
  auto &data_model_points{view->data_points};

  // Scale number of points with surface_area ratio
  int n_points = n_points_max_;
  if (use_adaptive_coverage_) {
    if (reference_surface_area_ > 0.0f)
      n_points = n_points_max_ *
                 std::min(1.0f, view->surface_area / reference_surface_area_);
    else
      n_points = n_points_max_ * view->surface_area /
                 depth_model_ptr_->max_surface_area();
  }
  if (n_points > data_model_points.size()) {
    std::cerr << "Number of model points too small: "
              << data_model_points.size() << " < " << n_points << std::endl;
    n_points = data_model_points.size();
  }

  // Iterate over n_points
  for (int j = 0; j < 2; ++j) {
    data_points_.clear();
    bool handle_occlusions =
        j == 0 && (iteration - first_iteration_) >= n_unoccluded_iterations_;
    for (int i = 0; i < n_points; ++i) {
      DataPoint data_point;
      CalculateBasicPointData(data_model_points[i], &data_point);
      if (!IsPointValid(
              data_point, use_silhouette_checking_ && body_visible_silhouette,
              handle_occlusions && measure_occlusions_,
              handle_occlusions && model_occlusions_ && body_visible_depth))
        continue;
      if (!FindCorrespondence(data_point,
                              &data_point.correspondence_center_f_camera))
        continue;
      data_points_.push_back(std::move(data_point));
    }
    if (data_points_.size() >= min_n_unoccluded_points_) break;
  }
  return true;
}

bool DepthModality::VisualizeCorrespondences(int save_idx) {
  if (!IsSetup()) return false;

  if (visualize_correspondences_correspondence_)
    VisualizeCorrespondences("correspondences_correspondence", save_idx);
  if (visualize_points_correspondence_)
    VisualizePointsDepthImage("depth_image_correspondence", save_idx);
  if (visualize_points_depth_rendering_correspondence_ && model_occlusions_)
    VisualizePointsDepthRendering("depth_rendering_correspondence", save_idx);
  if (visualize_points_silhouette_rendering_correspondence_ &&
      use_silhouette_checking_)
    VisualizePointsSilhouetteRendering("silhouette_rendering_correspondence",
                                       save_idx);
  return true;
}

bool DepthModality::CalculateGradientAndHessian(int iteration,
                                                int corr_iteration,
                                                int opt_iteration) {
  if (!IsSetup()) return false;

  PrecalculatePoseVariables();
  gradient_.setZero();
  hessian_.setZero();

  for (auto &data_point : data_points_) {
    // Calculate correspondence point coordinates in body frame
    Eigen::Vector3f correspondence_center_f_body =
        camera2body_pose_ * data_point.correspondence_center_f_camera;

    // Calculate intermediate variables
    float epsilon = data_point.normal_f_body.dot(data_point.center_f_body -
                                                 correspondence_center_f_body);
    Eigen::Vector3f correspondence_point_cross_normal =
        correspondence_center_f_body.cross(data_point.normal_f_body);

    // Calculate weight
    float correspondence_depth = data_point.correspondence_center_f_camera(2);
    float weight = 1.0f / (standard_deviation_ * correspondence_depth);
    float squared_weight = weight * weight;

    // Calculate weighted vectors
    Eigen::Vector3f weighted_correspondence_point_cross_normal =
        weight * correspondence_point_cross_normal;
    Eigen::Vector3f weighted_normal = weight * data_point.normal_f_body;

    // Calculate gradient
    gradient_.head<3>() -=
        (squared_weight * epsilon) * correspondence_point_cross_normal;
    gradient_.tail<3>() -=
        (squared_weight * epsilon) * data_point.normal_f_body;

    // Calculate hessian
    hessian_.topLeftCorner<3, 3>().triangularView<Eigen::Upper>() -=
        weighted_correspondence_point_cross_normal *
        weighted_correspondence_point_cross_normal.transpose();
    hessian_.topRightCorner<3, 3>() -=
        weighted_correspondence_point_cross_normal *
        weighted_normal.transpose();
    hessian_.bottomRightCorner<3, 3>().triangularView<Eigen::Upper>() -=
        weighted_normal * weighted_normal.transpose();
  }
  hessian_ = hessian_.selfadjointView<Eigen::Upper>();
  return true;
}

bool DepthModality::VisualizeOptimization(int save_idx) {
  if (!IsSetup()) return false;

  if (visualize_points_optimization_)
    VisualizePointsDepthImage("depth_image_optimization", save_idx);
  if (visualize_gradient_optimization_) VisualizeGradient();
  if (visualize_hessian_optimization_) VisualizeHessian();
  return true;
}

bool DepthModality::CalculateResults(int iteration) { return IsSetup(); }

bool DepthModality::VisualizeResults(int save_idx) {
  if (!IsSetup()) return false;

  if (visualize_points_result_)
    VisualizePointsDepthImage("depth_image_result", save_idx);
  if (visualize_pose_result_) VisualizePose();
  return true;
}

const std::shared_ptr<DepthCamera> &DepthModality::depth_camera_ptr() const {
  return depth_camera_ptr_;
}

const std::shared_ptr<DepthModel> &DepthModality::depth_model_ptr() const {
  return depth_model_ptr_;
}

const std::shared_ptr<FocusedDepthRenderer> &DepthModality::depth_renderer_ptr()
    const {
  return depth_renderer_ptr_;
}

const std::shared_ptr<FocusedSilhouetteRenderer>
    &DepthModality::silhouette_renderer_ptr() const {
  return silhouette_renderer_ptr_;
}

std::shared_ptr<Model> DepthModality::model_ptr() const {
  return depth_model_ptr_;
}

std::vector<std::shared_ptr<Camera>> DepthModality::camera_ptrs() const {
  return {depth_camera_ptr_};
}

std::vector<std::shared_ptr<Renderer>>
DepthModality::start_modality_renderer_ptrs() const {
  return {};
}

std::vector<std::shared_ptr<Renderer>>
DepthModality::correspondence_renderer_ptrs() const {
  return {depth_renderer_ptr_, silhouette_renderer_ptr_};
}

std::vector<std::shared_ptr<Renderer>> DepthModality::results_renderer_ptrs()
    const {
  return {};
}

int DepthModality::n_points_max() const { return n_points_max_; }

bool DepthModality::use_adaptive_coverage() const {
  return use_adaptive_coverage_;
}

bool DepthModality::use_depth_scaling() const { return use_depth_scaling_; }

float DepthModality::reference_surface_area() const {
  return reference_surface_area_;
}

float DepthModality::stride_length() const { return stride_length_; }

const std::vector<float> &DepthModality::considered_distances() const {
  return considered_distances_;
}

const std::vector<float> &DepthModality::standard_deviations() const {
  return standard_deviations_;
}

bool DepthModality::use_silhouette_checking() const {
  return use_silhouette_checking_;
}

bool DepthModality::measure_occlusions() const { return measure_occlusions_; }

float DepthModality::measured_depth_offset_radius() const {
  return measured_depth_offset_radius_;
}

float DepthModality::measured_occlusion_radius() const {
  return measured_occlusion_radius_;
}

float DepthModality::measured_occlusion_threshold() const {
  return measured_occlusion_threshold_;
}

bool DepthModality::model_occlusions() const { return model_occlusions_; }

float DepthModality::modeled_depth_offset_radius() const {
  return modeled_depth_offset_radius_;
}

float DepthModality::modeled_occlusion_radius() const {
  return modeled_occlusion_radius_;
}

float DepthModality::modeled_occlusion_threshold() const {
  return modeled_occlusion_threshold_;
}

int DepthModality::n_unoccluded_iterations() const {
  return n_unoccluded_iterations_;
}

int DepthModality::min_n_unoccluded_points() const {
  return min_n_unoccluded_points_;
}

bool DepthModality::visualize_correspondences_correspondence() const {
  return visualize_correspondences_correspondence_;
}

bool DepthModality::visualize_points_correspondence() const {
  return visualize_points_correspondence_;
}

bool DepthModality::visualize_points_depth_rendering_correspondence() const {
  return visualize_points_depth_rendering_correspondence_;
}

bool DepthModality::visualize_points_silhouette_rendering_correspondence()
    const {
  return visualize_points_silhouette_rendering_correspondence_;
}

bool DepthModality::visualize_points_optimization() const {
  return visualize_points_optimization_;
}

bool DepthModality::visualize_points_result() const {
  return visualize_points_result_;
}

float DepthModality::visualization_min_depth() const {
  return visualization_min_depth_;
}

float DepthModality::visualization_max_depth() const {
  return visualization_max_depth_;
}

bool DepthModality::LoadMetaData() {
  // Open file storage from yaml
  cv::FileStorage fs;
  if (!OpenYamlFileStorage(metafile_path_, &fs)) return false;

  // Read parameters from yaml file for general distribution
  ReadOptionalValueFromYaml(fs, "n_points_max", &n_points_max_);
  ReadOptionalValueFromYaml(fs, "use_adaptive_coverage",
                            &use_adaptive_coverage_);
  ReadOptionalValueFromYaml(fs, "use_depth_scaling", &use_depth_scaling_);
  ReadOptionalValueFromYaml(fs, "reference_surface_area",
                            &reference_surface_area_);
  ReadOptionalValueFromYaml(fs, "stride_length", &stride_length_);
  ReadOptionalValueFromYaml(fs, "considered_distances", &considered_distances_);
  ReadOptionalValueFromYaml(fs, "standard_deviations", &standard_deviations_);

  // Read parameters from yaml file for occlusion handling
  ReadOptionalValueFromYaml(fs, "measure_occlusions", &measure_occlusions_);
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
  ReadOptionalValueFromYaml(fs, "min_n_unoccluded_points",
                            &min_n_unoccluded_points_);

  // Read parameters from yaml for visualization
  ReadOptionalValueFromYaml(fs, "visualize_pose_result",
                            &visualize_pose_result_);
  ReadOptionalValueFromYaml(fs, "visualize_gradient_optimization",
                            &visualize_gradient_optimization_);
  ReadOptionalValueFromYaml(fs, "visualize_hessian_optimization",
                            &visualize_hessian_optimization_);
  ReadOptionalValueFromYaml(fs, "visualize_correspondences_correspondence",
                            &visualize_correspondences_correspondence_);
  ReadOptionalValueFromYaml(fs, "visualize_points_correspondence",
                            &visualize_points_correspondence_);
  ReadOptionalValueFromYaml(fs,
                            "visualize_points_depth_rendering_correspondence",
                            &visualize_points_depth_rendering_correspondence_);
  ReadOptionalValueFromYaml(
      fs, "visualize_points_silhouette_rendering_correspondence",
      &visualize_points_silhouette_rendering_correspondence_);
  ReadOptionalValueFromYaml(fs, "visualize_points_optimization",
                            &visualize_points_optimization_);
  ReadOptionalValueFromYaml(fs, "visualize_points_result",
                            &visualize_points_result_);
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

void DepthModality::SetImshowVariables() {
  imshow_correspondence_ =
      display_visualization_ &&
      (visualize_correspondences_correspondence_ ||
       visualize_points_correspondence_ ||
       (visualize_points_depth_rendering_correspondence_ &&
        model_occlusions_) ||
       (visualize_points_silhouette_rendering_correspondence_ &&
        use_silhouette_checking_));
  imshow_optimization_ =
      display_visualization_ && visualize_points_optimization_;
  imshow_result_ = display_visualization_ && visualize_points_result_;
}

void DepthModality::PrecalculateCameraVariables() {
  fu_ = depth_camera_ptr_->intrinsics().fu;
  fv_ = depth_camera_ptr_->intrinsics().fv;
  ppu_ = depth_camera_ptr_->intrinsics().ppu;
  ppv_ = depth_camera_ptr_->intrinsics().ppv;
  image_width_minus_1_ = depth_camera_ptr_->intrinsics().width - 1;
  image_height_minus_1_ = depth_camera_ptr_->intrinsics().height - 1;
  depth_scale_ = depth_camera_ptr_->depth_scale();
}

void DepthModality::PrecalculateRendererVariables() {
  if (model_occlusions_)
    depth_image_size_minus_1_ = depth_renderer_ptr_->image_size() - 1;
}

void DepthModality::PrecalculatePoseVariables() {
  body2camera_pose_ =
      depth_camera_ptr_->world2camera_pose() * body_ptr_->body2world_pose();
  camera2body_pose_ = body2camera_pose_.inverse();
  body2camera_rotation_ = body2camera_pose_.rotation().matrix();
}

void DepthModality::PrecalculateIterationDependentVariables(
    int corr_iteration) {
  considered_distance_ = LastValidValue(considered_distances_, corr_iteration);
  max_n_strides_ = int(considered_distance_ / stride_length_ + 0.5f);

  standard_deviation_ = LastValidValue(standard_deviations_, corr_iteration);
}

void DepthModality::CalculateBasicPointData(
    const DepthModel::DataPoint &data_model_point,
    DataPoint *data_point) const {
  Eigen::Vector3f center_f_camera{body2camera_pose_ *
                                  data_model_point.center_f_body};
  data_point->center_f_body = data_model_point.center_f_body;
  data_point->normal_f_body = data_model_point.normal_f_body;
  data_point->center_u = center_f_camera(0) * fu_ / center_f_camera(2) + ppu_;
  data_point->center_v = center_f_camera(1) * fv_ / center_f_camera(2) + ppv_;
  data_point->depth = center_f_camera(2);
  data_point->center_f_camera = std::move(center_f_camera);

  // Select depth offset
  float stride = depth_model_ptr_->stride_depth_offset();
  float max_radius = depth_model_ptr_->max_radius_depth_offset();
  if (measure_occlusions_) {
    float radius = measured_depth_offset_radius_;
    if (use_depth_scaling_) radius *= data_point->depth;
    int id = int(radius / stride + 0.5f);
    if (id >= data_model_point.depth_offsets.size()) {
      std::cerr << "Measured depth offset radius too large: " << radius
                << std::endl;
      data_point->measured_depth_offset = data_model_point.depth_offsets.back();
    } else {
      data_point->measured_depth_offset = data_model_point.depth_offsets[id];
    }
  }
  if (model_occlusions_) {
    float radius = modeled_depth_offset_radius_;
    if (use_depth_scaling_) radius *= data_point->depth;
    int id = int(radius / stride + 0.5f);
    if (id >= data_model_point.depth_offsets.size()) {
      std::cerr << "Modelled depth offset radius too large: " << radius
                << std::endl;
      data_point->modeled_depth_offset = data_model_point.depth_offsets.back();
    } else {
      data_point->modeled_depth_offset = data_model_point.depth_offsets[id];
    }
  }
}

bool DepthModality::IsPointValid(const DataPoint &data_point,
                                 bool use_silhouette_checking,
                                 bool measure_occlusions,
                                 bool model_occlusions) const {
  // Check if point is in front of image
  if (data_point.depth <= 0.0f) return false;

  // Check if image coordinate is on image
  int i_center_u = int(data_point.center_u + 0.5f);
  int i_center_v = int(data_point.center_v + 0.5f);
  if (i_center_u < 0 || i_center_u > image_width_minus_1_ || i_center_v < 0 ||
      i_center_v > image_height_minus_1_)
    return false;

  // Check silhouette
  if (use_silhouette_checking) {
    if (!IsPointOnValidSilhouette(data_point)) return false;
  }

  // Check measured occlusions
  if (measure_occlusions) {
    if (!IsPointUnoccludedMeasured(data_point)) return false;
  }

  // Check modeled occlusions
  if (model_occlusions) {
    if (!IsPointUnoccludedModeled(data_point)) return false;
  }
  return true;
}

bool DepthModality::IsPointOnValidSilhouette(
    const DataPoint &data_point) const {
  cv::Point2i center{int(data_point.center_u + 0.5f),
                     int(data_point.center_v + 0.5f)};
  uchar silhouette_value = silhouette_renderer_ptr_->SilhouetteValue(center);
  return silhouette_value == body_ptr_->body_id();
}

bool DepthModality::IsPointUnoccludedMeasured(
    const DataPoint &data_point) const {
  // Precalculate variables in pixel coordinates
  float diameter = 2.0f * measured_occlusion_radius_ * fu_;
  if (!use_depth_scaling_) diameter /= data_point.depth;
  int stride = int(diameter / kMaxNOcclusionStrides + 1.0f);
  int n_strides = int(diameter / stride + 0.5f);
  int rounded_diameter = n_strides * stride;
  float rounded_radius = 0.5f * float(rounded_diameter);

  // Calculate limits for iteration
  int u_min = int(data_point.center_u - rounded_radius + 0.5f);
  int v_min = int(data_point.center_v - rounded_radius + 0.5f);
  int u_max = u_min + rounded_diameter;
  int v_max = v_min + rounded_diameter;
  u_min = std::max(u_min, 0);
  v_min = std::max(v_min, 0);
  u_max = std::min(u_max, image_width_minus_1_);
  v_max = std::min(v_max, image_height_minus_1_);

  // Compute minimum valid depth
  float threshold = measured_occlusion_threshold_;
  if (use_depth_scaling_) threshold *= data_point.depth;
  ushort min_depth =
      ushort((data_point.depth - data_point.measured_depth_offset - threshold) /
             depth_scale_);

  // Iterate over pixels to check if depth value is big enough
  ushort depth;
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

bool DepthModality::IsPointUnoccludedModeled(
    const DataPoint &data_point) const {
  // Precalculate variables in pixel coordinates of focused image
  float meter_to_pixel = fu_ * depth_renderer_ptr_->scale();
  if (!use_depth_scaling_) meter_to_pixel /= data_point.depth;
  float diameter = 2.0f * modeled_occlusion_radius_ * meter_to_pixel;
  int stride = int(diameter / kMaxNOcclusionStrides + 1.0f);
  int n_strides = int(diameter / stride + 0.5f);
  int rounded_diameter = n_strides * stride;
  float rounded_radius = 0.5f * float(rounded_diameter);

  // Calculate limits for iteration in focused image
  float focused_center_u =
      (data_point.center_u - depth_renderer_ptr_->corner_u()) *
      depth_renderer_ptr_->scale();
  float focused_center_v =
      (data_point.center_v - depth_renderer_ptr_->corner_v()) *
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

  // Validate minimum depth value
  float threshold = modeled_occlusion_threshold_;
  if (use_depth_scaling_) threshold *= data_point.depth;
  float min_allowed_depth =
      data_point.depth - data_point.modeled_depth_offset - threshold;
  float min_depth = depth_renderer_ptr_->Depth(min_depth_value);
  return min_depth > min_allowed_depth;
}

bool DepthModality::FindCorrespondence(
    const DataPoint &data_point,
    Eigen::Vector3f *correspondence_center_f_camera) const {
  float considered_distance = considered_distance_;
  if (use_depth_scaling_) considered_distance *= data_point.depth;

  // Precalculate variables in pixel coordinates
  float meter_to_pixel = fu_ / data_point.depth;
  float diameter = 2.0f * considered_distance * meter_to_pixel;
  int stride = int(diameter / max_n_strides_ + 1.0f);
  int n_strides = int(diameter / stride + 0.5f);
  int rounded_diameter = n_strides * stride;
  float rounded_radius = 0.5f * float(rounded_diameter);

  // Calculate limits for iteration
  int u_min = int(data_point.center_u - rounded_radius + 0.5f);
  int v_min = int(data_point.center_v - rounded_radius + 0.5f);
  int u_max = u_min + rounded_diameter;
  int v_max = v_min + rounded_diameter;
  u_min = std::max(u_min, 0);
  v_min = std::max(v_min, 0);
  u_max = std::min(u_max, image_width_minus_1_);
  v_max = std::min(v_max, image_height_minus_1_);

  // Calculate limits for depth value
  float min_depth_value =
      std::min(0.0f, (data_point.depth - considered_distance) / depth_scale_);
  float max_depth_value =
      (data_point.depth + considered_distance) / depth_scale_;

  // Iterate over all pixels to find closest point
  float min_considered_distance_square = square(considered_distance);
  float min_measured_distance_square = min_considered_distance_square;
  float measured_distance_square;
  int u, v;
  const cv::Mat &image{depth_camera_ptr_->image()};
  const ushort *ptr_image;
  float depth;
  Eigen::Vector3f temp_point;
  const Eigen::Vector3f &center_f_camera{data_point.center_f_camera};
  for (v = v_min; v <= v_max; v += stride) {
    ptr_image = image.ptr<ushort>(v);
    for (u = u_min; u <= u_max; u += stride) {
      depth = float(ptr_image[u]);
      if (depth > min_depth_value && depth < max_depth_value) {
        depth *= depth_scale_;
        temp_point(0) = (float(u) - ppu_) * depth / fu_;
        temp_point(1) = (float(v) - ppv_) * depth / fv_;
        temp_point(2) = depth;
        measured_distance_square = (temp_point - center_f_camera).squaredNorm();
        if (measured_distance_square < min_measured_distance_square) {
          *correspondence_center_f_camera = temp_point;
          min_measured_distance_square = measured_distance_square;
        }
      }
    }
  }
  return min_measured_distance_square != min_considered_distance_square;
}

void DepthModality::ShowAndSaveImage(const std::string &title, int save_index,
                                     const cv::Mat &image) const {
  if (display_visualization_) cv::imshow(title, image);
  if (save_visualizations_) {
    std::filesystem::path path{
        save_directory_ /
        (title + "_" + std::to_string(save_index) + "." + save_image_type_)};
    cv::imwrite(path.string(), image);
  }
}

void DepthModality::VisualizePointsDepthImage(const std::string &title,
                                              int save_index) const {
  cv::Mat visualization_image;
  cv::cvtColor(depth_camera_ptr_->NormalizedDepthImage(
                   visualization_min_depth_, visualization_max_depth_),
               visualization_image, cv::COLOR_GRAY2BGR);
  DrawPoints(cv::Vec3b{187, 117, 0}, &visualization_image);
  ShowAndSaveImage(name_ + "_" + title, save_index, visualization_image);
}

void DepthModality::VisualizePointsDepthRendering(const std::string &title,
                                                  int save_index) const {
  cv::Mat visualization_image;
  depth_renderer_ptr_->FetchDepthImage();
  cv::cvtColor(depth_renderer_ptr_->NormalizedFocusedDepthImage(
                   visualization_min_depth_, visualization_max_depth_),
               visualization_image, cv::COLOR_GRAY2BGR);
  DrawFocusedPoints(depth_renderer_ptr_, cv::Vec3b{187, 117, 0},
                    &visualization_image);
  ShowAndSaveImage(name_ + "_" + title, save_index, visualization_image);
}

void DepthModality::VisualizePointsSilhouetteRendering(const std::string &title,
                                                       int save_index) const {
  cv::Mat visualization_image;
  silhouette_renderer_ptr_->FetchSilhouetteImage();
  cv::cvtColor(silhouette_renderer_ptr_->focused_silhouette_image(),
               visualization_image, cv::COLOR_GRAY2BGR);
  DrawFocusedPoints(silhouette_renderer_ptr_, cv::Vec3b{24, 184, 234},
                    &visualization_image);
  ShowAndSaveImage(name_ + "_" + title, save_index, visualization_image);
}

void DepthModality::VisualizeCorrespondences(const std::string &title,
                                             int save_index) const {
  cv::Mat visualization_image;
  cv::cvtColor(depth_camera_ptr_->NormalizedDepthImage(
                   visualization_min_depth_, visualization_max_depth_),
               visualization_image, cv::COLOR_GRAY2BGR);
  DrawCorrespondences(cv::Vec3b{187, 117, 0}, cv::Vec3b{24, 184, 234},
                      cv::Vec3b{61, 63, 179}, &visualization_image);
  ShowAndSaveImage(name_ + "_" + title, save_index, visualization_image);
}

void DepthModality::DrawPoints(const cv::Vec3b &color_point,
                               cv::Mat *image) const {
  for (const auto &data_point : data_points_) {
    DrawPointInImage(body2camera_pose_ * data_point.center_f_body, color_point,
                     depth_camera_ptr_->intrinsics(), image);
  }
}

void DepthModality::DrawFocusedPoints(
    const std::shared_ptr<FocusedRenderer> &renderer_ptr,
    const cv::Vec3b &color_point, cv::Mat *image) const {
  for (const auto &data_point : data_points_) {
    DrawFocusedPointInImage(body2camera_pose_ * data_point.center_f_body,
                            color_point, renderer_ptr->intrinsics(),
                            renderer_ptr->corner_u(), renderer_ptr->corner_v(),
                            renderer_ptr->scale(), image);
  }
}

void DepthModality::DrawCorrespondences(
    const cv::Vec3b &color_point, const cv::Vec3b &color_correspondence_point,
    const cv::Vec3b &color_correspondence, cv::Mat *image) const {
  for (const auto &data_point : data_points_) {
    DrawLineInImage(body2camera_pose_ * data_point.center_f_body,
                    data_point.correspondence_center_f_camera,
                    color_correspondence, depth_camera_ptr_->intrinsics(),
                    image);
    DrawPointInImage(body2camera_pose_ * data_point.center_f_body, color_point,
                     depth_camera_ptr_->intrinsics(), image);
    DrawPointInImage(data_point.correspondence_center_f_camera,
                     color_correspondence_point,
                     depth_camera_ptr_->intrinsics(), image);
  }
}

bool DepthModality::IsSetup() const {
  if (!set_up_) {
    std::cerr << "Set up depth modality " << name_ << " first" << std::endl;
    return false;
  }
  return true;
}

}  // namespace m3t
