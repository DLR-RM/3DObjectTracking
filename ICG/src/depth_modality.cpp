// SPDX-License-Identifier: MIT
// Copyright (c) 2022 Manuel Stoiber, German Aerospace Center (DLR)

#include <icg/depth_modality.h>

namespace icg {

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

  PrecalculateCameraVariables();
  if (!PrecalculateModelVariables()) return false;
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

void DepthModality::set_n_points(int n_points) { n_points_ = n_points; }

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

  // Search closest template view
  const DepthModel::View *view;
  depth_model_ptr_->GetClosestView(body2camera_pose_, &view);
  auto &data_model_points{view->data_points};

  // Iterate over n_points
  for (int j = 0; j < 2; ++j) {
    data_points_.clear();
    bool handle_occlusions =
        j == 0 && (iteration - first_iteration_) >= n_unoccluded_iterations_;
    for (int i = 0; i < n_points_; ++i) {
      DataPoint data_point;
      CalculateBasicPointData(data_model_points[i], &data_point);
      if (!IsPointValid(
              data_point, handle_occlusions && measure_occlusions_,
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

  if (visualize_points_optimization_) {
    VisualizePointsDepthImage("depth_image_optimization", save_idx);
  }
  if (visualize_gradient_optimization_) {
    VisualizeGradient();
  }
  if (visualize_hessian_optimization_) {
    VisualizeHessian();
  }
  return true;
}

bool DepthModality::CalculateResults(int iteration) { return IsSetup(); }

bool DepthModality::VisualizeResults(int save_idx) {
  if (!IsSetup()) return false;

  if (visualize_points_result_) {
    VisualizePointsDepthImage("depth_image_result", save_idx);
  }
  if (visualize_pose_result_) {
    VisualizePose();
  }
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

std::shared_ptr<Model> DepthModality::model_ptr() const {
  return depth_model_ptr_;
}

std::vector<std::shared_ptr<Camera>> DepthModality::camera_ptrs() const {
  return {depth_camera_ptr_};
}

std::vector<std::shared_ptr<Renderer>>
DepthModality::start_modality_renderer_ptrs() const {
  return {depth_renderer_ptr_};
}

std::vector<std::shared_ptr<Renderer>>
DepthModality::correspondence_renderer_ptrs() const {
  return {depth_renderer_ptr_};
}

std::vector<std::shared_ptr<Renderer>> DepthModality::results_renderer_ptrs()
    const {
  return {depth_renderer_ptr_};
}

int DepthModality::n_points() const { return n_points_; }

float DepthModality::stride_length() const { return stride_length_; }

const std::vector<float> &DepthModality::considered_distances() const {
  return considered_distances_;
}

const std::vector<float> &DepthModality::standard_deviations() const {
  return standard_deviations_;
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
  ReadOptionalValueFromYaml(fs, "n_points", &n_points_);
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
       (visualize_points_depth_rendering_correspondence_ && model_occlusions_));
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

bool DepthModality::PrecalculateModelVariables() {
  float stride = depth_model_ptr_->stride_depth_offset();
  float max_radius = depth_model_ptr_->max_radius_depth_offset();
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

void DepthModality::PrecalculateRendererVariables() {
  if (model_occlusions_) {
    image_size_minus_1_ = depth_renderer_ptr_->image_size() - 1;
  }
}

void DepthModality::PrecalculatePoseVariables() {
  body2camera_pose_ =
      depth_camera_ptr_->world2camera_pose() * body_ptr_->body2world_pose();
  camera2body_pose_ = body2camera_pose_.inverse();
  body2camera_rotation_ = body2camera_pose_.rotation().matrix();
}

void DepthModality::PrecalculateIterationDependentVariables(
    int corr_iteration) {
  if (corr_iteration < int(considered_distances_.size()))
    considered_distance_ = considered_distances_[corr_iteration];
  else
    considered_distance_ = considered_distances_.back();
  considered_distance_square_ = powf(considered_distance_, 2.0f);
  max_n_strides_ = int(considered_distance_ / stride_length_ + 0.5f);

  if (corr_iteration < int(standard_deviations_.size()))
    standard_deviation_ = standard_deviations_[corr_iteration];
  else
    standard_deviation_ = standard_deviations_.back();
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
  data_point->measured_depth_offset =
      data_model_point.depth_offsets[measured_depth_offset_id_];
  data_point->modeled_depth_offset =
      data_model_point.depth_offsets[modeled_depth_offset_id_];
  data_point->center_f_camera = std::move(center_f_camera);
}

bool DepthModality::IsPointValid(const DataPoint &data_point,
                                 bool measure_occlusions,
                                 bool model_occlusions) const {
  // Check if point is in front of image
  if (data_point.center_f_camera(2) <= 0.0f) return false;

  // Check if image coordinate is on image
  int i_center_u = int(data_point.center_u + 0.5f);
  int i_center_v = int(data_point.center_v + 0.5f);
  if (i_center_u < 0 || i_center_u > image_width_minus_1_ || i_center_v < 0 ||
      i_center_v > image_height_minus_1_)
    return false;

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

bool DepthModality::IsPointUnoccludedMeasured(
    const DataPoint &data_point) const {
  // Precalculate variables in pixel coordinates
  float meter_to_pixel = fu_ / data_point.depth;
  float diameter = 2.0f * measured_occlusion_radius_ * meter_to_pixel;
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

  // Iterate over pixels to check if depth value is big enough
  ushort depth;
  ushort min_depth =
      ushort((data_point.depth - data_point.measured_depth_offset -
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

bool DepthModality::IsPointUnoccludedModeled(
    const DataPoint &data_point) const {
  // Precalculate variables in pixel coordinates of focused image
  float meter_to_pixel =
      (fu_ / data_point.depth) * depth_renderer_ptr_->scale();
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
  u_max = std::min(u_max, image_size_minus_1_);
  v_max = std::min(v_max, image_size_minus_1_);

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
  float min_allowed_depth = data_point.depth - data_point.modeled_depth_offset -
                            modeled_occlusion_threshold_;
  return min_depth > min_allowed_depth;
}

bool DepthModality::FindCorrespondence(
    const DataPoint &data_point,
    Eigen::Vector3f *correspondence_center_f_camera) const {
  // Precalculate variables in pixel coordinates
  float meter_to_pixel = fu_ / data_point.depth;
  float diameter = 2.0f * considered_distance_ * meter_to_pixel;
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
      std::min(0.0f, (data_point.depth - considered_distance_) / depth_scale_);
  float max_depth_value =
      (data_point.depth + considered_distance_) / depth_scale_;

  // Iterate over all pixels to find closest point
  float min_distance_square = considered_distance_square_;
  float distance_square;
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
        distance_square = (temp_point - center_f_camera).squaredNorm();
        if (distance_square < min_distance_square) {
          *correspondence_center_f_camera = temp_point;
          min_distance_square = distance_square;
        }
      }
    }
  }
  return min_distance_square != considered_distance_square_;
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
  DrawFocusedPoints(cv::Vec3b{187, 117, 0}, &visualization_image);
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

void DepthModality::DrawFocusedPoints(const cv::Vec3b &color_point,
                                      cv::Mat *image) const {
  for (const auto &data_point : data_points_) {
    DrawFocusedPointInImage(
        body2camera_pose_ * data_point.center_f_body, color_point,
        depth_renderer_ptr_->intrinsics(), depth_renderer_ptr_->corner_u(),
        depth_renderer_ptr_->corner_v(), depth_renderer_ptr_->scale(), image);
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

}  // namespace icg
