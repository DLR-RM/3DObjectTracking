// SPDX-License-Identifier: MIT
// Copyright (c) 2023 Manuel Stoiber, German Aerospace Center (DLR)

#include "opt_evaluator.h"

OPTEvaluator::OPTEvaluator(const std::string &name,
                           const std::filesystem::path &dataset_directory,
                           const std::filesystem::path &external_directory,
                           const std::vector<std::string> &body_names,
                           const std::vector<std::string> &body_orientations,
                           const std::vector<std::string> &motion_patterns)
    : name_{name},
      dataset_directory_{dataset_directory},
      external_directory_{external_directory},
      body_names_{body_names},
      body_orientations_{body_orientations},
      motion_patterns_{motion_patterns} {
  // Compute thresholds used to compute AUC score
  float threshold_step = kThresholdMax / float(kNCurveValues);
  for (size_t i = 0; i < kNCurveValues; ++i) {
    thresholds_[i] = threshold_step * (0.5f + float(i));
  }
}

bool OPTEvaluator::SetUp() {
  set_up_ = false;

  CreateRunConfigurations();
  if (!LoadBodies()) return false;
  if (!GenerateModels()) return false;
  GenderateReducedVertices();
  if (calculate_diameters_) CalculateDiameters();

  set_up_ = true;
  return true;
}

void OPTEvaluator::set_run_sequentially(bool run_sequentially) {
  run_sequentially_ = run_sequentially;
}

void OPTEvaluator::set_use_random_seed(bool use_random_seed) {
  use_random_seed_ = use_random_seed;
}

void OPTEvaluator::set_n_vertices_evaluation(int n_vertices_evaluation) {
  n_vertices_evaluation_ = n_vertices_evaluation;
}

void OPTEvaluator::set_calculate_diameters(bool calculate_diameters) {
  calculate_diameters_ = calculate_diameters;
}

void OPTEvaluator::set_visualize_tracking(bool visualize_tracking) {
  visualize_tracking_ = visualize_tracking;
}

void OPTEvaluator::set_visualize_frame_results(bool visualize_frame_results) {
  visualize_frame_results_ = visualize_frame_results;
}

void OPTEvaluator::set_use_region_modality(bool use_region_modality) {
  use_region_modality_ = use_region_modality;
}

void OPTEvaluator::set_use_depth_modality(bool use_depth_modality) {
  use_depth_modality_ = use_depth_modality;
}

void OPTEvaluator::set_use_texture_modality(bool use_texture_modality) {
  use_texture_modality_ = use_texture_modality;
}

void OPTEvaluator::set_tracker_setter(
    const std::function<void(std::shared_ptr<m3t::Tracker>)> &tracker_setter) {
  tracker_setter_ = tracker_setter;
}

void OPTEvaluator::set_optimizer_setter(
    const std::function<void(std::shared_ptr<m3t::Optimizer>)>
        &optimizer_setter) {
  optimizer_setter_ = optimizer_setter;
}

void OPTEvaluator::set_region_modality_setter(
    const std::function<void(std::shared_ptr<m3t::RegionModality>)>
        &region_modality_setter) {
  region_modality_setter_ = region_modality_setter;
}

void OPTEvaluator::set_region_model_setter(
    const std::function<void(std::shared_ptr<m3t::RegionModel>)>
        &region_model_setter) {
  region_model_setter_ = region_model_setter;
  set_up_ = false;
}

void OPTEvaluator::set_depth_modality_setter(
    const std::function<void(std::shared_ptr<m3t::DepthModality>)>
        &depth_modality_setter) {
  depth_modality_setter_ = depth_modality_setter;
}

void OPTEvaluator::set_depth_model_setter(
    const std::function<void(std::shared_ptr<m3t::DepthModel>)>
        &depth_model_setter) {
  depth_model_setter_ = depth_model_setter;
  set_up_ = false;
}

void OPTEvaluator::set_texture_modality_setter(
    const std::function<void(std::shared_ptr<m3t::TextureModality>)>
        &texture_modality_setter) {
  texture_modality_setter_ = texture_modality_setter;
}

bool OPTEvaluator::Evaluate() {
  if (!set_up_) {
    std::cerr << "Set up evaluator " << name_ << " first" << std::endl;
    return false;
  }

  // Evaluate all run configuration
  results_.clear();
  final_results_.clear();
  if (run_sequentially_ || visualize_tracking_ || visualize_frame_results_) {
    auto renderer_geometry_ptr{std::make_shared<m3t::RendererGeometry>("rg")};
    renderer_geometry_ptr->SetUp();
    for (size_t i = 0; i < int(run_configurations_.size()); ++i) {
      SequenceResult sequence_result;
      if (!EvaluateRunConfiguration(run_configurations_[i],
                                    renderer_geometry_ptr, &sequence_result))
        continue;
      VisualizeResult(sequence_result.average_result,
                      run_configurations_[i].sequence_name);
      results_.push_back(std::move(sequence_result));
    }
  } else {
    std::vector<std::shared_ptr<m3t::RendererGeometry>> renderer_geometry_ptrs(
        omp_get_max_threads());
    for (auto &renderer_geometry_ptr : renderer_geometry_ptrs) {
      renderer_geometry_ptr = std::make_shared<m3t::RendererGeometry>("rg");
      renderer_geometry_ptr->SetUp();
    }
#pragma omp parallel for schedule(dynamic, 1)
    for (int i = 0; i < int(run_configurations_.size()); ++i) {
      SequenceResult sequence_result;
      if (!EvaluateRunConfiguration(
              run_configurations_[i],
              renderer_geometry_ptrs[omp_get_thread_num()], &sequence_result))
        continue;
#pragma omp critical
      {
        VisualizeResult(sequence_result.average_result,
                        run_configurations_[i].sequence_name);
        results_.push_back(std::move(sequence_result));
      }
    }
  }

  // Calculate body results
  std::cout << std::endl << std::string(80, '-') << std::endl;
  for (const auto &body_name : body_names_) {
    Result result{CalculateAverageBodyResult({body_name})};
    VisualizeResult(result, body_name);
    final_results_.insert({body_name, std::move(result)});
  }

  // Calculate average results
  Result final_result_all{CalculateAverageBodyResult(body_names_)};
  VisualizeResult(final_result_all, "all");
  final_results_.insert({"all", std::move(final_result_all)});
  std::cout << std::string(80, '-') << std::endl;
  return true;
}

void OPTEvaluator::SaveResults(std::filesystem::path path) const {
  std::ofstream ofs{path};
  for (auto const &[body_name, result] : final_results_) {
    ofs << body_name << "," << result.area_under_curve << ","
        << result.execution_times.complete_cycle << ","
        << result.execution_times.calculate_correspondences << ","
        << result.execution_times.calculate_gradient_and_hessian << ","
        << result.execution_times.calculate_optimization << ","
        << result.execution_times.calculate_results << std::endl;
  }
  ofs.flush();
  ofs.close();
}

float OPTEvaluator::area_under_curve() const {
  return final_results_.at("all").area_under_curve;
}

float OPTEvaluator::execution_time() const {
  return final_results_.at("all").execution_times.complete_cycle;
}

std::map<std::string, OPTEvaluator::Result> OPTEvaluator::final_results()
    const {
  return final_results_;
}

bool OPTEvaluator::EvaluateRunConfiguration(
    const RunConfiguration &run_configuration,
    const std::shared_ptr<m3t::RendererGeometry> &renderer_geometry_ptr,
    SequenceResult *sequence_result) const {
  // Read ground truth poses
  std::vector<m3t::Transform3fA> gt_body2world_pose;
  if (!GetGTPosesOPTDataset(run_configuration, &gt_body2world_pose))
    return false;

  // Initialize tracker
  std::shared_ptr<m3t::Tracker> tracker_ptr;
  if (!SetUpTracker(run_configuration, renderer_geometry_ptr, &tracker_ptr))
    return false;

  // Initialize body pose and start modalities
  const auto &body_ptr{tracker_ptr->optimizer_ptrs()[0]
                           ->root_link_ptr()
                           ->modality_ptrs()[0]
                           ->body_ptr()};
  body_ptr->set_body2world_pose(gt_body2world_pose[0]);
  tracker_ptr->StartModalities(0);

  // Init results
  sequence_result->body_name = run_configuration.body_name;
  sequence_result->body_orientation = run_configuration.body_orientation;
  sequence_result->motion_pattern = run_configuration.motion_pattern;
  sequence_result->frame_results.clear();

  // Iterate over all frames
  for (int i = 0; i < gt_body2world_pose.size() - 1; ++i) {
    Result result;
    result.frame_index = i;
    ExecuteMeasuredTrackingCycle(tracker_ptr, i, &result.execution_times);

    // Calculate results
    CalculatePoseResults(run_configuration.body_name,
                         body_ptr->body2world_pose(), gt_body2world_pose[i + 1],
                         &result);
    if (visualize_frame_results_)
      VisualizeResult(result, run_configuration.sequence_name);
    sequence_result->frame_results.push_back(std::move(result));
  }

  // Calculate Average Results
  sequence_result->average_result =
      CalculateAverageResult(sequence_result->frame_results);
  return true;
}

bool OPTEvaluator::SetUpTracker(
    const RunConfiguration &run_configuration,
    const std::shared_ptr<m3t::RendererGeometry> &renderer_geometry_ptr,
    std::shared_ptr<m3t::Tracker> *tracker_ptr) const {
  renderer_geometry_ptr->ClearBodies();
  *tracker_ptr = std::make_shared<m3t::Tracker>("tracker");

  // Init cameras
  std::filesystem::path sequence_directory{dataset_directory_ / "3D" /
                                           run_configuration.sequence_name};
  auto color_camera_ptr{std::make_shared<m3t::LoaderColorCamera>(
      "color_camera", sequence_directory / "color", kOPTIntrinsics, "", 1, 4)};
  color_camera_ptr->SetUp();
  auto depth_camera_ptr{std::make_shared<m3t::LoaderDepthCamera>(
      "depth_camera", sequence_directory / "depth", kOPTIntrinsics, 0.001f, "",
      1, 4)};
  depth_camera_ptr->set_camera2world_pose(
      m3t::Transform3fA{Eigen::Matrix4f{kDepth2Color_Pose.data()}});
  depth_camera_ptr->SetUp();

  // Init visualizer
  if (use_region_modality_ && visualize_tracking_) {
    auto color_viewer_ptr{std::make_shared<m3t::NormalColorViewer>(
        "color_viewer", color_camera_ptr, renderer_geometry_ptr)};
    color_viewer_ptr->SetUp();
    (*tracker_ptr)->AddViewer(color_viewer_ptr);
  }
  if (use_region_modality_ && visualize_tracking_) {
    auto depth_viewer_ptr{std::make_shared<m3t::NormalDepthViewer>(
        "depth_viewer", depth_camera_ptr, renderer_geometry_ptr, 0.0f, 1.0f)};
    depth_viewer_ptr->SetUp();
    (*tracker_ptr)->AddViewer(depth_viewer_ptr);
  }

  // Init body
  const auto &body_name{run_configuration.body_name};
  auto body_ptr{std::make_shared<m3t::Body>(*body2body_ptr_map_.at(body_name))};
  renderer_geometry_ptr->AddBody(body_ptr);

  // Init link
  auto link_ptr{std::make_shared<m3t::Link>(body_name + "_link", body_ptr)};

  // Init modalities
  if (use_region_modality_) {
    auto region_modality_ptr{std::make_shared<m3t::RegionModality>(
        body_name + "_region_modality", body_ptr, color_camera_ptr,
        body2region_model_ptr_map_.at(body_name))};
    region_modality_setter_(region_modality_ptr);
    region_modality_ptr->SetUp();
    link_ptr->AddModality(region_modality_ptr);
  }

  if (use_depth_modality_) {
    auto depth_modality_ptr{std::make_shared<m3t::DepthModality>(
        body_name + "_depth_modality", body_ptr, depth_camera_ptr,
        body2depth_model_ptr_map_.at(body_name))};
    depth_modality_setter_(depth_modality_ptr);
    depth_modality_ptr->SetUp();
    link_ptr->AddModality(depth_modality_ptr);
  }

  if (use_texture_modality_) {
    auto silhouette_renderer_ptr{
        std::make_shared<m3t::FocusedSilhouetteRenderer>(
            body_name + "_silhouette_renderer", renderer_geometry_ptr,
            color_camera_ptr)};
    silhouette_renderer_ptr->AddReferencedBody(body_ptr);
    silhouette_renderer_ptr->SetUp();
    auto texture_modality_ptr{std::make_shared<m3t::TextureModality>(
        body_name + "_texture_modality", body_ptr, color_camera_ptr,
        silhouette_renderer_ptr)};
    texture_modality_setter_(texture_modality_ptr);
    texture_modality_ptr->SetUp();
    link_ptr->AddModality(texture_modality_ptr);
  }

  // Init optimizer and tracker
  link_ptr->SetUp();
  auto optimizer_ptr{
      std::make_shared<m3t::Optimizer>(body_name + "_optimizer", link_ptr)};
  optimizer_setter_(optimizer_ptr);
  optimizer_ptr->SetUp();
  (*tracker_ptr)->AddOptimizer(optimizer_ptr);
  tracker_setter_(*tracker_ptr);
  return (*tracker_ptr)->SetUp(false);
}

void OPTEvaluator::ExecuteMeasuredTrackingCycle(
    const std::shared_ptr<m3t::Tracker> &tracker_ptr, int iteration,
    ExecutionTimes *execution_times) const {
  std::chrono::high_resolution_clock::time_point begin_time;

  // Update Cameras
  tracker_ptr->UpdateCameras(iteration);

  execution_times->calculate_correspondences = 0.0f;
  execution_times->calculate_gradient_and_hessian = 0.0f;
  execution_times->calculate_optimization = 0.0f;
  for (int corr_iteration = 0;
       corr_iteration < tracker_ptr->n_corr_iterations(); ++corr_iteration) {
    // Calculate correspondences
    begin_time = std::chrono::high_resolution_clock::now();
    tracker_ptr->CalculateCorrespondences(iteration, corr_iteration);
    execution_times->calculate_correspondences += ElapsedTime(begin_time);

    // Visualize correspondences
    int corr_save_idx =
        iteration * tracker_ptr->n_corr_iterations() + corr_iteration;
    tracker_ptr->VisualizeCorrespondences(corr_save_idx);

    for (int update_iteration = 0;
         update_iteration < tracker_ptr->n_update_iterations();
         ++update_iteration) {
      // Calculate gradient and hessian
      begin_time = std::chrono::high_resolution_clock::now();
      tracker_ptr->CalculateGradientAndHessian(iteration, corr_iteration,
                                               update_iteration);
      execution_times->calculate_gradient_and_hessian +=
          ElapsedTime(begin_time);

      // Calculate optimization
      begin_time = std::chrono::high_resolution_clock::now();
      tracker_ptr->CalculateOptimization(iteration, corr_iteration,
                                         update_iteration);
      execution_times->calculate_optimization += ElapsedTime(begin_time);

      // Visualize pose update
      int update_save_idx =
          corr_save_idx * tracker_ptr->n_update_iterations() + update_iteration;
      tracker_ptr->VisualizeOptimization(update_save_idx);
    }
  }

  // Calculate results
  begin_time = std::chrono::high_resolution_clock::now();
  tracker_ptr->CalculateResults(iteration);
  execution_times->calculate_results = ElapsedTime(begin_time);

  // Visualize results and update viewers
  tracker_ptr->VisualizeResults(iteration);
  if (visualize_tracking_) tracker_ptr->UpdateViewers(iteration);

  execution_times->complete_cycle =
      execution_times->calculate_correspondences +
      execution_times->calculate_gradient_and_hessian +
      execution_times->calculate_optimization +
      execution_times->calculate_results;
}

OPTEvaluator::Result OPTEvaluator::CalculateAverageBodyResult(
    const std::vector<std::string> &body_names) const {
  size_t n_results = 0;
  std::vector<Result> result_sums;
  for (const auto &result : results_) {
    if (std::find(begin(body_names), end(body_names), result.body_name) !=
        end(body_names)) {
      n_results += result.frame_results.size();
      result_sums.push_back(SumResults(result.frame_results));
    }
  }
  return DivideResult(SumResults(result_sums), n_results);
}

OPTEvaluator::Result OPTEvaluator::CalculateAverageResult(
    const std::vector<Result> &results) {
  return DivideResult(SumResults(results), results.size());
}

OPTEvaluator::Result OPTEvaluator::SumResults(
    const std::vector<Result> &results) {
  Result sum;
  for (const auto &result : results) {
    sum.area_under_curve += result.area_under_curve;
    std::transform(begin(result.curve_values), end(result.curve_values),
                   begin(sum.curve_values), begin(sum.curve_values),
                   [](float a, float b) { return a + b; });
    sum.execution_times.calculate_correspondences +=
        result.execution_times.calculate_correspondences;
    sum.execution_times.calculate_gradient_and_hessian +=
        result.execution_times.calculate_gradient_and_hessian;
    sum.execution_times.calculate_optimization +=
        result.execution_times.calculate_optimization;
    sum.execution_times.calculate_results +=
        result.execution_times.calculate_results;
    sum.execution_times.complete_cycle += result.execution_times.complete_cycle;
  }
  return sum;
}

OPTEvaluator::Result OPTEvaluator::DivideResult(const Result &result,
                                                size_t n) {
  Result divided;
  divided.area_under_curve = result.area_under_curve / float(n);
  std::transform(begin(result.curve_values), end(result.curve_values),
                 begin(divided.curve_values),
                 [&](float a) { return a / float(n); });
  divided.execution_times.calculate_correspondences =
      result.execution_times.calculate_correspondences / float(n);
  divided.execution_times.calculate_gradient_and_hessian =
      result.execution_times.calculate_gradient_and_hessian / float(n);
  divided.execution_times.calculate_optimization =
      result.execution_times.calculate_optimization / float(n);
  divided.execution_times.calculate_results =
      result.execution_times.calculate_results / float(n);
  divided.execution_times.complete_cycle =
      result.execution_times.complete_cycle / float(n);
  return divided;
}

void OPTEvaluator::CalculatePoseResults(
    const std::string &body_name, const m3t::Transform3fA &body2world_pose,
    const m3t::Transform3fA &gt_body2world_pose, Result *result) const {
  float error = 0.0f;
  m3t::Transform3fA delta_pose{
      (body2world_pose * kBody2Geometry2BodyPoseMap.at(body_name)).inverse() *
      gt_body2world_pose * kBody2Geometry2BodyPoseMap.at(body_name)};
  const auto &vertices{body2reduced_vertice_map_.at(body_name)};
  for (const auto &vertice : vertices)
    error += (vertice - delta_pose * vertice).norm();
  error /= vertices.size();

  float diameter;
  if (calculate_diameters_)
    diameter = body2diameter_map_.at(body_name);
  else
    diameter = kBody2PrecomputedDiametersMap.at(body_name);
  std::fill(begin(result->curve_values), end(result->curve_values), 1.0f);
  for (size_t i = 0; i < kNCurveValues; ++i) {
    if (error < diameter * thresholds_[i]) break;
    result->curve_values[i] = 0.0f;
  }

  float threshold = diameter * kThresholdMax;
  result->area_under_curve =
      kThresholdMax * (1.0f - std::min(error / threshold, 1.0f));
}

void OPTEvaluator::VisualizeResult(const Result &result,
                                   const std::string &title) {
  std::cout << title << ": ";
  if (result.frame_index) std::cout << "frame " << result.frame_index << ": ";
  std::cout << "execution_time = " << result.execution_times.complete_cycle
            << " us, "
            << "area_under_curve = " << result.area_under_curve << std::endl;
}

void OPTEvaluator::CreateRunConfigurations() {
  run_configurations_.clear();
  for (const auto &body_name : body_names_) {
    for (const auto &body_orientation : body_orientations_) {
      for (const auto &motion_pattern : motion_patterns_) {
        std::string sequence_name{body_name.substr(0, 2) + "_" +
                                  motion_pattern + "_" + body_orientation};
        run_configurations_.push_back(RunConfiguration{
            body_name, body_orientation, motion_pattern, sequence_name});
      }
    }
  }
}

bool OPTEvaluator::LoadBodies() {
  body2body_ptr_map_.clear();
  for (const auto &body_name : body_names_) {
    std::filesystem::path directory{dataset_directory_ / "Model3D" / body_name};
    auto body_ptr{std::make_shared<m3t::Body>(
        body_name, directory / (body_name + ".obj"), 1.0f, true, true,
        kBody2Geometry2BodyPoseMap.at(body_name))};
    if (!body_ptr->SetUp()) return false;
    body2body_ptr_map_.insert({body_name, std::move(body_ptr)});
  }
  return true;
}

bool OPTEvaluator::GenerateModels() {
  std::filesystem::path directory{external_directory_ / "models"};
  std::filesystem::create_directories(directory);
  for (auto body_name : body_names_) {
    if (use_region_modality_) {
      auto model_ptr{std::make_shared<m3t::RegionModel>(
          body_name + "_region_model", body2body_ptr_map_[body_name],
          directory / (body_name + "_region_model.bin"), 0.8f, 4, 500, 0.05f,
          0.002f, false, 2000)};
      region_model_setter_(model_ptr);
      if (!model_ptr->SetUp()) return false;
      body2region_model_ptr_map_.insert({body_name, std::move(model_ptr)});
    }
    if (use_depth_modality_) {
      auto model_ptr{std::make_shared<m3t::DepthModel>(
          body_name + "_depth_model", body2body_ptr_map_[body_name],
          directory / (body_name + "_depth_model.bin"), 0.8f, 4, 500, 0.05f,
          0.002f, false, 2000)};
      depth_model_setter_(model_ptr);
      if (!model_ptr->SetUp()) return false;
      body2depth_model_ptr_map_.insert({body_name, std::move(model_ptr)});
    }
  }
  return true;
}

void OPTEvaluator::GenderateReducedVertices() {
#pragma omp parallel for
  for (int i = 0; i < body_names_.size(); ++i) {
    auto &body_name{body_names_[i]};
    const auto &vertices{body2body_ptr_map_[body_name]->vertices()};
    if (n_vertices_evaluation_ <= 0 ||
        n_vertices_evaluation_ >= vertices.size()) {
#pragma omp critical
      body2reduced_vertice_map_[body_name] = vertices;
      continue;
    }

    std::mt19937 generator{7};
    if (use_random_seed_)
      generator.seed(unsigned(
          std::chrono::system_clock::now().time_since_epoch().count()));

    std::vector<Eigen::Vector3f> reduced_vertices(n_vertices_evaluation_);
    int n_vertices = vertices.size();
    for (auto &v : reduced_vertices) {
      int idx = int(generator() % n_vertices);
      v = vertices[idx];
    }
#pragma omp critical
    body2reduced_vertice_map_.insert({body_name, std::move(reduced_vertices)});
  }
}

void OPTEvaluator::CalculateDiameters() {
  for (const auto &body_name : body_names_) {
    float d_max = 0.0f;
    const auto &vertices{body2body_ptr_map_.at(body_name)->vertices()};
    long long size = vertices.size();
    long long size_squared = size * size;
#pragma omp parallel
    {
      float d_max_thread = 0.0f;
#pragma omp for
      for (long long i = 0; i < size_squared; ++i) {
        d_max_thread = std::max(
            d_max_thread, (vertices[i / size] - vertices[i % size]).norm());
      }
#pragma omp critical
      d_max = std::max(d_max, d_max_thread);
    }
    std::cout << body_name << ": " << d_max << std::endl;
    body2diameter_map_.insert({body_name, d_max});
  }
}

bool OPTEvaluator::GetGTPosesOPTDataset(
    const RunConfiguration &run_configuration,
    std::vector<m3t::Transform3fA> *gt_body2world_poses) const {
  std::filesystem::path path{dataset_directory_ / "3D" / "poses" /
                             (run_configuration.sequence_name + ".txt")};
  std::ifstream ifs{path.string(), std::ios::binary};
  if (!ifs.is_open() || ifs.fail()) {
    ifs.close();
    std::cerr << "Could not open file stream " << path.string() << std::endl;
    return false;
  }

  gt_body2world_poses->clear();
  std::string parsed;
  for (std::string line; std::getline(ifs, line);) {
    m3t::Transform3fA pose{m3t::Transform3fA::Identity()};
    std::stringstream stringstream{line};
    for (int i = 0; i < 4; ++i) {
      for (int j = 0; j < 3; ++j) {
        std::getline(stringstream, parsed, ' ');
        pose.matrix()(j, i) = stof(parsed);
      }
    }
    pose = pose *
           kBody2Geometry2BodyPoseMap.at(run_configuration.body_name).inverse();
    gt_body2world_poses->push_back(std::move(pose));
  }
  return true;
}

float OPTEvaluator::ElapsedTime(
    const std::chrono::high_resolution_clock::time_point &begin_time) {
  auto end_time{std::chrono::high_resolution_clock::now()};
  return float(std::chrono::duration_cast<std::chrono::microseconds>(end_time -
                                                                     begin_time)
                   .count());
}
