// SPDX-License-Identifier: MIT
// Copyright (c) 2023 Manuel Stoiber, German Aerospace Center (DLR)

#include "choi_evaluator.h"

ChoiEvaluator::ChoiEvaluator(const std::string &name,
                             const std::filesystem::path &dataset_directory,
                             const std::filesystem::path &external_directory,
                             const std::vector<std::string> &body_names)
    : name_{name},
      dataset_directory_{dataset_directory},
      external_directory_{external_directory},
      body_names_{body_names} {}

bool ChoiEvaluator::SetUp() {
  set_up_ = false;

  if (!LoadBodies()) return false;
  if (!GenerateModels()) return false;

  set_up_ = true;
  return true;
}

void ChoiEvaluator::set_run_sequentially(bool run_sequentially) {
  run_sequentially_ = run_sequentially;
}

void ChoiEvaluator::set_visualize_tracking(bool visualize_tracking) {
  visualize_tracking_ = visualize_tracking;
}

void ChoiEvaluator::set_visualize_frame_results(bool visualize_frame_results) {
  visualize_frame_results_ = visualize_frame_results;
}

void ChoiEvaluator::set_use_region_modality(bool use_region_modality) {
  use_region_modality_ = use_region_modality;
}

void ChoiEvaluator::set_use_depth_modality(bool use_depth_modality) {
  use_depth_modality_ = use_depth_modality;
}

void ChoiEvaluator::set_use_texture_modality(bool use_texture_modality) {
  use_texture_modality_ = use_texture_modality;
}

void ChoiEvaluator::set_measure_occlusions_region(
    bool measure_occlusions_region) {
  measure_occlusions_region_ = measure_occlusions_region;
}

void ChoiEvaluator::set_measure_occlusions_depth(
    bool measure_occlusions_depth) {
  measure_occlusions_depth_ = measure_occlusions_depth;
}

void ChoiEvaluator::set_measure_occlusions_texture(
    bool measure_occlusions_texture) {
  measure_occlusions_texture_ = measure_occlusions_texture;
}

void ChoiEvaluator::set_tracker_setter(
    const std::function<void(std::shared_ptr<m3t::Tracker>)> &tracker_setter) {
  tracker_setter_ = tracker_setter;
}

void ChoiEvaluator::set_optimizer_setter(
    const std::function<void(std::shared_ptr<m3t::Optimizer>)>
        &optimizer_setter) {
  optimizer_setter_ = optimizer_setter;
}

void ChoiEvaluator::set_region_modality_setter(
    const std::function<void(std::shared_ptr<m3t::RegionModality>)>
        &region_modality_setter) {
  region_modality_setter_ = region_modality_setter;
}

void ChoiEvaluator::set_region_model_setter(
    const std::function<void(std::shared_ptr<m3t::RegionModel>)>
        &region_model_setter) {
  region_model_setter_ = region_model_setter;
  set_up_ = false;
}

void ChoiEvaluator::set_depth_modality_setter(
    const std::function<void(std::shared_ptr<m3t::DepthModality>)>
        &depth_modality_setter) {
  depth_modality_setter_ = depth_modality_setter;
}

void ChoiEvaluator::set_depth_model_setter(
    const std::function<void(std::shared_ptr<m3t::DepthModel>)>
        &depth_model_setter) {
  depth_model_setter_ = depth_model_setter;
  set_up_ = false;
}

void ChoiEvaluator::set_texture_modality_setter(
    const std::function<void(std::shared_ptr<m3t::TextureModality>)>
        &texture_modality_setter) {
  texture_modality_setter_ = texture_modality_setter;
}

bool ChoiEvaluator::Evaluate() {
  if (!set_up_) {
    std::cerr << "Set up evaluator " << name_ << " first" << std::endl;
    return false;
  }

  // Evaluate all run configuration
  final_results_.clear();
  if (run_sequentially_ || visualize_tracking_ || visualize_frame_results_) {
    auto renderer_geometry_ptr{std::make_shared<m3t::RendererGeometry>("rg")};
    renderer_geometry_ptr->SetUp();
    for (size_t i = 0; i < int(body_names_.size()); ++i) {
      SequenceResult sequence_result;
      if (!EvaluateBody(body_names_[i], renderer_geometry_ptr,
                        &sequence_result))
        continue;
      VisualizeSequenceResult(sequence_result, body_names_[i]);
      final_results_.insert({body_names_[i], std::move(sequence_result)});
    }
  } else {
    std::vector<std::shared_ptr<m3t::RendererGeometry>> renderer_geometry_ptrs(
        omp_get_max_threads());
    for (auto &renderer_geometry_ptr : renderer_geometry_ptrs) {
      renderer_geometry_ptr = std::make_shared<m3t::RendererGeometry>("rg");
      renderer_geometry_ptr->SetUp();
    }
#pragma omp parallel for schedule(dynamic, 1)
    for (int i = 0; i < int(body_names_.size()); ++i) {
      SequenceResult sequence_result;
      if (!EvaluateBody(body_names_[i],
                        renderer_geometry_ptrs[omp_get_thread_num()],
                        &sequence_result))
        continue;
#pragma omp critical
      {
        VisualizeSequenceResult(sequence_result, body_names_[i]);
        final_results_.insert({body_names_[i], std::move(sequence_result)});
      }
    }
  }

  // Calculate average results
  auto final_result_average{CalculateAverageFinalResult()};
  VisualizeSequenceResult(final_result_average, "average");
  final_results_.insert({"average", std::move(final_result_average)});
  std::cout << std::string(80, '-') << std::endl;
  return true;
}

void ChoiEvaluator::SaveResults(std::filesystem::path path) const {
  std::ofstream ofs{path};
  for (auto const &[body_name, result] : final_results_) {
    ofs << body_name << "," << result.rms_error_x << "," << result.rms_error_y
        << "," << result.rms_error_z << "," << result.rms_error_alpha << ","
        << result.rms_error_beta << "," << result.rms_error_gamma << ","
        << result.mean_translation_error << "," << result.mean_rotation_error
        << "," << result.mean_execution_times.complete_cycle << ","
        << result.mean_execution_times.calculate_correspondences << ","
        << result.mean_execution_times.calculate_gradient_and_hessian << ","
        << result.mean_execution_times.calculate_optimization << ","
        << result.mean_execution_times.calculate_results << std::endl;
  }
  ofs.flush();
  ofs.close();
}

float ChoiEvaluator::mean_translation_error() const {
  return final_results_.at("average").mean_translation_error;
}

float ChoiEvaluator::mean_rotation_error() const {
  return final_results_.at("average").mean_rotation_error;
}

float ChoiEvaluator::execution_time() const {
  return final_results_.at("average").mean_execution_times.complete_cycle;
}

std::map<std::string, ChoiEvaluator::SequenceResult>
ChoiEvaluator::final_results() const {
  return final_results_;
}

bool ChoiEvaluator::EvaluateBody(
    const std::string &body_name,
    const std::shared_ptr<m3t::RendererGeometry> &renderer_geometry_ptr,
    SequenceResult *sequence_result) const {
  // Read ground truth poses
  std::vector<m3t::Transform3fA> gt_body2world_pose;
  if (!GetGTPosesChoiDataset(body_name, &gt_body2world_pose)) return false;

  // Initialize tracker
  std::shared_ptr<m3t::Tracker> tracker_ptr;
  if (!SetUpTracker(body_name, renderer_geometry_ptr, &tracker_ptr))
    return false;

  // Initialize body pose and start modalities
  const auto &body_ptr{tracker_ptr->optimizer_ptrs()[0]
                           ->root_link_ptr()
                           ->modality_ptrs()[0]
                           ->body_ptr()};
  body_ptr->set_body2world_pose(gt_body2world_pose[0]);
  tracker_ptr->StartModalities(0);

  // Iterate over all frames
  sequence_result->frame_results.clear();
  for (int i = 0; i < gt_body2world_pose.size() - 1; ++i) {
    Result result;
    result.frame_index = i;
    ExecuteMeasuredTrackingCycle(tracker_ptr, i, &result.execution_times);

    // Calculate results
    CalculatePoseResults(body_ptr->body2world_pose(), gt_body2world_pose[i + 1],
                         &result);
    if (visualize_frame_results_) VisualizeFrameResult(result, body_name);
    sequence_result->frame_results.push_back(std::move(result));
  }

  // Calculate Average Results
  CalculateAverageSequenceResult(sequence_result);
  return true;
}

bool ChoiEvaluator::SetUpTracker(
    const std::string &body_name,
    const std::shared_ptr<m3t::RendererGeometry> &renderer_geometry_ptr,
    std::shared_ptr<m3t::Tracker> *tracker_ptr) const {
  renderer_geometry_ptr->ClearBodies();
  *tracker_ptr = std::make_shared<m3t::Tracker>("tracker");

  // Init cameras
  std::filesystem::path camera_directory{
      external_directory_ / ("seq_synth_" + body_name + "_kitchen")};
  auto color_camera_ptr{std::make_shared<m3t::LoaderColorCamera>(
      "color_camera", camera_directory, kChoiIntrinsics, "color", 0, 4)};
  color_camera_ptr->SetUp();
  auto depth_camera_ptr{std::make_shared<m3t::LoaderDepthCamera>(
      "depth_camera", camera_directory, kChoiIntrinsics, 0.0001f, "depth", 0,
      4)};
  depth_camera_ptr->SetUp();

  // Init visualizer
  if ((use_region_modality_ || use_texture_modality_) && visualize_tracking_) {
    auto color_viewer_ptr{std::make_shared<m3t::NormalColorViewer>(
        "color_viewer", color_camera_ptr, renderer_geometry_ptr)};
    color_viewer_ptr->SetUp();
    (*tracker_ptr)->AddViewer(color_viewer_ptr);
  }
  if (use_depth_modality_ && visualize_tracking_) {
    auto depth_viewer_ptr{std::make_shared<m3t::NormalDepthViewer>(
        "depth_viewer", depth_camera_ptr, renderer_geometry_ptr, 0.3f, 1.5f)};
    depth_viewer_ptr->SetUp();
    (*tracker_ptr)->AddViewer(depth_viewer_ptr);
  }

  // Init body
  auto body_ptr{std::make_shared<m3t::Body>(*body2body_ptr_map_.at(body_name))};
  renderer_geometry_ptr->AddBody(body_ptr);

  // Init link
  auto link_ptr{std::make_shared<m3t::Link>(body_name + "_link", body_ptr)};

  // Init region modality
  if (use_region_modality_) {
    auto region_modality_ptr{std::make_shared<m3t::RegionModality>(
        body_name + "_region_modality", body_ptr, color_camera_ptr,
        body2region_model_ptr_map_.at(body_name))};
    region_modality_setter_(region_modality_ptr);
    if (measure_occlusions_region_)
      region_modality_ptr->MeasureOcclusions(depth_camera_ptr);
    region_modality_ptr->SetUp();
    link_ptr->AddModality(region_modality_ptr);
  }

  // Init depth modality
  if (use_depth_modality_) {
    auto depth_modality_ptr{std::make_shared<m3t::DepthModality>(
        body_name + "_depth_modality", body_ptr, depth_camera_ptr,
        body2depth_model_ptr_map_.at(body_name))};
    depth_modality_setter_(depth_modality_ptr);
    if (measure_occlusions_depth_) depth_modality_ptr->MeasureOcclusions();
    depth_modality_ptr->SetUp();
    link_ptr->AddModality(depth_modality_ptr);
  }

  // Init texture modality
  if (use_texture_modality_) {
    auto silhouette_renderer_ptr{
        std::make_shared<m3t::FocusedSilhouetteRenderer>(
            body_name + "_silhouette_renderer", renderer_geometry_ptr,
            color_camera_ptr, m3t::IDType::BODY)};
    silhouette_renderer_ptr->AddReferencedBody(body_ptr);
    silhouette_renderer_ptr->SetUp();
    auto texture_modality_ptr{std::make_shared<m3t::TextureModality>(
        body_name + "_texture_modality", body_ptr, color_camera_ptr,
        silhouette_renderer_ptr)};
    texture_modality_setter_(texture_modality_ptr);
    if (measure_occlusions_texture_)
      texture_modality_ptr->MeasureOcclusions(depth_camera_ptr);
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

void ChoiEvaluator::ExecuteMeasuredTrackingCycle(
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

ChoiEvaluator::SequenceResult ChoiEvaluator::CalculateAverageFinalResult()
    const {
  SequenceResult result;
  for (const auto &body_name : body_names_) {
    result.mean_translation_error +=
        final_results_.at(body_name).mean_translation_error;
    result.mean_rotation_error +=
        final_results_.at(body_name).mean_rotation_error;
    result.mean_execution_times.complete_cycle +=
        final_results_.at(body_name).mean_execution_times.complete_cycle;
    result.mean_execution_times.calculate_correspondences +=
        final_results_.at(body_name)
            .mean_execution_times.calculate_correspondences;
    result.mean_execution_times.calculate_gradient_and_hessian +=
        final_results_.at(body_name)
            .mean_execution_times.calculate_gradient_and_hessian;
    result.mean_execution_times.calculate_optimization +=
        final_results_.at(body_name)
            .mean_execution_times.calculate_optimization;
    result.mean_execution_times.calculate_results +=
        final_results_.at(body_name).mean_execution_times.calculate_results;
  }
  float n = float(body_names_.size());
  result.mean_translation_error /= n;
  result.mean_rotation_error /= n;
  result.mean_execution_times.complete_cycle /= n;
  result.mean_execution_times.calculate_correspondences /= n;
  result.mean_execution_times.calculate_gradient_and_hessian /= n;
  result.mean_execution_times.calculate_optimization /= n;
  result.mean_execution_times.calculate_results /= n;
  return result;
}

void ChoiEvaluator::CalculateAverageSequenceResult(SequenceResult *result) {
  result->rms_error_x = 0.0f;
  result->rms_error_y = 0.0f;
  result->rms_error_z = 0.0f;
  result->rms_error_alpha = 0.0f;
  result->rms_error_beta = 0.0f;
  result->rms_error_gamma = 0.0f;
  for (const auto &frame_result : result->frame_results) {
    result->rms_error_x += m3t::square(frame_result.error_x);
    result->rms_error_y += m3t::square(frame_result.error_y);
    result->rms_error_z += m3t::square(frame_result.error_z);
    result->rms_error_alpha += m3t::square(frame_result.error_alpha);
    result->rms_error_beta += m3t::square(frame_result.error_beta);
    result->rms_error_gamma += m3t::square(frame_result.error_gamma);
    result->mean_execution_times.calculate_correspondences +=
        frame_result.execution_times.calculate_correspondences;
    result->mean_execution_times.calculate_gradient_and_hessian +=
        frame_result.execution_times.calculate_gradient_and_hessian;
    result->mean_execution_times.calculate_optimization +=
        frame_result.execution_times.calculate_optimization;
    result->mean_execution_times.calculate_results +=
        frame_result.execution_times.calculate_results;
    result->mean_execution_times.complete_cycle +=
        frame_result.execution_times.complete_cycle;
  }
  float n = float(result->frame_results.size());
  result->rms_error_x = sqrtf(result->rms_error_x / n);
  result->rms_error_y = sqrtf(result->rms_error_y / n);
  result->rms_error_z = sqrtf(result->rms_error_z / n);
  result->rms_error_alpha = sqrtf(result->rms_error_alpha / n);
  result->rms_error_beta = sqrtf(result->rms_error_beta / n);
  result->rms_error_gamma = sqrtf(result->rms_error_gamma / n);
  result->mean_execution_times.calculate_correspondences /= n;
  result->mean_execution_times.calculate_gradient_and_hessian /= n;
  result->mean_execution_times.calculate_optimization /= n;
  result->mean_execution_times.calculate_results /= n;
  result->mean_execution_times.complete_cycle /= n;
  result->mean_translation_error =
      (result->rms_error_x + result->rms_error_y + result->rms_error_z) / 3.0f;
  result->mean_rotation_error =
      (result->rms_error_alpha + result->rms_error_beta +
       result->rms_error_gamma) /
      3.0f;
}

void ChoiEvaluator::CalculatePoseResults(
    const m3t::Transform3fA &body2world_pose,
    const m3t::Transform3fA &gt_body2world_pose, Result *result) const {
  auto trans_error = body2world_pose.translation().matrix() -
                     gt_body2world_pose.translation().matrix();
  result->error_x = trans_error.x() * 1000.0f;
  result->error_y = trans_error.y() * 1000.0f;
  result->error_z = trans_error.z() * 1000.0f;

  auto rpy_error = body2world_pose.rotation().matrix().eulerAngles(0, 1, 2) -
                   gt_body2world_pose.rotation().matrix().eulerAngles(0, 1, 2);
  result->error_alpha = rpy_error.x() * 180.0f / m3t::kPi;
  result->error_beta = rpy_error.y() * 180.0f / m3t::kPi;
  result->error_gamma = rpy_error.z() * 180.0f / m3t::kPi;
}

void ChoiEvaluator::VisualizeFrameResult(const Result &result,
                                         const std::string &title) {
  std::cout << title << ": "
            << "frame " << result.frame_index << ": "
            << "execution_time = " << result.execution_times.complete_cycle
            << " us, "
            << "error_x = " << result.error_x << " mm, "
            << "error_y = " << result.error_y << " mm, "
            << "error_z = " << result.error_z << " mm, "
            << "error_alpha = " << result.error_alpha << " deg, "
            << "error_beta = " << result.error_beta << " deg, "
            << "error_gamma = " << result.error_gamma << " deg" << std::endl;
}

void ChoiEvaluator::VisualizeSequenceResult(const SequenceResult &result,
                                            const std::string &title) {
  std::cout << title << ": " << std::endl;
  if (result.rms_error_x != 0.0f) {
    std::cout << "rms_error_x = " << result.rms_error_x << " mm, "
              << "rms_error_y = " << result.rms_error_y << " mm, "
              << "rms_error_z = " << result.rms_error_z << " mm" << std::endl;
    std::cout << "rms_error_alpha = " << result.rms_error_alpha << " deg, "
              << "rms_error_beta = " << result.rms_error_beta << " deg, "
              << "rms_error_gamma = " << result.rms_error_gamma << " deg"
              << std::endl;
  }
  std::cout << "execution_time = " << result.mean_execution_times.complete_cycle
            << " us, "
            << "mean_translation_error = " << result.mean_translation_error
            << " mm, "
            << "mean_rotation_error = " << result.mean_rotation_error << " deg"
            << std::endl;
}

bool ChoiEvaluator::LoadBodies() {
  body2body_ptr_map_.clear();
  std::filesystem::path directory{external_directory_ / "models"};
  for (const auto &body_name : body_names_) {
    auto body_ptr{std::make_shared<m3t::Body>(
        body_name, directory / (body_name + ".obj"), 1.0f, true, true,
        m3t::Transform3fA::Identity())};
    if (!body_ptr->SetUp()) return false;
    body2body_ptr_map_.insert({body_name, std::move(body_ptr)});
  }
  return true;
}

bool ChoiEvaluator::GenerateModels() {
  std::filesystem::path directory{external_directory_ / "models"};
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

bool ChoiEvaluator::GetGTPosesChoiDataset(
    const std::string &body_name,
    std::vector<m3t::Transform3fA> *gt_body2world_poses) const {
  std::filesystem::path path{dataset_directory_ / "ground_truth" /
                             (body_name + "_kitchen.motion")};
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
    for (int i = 0; i < 3; ++i) {
      for (int j = 0; j < 4; ++j) {
        std::getline(stringstream, parsed, ' ');
        pose.matrix()(i, j) = stof(parsed);
      }
    }
    gt_body2world_poses->push_back(std::move(pose));
  }
  return true;
}

float ChoiEvaluator::ElapsedTime(
    const std::chrono::high_resolution_clock::time_point &begin_time) {
  auto end_time{std::chrono::high_resolution_clock::now()};
  return float(std::chrono::duration_cast<std::chrono::microseconds>(end_time -
                                                                     begin_time)
                   .count());
}
