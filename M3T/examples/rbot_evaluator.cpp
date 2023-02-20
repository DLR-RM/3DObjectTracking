// SPDX-License-Identifier: MIT
// Copyright (c) 2023 Manuel Stoiber, German Aerospace Center (DLR)

#include "rbot_evaluator.h"

RBOTEvaluator::RBOTEvaluator(const std::string &name,
                             const std::filesystem::path &dataset_directory,
                             const std::filesystem::path &external_directory,
                             const std::vector<std::string> &body_names,
                             const std::vector<std::string> &sequence_names,
                             const std::vector<bool> &sequence_occlusions)
    : name_{name},
      dataset_directory_{dataset_directory},
      external_directory_{external_directory},
      body_names_{body_names},
      sequence_names_{sequence_names},
      sequence_occlusions_{sequence_occlusions} {}

bool RBOTEvaluator::SetUp() {
  set_up_ = false;

  // Create run configurations
  run_configurations_.clear();
  for (size_t i = 0; i < sequence_names_.size(); ++i) {
    for (size_t j = 0; j < body_names_.size(); ++j) {
      run_configurations_.push_back(RunConfiguration{
          sequence_names_[i], sequence_occlusions_[i], body_names_[j]});
    }
  }

  // Read poses
  if (!ReadPosesRBOTDataset(dataset_directory_ / "poses_first.txt",
                            &poses_gt_first_))
    return false;
  if (!ReadPosesRBOTDataset(dataset_directory_ / "poses_second.txt",
                            &poses_gt_second_))
    return false;

  // Load bodies and generate models
  LoadBodies();
  GenerateModels();

  set_up_ = true;
  return true;
}

void RBOTEvaluator::set_translation_error_threshold(
    float translation_error_threshold) {
  translation_error_threshold_ = translation_error_threshold;
}

void RBOTEvaluator::set_rotation_error_threshold(
    float rotation_error_threshold) {
  rotation_error_threshold_ = rotation_error_threshold;
}

void RBOTEvaluator::set_run_sequentially(bool run_sequentially) {
  run_sequentially_ = run_sequentially;
}

void RBOTEvaluator::set_visualize_all_results(bool visualize_all_results) {
  visualize_all_results_ = visualize_all_results;
}

void RBOTEvaluator::SaveResults(std::filesystem::path save_directory) {
  save_directory_ = save_directory;
  save_results_ = true;
}

void RBOTEvaluator::DoNotSaveResults() { save_results_ = false; }

void RBOTEvaluator::set_use_region_modality(bool use_region_modality) {
  use_region_modality_ = use_region_modality;
}

void RBOTEvaluator::set_use_texture_modality(bool use_texture_modality) {
  use_texture_modality_ = use_texture_modality;
}

void RBOTEvaluator::set_tracker_setter(
    const std::function<void(std::shared_ptr<m3t::Tracker>)> &tracker_setter) {
  tracker_setter_ = tracker_setter;
}

void RBOTEvaluator::set_optimizer_setter(
    const std::function<void(std::shared_ptr<m3t::Optimizer>)>
        &optimizer_setter) {
  optimizer_setter_ = optimizer_setter;
}

void RBOTEvaluator::set_region_modality_setter(
    const std::function<void(std::shared_ptr<m3t::RegionModality>)>
        &region_modality_setter) {
  region_modality_setter_ = region_modality_setter;
}

void RBOTEvaluator::set_region_model_setter(
    const std::function<void(std::shared_ptr<m3t::RegionModel>)>
        &region_model_setter) {
  region_model_setter_ = region_model_setter;
  set_up_ = false;
}

void RBOTEvaluator::set_texture_modality_setter(
    const std::function<void(std::shared_ptr<m3t::TextureModality>)>
        &texture_modality_setter) {
  texture_modality_setter_ = texture_modality_setter;
}

void RBOTEvaluator::set_depth_renderer_setter(
    const std::function<void(std::shared_ptr<m3t::FocusedBasicDepthRenderer>)>
        &depth_renderer_setter) {
  depth_renderer_setter_ = depth_renderer_setter;
}

bool RBOTEvaluator::Evaluate() {
  if (!set_up_) {
    std::cerr << "Set up evaluator " << name_ << " first" << std::endl;
    return false;
  }

  // Evaluate all run configuration
  results_.resize(run_configurations_.size());
  if (run_sequentially_ || visualize_all_results_) {
    auto renderer_geometry_ptr{std::make_shared<m3t::RendererGeometry>("rg")};
    renderer_geometry_ptr->SetUp();
    for (size_t i = 0; i < int(run_configurations_.size()); ++i) {
      EvaluateRunConfiguration(run_configurations_[i], renderer_geometry_ptr,
                               &results_[i]);

      std::string title{run_configurations_[i].sequence_name + "_" +
                        (run_configurations_[i].occlusions ? "modeled_" : "") +
                        run_configurations_[i].body_name};
      if (save_results_) SaveFinalResult(results_[i], title);
      VisualizeFinalResult(results_[i], title);
    }
  } else {
    std::vector<std::shared_ptr<m3t::RendererGeometry>> renderer_geometry_ptrs(
        omp_get_max_threads());
    for (auto &renderer_geometry_ptr : renderer_geometry_ptrs) {
      renderer_geometry_ptr = std::make_shared<m3t::RendererGeometry>("rg");
      renderer_geometry_ptr->SetUp();
    }
#pragma omp parallel for
    for (int i = 0; i < int(run_configurations_.size()); ++i) {
      EvaluateRunConfiguration(run_configurations_[i],
                               renderer_geometry_ptrs[omp_get_thread_num()],
                               &results_[i]);

      std::string title{run_configurations_[i].sequence_name + "_" +
                        (run_configurations_[i].occlusions ? "modeled_" : "") +
                        run_configurations_[i].body_name};
      if (save_results_) SaveFinalResult(results_[i], title);
#pragma omp critical
      VisualizeFinalResult(results_[i], title);
    }
  }

  // Calculate final results
  CalculateAverageResult(results_, &final_result_);
  if (save_results_) SaveFinalResult(final_result_, "all_sequences_all_bodies");
  VisualizeFinalResult(final_result_, "all_sequences_all_bodies");
  return true;
}

float RBOTEvaluator::tracking_success() const {
  return final_result_.tracking_success;
}

float RBOTEvaluator::execution_time() const {
  return final_result_.execution_times.complete_cycle;
}

void RBOTEvaluator::EvaluateRunConfiguration(
    const RunConfiguration &run_configuration,
    const std::shared_ptr<m3t::RendererGeometry> &renderer_geometry_ptr,
    Result *average_result) const {
  std::shared_ptr<m3t::Tracker> tracker_ptr;
  SetUpTracker(run_configuration, renderer_geometry_ptr, &tracker_ptr);
  const auto &body_ptr{tracker_ptr->optimizer_ptrs()[0]
                           ->root_link_ptr()
                           ->modality_ptrs()[0]
                           ->body_ptr()};
  ResetBody(tracker_ptr, 0);
  if (run_configuration.occlusions) ResetOcclusionBody(tracker_ptr, 0);

  // Iterate over all frames
  std::vector<Result> results(kNFrames_);
  for (int i = 0; i < kNFrames_; ++i) {
    results[i].frame_index = i;
    ExecuteMeasuredTrackingCycle(tracker_ptr, i, &results[i].execution_times);

    // Calculate results for main body
    CalculatePoseResults(body_ptr->body2world_pose(), poses_gt_first_[i + 1],
                         &results[i]);
    if (visualize_all_results_)
      VisualizeFrameResult(results[i], run_configuration.sequence_name + ": " +
                                           run_configuration.body_name);
    if (results[i].tracking_success == 0.0f) ResetBody(tracker_ptr, i + 1);

    // Calculate results for occluding body
    if (run_configuration.occlusions) {
      Result occlusion_result;
      CalculatePoseResults(body_ptr->body2world_pose(), poses_gt_second_[i + 1],
                           &occlusion_result);
      if (occlusion_result.tracking_success == 0.0f)
        ResetOcclusionBody(tracker_ptr, i + 1);
    }
  }
  CalculateAverageResult(results, average_result);
}

void RBOTEvaluator::SetUpTracker(
    const RunConfiguration &run_configuration,
    const std::shared_ptr<m3t::RendererGeometry> &renderer_geometry_ptr,
    std::shared_ptr<m3t::Tracker> *tracker_ptr) const {
  *tracker_ptr = std::make_shared<m3t::Tracker>("tracker");
  renderer_geometry_ptr->ClearBodies();

  // Init camera
  auto camera_ptr{std::make_shared<m3t::LoaderColorCamera>(
      "camera", dataset_directory_ / run_configuration.body_name / "frames",
      kRBOTIntrinsics, run_configuration.sequence_name, 0, 4)};
  camera_ptr->SetUp();

  // Init Viewer
  if (visualize_all_results_) {
    auto viewer_ptr{std::make_shared<m3t::NormalColorViewer>(
        "viewer", camera_ptr, renderer_geometry_ptr)};
    viewer_ptr->SetUp();
    (*tracker_ptr)->AddViewer(viewer_ptr);
  }

  // Init bodies
  const std::string &body_name{run_configuration.body_name};
  auto body_ptr{std::make_shared<m3t::Body>(*body2body_ptr_map_.at(body_name))};
  renderer_geometry_ptr->AddBody(body_ptr);
  auto occlusion_body_ptr{
      std::make_shared<m3t::Body>(*body2body_ptr_map_.at(kOcclusionBodyName))};
  if (run_configuration.occlusions)
    renderer_geometry_ptr->AddBody(occlusion_body_ptr);

  // Init renderer
  auto silhouette_renderer_ptr{std::make_shared<m3t::FocusedSilhouetteRenderer>(
      "silhouette_renderer", renderer_geometry_ptr, camera_ptr)};
  silhouette_renderer_ptr->AddReferencedBody(body_ptr);
  if (run_configuration.occlusions)
    silhouette_renderer_ptr->AddReferencedBody(occlusion_body_ptr);
  silhouette_renderer_ptr->SetUp();

  auto depth_renderer_ptr{std::make_shared<m3t::FocusedBasicDepthRenderer>(
      "depth_renderer", renderer_geometry_ptr, camera_ptr)};
  depth_renderer_ptr->AddReferencedBody(body_ptr);
  depth_renderer_ptr->AddReferencedBody(occlusion_body_ptr);
  depth_renderer_setter_(depth_renderer_ptr);
  depth_renderer_ptr->SetUp();

  // Init link
  auto link_ptr{std::make_shared<m3t::Link>(body_name + "_link", body_ptr)};

  // Init modalities
  if (use_region_modality_) {
    auto region_modality_ptr{std::make_shared<m3t::RegionModality>(
        body_name + "_region_modality", body_ptr, camera_ptr,
        body2model_ptr_map_.at(body_name))};
    region_modality_setter_(region_modality_ptr);
    region_modality_ptr->set_n_unoccluded_iterations(0);
    if (run_configuration.occlusions)
      region_modality_ptr->ModelOcclusions(depth_renderer_ptr);
    region_modality_ptr->SetUp();
    link_ptr->AddModality(region_modality_ptr);
  }

  if (use_texture_modality_) {
    auto texture_modality_ptr{std::make_shared<m3t::TextureModality>(
        body_name + "_texture_modality", body_ptr, camera_ptr,
        silhouette_renderer_ptr)};
    texture_modality_setter_(texture_modality_ptr);
    if (run_configuration.occlusions)
      texture_modality_ptr->ModelOcclusions(depth_renderer_ptr);
    texture_modality_ptr->SetUp();
    link_ptr->AddModality(texture_modality_ptr);
  }

  // Init optimizer
  link_ptr->SetUp();
  auto optimizer_ptr{
      std::make_shared<m3t::Optimizer>(body_name + "_optimizer", link_ptr)};
  optimizer_setter_(optimizer_ptr);
  optimizer_ptr->SetUp();
  (*tracker_ptr)->AddOptimizer(optimizer_ptr);

  if (run_configuration.occlusions) {
    // Init occlusion link
    auto occlusion_link_ptr{std::make_shared<m3t::Link>(
        kOcclusionBodyName + "_link", occlusion_body_ptr)};

    // Init occlusion modalities
    if (use_region_modality_) {
      auto occlusion_region_modality_ptr{std::make_shared<m3t::RegionModality>(
          kOcclusionBodyName + "_region_modality", occlusion_body_ptr,
          camera_ptr, body2model_ptr_map_.at(kOcclusionBodyName))};
      region_modality_setter_(occlusion_region_modality_ptr);
      occlusion_region_modality_ptr->set_n_unoccluded_iterations(0);
      occlusion_region_modality_ptr->ModelOcclusions(depth_renderer_ptr);
      occlusion_region_modality_ptr->SetUp();
      occlusion_link_ptr->AddModality(occlusion_region_modality_ptr);
    }

    if (use_texture_modality_) {
      auto occlusion_texture_modality_ptr{
          std::make_shared<m3t::TextureModality>(
              kOcclusionBodyName + "_texture_modality", occlusion_body_ptr,
              camera_ptr, silhouette_renderer_ptr)};
      texture_modality_setter_(occlusion_texture_modality_ptr);
      occlusion_texture_modality_ptr->ModelOcclusions(depth_renderer_ptr);
      occlusion_texture_modality_ptr->SetUp();
      occlusion_link_ptr->AddModality(occlusion_texture_modality_ptr);
    }

    // Init occlusion optimizer
    occlusion_link_ptr->SetUp();
    auto occlusion_optimizer_ptr{std::make_shared<m3t::Optimizer>(
        kOcclusionBodyName + "_optimizer", occlusion_link_ptr)};
    optimizer_setter_(occlusion_optimizer_ptr);
    occlusion_optimizer_ptr->SetUp();
    (*tracker_ptr)->AddOptimizer(occlusion_optimizer_ptr);
  }

  tracker_setter_(*tracker_ptr);
  (*tracker_ptr)->SetUp(false);
}

void RBOTEvaluator::ResetBody(const std::shared_ptr<m3t::Tracker> &tracker_ptr,
                              int i_frame) const {
  const auto &link_ptr{tracker_ptr->optimizer_ptrs()[0]->root_link_ptr()};
  link_ptr->body_ptr()->set_body2world_pose(poses_gt_first_[i_frame]);
  for (const auto &renderer_ptr : tracker_ptr->start_modality_renderer_ptrs())
    renderer_ptr->StartRendering();
  for (const auto &modality_ptr : link_ptr->modality_ptrs())
    modality_ptr->StartModality(0, 0);
}

void RBOTEvaluator::ResetOcclusionBody(
    const std::shared_ptr<m3t::Tracker> &tracker_ptr, int i_frame) const {
  const auto &link_ptr{tracker_ptr->optimizer_ptrs()[1]->root_link_ptr()};
  link_ptr->body_ptr()->set_body2world_pose(poses_gt_second_[i_frame]);
  for (const auto &renderer_ptr : tracker_ptr->start_modality_renderer_ptrs())
    renderer_ptr->StartRendering();
  for (const auto &modality_ptr : link_ptr->modality_ptrs())
    modality_ptr->StartModality(0, 0);
}

void RBOTEvaluator::ExecuteMeasuredTrackingCycle(
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

      // Visualize optimization
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
  if (visualize_all_results_) tracker_ptr->UpdateViewers(iteration);

  execution_times->complete_cycle =
      execution_times->calculate_correspondences +
      execution_times->calculate_gradient_and_hessian +
      execution_times->calculate_optimization +
      execution_times->calculate_results;
}

void RBOTEvaluator::CalculatePoseResults(
    const m3t::Transform3fA &body2world_pose,
    const m3t::Transform3fA &body2world_pose_gt, Result *result) const {
  result->translation_error = (body2world_pose.translation().matrix() -
                               body2world_pose_gt.translation().matrix())
                                  .norm();
  result->rotation_error =
      acos(((body2world_pose.rotation().matrix().transpose() *
             body2world_pose_gt.rotation().matrix())
                .trace() -
            1.0f) /
           2.0f);
  if (result->translation_error > translation_error_threshold_ ||
      result->rotation_error > rotation_error_threshold_)
    result->tracking_success = 0.0f;
  else
    result->tracking_success = 1.0f;
}

void RBOTEvaluator::CalculateAverageResult(const std::vector<Result> &results,
                                           Result *average_result) {
  average_result->rotation_error = 0.0f;
  average_result->translation_error = 0.0f;
  average_result->tracking_success = 0.0f;
  average_result->execution_times.calculate_correspondences = 0.0f;
  average_result->execution_times.calculate_gradient_and_hessian = 0.0f;
  average_result->execution_times.calculate_optimization = 0.0f;
  average_result->execution_times.calculate_results = 0.0f;
  average_result->execution_times.complete_cycle = 0.0f;

  for (auto &result : results) {
    average_result->rotation_error += result.rotation_error;
    average_result->translation_error += result.translation_error;
    average_result->tracking_success += result.tracking_success;
    average_result->execution_times.calculate_correspondences +=
        result.execution_times.calculate_correspondences;
    average_result->execution_times.calculate_gradient_and_hessian +=
        result.execution_times.calculate_gradient_and_hessian;
    average_result->execution_times.calculate_optimization +=
        result.execution_times.calculate_optimization;
    average_result->execution_times.calculate_results +=
        result.execution_times.calculate_results;
    average_result->execution_times.complete_cycle +=
        result.execution_times.complete_cycle;
  }

  float n = float(results.size());
  average_result->frame_index = 0;
  average_result->rotation_error /= n;
  average_result->translation_error /= n;
  average_result->tracking_success /= n;
  average_result->execution_times.calculate_correspondences /= n;
  average_result->execution_times.calculate_gradient_and_hessian /= n;
  average_result->execution_times.calculate_optimization /= n;
  average_result->execution_times.calculate_results /= n;
  average_result->execution_times.complete_cycle /= n;
}

void RBOTEvaluator::VisualizeFrameResult(const Result &result,
                                         const std::string &title) {
  std::cout << title << ": "
            << "frame " << result.frame_index << ": "
            << "execution_time = " << result.execution_times.complete_cycle
            << " us "
            << "rotation_error = " << result.rotation_error * 180.0f / m3t::kPi
            << ", "
            << "translation_error = " << result.translation_error << ", "
            << "tracking success = " << result.tracking_success << std::endl;
}

void RBOTEvaluator::VisualizeFinalResult(const Result &results,
                                         const std::string &title) {
  std::cout << std::string(80, '-') << std::endl;
  std::cout << title << ":" << std::endl;
  std::cout << "success rate = " << results.tracking_success << std::endl;
  std::cout << "execution times:" << std::endl;
  std::cout << "complete cycle = " << results.execution_times.complete_cycle
            << " us" << std::endl;
  std::cout << "calculate correspondences = "
            << results.execution_times.calculate_correspondences << " us"
            << std::endl;
  std::cout << "calculate gradient and hessian = "
            << results.execution_times.calculate_gradient_and_hessian << " us"
            << std::endl;
  std::cout << "calculate optimization = "
            << results.execution_times.calculate_optimization << " us"
            << std::endl;
  std::cout << "calculate results = "
            << results.execution_times.calculate_results << " us" << std::endl;
}

void RBOTEvaluator::SaveFinalResult(const Result &result,
                                    const std::string &title) const {
  std::ofstream ofs{save_directory_ / ("results_" + title + ".txt")};
  ofs << result.tracking_success << "," << result.execution_times.complete_cycle
      << "," << result.execution_times.calculate_correspondences << ","
      << result.execution_times.calculate_gradient_and_hessian << ","
      << result.execution_times.calculate_optimization << ","
      << result.execution_times.calculate_results << std::endl;
  ofs.flush();
  ofs.close();
}

bool RBOTEvaluator::LoadBodies() {
  for (auto &body_name : body_names_) {
    if (!LoadSingleBody(body_name, dataset_directory_ / body_name))
      return false;
  }
  return LoadSingleBody(kOcclusionBodyName, dataset_directory_);
}

bool RBOTEvaluator::LoadSingleBody(const std::string &body_name,
                                   const std::filesystem::path &directory) {
  auto body_ptr{std::make_shared<m3t::Body>(
      body_name, directory / (body_name + ".obj"), 0.001f, true, false,
      m3t::Transform3fA::Identity())};
  if (!body_ptr->SetUp()) return false;
  body2body_ptr_map_.insert({body_name, std::move(body_ptr)});
  return true;
}

bool RBOTEvaluator::GenerateModels() {
  std::filesystem::path directory{external_directory_ / "models"};
  std::filesystem::create_directories(directory);
  for (auto &body_name : body_names_) {
    if (!GenerateSingleModel(body_name, directory)) return false;
  }
  return GenerateSingleModel(kOcclusionBodyName, directory);
}

bool RBOTEvaluator::GenerateSingleModel(
    const std::string &body_name, const std::filesystem::path &directory) {
  auto region_model_ptr{std::make_shared<m3t::RegionModel>(
      body_name + "_model", body2body_ptr_map_[body_name],
      directory / (body_name + "_model.bin"), 0.8f, 4, 200, 0.01f, 0.002f,
      false, 2000)};
  region_model_setter_(region_model_ptr);
  if (!region_model_ptr->SetUp()) return false;
  body2model_ptr_map_.insert({body_name, std::move(region_model_ptr)});
  return true;
}

bool RBOTEvaluator::ReadPosesRBOTDataset(
    const std::filesystem::path &path, std::vector<m3t::Transform3fA> *poses) {
  std::ifstream ifs{path.string(), std::ios::binary};
  if (!ifs.is_open() || ifs.fail()) {
    ifs.close();
    std::cerr << "Could not open file stream " << path.string() << std::endl;
    return false;
  }

  poses->resize(kNFrames_ + 1);
  std::string parsed;
  std::getline(ifs, parsed);
  for (auto &pose : *poses) {
    for (int i = 0; i < 3; ++i) {
      for (int j = 0; j < 3; ++j) {
        std::getline(ifs, parsed, '\t');
        pose.matrix()(i, j) = stof(parsed);
      }
    }
    std::getline(ifs, parsed, '\t');
    pose.matrix()(0, 3) = stof(parsed) * 0.001f;
    std::getline(ifs, parsed, '\t');
    pose.matrix()(1, 3) = stof(parsed) * 0.001f;
    std::getline(ifs, parsed);
    pose.matrix()(2, 3) = stof(parsed) * 0.001f;
  }
  return true;
}

float RBOTEvaluator::ElapsedTime(
    const std::chrono::high_resolution_clock::time_point &begin_time) {
  auto end_time{std::chrono::high_resolution_clock::now()};
  return float(std::chrono::duration_cast<std::chrono::microseconds>(end_time -
                                                                     begin_time)
                   .count());
}
