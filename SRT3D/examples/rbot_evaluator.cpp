// SPDX-License-Identifier: MIT
// Copyright (c) 2021 Manuel Stoiber, German Aerospace Center (DLR)

#include "rbot_evaluator.h"

RBOTEvaluator::RBOTEvaluator(const std::string &name,
                             const std::filesystem::path &dataset_directory,
                             const std::vector<std::string> &body_names,
                             const std::vector<std::string> &sequence_names,
                             const std::vector<bool> &sequence_occlusions)
    : name_{name},
      dataset_directory_{dataset_directory},
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

  // Generate models
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

void RBOTEvaluator::set_visualize_all_results(bool visualize_all_results) {
  visualize_all_results_ = visualize_all_results;
}

void RBOTEvaluator::SaveResults(std::filesystem::path save_directory) {
  save_directory_ = save_directory;
  save_results_ = true;
}

void RBOTEvaluator::DoNotSaveResults() { save_results_ = false; }

void RBOTEvaluator::set_tracker_setter(
    const std::function<void(std::shared_ptr<srt3d::Tracker>)>
        &tracker_setter) {
  tracker_setter_ = tracker_setter;
}

void RBOTEvaluator::set_region_modality_setter(
    const std::function<void(std::shared_ptr<srt3d::RegionModality>)>
        &region_modality_setter) {
  region_modality_setter_ = region_modality_setter;
}

void RBOTEvaluator::set_model_setter(
    const std::function<void(std::shared_ptr<srt3d::Model>)> &model_setter) {
  model_setter_ = model_setter;
  set_up_ = false;
}

void RBOTEvaluator::set_occlusion_renderer_setter(
    const std::function<void(std::shared_ptr<srt3d::OcclusionRenderer>)>
        &occlusion_renderer_setter) {
  occlusion_renderer_setter_ = occlusion_renderer_setter;
}

bool RBOTEvaluator::Evaluate() {
  if (!set_up_) {
    std::cerr << "Set up evaluator " << name_ << " first" << std::endl;
    return false;
  }

  // Evaluate all run configuration
  results_.resize(run_configurations_.size());
  if (visualize_all_results_) {
    auto renderer_geometry_ptr{std::make_shared<srt3d::RendererGeometry>("rg")};
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
    std::vector<std::shared_ptr<srt3d::RendererGeometry>>
        renderer_geometry_ptrs(omp_get_max_threads());
    for (auto &renderer_geometry_ptr : renderer_geometry_ptrs) {
      renderer_geometry_ptr = std::make_shared<srt3d::RendererGeometry>("rg");
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

float RBOTEvaluator::tracking_success() {
  return final_result_.tracking_success;
}

void RBOTEvaluator::EvaluateRunConfiguration(
    RunConfiguration run_configuration,
    std::shared_ptr<srt3d::RendererGeometry> renderer_geometry_ptr,
    DataResult *average_result) {
  auto tracker_ptr{std::make_shared<srt3d::Tracker>("tracker")};
  SetUpTracker(run_configuration, renderer_geometry_ptr, tracker_ptr);
  ResetBody(tracker_ptr, 0);
  if (run_configuration.occlusions) ResetOcclusionBody(tracker_ptr, 0);

  // Iterate over all frames
  std::vector<DataResult> results(kNFrames_);
  for (int i = 0; i < kNFrames_; ++i) {
    results[i].frame_index = i;
    ExecuteMeasuredTrackingCycle(tracker_ptr, i, &results[i].execution_times);

    // Calculate results for main body
    CalculatePoseResults(
        tracker_ptr->region_modality_ptrs()[0]->body_ptr()->body2world_pose(),
        poses_gt_first_[i + 1], &results[i]);
    if (visualize_all_results_)
      VisualizeFrameResult(results[i], run_configuration.sequence_name + ": " +
                                           run_configuration.body_name);
    if (results[i].tracking_success == 0.0f) ResetBody(tracker_ptr, i + 1);

    // Calculate results for occluding body
    if (run_configuration.occlusions) {
      DataResult occlusion_result;
      CalculatePoseResults(
          tracker_ptr->region_modality_ptrs()[1]->body_ptr()->body2world_pose(),
          poses_gt_second_[i + 1], &occlusion_result);
      if (occlusion_result.tracking_success == 0.0f)
        ResetOcclusionBody(tracker_ptr, i + 1);
    }
  }
  CalculateAverageResult(results, average_result);
}

void RBOTEvaluator::SetUpTracker(
    const RunConfiguration &run_configuration,
    std::shared_ptr<srt3d::RendererGeometry> renderer_geometry_ptr,
    std::shared_ptr<srt3d::Tracker> tracker_ptr) {
  renderer_geometry_ptr->ClearBodies();

  // Init camera
  auto camera_ptr{std::make_shared<srt3d::LoaderCamera>(
      "camera", dataset_directory_ / run_configuration.body_name / "frames",
      kRBOTIntrinsics, run_configuration.sequence_name, 0, 4)};

  // Init Viewer
  if (visualize_all_results_) {
    auto viewer_ptr{std::make_shared<srt3d::NormalViewer>(
        "viewer", camera_ptr, renderer_geometry_ptr)};
    tracker_ptr->AddViewer(viewer_ptr);
  }

  // Init body
  std::filesystem::path geometry_path{dataset_directory_ /
                                      run_configuration.body_name /
                                      (run_configuration.body_name + ".obj")};
  auto body_ptr{std::make_shared<srt3d::Body>(
      run_configuration.body_name, geometry_path, 0.001f, true, false, 0.3f,
      srt3d::Transform3fA::Identity(), 7)};
  renderer_geometry_ptr->AddBody(body_ptr);

  // Init model
  auto model_ptr{std::make_shared<srt3d::Model>(
      "model", body_ptr, dataset_directory_ / run_configuration.body_name,
      run_configuration.body_name + "_model.bin", 0.8, 4, 200, false, 2000)};
  model_setter_(model_ptr);

  // Init region modality
  auto region_modality_ptr{std::make_shared<srt3d::RegionModality>(
      "region_modality", body_ptr, model_ptr, camera_ptr)};
  region_modality_setter_(region_modality_ptr);
  tracker_ptr->AddRegionModality(region_modality_ptr);

  if (run_configuration.occlusions) {
    // Init occlusion body
    auto occlusion_body_ptr{std::make_shared<srt3d::Body>(
        "squirrel_small", dataset_directory_ / "squirrel_small.obj", 0.001f,
        true, false, 0.3f, srt3d::Transform3fA::Identity(), 1)};
    renderer_geometry_ptr->AddBody(occlusion_body_ptr);

    // Init occlusion model
    auto occlusion_model_ptr{std::make_shared<srt3d::Model>(
        "occlusion_model", occlusion_body_ptr, dataset_directory_,
        "squirrel_small_model.bin", 0.8, 4, 200, false, 2000)};
    model_setter_(occlusion_model_ptr);

    // Init occlusion region modality
    auto occlusion_region_modality_ptr{std::make_shared<srt3d::RegionModality>(
        "occlusion_region_modality", occlusion_body_ptr, occlusion_model_ptr,
        camera_ptr)};
    region_modality_setter_(occlusion_region_modality_ptr);
    tracker_ptr->AddRegionModality(occlusion_region_modality_ptr);

    // Init occlusion renderer
    auto occlusion_renderer_ptr{std::make_shared<srt3d::OcclusionRenderer>(
        "occlusion_renderer", renderer_geometry_ptr, camera_ptr)};
    occlusion_renderer_setter_(occlusion_renderer_ptr);
    region_modality_ptr->UseOcclusionHandling(occlusion_renderer_ptr);
    occlusion_region_modality_ptr->UseOcclusionHandling(occlusion_renderer_ptr);
  }

  tracker_setter_(tracker_ptr);
  tracker_ptr->SetUpTracker();
}

void RBOTEvaluator::ResetBody(std::shared_ptr<srt3d::Tracker> tracker_ptr,
                              int i_frame) {
  tracker_ptr->region_modality_ptrs()[0]->body_ptr()->set_body2world_pose(
      poses_gt_first_[i_frame]);
  tracker_ptr->region_modality_ptrs()[0]->StartModality();
}

void RBOTEvaluator::ResetOcclusionBody(
    std::shared_ptr<srt3d::Tracker> tracker_ptr, int i_frame) {
  tracker_ptr->region_modality_ptrs()[1]->body_ptr()->set_body2world_pose(
      poses_gt_second_[i_frame]);
  tracker_ptr->region_modality_ptrs()[1]->StartModality();
}

void RBOTEvaluator::ExecuteMeasuredTrackingCycle(
    std::shared_ptr<srt3d::Tracker> tracker_ptr, int iteration,
    DataExecutionTimes *execution_times) {
  // Calculate before camera update and camera update
  auto begin_time{std::chrono::high_resolution_clock::now()};
  tracker_ptr->CalculateBeforeCameraUpdate();
  execution_times->calculate_before_camera_update = ElapsedTime(begin_time);
  tracker_ptr->UpdateCameras();

  execution_times->start_occlusion_rendering = 0.0f;
  execution_times->calculate_correspondences = 0.0f;
  execution_times->calculate_pose_update = 0.0f;
  for (int corr_iteration = 0;
       corr_iteration < tracker_ptr->n_corr_iterations(); ++corr_iteration) {
    // Start occlusion rendering
    begin_time = std::chrono::high_resolution_clock::now();
    tracker_ptr->StartOcclusionRendering();
    execution_times->start_occlusion_rendering += ElapsedTime(begin_time);

    // Calculate correspondences
    begin_time = std::chrono::high_resolution_clock::now();
    tracker_ptr->CalculateCorrespondences(corr_iteration);
    execution_times->calculate_correspondences += ElapsedTime(begin_time);

    // Visualize correspondences
    int corr_save_idx =
        iteration * tracker_ptr->n_corr_iterations() + corr_iteration;
    tracker_ptr->VisualizeCorrespondences(corr_save_idx);

    for (int update_iteration = 0;
         update_iteration < tracker_ptr->n_update_iterations();
         ++update_iteration) {
      // Calculate pose update
      begin_time = std::chrono::high_resolution_clock::now();
      tracker_ptr->CalculatePoseUpdate(corr_iteration, update_iteration);
      execution_times->calculate_pose_update += ElapsedTime(begin_time);

      // Visualize pose update
      int update_save_idx =
          corr_save_idx * tracker_ptr->n_update_iterations() + update_iteration;
      tracker_ptr->VisualizePoseUpdate(update_save_idx);
    }
  }

  // Visualize results and update viewers
  tracker_ptr->VisualizeResults(iteration);
  if (visualize_all_results_) tracker_ptr->UpdateViewers(iteration);

  execution_times->complete_cycle =
      execution_times->calculate_before_camera_update +
      execution_times->start_occlusion_rendering +
      execution_times->calculate_correspondences +
      execution_times->calculate_pose_update;
}

void RBOTEvaluator::CalculatePoseResults(
    const srt3d::Transform3fA &body2world_pose,
    const srt3d::Transform3fA &body2world_pose_gt, DataResult *result) const {
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

void RBOTEvaluator::CalculateAverageResult(
    const std::vector<DataResult> &results, DataResult *average_result) {
  average_result->rotation_error = 0.0f;
  average_result->translation_error = 0.0f;
  average_result->tracking_success = 0.0f;
  average_result->execution_times.calculate_before_camera_update = 0.0;
  average_result->execution_times.calculate_correspondences = 0.0f;
  average_result->execution_times.calculate_pose_update = 0.0f;
  average_result->execution_times.complete_cycle = 0.0f;
  average_result->execution_times.start_occlusion_rendering = 0.0f;

  for (auto &result : results) {
    average_result->rotation_error += result.rotation_error;
    average_result->translation_error += result.translation_error;
    average_result->tracking_success += result.tracking_success;
    average_result->execution_times.calculate_before_camera_update +=
        result.execution_times.calculate_before_camera_update;
    average_result->execution_times.calculate_correspondences +=
        result.execution_times.calculate_correspondences;
    average_result->execution_times.calculate_pose_update +=
        result.execution_times.calculate_pose_update;
    average_result->execution_times.complete_cycle +=
        result.execution_times.complete_cycle;
    average_result->execution_times.start_occlusion_rendering +=
        result.execution_times.start_occlusion_rendering;
  }

  float n = float(results.size());
  average_result->frame_index = 0;
  average_result->rotation_error /= n;
  average_result->translation_error /= n;
  average_result->tracking_success /= n;
  average_result->execution_times.calculate_before_camera_update /= n;
  average_result->execution_times.calculate_correspondences /= n;
  average_result->execution_times.calculate_pose_update /= n;
  average_result->execution_times.complete_cycle /= n;
  average_result->execution_times.start_occlusion_rendering /= n;
}

void RBOTEvaluator::VisualizeFrameResult(const DataResult &result,
                                         const std::string &title) {
  std::cout << title << ": "
            << "frame " << result.frame_index << ": "
            << "execution_time = " << result.execution_times.complete_cycle
            << " us "
            << "rotation_error = "
            << result.rotation_error * 180.0f / srt3d::kPi << ", "
            << "translation_error = " << result.translation_error << ", "
            << "tracking success = " << result.tracking_success << std::endl;
}

void RBOTEvaluator::VisualizeFinalResult(const DataResult &results,
                                         const std::string &title) {
  std::cout << std::string(80, '-') << std::endl;
  std::cout << title << ":" << std::endl;
  std::cout << "success rate = " << results.tracking_success << std::endl;
  std::cout << "execution times:" << std::endl;
  std::cout << "complete cycle = " << results.execution_times.complete_cycle
            << " us" << std::endl;
  std::cout << "calculate before camera update = "
            << results.execution_times.calculate_before_camera_update << " us"
            << std::endl;
  std::cout << "start occlusion rendering = "
            << results.execution_times.start_occlusion_rendering << " us"
            << std::endl;
  std::cout << "calculate correspondences = "
            << results.execution_times.calculate_correspondences << " us"
            << std::endl;
  std::cout << "calculate pose update = "
            << results.execution_times.calculate_pose_update << " us"
            << std::endl;
}

void RBOTEvaluator::SaveFinalResult(const DataResult &result,
                                    const std::string &title) const {
  std::ofstream ofs{save_directory_ / ("results_" + title + ".txt")};
  ofs << result.tracking_success << "," << result.execution_times.complete_cycle
      << "," << result.execution_times.calculate_before_camera_update << ","
      << result.execution_times.start_occlusion_rendering << ","
      << result.execution_times.calculate_correspondences << ","
      << result.execution_times.calculate_pose_update << std::endl;
  ofs.flush();
  ofs.close();
}

bool RBOTEvaluator::GenerateModels() {
  for (auto &body_name : body_names_) {
    if (!GenerateSingleModel(body_name, dataset_directory_ / body_name))
      return false;
  }
  if (!GenerateSingleModel("squirrel_small", dataset_directory_)) return false;
  return true;
}

bool RBOTEvaluator::GenerateSingleModel(
    const std::string &body_name, const std::filesystem::path &directory) {
  auto body_ptr{std::make_shared<srt3d::Body>(
      body_name, directory / (body_name + ".obj"), 0.001f, true, false, 0.3f,
      srt3d::Transform3fA::Identity())};
  auto model_ptr{std::make_shared<srt3d::Model>("model", body_ptr, directory,
                                                body_name + "_model.bin", 0.8f,
                                                4, 200, false, 2000)};
  model_setter_(model_ptr);
  return model_ptr->SetUp();
}

bool RBOTEvaluator::ReadPosesRBOTDataset(
    const std::filesystem::path &path,
    std::vector<srt3d::Transform3fA> *poses) {
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
