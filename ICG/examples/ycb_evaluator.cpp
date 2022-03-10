// SPDX-License-Identifier: MIT
// Copyright (c) 2022 Manuel Stoiber, German Aerospace Center (DLR)

#include "ycb_evaluator.h"

YCBEvaluator::YCBEvaluator(const std::string &name,
                           const std::filesystem::path &dataset_directory,
                           const std::filesystem::path &external_directory,
                           const std::vector<int> &sequence_ids,
                           const std::vector<std::string> &evaluated_body_names)
    : name_{name},
      dataset_directory_{dataset_directory},
      external_directory_{external_directory},
      sequence_ids_{sequence_ids},
      evaluated_body_names_{evaluated_body_names} {
  // Compute thresholds used to compute AUC score
  float threshold_step = kThresholdMax / float(kNCurveValues);
  for (size_t i = 0; i < kNCurveValues; ++i) {
    thresholds_[i] = threshold_step * (0.5f + float(i));
  }
}

bool YCBEvaluator::SetUp() {
  set_up_ = false;

  if (!CreateRunConfigurations()) return false;
  AssambleTrackedBodyNames();
  if (!LoadBodies()) return false;
  if (!GenerateModels()) return false;
  if (!LoadKeyframes()) return false;
  LoadNFrames();
  LoadPoseBegin();
  GenderateReducedVertices();
  GenerateKDTrees();

  set_up_ = true;
  return true;
}

void YCBEvaluator::set_detector_folder(const std::string &detector_folder) {
  detector_folder_ = detector_folder;
}

void YCBEvaluator::set_evaluate_refinement(bool evaluate_refinement) {
  evaluate_refinement_ = evaluate_refinement;
}

void YCBEvaluator::set_use_detector_initialization(
    bool use_detector_initialization) {
  use_detector_initialization_ = use_detector_initialization;
}

void YCBEvaluator::set_use_matlab_gt_poses(bool use_matlab_gt_poses) {
  use_matlab_gt_poses_ = use_matlab_gt_poses;
}

void YCBEvaluator::set_run_sequentially(bool run_sequentially) {
  run_sequentially_ = run_sequentially;
}

void YCBEvaluator::set_use_random_seed(bool use_random_seed) {
  use_random_seed_ = use_random_seed;
}

void YCBEvaluator::set_n_vertices_evaluation(int n_vertices_evaluation) {
  n_vertices_evaluation_ = n_vertices_evaluation;
}

void YCBEvaluator::set_visualize_tracking(bool visualize_tracking) {
  visualize_tracking_ = visualize_tracking;
}

void YCBEvaluator::set_visualize_frame_results(bool visualize_frame_results) {
  visualize_frame_results_ = visualize_frame_results;
}

void YCBEvaluator::StartSavingImages(
    const std::filesystem::path &save_directory) {
  save_images_ = true;
  save_directory_ = save_directory;
}

void YCBEvaluator::StopSavingImages() { save_images_ = false; }

void YCBEvaluator::set_use_region_modality(bool use_region_modality) {
  use_region_modality_ = use_region_modality;
}

void YCBEvaluator::set_use_depth_modality(bool use_depth_modality) {
  use_depth_modality_ = use_depth_modality;
}

void YCBEvaluator::set_measure_occlusions_region(
    bool measure_occlusions_region) {
  measure_occlusions_region_ = measure_occlusions_region;
}

void YCBEvaluator::set_measure_occlusions_depth(bool measure_occlusions_depth) {
  measure_occlusions_depth_ = measure_occlusions_depth;
}

void YCBEvaluator::set_model_occlusions_region(bool model_occlusions_region) {
  model_occlusions_region_ = model_occlusions_region;
}

void YCBEvaluator::set_model_occlusions_depth(bool model_occlusions_depth) {
  model_occlusions_depth_ = model_occlusions_depth;
}

void YCBEvaluator::set_tracker_setter(
    const std::function<void(std::shared_ptr<icg::Tracker>)> &tracker_setter) {
  tracker_setter_ = tracker_setter;
}

void YCBEvaluator::set_refiner_setter(
    const std::function<void(std::shared_ptr<icg::Refiner>)> &refiner_setter) {
  refiner_setter_ = refiner_setter;
}

void YCBEvaluator::set_optimizer_setter(
    const std::function<void(std::shared_ptr<icg::Optimizer>)>
        &optimizer_setter) {
  optimizer_setter_ = optimizer_setter;
}

void YCBEvaluator::set_region_modality_setter(
    const std::function<void(std::shared_ptr<icg::RegionModality>)>
        &region_modality_setter) {
  region_modality_setter_ = region_modality_setter;
}

void YCBEvaluator::set_region_model_setter(
    const std::function<void(std::shared_ptr<icg::RegionModel>)>
        &region_model_setter) {
  region_model_setter_ = region_model_setter;
  set_up_ = false;
}

void YCBEvaluator::set_depth_modality_setter(
    const std::function<void(std::shared_ptr<icg::DepthModality>)>
        &depth_modality_setter) {
  depth_modality_setter_ = depth_modality_setter;
}

void YCBEvaluator::set_depth_model_setter(
    const std::function<void(std::shared_ptr<icg::DepthModel>)>
        &depth_model_setter) {
  depth_model_setter_ = depth_model_setter;
  set_up_ = false;
}

bool YCBEvaluator::Evaluate() {
  if (!set_up_) {
    std::cerr << "Set up evaluator " << name_ << " first" << std::endl;
    return false;
  }
  if (run_configurations_.empty()) return false;

  // Evaluate all run configurations
  results_.clear();
  final_results_.clear();
  if (run_sequentially_ || visualize_tracking_ || visualize_frame_results_) {
    auto renderer_geometry_ptr{std::make_shared<icg::RendererGeometry>("rg")};
    renderer_geometry_ptr->SetUp();
    for (size_t i = 0; i < int(run_configurations_.size()); ++i) {
      std::vector<SequenceResult> sequence_results;
      if (!EvaluateRunConfiguration(run_configurations_[i],
                                    renderer_geometry_ptr, &sequence_results))
        continue;
      results_.insert(end(results_), begin(sequence_results),
                      end(sequence_results));
      for (const auto &sequence_result : sequence_results) {
        std::string title{sequence_result.sequence_name + ": " +
                          sequence_result.body_name};
        VisualizeResult(sequence_result.average_result, title);
      }
    }
  } else {
    std::vector<std::shared_ptr<icg::RendererGeometry>> renderer_geometry_ptrs(
        omp_get_max_threads());
    for (auto &renderer_geometry_ptr : renderer_geometry_ptrs) {
      renderer_geometry_ptr = std::make_shared<icg::RendererGeometry>("rg");
      renderer_geometry_ptr->SetUp();
    }
#pragma omp parallel for schedule(dynamic, 1)
    for (int i = 0; i < int(run_configurations_.size()); ++i) {
      std::vector<SequenceResult> sequence_results;
      if (!EvaluateRunConfiguration(
              run_configurations_[i],
              renderer_geometry_ptrs[omp_get_thread_num()], &sequence_results))
        continue;
#pragma omp critical
      {
        results_.insert(end(results_), begin(sequence_results),
                        end(sequence_results));
        for (const auto &sequence_result : sequence_results) {
          std::string title{sequence_result.sequence_name + ": " +
                            sequence_result.body_name};
          VisualizeResult(sequence_result.average_result, title);
        }
      }
    }
  }

  // Calculate sequnce results
  std::cout << std::endl << std::string(80, '-') << std::endl;
  for (int sequence_id : sequence_ids_) {
    auto sequence_name{SequenceIDToName(sequence_id)};
    Result result{CalculateAverageSequenceResult({sequence_name})};
    VisualizeResult(result, sequence_name);
  }

  // Calculate body results
  std::cout << std::endl << std::string(80, '-') << std::endl;
  for (const auto &body_name : evaluated_body_names_) {
    Result result{CalculateAverageBodyResult({body_name})};
    final_results_.insert({body_name, result});
    VisualizeResult(result, body_name);
  }

  // Calculate average results
  Result final_result_all{CalculateAverageBodyResult({evaluated_body_names_})};
  final_results_.insert({"all", final_result_all});
  VisualizeResult(final_result_all, "all");
  std::cout << std::string(80, '-') << std::endl;
  return true;
}

void YCBEvaluator::SaveResults(std::filesystem::path path) const {
  std::ofstream ofs{path};
  for (auto const &[body_name, result] : final_results_) {
    ofs << body_name << "," << result.add_auc << "," << result.adds_auc << ","
        << result.execution_times.complete_cycle << ","
        << result.execution_times.start_modalities << ","
        << result.execution_times.calculate_correspondences << ","
        << result.execution_times.calculate_gradient_and_hessian << ","
        << result.execution_times.calculate_optimization << ","
        << result.execution_times.calculate_results << std::endl;
  }
  ofs.flush();
  ofs.close();
}

float YCBEvaluator::add_auc() const { return final_results_.at("all").add_auc; }

float YCBEvaluator::adds_auc() const {
  return final_results_.at("all").adds_auc;
}

float YCBEvaluator::execution_time() const {
  return final_results_.at("all").execution_times.complete_cycle;
}

std::map<std::string, YCBEvaluator::Result> YCBEvaluator::final_results()
    const {
  return final_results_;
}

bool YCBEvaluator::EvaluateRunConfiguration(
    const RunConfiguration &run_configuration,
    const std::shared_ptr<icg::RendererGeometry> &renderer_geometry_ptr,
    std::vector<SequenceResult> *sequence_results) const {
  const auto &sequence_name{run_configuration.sequence_name};
  const auto &evaluated_body_names{run_configuration.evaluated_body_names};
  const auto &tracked_body_names{run_configuration.tracked_body_names};
  const auto &keyframes{sequence2keyframes_map_.at(sequence_name)};
  size_t n_keyframes = keyframes.size();

  // Initialize tracker
  std::shared_ptr<icg::Tracker> tracker_ptr;
  if (!SetUpTracker(run_configuration, renderer_geometry_ptr, &tracker_ptr))
    return false;

  // Read gt poses and detector poses
  std::map<std::string, std::vector<icg::Transform3fA>> gt_body2world_poses;
  std::map<std::string, std::vector<icg::Transform3fA>>
      detector_body2world_poses;
  std::map<std::string, std::vector<bool>> body_detected;
  for (const auto &body_name : tracked_body_names) {
    if (use_matlab_gt_poses_) {
      if (!LoadMatlabGTPoses(sequence_name, body_name,
                             &gt_body2world_poses[body_name]))
        return false;
    } else {
      if (!LoadGTPoses(sequence_name, body_name,
                       &gt_body2world_poses[body_name]))
        return false;
    }
    if (evaluate_refinement_) {
      if (!LoadDetectorPoses(sequence_name, body_name,
                             &detector_body2world_poses[body_name],
                             &body_detected[body_name]))
        return false;
    }
  }

  // Init results
  sequence_results->resize(evaluated_body_names.size());
  for (int i = 0; i < evaluated_body_names.size(); ++i) {
    (*sequence_results)[i].sequence_name = sequence_name;
    (*sequence_results)[i].body_name = evaluated_body_names[i];
    (*sequence_results)[i].frame_results.clear();
  }

  // Iterate over all frames
  for (int i = 0; i < n_keyframes; ++i) {
    Result general_results;
    general_results.frame_index = i;

    // Execute different evaluations
    if (evaluate_refinement_) {
      ResetBodies(tracker_ptr, tracked_body_names, detector_body2world_poses,
                  i);
      ExecuteMeasuredRefinementCycle(tracker_ptr, keyframes[i], i,
                                     &general_results.execution_times);
    } else {
      if (i == 0) {
        ResetBodies(tracker_ptr, tracked_body_names, gt_body2world_poses, 0);
        UpdateCameras(tracker_ptr, keyframes[0]);
        tracker_ptr->StartModalities(0);
      }
      ExecuteMeasuredTrackingCycle(tracker_ptr, keyframes[i], i,
                                   &general_results.execution_times);
    }

    // Calculate results
    for (int j = 0; j < evaluated_body_names.size(); ++j) {
      const auto &body_name{evaluated_body_names[j]};
      Result result{general_results};
      CalculatePoseResults(tracker_ptr, body_name,
                           gt_body2world_poses[body_name][i], &result);
      if (visualize_frame_results_)
        VisualizeResult(result, sequence_name + ": " + body_name);
      (*sequence_results)[j].frame_results.push_back(std::move(result));
    }
  }

  // Calculate Average Results
  for (auto &sequence_result : *sequence_results)
    sequence_result.average_result =
        CalculateAverageResult(sequence_result.frame_results);
  return true;
}

bool YCBEvaluator::SetUpTracker(
    const RunConfiguration &run_configuration,
    const std::shared_ptr<icg::RendererGeometry> &renderer_geometry_ptr,
    std::shared_ptr<icg::Tracker> *tracker_ptr) const {
  renderer_geometry_ptr->ClearBodies();
  *tracker_ptr = std::make_shared<icg::Tracker>("tracker");
  auto refiner_ptr{std::make_shared<icg::Refiner>("refiner")};

  // Init cameras
  std::filesystem::path camera_directory{dataset_directory_ / "data" /
                                         run_configuration.sequence_name};
  auto color_camera_ptr{std::make_shared<icg::LoaderColorCamera>(
      "color_camera", camera_directory, kYCBIntrinsics, "", 1, 6, "-color")};
  color_camera_ptr->SetUp();
  auto depth_camera_ptr{std::make_shared<icg::LoaderDepthCamera>(
      "depth_camera", camera_directory, kYCBIntrinsics, 0.0001f, "", 1, 6,
      "-depth")};
  depth_camera_ptr->SetUp();

  // Init visualizer
  auto save_directory_sequence{save_directory_ /
                               run_configuration.sequence_name};
  if (save_images_) std::filesystem::create_directory(save_directory_sequence);
  if (use_region_modality_ && (visualize_tracking_ || save_images_)) {
    auto color_viewer_ptr{std::make_shared<icg::NormalColorViewer>(
        "color_viewer", color_camera_ptr, renderer_geometry_ptr)};
    color_viewer_ptr->set_display_images(visualize_tracking_);
    if (save_images_)
      color_viewer_ptr->StartSavingImages(save_directory_sequence);
    color_viewer_ptr->SetUp();
    (*tracker_ptr)->AddViewer(color_viewer_ptr);
  }
  if (use_depth_modality_ && (visualize_tracking_ || save_images_)) {
    auto depth_viewer_ptr{std::make_shared<icg::NormalDepthViewer>(
        "depth_viewer", depth_camera_ptr, renderer_geometry_ptr, 0.3f, 1.0f)};
    depth_viewer_ptr->set_display_images(visualize_tracking_);
    if (save_images_)
      depth_viewer_ptr->StartSavingImages(save_directory_sequence);
    depth_viewer_ptr->SetUp();
    (*tracker_ptr)->AddViewer(depth_viewer_ptr);
  }

  // Init renderer
  auto color_depth_renderer_ptr{
      std::make_shared<icg::FocusedBasicDepthRenderer>(
          "color_depth_renderer", renderer_geometry_ptr, color_camera_ptr)};
  auto depth_depth_renderer_ptr{
      std::make_shared<icg::FocusedBasicDepthRenderer>(
          "depth_depth_renderer", renderer_geometry_ptr, depth_camera_ptr)};

  // Iterate over tracked bodies
  for (const auto &body_name : run_configuration.tracked_body_names) {
    // Init body
    auto body_ptr{
        std::make_shared<icg::Body>(*body2body_ptr_map_.at(body_name))};
    renderer_geometry_ptr->AddBody(body_ptr);

    // Init optimizer
    auto optimizer_ptr{
        std::make_shared<icg::Optimizer>(body_name + "_optimizer")};

    // Init region modality
    if (use_region_modality_) {
      auto region_modality_ptr{std::make_shared<icg::RegionModality>(
          body_name + "_region_modality", body_ptr, color_camera_ptr,
          body2region_model_ptr_map_.at(body_name))};
      region_modality_ptr->set_n_unoccluded_iterations(0);
      region_modality_setter_(region_modality_ptr);
      if (measure_occlusions_region_)
        region_modality_ptr->MeasureOcclusions(depth_camera_ptr);
      if (model_occlusions_region_) {
        color_depth_renderer_ptr->AddReferencedBody(body_ptr);
        color_depth_renderer_ptr->SetUp();
        region_modality_ptr->ModelOcclusions(color_depth_renderer_ptr);
      }
      region_modality_ptr->SetUp();
      optimizer_ptr->AddModality(region_modality_ptr);
    }

    // Init depth modality
    if (use_depth_modality_) {
      auto depth_modality_ptr{std::make_shared<icg::DepthModality>(
          body_name + "_depth_modality", body_ptr, depth_camera_ptr,
          body2depth_model_ptr_map_.at(body_name))};
      depth_modality_ptr->set_n_unoccluded_iterations(0);
      depth_modality_setter_(depth_modality_ptr);
      if (measure_occlusions_depth_) depth_modality_ptr->MeasureOcclusions();
      if (model_occlusions_depth_) {
        depth_depth_renderer_ptr->AddReferencedBody(body_ptr);
        depth_depth_renderer_ptr->SetUp();
        depth_modality_ptr->ModelOcclusions(depth_depth_renderer_ptr);
      }
      depth_modality_ptr->SetUp();
      optimizer_ptr->AddModality(depth_modality_ptr);
    }

    // Init optimizer
    optimizer_setter_(optimizer_ptr);
    optimizer_ptr->SetUp();
    (*tracker_ptr)->AddOptimizer(optimizer_ptr);
    refiner_ptr->AddOptimizer(optimizer_ptr);
  }

  // Init refiner
  refiner_setter_(refiner_ptr);
  refiner_ptr->SetUp(false);
  (*tracker_ptr)->AddRefiner(refiner_ptr);

  // Init tracker
  tracker_setter_(*tracker_ptr);
  return (*tracker_ptr)->SetUp(false);
}

void YCBEvaluator::ResetBodies(
    const std::shared_ptr<icg::Tracker> &tracker_ptr,
    const std::vector<std::string> &body_names,
    const std::map<std::string, std::vector<icg::Transform3fA>>
        &body2world_poses,
    int idx) const {
  for (const auto &body_name : body_names) {
    for (auto &modality_ptr : tracker_ptr->modality_ptrs()) {
      if (modality_ptr->body_ptr()->name() == body_name)
        modality_ptr->body_ptr()->set_body2world_pose(
            body2world_poses.at(body_name)[idx]);
    }
  }
}

void YCBEvaluator::ExecuteMeasuredRefinementCycle(
    const std::shared_ptr<icg::Tracker> &tracker_ptr, int load_index,
    int iteration, ExecutionTimes *execution_times) const {
  std::chrono::high_resolution_clock::time_point begin_time;
  auto refiner_ptr{tracker_ptr->refiner_ptrs()[0]};

  // Update Cameras
  UpdateCameras(tracker_ptr, load_index);

  execution_times->start_modalities = 0.0f;
  execution_times->calculate_correspondences = 0.0f;
  execution_times->calculate_gradient_and_hessian = 0.0f;
  execution_times->calculate_optimization = 0.0f;
  for (int corr_iteration = 0;
       corr_iteration < refiner_ptr->n_corr_iterations(); ++corr_iteration) {
    // Start modalities
    begin_time = std::chrono::high_resolution_clock::now();
    refiner_ptr->StartModalities(corr_iteration);
    execution_times->start_modalities += ElapsedTime(begin_time);

    // Calculate correspondences
    begin_time = std::chrono::high_resolution_clock::now();
    refiner_ptr->CalculateCorrespondences(corr_iteration);
    execution_times->calculate_correspondences += ElapsedTime(begin_time);

    // Visualize correspondences
    int corr_save_idx = corr_iteration;
    refiner_ptr->VisualizeCorrespondences(corr_save_idx);

    for (int update_iteration = 0;
         update_iteration < refiner_ptr->n_update_iterations();
         ++update_iteration) {
      // Calculate gradient and hessian
      begin_time = std::chrono::high_resolution_clock::now();
      refiner_ptr->CalculateGradientAndHessian(corr_iteration,
                                               update_iteration);
      execution_times->calculate_gradient_and_hessian +=
          ElapsedTime(begin_time);

      // Calculate optimization
      begin_time = std::chrono::high_resolution_clock::now();
      refiner_ptr->CalculateOptimization(corr_iteration, update_iteration);
      execution_times->calculate_optimization += ElapsedTime(begin_time);

      // Visualize pose update
      int update_save_idx =
          corr_save_idx * tracker_ptr->n_update_iterations() + update_iteration;
      refiner_ptr->VisualizeOptimization(update_save_idx);
    }
  }

  // Update viewer
  if (visualize_tracking_ || save_images_)
    tracker_ptr->UpdateViewers(iteration);

  execution_times->calculate_results = 0.0f;
  execution_times->complete_cycle =
      execution_times->start_modalities +
      execution_times->calculate_correspondences +
      execution_times->calculate_gradient_and_hessian +
      execution_times->calculate_optimization;
}

void YCBEvaluator::ExecuteMeasuredTrackingCycle(
    const std::shared_ptr<icg::Tracker> &tracker_ptr, int load_index,
    int iteration, ExecutionTimes *execution_times) const {
  std::chrono::high_resolution_clock::time_point begin_time;

  // Update Cameras
  UpdateCameras(tracker_ptr, load_index);

  execution_times->start_modalities = 0.0f;
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
  if (visualize_tracking_ || save_images_)
    tracker_ptr->UpdateViewers(iteration);

  execution_times->complete_cycle =
      execution_times->calculate_correspondences +
      execution_times->calculate_gradient_and_hessian +
      execution_times->calculate_optimization +
      execution_times->calculate_results;
}

void YCBEvaluator::UpdateCameras(
    const std::shared_ptr<icg::Tracker> &tracker_ptr, int load_index) const {
  if (use_region_modality_) {
    auto color_camera{std::static_pointer_cast<icg::LoaderColorCamera>(
        tracker_ptr->main_camera_ptrs()[0])};
    color_camera->set_load_index(load_index);
    color_camera->SetUp();
  }
  if (!use_region_modality_ && use_depth_modality_) {
    auto depth_camera{std::static_pointer_cast<icg::LoaderDepthCamera>(
        tracker_ptr->main_camera_ptrs()[0])};
    depth_camera->set_load_index(load_index);
    depth_camera->SetUp();
  }
  if (use_region_modality_ &&
      (use_depth_modality_ || measure_occlusions_region_)) {
    auto depth_camera{std::static_pointer_cast<icg::LoaderDepthCamera>(
        tracker_ptr->main_camera_ptrs()[1])};
    depth_camera->set_load_index(load_index);
    depth_camera->SetUp();
  }
}

YCBEvaluator::Result YCBEvaluator::CalculateAverageBodyResult(
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

YCBEvaluator::Result YCBEvaluator::CalculateAverageSequenceResult(
    const std::vector<std::string> &sequence_names) const {
  size_t n_results = 0;
  std::vector<Result> result_sums;
  for (const auto &result : results_) {
    if (std::find(begin(sequence_names), end(sequence_names),
                  result.sequence_name) != end(sequence_names)) {
      n_results += result.frame_results.size();
      result_sums.push_back(SumResults(result.frame_results));
    }
  }
  return DivideResult(SumResults(result_sums), n_results);
}

YCBEvaluator::Result YCBEvaluator::CalculateAverageResult(
    const std::vector<Result> &results) {
  return DivideResult(SumResults(results), results.size());
}

YCBEvaluator::Result YCBEvaluator::SumResults(
    const std::vector<Result> &results) {
  Result sum;
  for (const auto &result : results) {
    sum.add_auc += result.add_auc;
    sum.adds_auc += result.adds_auc;
    std::transform(begin(result.add_curve), end(result.add_curve),
                   begin(sum.add_curve), begin(sum.add_curve),
                   [](float a, float b) { return a + b; });
    std::transform(begin(result.adds_curve), end(result.adds_curve),
                   begin(sum.adds_curve), begin(sum.adds_curve),
                   [](float a, float b) { return a + b; });
    sum.execution_times.start_modalities +=
        result.execution_times.start_modalities;
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

YCBEvaluator::Result YCBEvaluator::DivideResult(const Result &result,
                                                size_t n) {
  Result divided;
  divided.add_auc = result.add_auc / float(n);
  divided.adds_auc = result.adds_auc / float(n);
  std::transform(begin(result.add_curve), end(result.add_curve),
                 begin(divided.add_curve),
                 [&](float a) { return a / float(n); });
  std::transform(begin(result.adds_curve), end(result.adds_curve),
                 begin(divided.adds_curve),
                 [&](float a) { return a / float(n); });
  divided.execution_times.start_modalities =
      result.execution_times.start_modalities / float(n);
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

void YCBEvaluator::CalculatePoseResults(
    const std::shared_ptr<icg::Tracker> tracker_ptr,
    const std::string &body_name, const icg::Transform3fA &gt_body2world_pose,
    Result *result) const {
  const auto &vertices{body2reduced_vertice_map_.at(body_name)};
  const auto &kdtree_index{body2kdtree_ptr_map_.at(body_name)->index};

  // Calculate pose error
  icg::Transform3fA body2world_pose;
  for (const auto &body_ptr : tracker_ptr->body_ptrs()) {
    if (body_ptr->name() == body_name)
      body2world_pose = body_ptr->body2world_pose();
  }
  icg::Transform3fA delta_pose{body2world_pose.inverse() * gt_body2world_pose};

  // Calcualte add and adds error
  float add_error = 0.0f;
  float adds_error = 0.0f;
  size_t ret_index;
  float dist_sqrt;
  Eigen::Vector3f v;
  for (const auto &vertice : vertices) {
    v = delta_pose * vertice;
    add_error += (vertice - v).norm();
    kdtree_index->knnSearch(v.data(), 1, &ret_index, &dist_sqrt);
    adds_error += std::sqrt(dist_sqrt);
  }
  add_error /= vertices.size();
  adds_error /= vertices.size();

  // Calculate curve (tracking loss distribution)
  std::fill(begin(result->add_curve), end(result->add_curve), 1.0f);
  for (size_t i = 0; i < kNCurveValues; ++i) {
    if (add_error < thresholds_[i]) break;
    result->add_curve[i] = 0.0f;
  }
  std::fill(begin(result->adds_curve), end(result->adds_curve), 1.0f);
  for (size_t i = 0; i < kNCurveValues; ++i) {
    if (adds_error < thresholds_[i]) break;
    result->adds_curve[i] = 0.0f;
  }

  // Calculate area under curve
  result->add_auc = 1.0f - std::min(add_error / kThresholdMax, 1.0f);
  result->adds_auc = 1.0f - std::min(adds_error / kThresholdMax, 1.0f);
}

bool YCBEvaluator::LoadGTPoses(
    const std::string &sequence_name, const std::string &body_name,
    std::vector<icg::Transform3fA> *gt_body2world_poses) const {
  // Open poses file
  std::filesystem::path path{dataset_directory_ / "poses" /
                             (body_name + ".txt")};
  std::ifstream ifs{path.string(), std::ios::binary};
  if (!ifs.is_open() || ifs.fail()) {
    ifs.close();
    std::cerr << "Could not open file stream " << path.string() << std::endl;
    return false;
  }

  // Define begin index, n_frames and keyframes
  const auto idx_begin =
      body2sequence2pose_begin_.at(body_name).at(sequence_name);
  const int n_frames = sequence2nframes_map_.at(sequence_name);
  const auto keyframes = sequence2keyframes_map_.at(sequence_name);

  // Skip poses of other sequences
  std::string parsed;
  for (int idx = 0; idx < idx_begin; ++idx) {
    std::getline(ifs, parsed);
  }

  // Read poses
  int i_keyframe = 0;
  gt_body2world_poses->clear();
  for (int idx = 1; idx <= n_frames && i_keyframe < keyframes.size(); ++idx) {
    if (idx == keyframes[i_keyframe]) {
      Eigen::Quaternionf quaternion;
      Eigen::Translation3f translation;
      std::getline(ifs, parsed, ' ');
      quaternion.w() = stof(parsed);
      std::getline(ifs, parsed, ' ');
      quaternion.x() = stof(parsed);
      std::getline(ifs, parsed, ' ');
      quaternion.y() = stof(parsed);
      std::getline(ifs, parsed, ' ');
      quaternion.z() = stof(parsed);
      std::getline(ifs, parsed, ' ');
      translation.x() = stof(parsed);
      std::getline(ifs, parsed, ' ');
      translation.y() = stof(parsed);
      std::getline(ifs, parsed);
      translation.z() = stof(parsed);
      quaternion.normalize();
      icg::Transform3fA pose{translation * quaternion};
      gt_body2world_poses->push_back(std::move(pose));
      i_keyframe++;
    } else {
      std::getline(ifs, parsed);
    }
  }
  return true;
}

bool YCBEvaluator::LoadMatlabGTPoses(
    const std::string &sequence_name, const std::string &body_name,
    std::vector<icg::Transform3fA> *gt_body2world_poses) const {
  // Open poses file
  std::filesystem::path path{external_directory_ / "poses" / "ground_truth" /
                             (sequence_name + "_" + body_name + ".txt")};
  std::ifstream ifs{path.string(), std::ios::binary};
  if (!ifs.is_open() || ifs.fail()) {
    ifs.close();
    std::cerr << "Could not open file stream " << path.string() << std::endl;
    return false;
  }

  // Read poses
  gt_body2world_poses->clear();
  std::string parsed;
  while (std::getline(ifs, parsed)) {
    std::stringstream ss{parsed};
    Eigen::Quaternionf quaternion;
    Eigen::Translation3f translation;
    std::getline(ss, parsed, ' ');
    quaternion.w() = stof(parsed);
    std::getline(ss, parsed, ' ');
    quaternion.x() = stof(parsed);
    std::getline(ss, parsed, ' ');
    quaternion.y() = stof(parsed);
    std::getline(ss, parsed, ' ');
    quaternion.z() = stof(parsed);
    std::getline(ss, parsed, ' ');
    translation.x() = stof(parsed);
    std::getline(ss, parsed, ' ');
    translation.y() = stof(parsed);
    std::getline(ss, parsed);
    translation.z() = stof(parsed);
    quaternion.normalize();
    icg::Transform3fA pose{translation * quaternion};
    gt_body2world_poses->push_back(std::move(pose));
  }
  return true;
}

bool YCBEvaluator::LoadDetectorPoses(
    const std::string &sequence_name, const std::string &body_name,
    std::vector<icg::Transform3fA> *detector_body2world_poses,
    std::vector<bool> *body_detected) const {
  // Open detector poses file
  std::filesystem::path path{external_directory_ / "poses" / detector_folder_ /
                             (sequence_name + "_" + body_name + ".txt")};
  std::ifstream ifs{path.string(), std::ios::binary};
  if (!ifs.is_open() || ifs.fail()) {
    ifs.close();
    std::cerr << "Could not open file stream " << path.string() << std::endl;
    return false;
  }

  // Read poses
  body_detected->clear();
  detector_body2world_poses->clear();
  std::string parsed;
  while (std::getline(ifs, parsed)) {
    std::stringstream ss{parsed};
    Eigen::Quaternionf quaternion;
    Eigen::Translation3f translation;
    std::getline(ss, parsed, ' ');
    quaternion.w() = stof(parsed);
    std::getline(ss, parsed, ' ');
    quaternion.x() = stof(parsed);
    std::getline(ss, parsed, ' ');
    quaternion.y() = stof(parsed);
    std::getline(ss, parsed, ' ');
    quaternion.z() = stof(parsed);
    std::getline(ss, parsed, ' ');
    translation.x() = stof(parsed);
    std::getline(ss, parsed, ' ');
    translation.y() = stof(parsed);
    std::getline(ss, parsed);
    translation.z() = stof(parsed);
    if (translation.vector().isZero()) {
      detector_body2world_poses->push_back(icg::Transform3fA::Identity());
      body_detected->push_back(false);
    } else {
      quaternion.normalize();
      icg::Transform3fA pose{translation * quaternion};
      detector_body2world_poses->push_back(std::move(pose));
      body_detected->push_back(true);
    }
  }
  return true;
}

void YCBEvaluator::VisualizeResult(const Result &result,
                                   const std::string &title) {
  std::cout << title << ": ";
  if (result.frame_index) std::cout << "frame " << result.frame_index << ": ";
  std::cout << "execution_time = " << result.execution_times.complete_cycle
            << " us, "
            << "add auc = " << result.add_auc << ", "
            << "adds auc = " << result.adds_auc << std::endl;
}

bool YCBEvaluator::CreateRunConfigurations() {
  run_configurations_.clear();
  if (run_sequentially_ && !model_occlusions_region_ &&
      !model_occlusions_depth_) {
    for (int sequence_id : sequence_ids_) {
      std::string sequence_name{SequenceIDToName(sequence_id)};
      for (const auto &body_name : evaluated_body_names_) {
        if (BodyExistsInSequence(sequence_name, body_name)) {
          RunConfiguration run_configuration;
          run_configuration.sequence_id = sequence_id;
          run_configuration.sequence_name = sequence_name;
          run_configuration.evaluated_body_names.push_back(body_name);
          run_configuration.tracked_body_names.push_back(body_name);
          run_configurations_.push_back(std::move(run_configuration));
        }
      }
    }
  } else {
    for (int sequence_id : sequence_ids_) {
      RunConfiguration run_configuration;
      run_configuration.sequence_id = sequence_id;
      run_configuration.sequence_name = SequenceIDToName(sequence_id);
      for (const auto &body_name : evaluated_body_names_) {
        if (BodyExistsInSequence(run_configuration.sequence_name, body_name))
          run_configuration.evaluated_body_names.push_back(body_name);
      }
      if (run_configuration.evaluated_body_names.empty()) continue;
      if (model_occlusions_region_ || model_occlusions_depth_) {
        if (!SequenceBodyNames(run_configuration.sequence_name,
                               &run_configuration.tracked_body_names))
          return false;
      } else {
        run_configuration.tracked_body_names =
            run_configuration.evaluated_body_names;
      }
      run_configurations_.push_back(std::move(run_configuration));
    }
  }
  return true;
}

void YCBEvaluator::AssambleTrackedBodyNames() {
  tracked_body_names_.clear();
  for (const auto &run_configuration : run_configurations_) {
    for (const auto &tracked_body_name : run_configuration.tracked_body_names) {
      if (std::find(begin(tracked_body_names_), end(tracked_body_names_),
                    tracked_body_name) == end(tracked_body_names_))
        tracked_body_names_.push_back(tracked_body_name);
    }
  }
}

bool YCBEvaluator::LoadBodies() {
  bool success = true;
  body2body_ptr_map_.clear();
#pragma omp parallel for
  for (int i = 0; i < tracked_body_names_.size(); ++i) {
    auto &body_name{tracked_body_names_[i]};
    std::filesystem::path directory{dataset_directory_ / "models" / body_name};
    auto body_ptr{
        std::make_shared<icg::Body>(body_name, directory / "textured.obj", 1.0f,
                                    true, true, icg::Transform3fA::Identity())};
    if (!body_ptr->SetUp()) {
      success = false;
      continue;
    }
#pragma omp critical
    body2body_ptr_map_.insert({body_name, std::move(body_ptr)});
  }
  return success;
}

bool YCBEvaluator::GenerateModels() {
  std::filesystem::path directory{external_directory_ / "models"};
  std::filesystem::create_directory(directory);
  for (auto body_name : tracked_body_names_) {
    if (use_region_modality_) {
      auto model_ptr{std::make_shared<icg::RegionModel>(
          body_name + "_region_model", body2body_ptr_map_[body_name],
          directory / (body_name + "_region_model.bin"), 0.8f, 4, 500, 0.05f,
          0.002f, false, 2000)};
      region_model_setter_(model_ptr);
      if (!model_ptr->SetUp()) return false;
      body2region_model_ptr_map_.insert({body_name, std::move(model_ptr)});
    }
    if (use_depth_modality_) {
      auto model_ptr{std::make_shared<icg::DepthModel>(
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

bool YCBEvaluator::LoadKeyframes() {
  bool success = true;
  sequence2keyframes_map_.clear();
#pragma omp parallel for
  for (int i = 0; i < sequence_ids_.size(); ++i) {
    std::string sequence_name{SequenceIDToName(sequence_ids_[i])};

    // Open poses file
    std::filesystem::path path{dataset_directory_ / "image_sets" /
                               "keyframe.txt"};
    std::ifstream ifs{path.string(), std::ios::binary};
    if (!ifs.is_open() || ifs.fail()) {
      ifs.close();
      std::cerr << "Could not open file stream " << path.string() << std::endl;
      success = false;
      continue;
    }

    std::vector<int> keyframes;
    std::string parsed;
    while (std::getline(ifs, parsed, '/')) {
      if (parsed == sequence_name) {
        std::getline(ifs, parsed);
        keyframes.push_back(stoi(parsed));
      } else {
        std::getline(ifs, parsed);
      }
    }
#pragma omp critical
    sequence2keyframes_map_.insert({sequence_name, keyframes});
  }
  return success;
}

void YCBEvaluator::LoadNFrames() {
  sequence2nframes_map_.clear();
  int max_sequence_id =
      *std::max_element(begin(sequence_ids_), end(sequence_ids_));
#pragma omp parallel for
  for (int sequence_id = 0; sequence_id <= max_sequence_id; ++sequence_id) {
    std::string sequence_name{SequenceIDToName(sequence_id)};
    int nframes = NFramesInSequence(sequence_name);
#pragma omp critical
    sequence2nframes_map_.insert({sequence_name, nframes});
  }
}

void YCBEvaluator::LoadPoseBegin() {
  body2sequence2pose_begin_.clear();
#pragma omp parallel for
  for (int i = 0; i < tracked_body_names_.size(); ++i) {
    auto &body_name{tracked_body_names_[i]};
    std::map<std::string, int> sequence2pose_begin;
    for (int max_sequence_id : sequence_ids_) {
      int idx_begin = 0;
      for (int sequence_id = 0; sequence_id < max_sequence_id; ++sequence_id) {
        std::string sequence_name{SequenceIDToName(sequence_id)};
        if (BodyExistsInSequence(sequence_name, body_name))
          idx_begin += sequence2nframes_map_[sequence_name];
      }
      std::string max_sequence_name{SequenceIDToName(max_sequence_id)};
      sequence2pose_begin.insert({max_sequence_name, idx_begin});
    }
#pragma omp critical
    body2sequence2pose_begin_.insert({body_name, sequence2pose_begin});
  }
}

void YCBEvaluator::GenderateReducedVertices() {
#pragma omp parallel for
  for (int i = 0; i < tracked_body_names_.size(); ++i) {
    auto &body_name{tracked_body_names_[i]};
    const auto &vertices{body2body_ptr_map_[body_name]->vertices()};
    if (n_vertices_evaluation_ <= 0 ||
        n_vertices_evaluation_ >= vertices.size()) {
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

void YCBEvaluator::GenerateKDTrees() {
#pragma omp parallel for
  for (int i = 0; i < tracked_body_names_.size(); ++i) {
    auto &body_name{tracked_body_names_[i]};
    const auto &vertices{body2body_ptr_map_[body_name]->vertices()};
    auto kdtree_ptr{std::make_unique<KDTreeVector3f>(3, vertices, 10)};
    kdtree_ptr->index->buildIndex();
#pragma omp critical
    body2kdtree_ptr_map_.insert({body_name, std::move(kdtree_ptr)});
  }
}

bool YCBEvaluator::BodyExistsInSequence(const std::string &sequence_name,
                                        const std::string &body_name) const {
  std::filesystem::path path{dataset_directory_ / "data" / sequence_name /
                             "000001-box.txt"};
  std::ifstream ifs{path.string(), std::ios::binary};
  if (!ifs.is_open() || ifs.fail()) {
    ifs.close();
    std::cerr << "Could not open file stream " << path.string() << std::endl;
    return false;
  }

  std::string parsed;
  while (std::getline(ifs, parsed, ' ')) {
    if (parsed == body_name) return true;
    std::getline(ifs, parsed);
  }
  return false;
}

bool YCBEvaluator::SequenceBodyNames(
    const std::string &sequence_name,
    std::vector<std::string> *sequence_body_names) const {
  std::filesystem::path path{dataset_directory_ / "data" / sequence_name /
                             "000001-box.txt"};
  std::ifstream ifs{path.string(), std::ios::binary};
  if (!ifs.is_open() || ifs.fail()) {
    ifs.close();
    std::cerr << "Could not open file stream " << path.string() << std::endl;
    return false;
  }

  sequence_body_names->clear();
  std::string parsed;
  while (std::getline(ifs, parsed, ' ')) {
    sequence_body_names->push_back(parsed);
    std::getline(ifs, parsed);
  }
  return true;
}

int YCBEvaluator::NFramesInSequence(const std::string &sequence_name) const {
  std::filesystem::path directory{dataset_directory_ / "data" / sequence_name};
  for (int i = 1; true; ++i) {
    int n_zeros = 6 - int(std::to_string(i).length());
    std::filesystem::path path{directory / (std::string(n_zeros, '0') +
                                            std::to_string(i) + "-box.txt")};
    if (!std::filesystem::exists(path)) return i - 1;
  }
}

std::string YCBEvaluator::SequenceIDToName(int sequence_id) const {
  int n_zeros = 4 - int(std::to_string(sequence_id).length());
  return std::string(n_zeros, '0') + std::to_string(sequence_id);
}

float YCBEvaluator::ElapsedTime(
    const std::chrono::high_resolution_clock::time_point &begin_time) {
  auto end_time{std::chrono::high_resolution_clock::now()};
  return float(std::chrono::duration_cast<std::chrono::microseconds>(end_time -
                                                                     begin_time)
                   .count());
}
