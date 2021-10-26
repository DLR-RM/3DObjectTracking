// SPDX-License-Identifier: MIT
// Copyright (c) 2021 Manuel Stoiber, German Aerospace Center (DLR)

#include "opt_evaluator.h"

#include <tiny_obj_loader/tiny_obj_loader.h>

OPTEvaluator::OPTEvaluator(const std::string &name,
                           const std::filesystem::path &dataset_directory,
                           const std::vector<std::string> &body_names,
                           const std::vector<std::string> &body_orientations,
                           const std::vector<std::string> &motion_patterns)
    : name_{name},
      dataset_directory_{dataset_directory},
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

  // Create run configurations
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

  // Generate models, load vertices, and calculate diamters for used bodies
  if (!GenerateModels()) return false;
  if (!LoadVertices()) return false;
  if (calculate_diameters_) CalculateDiameters();

  set_up_ = true;
  return true;
}

void OPTEvaluator::set_visualize_tracking(bool visualize_tracking) {
  visualize_tracking_ = visualize_tracking;
}

void OPTEvaluator::set_visualize_frame_results(bool visualize_frame_results) {
  visualize_frame_results_ = visualize_frame_results;
}

void OPTEvaluator::set_calculate_diameters(bool calculate_diameters) {
  calculate_diameters_ = calculate_diameters;
}

void OPTEvaluator::SaveResults(std::filesystem::path save_directory) {
  save_directory_ = save_directory;
  save_results_ = true;
}

void OPTEvaluator::DoNotSaveResults() { save_results_ = false; }

void OPTEvaluator::set_tracker_setter(
    const std::function<void(std::shared_ptr<srt3d::Tracker>)>
        &tracker_setter) {
  tracker_setter_ = tracker_setter;
}

void OPTEvaluator::set_region_modality_setter(
    const std::function<void(std::shared_ptr<srt3d::RegionModality>)>
        &region_modality_setter) {
  region_modality_setter_ = region_modality_setter;
}

void OPTEvaluator::set_model_setter(
    const std::function<void(std::shared_ptr<srt3d::Model>)> &model_setter) {
  model_setter_ = model_setter;
  set_up_ = false;
}

bool OPTEvaluator::Evaluate() {
  if (!set_up_) {
    std::cerr << "Set up evaluator " << name_ << " first" << std::endl;
    return false;
  }

  // Evaluate all run configuration
  results_.resize(run_configurations_.size());
  if (visualize_tracking_ || visualize_frame_results_) {
    auto renderer_geometry_ptr{std::make_shared<srt3d::RendererGeometry>("rg")};
    renderer_geometry_ptr->SetUp();
    for (size_t i = 0; i < int(run_configurations_.size()); ++i) {
      EvaluateRunConfiguration(run_configurations_[i], renderer_geometry_ptr,
                               &results_[i]);
      DataResult average_result;
      CalculateAverageResult({results_[i]}, &average_result);
      VisualizeResult(average_result, run_configurations_[i].sequence_name);
      if (save_results_)
        SaveResults(results_[i], run_configurations_[i].sequence_name);
    }
  } else {
    std::vector<std::shared_ptr<srt3d::RendererGeometry>>
        renderer_geometry_ptrs(omp_get_max_threads());
    for (auto &renderer_geometry_ptr : renderer_geometry_ptrs) {
      renderer_geometry_ptr = std::make_shared<srt3d::RendererGeometry>("rg");
      renderer_geometry_ptr->SetUp();
    }
#pragma omp parallel for schedule(dynamic, 1)
    for (int i = 0; i < int(run_configurations_.size()); ++i) {
      EvaluateRunConfiguration(run_configurations_[i],
                               renderer_geometry_ptrs[omp_get_thread_num()],
                               &results_[i]);
      DataResult average_result;
      CalculateAverageResult({results_[i]}, &average_result);
#pragma omp critical
      VisualizeResult(average_result, run_configurations_[i].sequence_name);
      if (save_results_)
        SaveResults(results_[i], run_configurations_[i].sequence_name);
    }
  }

  // Calculate final results
  CalculateAverageResult(results_, &final_result_);
  VisualizeResult(final_result_, "all_sequences");
  return true;
}

float OPTEvaluator::area_under_curve() {
  return final_result_.area_under_curve;
}

void OPTEvaluator::EvaluateRunConfiguration(
    RunConfiguration run_configuration,
    std::shared_ptr<srt3d::RendererGeometry> renderer_geometry_ptr,
    std::vector<DataResult> *results) {
  // Read ground truth poses
  std::vector<srt3d::Transform3fA> gt_body2world_pose;
  GetGTPosesOPTDataset(run_configuration, &gt_body2world_pose);

  // Initialize tracker
  auto tracker_ptr{std::make_shared<srt3d::Tracker>("tracker")};
  SetUpTracker(run_configuration, renderer_geometry_ptr, tracker_ptr);
  tracker_ptr->region_modality_ptrs()[0]->body_ptr()->set_body2world_pose(
      gt_body2world_pose[0]);
  tracker_ptr->region_modality_ptrs()[0]->StartModality();

  // Iterate over all frames
  results->resize(gt_body2world_pose.size() - 1);
  for (int i = 0; i < gt_body2world_pose.size() - 1; ++i) {
    (*results)[i].frame_index = i;
    ExecuteMeasuredTrackingCycle(tracker_ptr, i,
                                 &(*results)[i].execution_times);

    // Calculate results for main body
    CalculatePoseResults(
        run_configuration.body_name,
        tracker_ptr->region_modality_ptrs()[0]->body_ptr()->body2world_pose(),
        gt_body2world_pose[i + 1], &(*results)[i]);
    if (visualize_frame_results_)
      VisualizeResult((*results)[i], run_configuration.sequence_name);
  }
}

void OPTEvaluator::SetUpTracker(
    const RunConfiguration &run_configuration,
    std::shared_ptr<srt3d::RendererGeometry> renderer_geometry_ptr,
    std::shared_ptr<srt3d::Tracker> tracker_ptr) {
  renderer_geometry_ptr->ClearBodies();

  // Init camera
  std::filesystem::path camera_directory{
      dataset_directory_ / "3D" / run_configuration.sequence_name / "color"};
  auto camera_ptr{std::make_shared<srt3d::LoaderCamera>(
      "camera", camera_directory, kOPTIntrinsics, "", 1, 4)};

  // Init visualizer
  if (visualize_tracking_) {
    auto viewer_ptr{std::make_shared<srt3d::NormalViewer>(
        "viewer", camera_ptr, renderer_geometry_ptr)};
    tracker_ptr->AddViewer(viewer_ptr);
  }

  // Init body
  std::filesystem::path directory{dataset_directory_ / "Model3D" /
                                  run_configuration.body_name};
  std::filesystem::path geometry_path{directory /
                                      (run_configuration.body_name + ".obj")};
  auto body_ptr{std::make_shared<srt3d::Body>(
      run_configuration.body_name, geometry_path, 1.0f, true, true, 0.15f,
      kBody2Geometry2BodyPoseMap.at(run_configuration.body_name))};
  renderer_geometry_ptr->AddBody(body_ptr);

  // Init model
  auto model_ptr{std::make_shared<srt3d::Model>(
      "model", body_ptr, directory, run_configuration.body_name + "_model.bin",
      0.8, 4, 200, false, 2000)};
  model_setter_(model_ptr);

  // Init region modality
  auto region_modality_ptr{std::make_shared<srt3d::RegionModality>(
      "region_modality", body_ptr, model_ptr, camera_ptr)};
  region_modality_setter_(region_modality_ptr);
  tracker_ptr->AddRegionModality(region_modality_ptr);

  // Init tracker
  tracker_setter_(tracker_ptr);
  tracker_ptr->SetUpTracker();
}

void OPTEvaluator::ExecuteMeasuredTrackingCycle(
    std::shared_ptr<srt3d::Tracker> tracker_ptr, int iteration,
    DataExecutionTimes *execution_times) {
  // Calculate before camera update and camera update
  auto begin_time{std::chrono::high_resolution_clock::now()};
  tracker_ptr->CalculateBeforeCameraUpdate();
  execution_times->calculate_before_camera_update = ElapsedTime(begin_time);
  tracker_ptr->UpdateCameras();

  execution_times->calculate_correspondences = 0.0f;
  execution_times->calculate_pose_update = 0.0f;
  for (int corr_iteration = 0;
       corr_iteration < tracker_ptr->n_corr_iterations(); ++corr_iteration) {
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
  if (visualize_tracking_) tracker_ptr->UpdateViewers(iteration);

  execution_times->complete_cycle =
      execution_times->calculate_before_camera_update +
      execution_times->calculate_correspondences +
      execution_times->calculate_pose_update;
}

void OPTEvaluator::CalculatePoseResults(
    const std::string &body_name, const srt3d::Transform3fA &body2world_pose,
    const srt3d::Transform3fA &gt_body2world_pose, DataResult *result) const {
  float error = 0.0f;
  srt3d::Transform3fA delta_pose{
      (body2world_pose * kBody2Geometry2BodyPoseMap.at(body_name)).inverse() *
      gt_body2world_pose * kBody2Geometry2BodyPoseMap.at(body_name)};
  const auto &vertices{body2vertice_map_.at(body_name)};
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

  result->area_under_curve = std::accumulate(begin(result->curve_values),
                                             end(result->curve_values), 0.0f);
  result->area_under_curve *=
      (100.0f * kThresholdMax / result->curve_values.size());
}

void OPTEvaluator::CalculateAverageResult(
    const std::vector<std::vector<DataResult>> &results,
    DataResult *average_result) {
  average_result->area_under_curve = 0.0f;
  std::fill(begin(average_result->curve_values),
            end(average_result->curve_values), 0.0f);
  average_result->execution_times.calculate_before_camera_update = 0.0;
  average_result->execution_times.calculate_correspondences = 0.0f;
  average_result->execution_times.calculate_pose_update = 0.0f;
  average_result->execution_times.complete_cycle = 0.0f;

  int n = 0;
  for (auto &sequence_results : results) {
    for (auto &result : sequence_results) {
      average_result->area_under_curve += result.area_under_curve;
      std::transform(begin(result.curve_values), end(result.curve_values),
                     begin(average_result->curve_values),
                     begin(average_result->curve_values),
                     [](float a, float b) { return a + b; });
      average_result->execution_times.calculate_before_camera_update +=
          result.execution_times.calculate_before_camera_update;
      average_result->execution_times.calculate_correspondences +=
          result.execution_times.calculate_correspondences;
      average_result->execution_times.calculate_pose_update +=
          result.execution_times.calculate_pose_update;
      average_result->execution_times.complete_cycle +=
          result.execution_times.complete_cycle;
      n++;
    }
  }

  average_result->frame_index = 0;
  average_result->area_under_curve /= float(n);
  std::transform(begin(average_result->curve_values),
                 end(average_result->curve_values),
                 begin(average_result->curve_values),
                 [&](float a) { return a / float(n); });
  average_result->execution_times.calculate_before_camera_update /= float(n);
  average_result->execution_times.calculate_correspondences /= float(n);
  average_result->execution_times.calculate_pose_update /= float(n);
  average_result->execution_times.complete_cycle /= float(n);
}

void OPTEvaluator::VisualizeResult(const DataResult &result,
                                   const std::string &title) {
  std::cout << title << ": "
            << "frame " << result.frame_index << ": "
            << "execution_time = " << result.execution_times.complete_cycle
            << " us, "
            << "area_under_curve = " << result.area_under_curve << std::endl;
}

void OPTEvaluator::SaveResults(const std::vector<DataResult> &results,
                               const std::string &title) const {
  std::ofstream ofs{save_directory_ / ("results_" + title + ".txt")};
  for (const auto &result : results) {
    ofs << result.area_under_curve << ","
        << result.execution_times.complete_cycle << ","
        << result.execution_times.calculate_before_camera_update << ","
        << result.execution_times.calculate_correspondences << ","
        << result.execution_times.calculate_pose_update << std::endl;
  }
  ofs.flush();
  ofs.close();
}

bool OPTEvaluator::GenerateModels() {
  for (auto body_name : body_names_) {
    std::filesystem::path directory{dataset_directory_ / "Model3D" / body_name};
    auto body_ptr{std::make_shared<srt3d::Body>(
        body_name, directory / (body_name + ".obj"), 1.0f, true, true, 0.15f,
        kBody2Geometry2BodyPoseMap.at(body_name))};
    auto model_ptr{std::make_shared<srt3d::Model>("model", body_ptr, directory,
                                                  body_name + "_model.bin",
                                                  0.8f, 4, 200, false, 2000)};
    model_setter_(model_ptr);
    if (!model_ptr->SetUp()) return false;
  }
  return true;
}

bool OPTEvaluator::LoadVertices() {
  tinyobj::attrib_t attributes;
  std::vector<tinyobj::shape_t> shapes;
  std::vector<tinyobj::material_t> materials;
  std::string warning;
  std::string error;
  body2vertice_map_.clear();
  for (const auto &body_name : body_names_) {
    std::filesystem::path path{dataset_directory_ / "Model3D" / body_name /
                               (body_name + ".obj")};
    if (!tinyobj::LoadObj(&attributes, &shapes, &materials, &warning, &error,
                          path.string().c_str(), nullptr, true, false)) {
      std::cerr << "TinyObjLoader failed to load data from " << path
                << std::endl;
      return false;
    }
    if (!error.empty()) std::cerr << error << std::endl;

    std::vector<Eigen::Vector3f> vertices_vector(attributes.vertices.size() /
                                                 3);
    memcpy(vertices_vector.data(), attributes.vertices.data(),
           sizeof(float) * attributes.vertices.size());
    body2vertice_map_.insert({body_name, std::move(vertices_vector)});
  }
  return true;
}

void OPTEvaluator::CalculateDiameters() {
  for (const auto &body_name : body_names_) {
    float d_max = 0.0f;
    auto &vertices{body2vertice_map_.at(body_name)};
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
    std::vector<srt3d::Transform3fA> *gt_body2world_poses) const {
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
    srt3d::Transform3fA pose{srt3d::Transform3fA::Identity()};
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
