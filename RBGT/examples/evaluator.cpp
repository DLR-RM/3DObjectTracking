// SPDX-License-Identifier: MIT
// Copyright (c) 2020 Manuel Stoiber, German Aerospace Center (DLR)

#include "evaluator.h"

bool Evaluator::Init(const std::filesystem::path &dataset_path,
                     const std::vector<std::string> &body_names,
                     const std::string &sequence_name, bool use_occlusions) {
  initialized_ = false;
  dataset_path_ = dataset_path;
  body_names_ = body_names;
  sequence_name_ = sequence_name;
  use_occlusions_ = use_occlusions;

  if (!ReadPosesRBOTDataset(dataset_path_ / "poses_first.txt", &poses_first_))
    return false;
  if (!ReadPosesRBOTDataset(dataset_path_ / "poses_second.txt", &poses_second_))
    return false;

  InitTracker();

  results_.resize(body_names_.size());
  for (auto &result : results_) result.resize(kNFrames_ - 1);
  initialized_ = true;
  return true;
}

void Evaluator::set_translation_error_threshold(
    float translation_error_threshold) {
  translation_error_threshold_ = translation_error_threshold;
}

void Evaluator::set_rotation_error_threshold(float rotation_error_threshold) {
  rotation_error_threshold_ = rotation_error_threshold;
}

void Evaluator::set_visualize_all_results(bool visualize_all_results) {
  visualize_all_results_ = visualize_all_results;
}

void Evaluator::set_sphere_radius(float sphere_radius) {
  sphere_radius_ = sphere_radius;
}

void Evaluator::set_n_divides(int n_divides) { n_divides_ = n_divides; }

void Evaluator::set_n_points(int n_points) { n_points_ = n_points; }

bool Evaluator::Evaluate() {
  if (!initialized_) {
    std::cerr << "Evaluator is not initialized" << std::endl;
    return false;
  }

  // Iterate over all bodies
  for (int i_body = 0; i_body < body_names_.size(); ++i_body) {
    auto body_name{body_names_[i_body]};
    InitBodies(body_name);
    ResetBody(0);
    if (use_occlusions_) ResetOcclusionBody(0);

    // Iterate over all frames
    for (int i_frame = 0; i_frame < kNFrames_ - 1; ++i_frame) {
      auto &result{results_[i_body][i_frame]};
      result.frame_index = i_frame;
      ExecuteMeasuredTrackingCycle(i_frame, &result.execution_times);

      // Calculate results for main body
      CalculatePoseResults(body_ptr_->body2world_pose(),
                           poses_first_[i_frame + 1], &result);
      if (visualize_all_results_) VisualizeResult(result, body_name);
      if (result.tracking_loss) ResetBody(i_frame + 1);

      // Calculate results for occluding body
      if (use_occlusions_) {
        DataResult occlusion_result;
        CalculatePoseResults(occlusion_body_ptr_->body2world_pose(),
                             poses_second_[i_frame + 1], &occlusion_result);
        if (occlusion_result.tracking_loss) ResetOcclusionBody(i_frame + 1);
      }
    }
    VisualizeAverageResults({results_[i_body]}, body_name);
  }
  VisualizeAverageResults(results_, "all bodies");
  return true;
}

void Evaluator::SaveResults(const std::filesystem::path &path) {
  for (int i_body = 0; i_body < body_names_.size(); ++i_body) {
    std::ofstream ofs{path / ("results_" + body_names_[i_body] + ".txt")};
    for (auto &result : results_[i_body]) {
      ofs << result.frame_index << "," << result.rotation_error << ","
          << result.translation_error << "," << result.tracking_loss << ","
          << result.execution_times.complete_cycle << ","
          << result.execution_times.calculate_before_camera_update << ","
          << result.execution_times.start_occlusion_mask_rendering << ","
          << result.execution_times.calculate_correspondences << ","
          << result.execution_times.calculate_pose_update << std::endl;
    }
    ofs.flush();
    ofs.close();
  }
}

std::shared_ptr<rbgt::Tracker> Evaluator::tracker_ptr() const {
  return tracker_ptr_;
}

std::shared_ptr<rbgt::RegionModality> Evaluator::region_modality_ptr() const {
  return region_modality_ptr_;
}

std::shared_ptr<rbgt::RegionModality> Evaluator::occlusion_region_modality_ptr()
    const {
  return occlusion_region_modality_ptr_;
}

std::shared_ptr<rbgt::OcclusionMaskRenderer>
Evaluator::occlusion_mask_renderer_ptr() const {
  return occlusion_mask_renderer_ptr_;
}

std::shared_ptr<rbgt::NormalImageViewer> Evaluator::viewer_ptr() const {
  return viewer_ptr_;
}

void Evaluator::InitTracker() {
  tracker_ptr_ = std::make_shared<rbgt::Tracker>();

  renderer_geometry_ptr_ = std::make_shared<rbgt::RendererGeometry>();
  camera_ptr_ = std::make_shared<rbgt::DatasetRBOTCamera>();
  camera_ptr_->Init("camera", dataset_path_, body_names_[0], sequence_name_, 0);
  viewer_ptr_ = std::make_shared<rbgt::NormalImageViewer>();
  viewer_ptr_->Init("viewer", renderer_geometry_ptr_, camera_ptr_);
  tracker_ptr_->AddViewer(viewer_ptr_);

  body_ptr_ = std::make_shared<rbgt::Body>("body", "path_placeholder", 0.001f,
                                           true, false, 0.3f);
  body_ptr_->set_occlusion_mask_id(7);
  model_ptr_ = std::make_shared<rbgt::Model>("model");
  model_ptr_->set_use_random_seed(false);
  region_modality_ptr_ = std::make_shared<rbgt::RegionModality>();
  region_modality_ptr_->Init("region_modality", body_ptr_, model_ptr_,
                             camera_ptr_);
  tracker_ptr_->AddRegionModality(region_modality_ptr_);

  if (use_occlusions_) {
    occlusion_body_ptr_ = std::make_shared<rbgt::Body>(
        "squirrel_small", dataset_path_ / "squirrel_small.obj", 0.001f, true,
        false, 0.3f);
    occlusion_body_ptr_->set_occlusion_mask_id(1);
    occlusion_model_ptr_ = std::make_shared<rbgt::Model>("occlusion_model");
    occlusion_model_ptr_->set_use_random_seed(false);
    occlusion_region_modality_ptr_ = std::make_shared<rbgt::RegionModality>();
    occlusion_region_modality_ptr_->Init("occlusion_region_modality",
                                         occlusion_body_ptr_,
                                         occlusion_model_ptr_, camera_ptr_);
    tracker_ptr_->AddRegionModality(occlusion_region_modality_ptr_);

    occlusion_mask_renderer_ptr_ =
        std::make_shared<rbgt::OcclusionMaskRenderer>();
    occlusion_mask_renderer_ptr_->InitFromCamera(
        "occlusion_mask_renderer", renderer_geometry_ptr_, *camera_ptr_);
    region_modality_ptr_->UseOcclusionHandling(occlusion_mask_renderer_ptr_);
    occlusion_region_modality_ptr_->UseOcclusionHandling(
        occlusion_mask_renderer_ptr_);
  }
  tracker_ptr_->SetUpObjects();
}

void Evaluator::InitBodies(const std::string &body_name) {
  camera_ptr_->Init("RBOT camera", dataset_path_, body_name, sequence_name_, 0);

  body_ptr_->set_name(body_name);
  body_ptr_->set_geometry_path(dataset_path_ / body_name /
                               (body_name + ".obj"));

  std::filesystem::path model_directory{dataset_path_ / body_name};
  std::string model_name{body_name + "_model"};
  if (!model_ptr_->LoadModel(model_directory, model_name)) {
    model_ptr_->GenerateModel(*body_ptr_, sphere_radius_, n_divides_,
                              n_points_);
    model_ptr_->SaveModel(model_directory, model_name);
  }

  renderer_geometry_ptr_->ClearBodies();
  renderer_geometry_ptr_->AddBody(body_ptr_);

  if (use_occlusions_) {
    std::string occlusion_model_name{"squirrel_small_model"};
    if (!occlusion_model_ptr_->LoadModel(dataset_path_, occlusion_model_name)) {
      occlusion_model_ptr_->GenerateModel(*occlusion_body_ptr_, sphere_radius_,
                                          n_divides_, n_points_);
      occlusion_model_ptr_->SaveModel(dataset_path_, occlusion_model_name);
    }

    renderer_geometry_ptr_->AddBody(occlusion_body_ptr_);
  }
}

void Evaluator::ResetBody(int i_frame) {
  body_ptr_->set_body2world_pose(poses_first_[i_frame]);
  region_modality_ptr_->StartModality();
}

void Evaluator::ResetOcclusionBody(int i_frame) {
  occlusion_body_ptr_->set_body2world_pose(poses_second_[i_frame]);
  occlusion_region_modality_ptr_->StartModality();
}

void Evaluator::ExecuteMeasuredTrackingCycle(
    int iteration, DataExecutionTimes *execution_times) {
  // Calculate before camera update and camera update
  auto begin_time{std::chrono::high_resolution_clock::now()};
  tracker_ptr_->CalculateBeforeCameraUpdate();
  execution_times->calculate_before_camera_update = ElapsedTime(begin_time);
  tracker_ptr_->UpdateCameras();

  execution_times->start_occlusion_mask_rendering = 0.0f;
  execution_times->calculate_correspondences = 0.0f;
  execution_times->calculate_pose_update = 0.0f;
  for (int corr_iteration = 0;
       corr_iteration < tracker_ptr_->n_corr_iterations(); ++corr_iteration) {
    // Start occlusion mask rendering
    begin_time = std::chrono::high_resolution_clock::now();
    tracker_ptr_->StartOcclusionMaskRendering();
    execution_times->start_occlusion_mask_rendering += ElapsedTime(begin_time);

    // Calculate correspondences
    begin_time = std::chrono::high_resolution_clock::now();
    tracker_ptr_->CalculateCorrespondences(corr_iteration);
    execution_times->calculate_correspondences += ElapsedTime(begin_time);

    // Visualize correspondences
    int corr_save_idx =
        iteration * tracker_ptr_->n_corr_iterations() + corr_iteration;
    tracker_ptr_->VisualizeCorrespondences(corr_save_idx);

    for (int update_iteration = 0;
         update_iteration < tracker_ptr_->n_update_iterations();
         ++update_iteration) {
      // Calculate pose update
      begin_time = std::chrono::high_resolution_clock::now();
      tracker_ptr_->CalculatePoseUpdate();
      execution_times->calculate_pose_update += ElapsedTime(begin_time);

      // Visualize pose update
      int update_save_idx =
          corr_save_idx * tracker_ptr_->n_update_iterations() +
          update_iteration;
      tracker_ptr_->VisualizePoseUpdate(update_save_idx);
    }
  }

  // Visualize results and update viewers
  tracker_ptr_->VisualizeResults(iteration);
  if (visualize_all_results_) tracker_ptr_->UpdateViewers(iteration);

  execution_times->complete_cycle =
      execution_times->calculate_before_camera_update +
      execution_times->start_occlusion_mask_rendering +
      execution_times->calculate_correspondences +
      execution_times->calculate_pose_update;
}

void Evaluator::CalculatePoseResults(
    const rbgt::Transform3fA &body2world_pose,
    const rbgt::Transform3fA &body2world_pose_gt, DataResult *result) const {
  result->translation_error = (body2world_pose.translation().matrix() -
                               body2world_pose_gt.translation().matrix())
                                  .norm();
  result->rotation_error =
      acos(((body2world_pose.rotation().matrix().transpose() *
             body2world_pose_gt.rotation().matrix())
                .trace() -
            1.0f) /
           2.0f);
  result->tracking_loss =
      result->translation_error > translation_error_threshold_ ||
      result->rotation_error > rotation_error_threshold_;
}

void Evaluator::VisualizeResult(const DataResult &result,
                                const std::string &body_name) {
  std::cout << body_name << ": "
            << "frame " << result.frame_index << ": "
            << "execution_time = " << result.execution_times.complete_cycle
            << " us "
            << "rotation_error = " << result.rotation_error * 180.0f / rbgt::kPi
            << ", "
            << "translation_error = " << result.translation_error << ", "
            << "tracking loss = " << result.tracking_loss << std::endl;
}

void Evaluator::VisualizeAverageResults(
    const std::vector<std::vector<DataResult>> &results,
    const std::string &title) {
  float tracking_loss = 0.0f;
  float execution_time_complete_cycle = 0.0f;
  float execution_time_calculate_before_camera_update = 0.0f;
  float execution_time_calculate_correspondences = 0.0f;
  float execution_time_calculate_pose_update = 0.0f;
  int n_results = 0;
  for (auto &results_body : results) {
    for (auto &result : results_body) {
      tracking_loss += float(result.tracking_loss);
      execution_time_complete_cycle += result.execution_times.complete_cycle;
      execution_time_calculate_before_camera_update +=
          result.execution_times.calculate_before_camera_update;
      execution_time_calculate_correspondences +=
          result.execution_times.calculate_correspondences;
      execution_time_calculate_pose_update +=
          result.execution_times.calculate_pose_update;
    }
    n_results += kNFrames_;
  }
  tracking_loss /= float(n_results);
  execution_time_complete_cycle /= float(n_results);
  execution_time_calculate_before_camera_update /= float(n_results);
  execution_time_calculate_correspondences /= float(n_results);
  execution_time_calculate_pose_update /= float(n_results);

  std::cout << "----------------------------------------"
            << "----------------------------------------" << std::endl;
  std::cout << title << ":" << std::endl;
  std::cout << "success rate = " << 1.0f - tracking_loss << std::endl;
  std::cout << "execution times:" << std::endl;
  std::cout << "complete cycle = " << execution_time_complete_cycle << " us"
            << std::endl;
  std::cout << "calculate before camera update = "
            << execution_time_calculate_before_camera_update << " us"
            << std::endl;
  std::cout << "calculate correspondences = "
            << execution_time_calculate_correspondences << " us" << std::endl;
  std::cout << "calculate pose update = "
            << execution_time_calculate_pose_update << " us" << std::endl;
}

bool Evaluator::ReadPosesRBOTDataset(const std::filesystem::path &path,
                                     std::vector<rbgt::Transform3fA> *poses) {
  std::ifstream ifs;
  ifs.open(path.string(), std::ios::binary);
  if (!ifs.is_open() || ifs.fail()) {
    ifs.close();
    std::cerr << "Could not open file stream " << path.string() << std::endl;
    return false;
  }

  poses->resize(kNFrames_);
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

float Evaluator::ElapsedTime(
    const std::chrono::high_resolution_clock::time_point &begin_time) {
  auto end_time{std::chrono::high_resolution_clock::now()};
  return std::chrono::duration_cast<std::chrono::microseconds>(end_time -
                                                               begin_time)
      .count();
}
