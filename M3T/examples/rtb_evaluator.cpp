// SPDX-License-Identifier: MIT
// Copyright (c) 2023 Manuel Stoiber, German Aerospace Center (DLR)

#include "rtb_evaluator.h"

RTBEvaluator::RTBEvaluator(const std::string &name,
                           const std::filesystem::path &dataset_directory,
                           const std::filesystem::path &external_directory,
                           const std::vector<std::string> &object_names,
                           const std::vector<std::string> &difficulty_levels,
                           const std::vector<std::string> &depth_names,
                           const std::vector<int> &sequence_numbers)
    : name_{name},
      dataset_directory_{dataset_directory},
      external_directory_{external_directory},
      object_names_{object_names},
      difficulty_levels_{difficulty_levels},
      depth_names_{depth_names},
      sequence_numbers_{sequence_numbers} {
  // Compute thresholds used to compute AUC score
  float threshold_step = 1.0f / float(kNCurveValues);
  for (size_t i = 0; i < kNCurveValues; ++i) {
    thresholds_[i] = threshold_step * (0.5f + float(i));
  }
}

bool RTBEvaluator::SetUp() {
  set_up_ = false;
  CreateRunConfigurations();
  set_up_ = true;
  return true;
}

void RTBEvaluator::set_evaluation_mode(EvaluationMode evaluation_mode) {
  evaluation_mode_ = evaluation_mode;
}

void RTBEvaluator::set_evaluate_external(bool evaluate_external) {
  evaluate_external_ = evaluate_external;
}

void RTBEvaluator::set_external_results_folder(
    const std::filesystem::path &external_results_folder) {
  external_results_folder_ = external_results_folder;
}

void RTBEvaluator::set_run_sequentially(bool run_sequentially) {
  run_sequentially_ = run_sequentially;
}

void RTBEvaluator::set_use_random_seed(bool use_random_seed) {
  use_random_seed_ = use_random_seed;
}

void RTBEvaluator::set_n_vertices_evaluation(int n_vertices_evaluation) {
  n_vertices_evaluation_ = n_vertices_evaluation;
}

void RTBEvaluator::set_visualize_tracking(bool visualize_tracking) {
  visualize_tracking_ = visualize_tracking;
}

void RTBEvaluator::set_visualize_frame_results(bool visualize_frame_results) {
  visualize_frame_results_ = visualize_frame_results;
}

void RTBEvaluator::StartSavingImages(
    const std::filesystem::path &save_directory) {
  save_images_ = true;
  save_directory_ = save_directory;
}

void RTBEvaluator::StopSavingImages() { save_images_ = false; }

void RTBEvaluator::set_use_shared_color_histograms(
    bool use_shared_color_histograms) {
  use_shared_color_histograms_ = use_shared_color_histograms;
}

void RTBEvaluator::set_use_region_checking(bool use_region_checking) {
  use_region_checking_ = use_region_checking;
}

void RTBEvaluator::set_use_silhouette_checking(bool use_silhouette_checking) {
  use_silhouette_checking_ = use_silhouette_checking;
}

void RTBEvaluator::set_tracker_setter(
    const std::function<void(std::shared_ptr<m3t::Tracker>)> &tracker_setter) {
  tracker_setter_ = tracker_setter;
}

void RTBEvaluator::set_optimizer_setter(
    const std::function<void(std::shared_ptr<m3t::Optimizer>)>
        &optimizer_setter) {
  optimizer_setter_ = optimizer_setter;
}

void RTBEvaluator::set_region_modality_setter(
    const std::function<void(std::shared_ptr<m3t::RegionModality>)>
        &region_modality_setter) {
  region_modality_setter_ = region_modality_setter;
}

void RTBEvaluator::set_color_histograms_setter(
    const std::function<void(std::shared_ptr<m3t::ColorHistograms>)>
        &color_histograms_setter) {
  color_histograms_setter_ = color_histograms_setter;
}

void RTBEvaluator::set_region_model_setter(
    const std::function<void(std::shared_ptr<m3t::RegionModel>)>
        &region_model_setter) {
  region_model_setter_ = region_model_setter;
  set_up_ = false;
}

void RTBEvaluator::set_depth_modality_setter(
    const std::function<void(std::shared_ptr<m3t::DepthModality>)>
        &depth_modality_setter) {
  depth_modality_setter_ = depth_modality_setter;
}

void RTBEvaluator::set_depth_model_setter(
    const std::function<void(std::shared_ptr<m3t::DepthModel>)>
        &depth_model_setter) {
  depth_model_setter_ = depth_model_setter;
  set_up_ = false;
}

bool RTBEvaluator::Evaluate() {
  if (!set_up_) {
    std::cerr << "Set up evaluator " << name_ << " first" << std::endl;
    return false;
  }
  if (run_configurations_.empty()) return false;

  // Evaluate all run configurations
  results_.clear();
  final_results_.clear();
  for (const auto &object_name : object_names_) {
    {
      std::vector<std::shared_ptr<m3t::Body>> body_ptrs;
      if (!LoadObjectBodies(object_name, &body_ptrs)) return false;
      if (!GenerateModels(body_ptrs, object_name)) return false;
      GenderateReducedVertices(body_ptrs);
      GenerateKDTrees(body_ptrs);
      LoadEvaluationData(object_name);
      body_name2idx_map_.clear();
      for (int i = 0; i < body_ptrs.size(); ++i)
        body_name2idx_map_.insert({body_ptrs[i]->name(), i});
    }

    if (run_sequentially_ || visualize_tracking_ || visualize_frame_results_) {
      // Set up renderer geometry
      auto renderer_geometry_ptr{std::make_shared<m3t::RendererGeometry>("rg")};
      if (!renderer_geometry_ptr->SetUp()) return false;

      // Set Up Tracker
      std::shared_ptr<m3t::Tracker> tracker_ptr;
      if (!SetUpTracker(object_name, renderer_geometry_ptr, &tracker_ptr))
        return false;

      // Iterate over run configuration
      for (size_t i = 0; i < run_configurations_.size(); ++i) {
        if (run_configurations_[i].object_name != object_name) continue;
        SequenceResult sequence_result;
        if (!EvaluateRunConfiguration(run_configurations_[i], tracker_ptr,
                                      &sequence_result))
          return false;
        VisualizeResult(sequence_result.average_result, sequence_result.title);
        results_.push_back(std::move(sequence_result));
      }
    } else {
      // Set up renderer geometry
      std::vector<std::shared_ptr<m3t::RendererGeometry>>
          renderer_geometry_ptrs(omp_get_max_threads());
      for (auto &renderer_geometry_ptr : renderer_geometry_ptrs) {
        renderer_geometry_ptr = std::make_shared<m3t::RendererGeometry>("rg");
        if (!renderer_geometry_ptr->SetUp()) return false;
      }

      bool cancel = false;
#pragma omp parallel
      {
        // Set Up Tracker
        std::shared_ptr<m3t::Tracker> tracker_ptr;
        if (!SetUpTracker(object_name,
                          renderer_geometry_ptrs[omp_get_thread_num()],
                          &tracker_ptr))
          cancel = true;

          // Iterate over run configuration
#pragma omp for schedule(dynamic, 1)
        for (int i = 0; i < int(run_configurations_.size()); ++i) {
          if (cancel) continue;
          if (run_configurations_[i].object_name != object_name) continue;
          SequenceResult sequence_result;
          if (!EvaluateRunConfiguration(run_configurations_[i], tracker_ptr,
                                        &sequence_result)) {
            cancel = true;
            continue;
          }
#pragma omp critical
          {
            VisualizeResult(sequence_result.average_result,
                            sequence_result.title);
            results_.push_back(std::move(sequence_result));
          }
        }
      }
    }
  }

  // Calculate difficulty results
  std::cout << std::endl << std::string(80, '-') << std::endl;
  for (const auto &object_name : object_names_) {
    for (const auto &difficulty_level : difficulty_levels_) {
      Result result{CalculateAverageResult({object_name}, {difficulty_level},
                                           depth_names_)};
      VisualizeResult(result, object_name + "_" + difficulty_level);
      final_results_.insert(
          {object_name + "_" + difficulty_level, std::move(result)});
    }
  }
  for (const auto &difficulty_level : difficulty_levels_) {
    Result result{CalculateAverageResult(object_names_, {difficulty_level},
                                         depth_names_)};
    VisualizeResult(result, "all_" + difficulty_level);
    final_results_.insert({"all_" + difficulty_level, std::move(result)});
  }

  // Calculate depth results
  std::cout << std::endl << std::string(80, '-') << std::endl;
  for (const auto &object_name : object_names_) {
    for (const auto depth_name : depth_names_) {
      Result result{CalculateAverageResult({object_name}, difficulty_levels_,
                                           {depth_name})};
      VisualizeResult(result, object_name + "_" + depth_name);
      final_results_.insert(
          {object_name + "_" + depth_name, std::move(result)});
    }
  }
  for (const auto depth_name : depth_names_) {
    Result result{CalculateAverageResult(object_names_, difficulty_levels_,
                                         {depth_name})};
    VisualizeResult(result, "all_" + depth_name);
    final_results_.insert({"all_" + depth_name, std::move(result)});
  }

  // Calculate object results
  std::cout << std::endl << std::string(80, '-') << std::endl;
  for (const auto &object_name : object_names_) {
    Result result{CalculateAverageResult({object_name}, difficulty_levels_,
                                         depth_names_)};
    VisualizeResult(result, object_name);
    final_results_.insert({object_name, std::move(result)});
  }
  Result result{
      CalculateAverageResult(object_names_, difficulty_levels_, depth_names_)};
  VisualizeResult(result, "all");
  final_results_.insert({"all", std::move(result)});
  std::cout << std::string(80, '-') << std::endl;
  return true;
}

void RTBEvaluator::SaveResults(std::filesystem::path path) const {
  std::ofstream ofs{path};
  for (auto const &[name, result] : final_results_) {
    ofs << name << "," << result.add_auc << "," << result.adds_auc << ","
        << result.execution_times.complete_cycle << ","
        << result.execution_times.calculate_correspondences << ","
        << result.execution_times.calculate_gradient_and_hessian << ","
        << result.execution_times.calculate_optimization << ","
        << result.execution_times.calculate_results << std::endl;
  }
  ofs.flush();
  ofs.close();
}

float RTBEvaluator::add_auc() const { return final_results_.at("all").add_auc; }

float RTBEvaluator::adds_auc() const {
  return final_results_.at("all").adds_auc;
}

float RTBEvaluator::execution_time() const {
  return final_results_.at("all").execution_times.complete_cycle;
}

std::map<std::string, RTBEvaluator::Result> RTBEvaluator::final_results()
    const {
  return final_results_;
}

void RTBEvaluator::CreateRunConfigurations() {
  run_configurations_.clear();
  for (const auto &object_name : object_names_) {
    for (const auto &difficulty_level : difficulty_levels_) {
      for (const auto &depth_name : depth_names_) {
        for (int sequence_number : sequence_numbers_) {
          int n_zeros = 6 - std::to_string(sequence_number).length();
          std::string sequence_name{std::string(n_zeros, '0') +
                                    std::to_string(sequence_number)};
          std::string title{object_name + "_" + difficulty_level + "_" +
                            sequence_name + "_" + depth_name};
          run_configurations_.push_back(RunConfiguration{
              object_name, difficulty_level, depth_name, sequence_name, title});
        }
      }
    }
  }
}

bool RTBEvaluator::LoadObjectBodies(
    const std::string &object_name,
    std::vector<std::shared_ptr<m3t::Body>> *body_ptrs) const {
  body_ptrs->clear();
  std::filesystem::path configfile_path{dataset_directory_ / object_name /
                                        "model" / "tracker_config" /
                                        "config.yaml"};
  cv::FileStorage fs;
  if (!m3t::OpenYamlFileStorage(configfile_path, &fs)) return false;
  if (!m3t::ConfigureObjectsMetafileRequired<m3t::Body>(configfile_path, fs,
                                                        "Body", body_ptrs))
    return false;
  for (auto &body_ptr : *body_ptrs)
    if (!body_ptr->SetUp()) return false;
  return true;
}

bool RTBEvaluator::GenerateModels(
    std::vector<std::shared_ptr<m3t::Body>> &body_ptrs,
    const std::string &object_name) {
  region_model_ptrs_.clear();
  depth_model_ptrs_.clear();
  max_model_contour_length_ = 0.0f;
  max_model_surface_area_ = 0.0f;

  std::filesystem::path configfile_path{dataset_directory_ / object_name /
                                        "model" / "tracker_config" /
                                        "config.yaml"};
  cv::FileStorage fs;
  if (!m3t::OpenYamlFileStorage(configfile_path, &fs)) return false;
  if (!m3t::ConfigureRegionModels(configfile_path, fs, body_ptrs,
                                  &region_model_ptrs_))
    return false;
  if (!m3t::ConfigureObjectsMetafileAndBodyRequired<m3t::DepthModel>(
          configfile_path, fs, "DepthModel", body_ptrs, &depth_model_ptrs_))
    return false;

  std::filesystem::path directory{external_directory_ / "models" / object_name};
  std::filesystem::create_directories(directory);
  for (auto &region_model_ptr : region_model_ptrs_) {
    const auto &body_ptr{region_model_ptr->body_ptr()};
    region_model_ptr = std::make_shared<m3t::RegionModel>(
        body_ptr->name() + "_region_model", body_ptr,
        directory / (body_ptr->name() + "_region_model.bin"), 1.5f, 4, 500,
        0.05f, 0.002f, false, 2000);
    region_model_setter_(region_model_ptr);
    if (!region_model_ptr->SetUp()) return false;
    max_model_contour_length_ = std::max(
        max_model_contour_length_, kObject2SizeMultiplier.at(object_name) *
                                       region_model_ptr->max_contour_length());
  }
  for (auto &depth_model_ptr : depth_model_ptrs_) {
    const auto &body_ptr{depth_model_ptr->body_ptr()};
    depth_model_ptr = std::make_shared<m3t::DepthModel>(
        body_ptr->name() + "_depth_model", body_ptr,
        directory / (body_ptr->name() + "_depth_model.bin"), 1.5f, 4, 500,
        0.05f, 0.002f, false, 2000);
    depth_model_setter_(depth_model_ptr);
    if (!depth_model_ptr->SetUp()) return false;
    max_model_surface_area_ = std::max(max_model_surface_area_,
                                       kObject2SizeMultiplier.at(object_name) *
                                           depth_model_ptr->max_surface_area());
  }
  return true;
}

void RTBEvaluator::GenderateReducedVertices(
    std::vector<std::shared_ptr<m3t::Body>> &body_ptrs) {
  reduced_vertices_vector_.resize(body_ptrs.size());
#pragma omp parallel for
  for (int i = 0; i < body_ptrs.size(); ++i) {
    const auto &vertices{body_ptrs[i]->vertices()};
    if (n_vertices_evaluation_ <= 0 ||
        n_vertices_evaluation_ >= vertices.size()) {
      reduced_vertices_vector_[i] = vertices;
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
    reduced_vertices_vector_[i] = std::move(reduced_vertices);
  }
}

void RTBEvaluator::GenerateKDTrees(
    std::vector<std::shared_ptr<m3t::Body>> &body_ptrs) {
  kdtree_ptr_vector_.resize(body_ptrs.size());
#pragma omp parallel for
  for (int i = 0; i < body_ptrs.size(); ++i) {
    const auto &vertices{body_ptrs[i]->vertices()};
    auto kdtree_ptr{std::make_unique<KDTreeVector3f>(3, vertices, 10)};
    kdtree_ptr->index->buildIndex();
    kdtree_ptr_vector_[i] = std::move(kdtree_ptr);
  }
}

bool RTBEvaluator::LoadEvaluationData(const std::string &object_name) {
  std::filesystem::path path{dataset_directory_ / object_name / "model" /
                             "evaluation_data.yaml"};
  cv::FileStorage fs;
  if (!m3t::OpenYamlFileStorage(path, &fs)) return false;
  fs["ErrorThreshold"] >> error_threshold_;

  cv::FileNode fn = fs["IDsCombinedBodies"];
  ids_combined_bodies_.clear();
  for (const auto &fn_body : fn) {
    std::vector<int> ids_combined_body;
    fn_body >> ids_combined_body;
    ids_combined_bodies_.push_back(std::move(ids_combined_body));
  }
  return true;
}

bool RTBEvaluator::EvaluateRunConfiguration(
    const RunConfiguration &run_configuration,
    const std::shared_ptr<m3t::Tracker> &tracker_ptr,
    SequenceResult *sequence_result) const {
  // Set run configuration
  if (!SetRunConfiguration(run_configuration, tracker_ptr)) return false;

  // Read gt poses
  std::vector<std::vector<m3t::Transform3fA>> gt_body2world_poses_sequence;
  std::filesystem::path gt_path{
      dataset_directory_ / run_configuration.object_name /
      run_configuration.difficulty_level / run_configuration.sequence_name /
      "scene_gt.json"};
  if (!LoadPoses(gt_path, 0, &gt_body2world_poses_sequence)) return false;

  // Read external poses
  std::vector<std::vector<m3t::Transform3fA>>
      external_body2world_poses_sequence;
  std::vector<float> external_execution_times;
  if (evaluate_external_) {
    std::filesystem::path external_path{external_directory_ / "poses" /
                                        external_results_folder_ /
                                        (run_configuration.title + ".json")};
    if (!LoadPoses(external_path, 1, &external_body2world_poses_sequence,
                   &external_execution_times))
      return false;
  }

  // Initialize poses and start modalities
  SetBodyAndJointPoses(gt_body2world_poses_sequence[0], tracker_ptr);
  tracker_ptr->StartModalities(0);

  // Init results
  sequence_result->object_name = run_configuration.object_name;
  sequence_result->difficulty_level = run_configuration.difficulty_level;
  sequence_result->depth_name = run_configuration.depth_name;
  sequence_result->sequence_name = run_configuration.sequence_name;
  sequence_result->title = run_configuration.title;
  sequence_result->frame_results.clear();

  // Iterate over all frames
  for (int i = 0; i < gt_body2world_poses_sequence.size() - 1; ++i) {
    Result result;
    result.frame_index = i;
    if (evaluate_external_) {
      SetBodyAndJointPoses(external_body2world_poses_sequence[i], tracker_ptr);
      result.execution_times.complete_cycle = external_execution_times[i];
      ExecuteViewingCycle(tracker_ptr, i);
    } else {
      ExecuteMeasuredTrackingCycle(tracker_ptr, i, &result.execution_times);
    }

    // Calculate results
    CalculatePoseResults(tracker_ptr, gt_body2world_poses_sequence[i + 1],
                         &result);
    if (visualize_frame_results_)
      VisualizeResult(result, run_configuration.title);
    sequence_result->frame_results.push_back(std::move(result));
  }

  // Calculate average results
  sequence_result->average_result =
      CalculateAverageResult(sequence_result->frame_results);
  return true;
}

bool RTBEvaluator::SetUpTracker(
    const std::string &object_name,
    const std::shared_ptr<m3t::RendererGeometry> &renderer_geometry_ptr,
    std::shared_ptr<m3t::Tracker> *tracker_ptr) const {
  *tracker_ptr = std::make_shared<m3t::Tracker>("tracker");

  // Create file storage
  cv::FileStorage fs;
  std::filesystem::path configfile_path;
  switch (evaluation_mode_) {
    case EvaluationMode::INDEPENDENT:
      configfile_path = dataset_directory_ / object_name / "model" /
                        "tracker_config" / "config_independent.yaml";
      break;
    case EvaluationMode::PROJECTED:
      configfile_path = dataset_directory_ / object_name / "model" /
                        "tracker_config" / "config_projected.yaml";
      break;
    case EvaluationMode::CONSTRAINED:
      configfile_path = dataset_directory_ / object_name / "model" /
                        "tracker_config" / "config_constrained.yaml";
      break;
    default:
      configfile_path = dataset_directory_ / object_name / "model" /
                        "tracker_config" / "config.yaml";
  }
  if (!m3t::OpenYamlFileStorage(configfile_path, &fs)) return false;

  // Init bodies
  std::vector<std::shared_ptr<m3t::Body>> body_ptrs;
  if (!LoadObjectBodies(object_name, &body_ptrs)) return false;

  // Init renderer geometry
  renderer_geometry_ptr->ClearBodies();
  for (const auto &body_ptr : body_ptrs)
    renderer_geometry_ptr->AddBody(body_ptr);

  // Init cameras
  std::filesystem::path color_camera_directory{
      dataset_directory_ / run_configurations_[0].object_name /
      run_configurations_[0].difficulty_level /
      run_configurations_[0].sequence_name / "rgb"};
  std::filesystem::path depth_camera_directory{
      dataset_directory_ / run_configurations_[0].object_name /
      run_configurations_[0].difficulty_level /
      run_configurations_[0].sequence_name / run_configurations_[0].depth_name};
  auto color_camera_ptr{std::make_shared<m3t::LoaderColorCamera>(
      "color_camera", color_camera_directory, kRTBIntrinsics, "", 0, 6)};
  if (!color_camera_ptr->SetUp()) return false;
  auto depth_camera_ptr{std::make_shared<m3t::LoaderDepthCamera>(
      "depth_camera", depth_camera_directory, kRTBIntrinsics, 0.001f, "", 0,
      6)};
  if (!depth_camera_ptr->SetUp()) return false;

  // Init viewers
  if (visualize_tracking_ || save_images_) {
    auto color_viewer_ptr{std::make_shared<m3t::NormalColorViewer>(
        "color_viewer", color_camera_ptr, renderer_geometry_ptr)};
    color_viewer_ptr->set_display_images(visualize_tracking_);
    if (!color_viewer_ptr->SetUp()) return false;
    (*tracker_ptr)->AddViewer(color_viewer_ptr);
    auto depth_viewer_ptr{std::make_shared<m3t::NormalDepthViewer>(
        "depth_viewer", depth_camera_ptr, renderer_geometry_ptr, 0.0f, 2.0f)};
    depth_viewer_ptr->set_display_images(visualize_tracking_);
    if (!depth_viewer_ptr->SetUp()) return false;
    (*tracker_ptr)->AddViewer(depth_viewer_ptr);
  }

  // Init renderer
  auto silhouette_renderer_color_ptr{
      std::make_shared<m3t::FocusedSilhouetteRenderer>(
          "silhouette_renderer_color", renderer_geometry_ptr, color_camera_ptr,
          m3t::IDType::REGION, 400)};
  auto silhouette_renderer_depth_ptr{
      std::make_shared<m3t::FocusedSilhouetteRenderer>(
          "silhouette_renderer_depth", renderer_geometry_ptr, depth_camera_ptr,
          m3t::IDType::BODY, 400)};
  for (const auto &body_ptr : body_ptrs) {
    silhouette_renderer_color_ptr->AddReferencedBody(body_ptr);
    silhouette_renderer_depth_ptr->AddReferencedBody(body_ptr);
  }
  if (!silhouette_renderer_color_ptr->SetUp()) return false;
  if (!silhouette_renderer_depth_ptr->SetUp()) return false;

  // Configure color histograms
  std::vector<std::shared_ptr<m3t::ColorHistograms>> color_histograms_ptrs;
  if (!m3t::ConfigureObjectsMetafileOptional<m3t::ColorHistograms>(
          configfile_path, fs, "ColorHistograms", &color_histograms_ptrs))
    return false;
  for (auto &color_histograms_ptr : color_histograms_ptrs) {
    color_histograms_setter_(color_histograms_ptr);
    if (!color_histograms_ptr->SetUp()) return false;
  }

  // Init region modalities
  std::vector<std::shared_ptr<m3t::Modality>> modality_ptrs;
  if (!m3t::ConfigureObjects<m3t::RegionModality>(
          fs, "RegionModality",
          {"name", "body", "color_camera", "region_model"},
          [&](const auto &file_node, auto *region_modality_ptr) {
            // Get objects required for region modality constructor
            std::shared_ptr<m3t::Body> body_ptr;
            std::shared_ptr<m3t::RegionModel> region_model_ptr;
            if (!m3t::GetObject(file_node, "body", "RegionModality", body_ptrs,
                                &body_ptr) ||
                !m3t::GetObject(file_node, "region_model", "RegionModality",
                                region_model_ptrs_, &region_model_ptr))
              return false;

            // Construct region modality
            *region_modality_ptr = std::make_shared<m3t::RegionModality>(
                m3t::Name(file_node), body_ptr, color_camera_ptr,
                region_model_ptr);

            // Add additional objects
            if (use_shared_color_histograms_) {
              if (!file_node["use_shared_color_histograms"].empty()) {
                if (!m3t::AddObject(file_node["use_shared_color_histograms"],
                                    "color_histograms", "RegionModality",
                                    color_histograms_ptrs,
                                    [&](const auto &color_histograms_ptr) {
                                      (*region_modality_ptr)
                                          ->UseSharedColorHistograms(
                                              color_histograms_ptr);
                                      return true;
                                    }))
                  return false;
              }
            }
            if (use_region_checking_)
              (*region_modality_ptr)
                  ->UseRegionChecking(silhouette_renderer_color_ptr);

            // Set parameters
            region_modality_setter_(*region_modality_ptr);
            (*region_modality_ptr)->set_n_unoccluded_iterations(0);
            (*region_modality_ptr)
                ->set_reference_contour_length(max_model_contour_length_);
            return (*region_modality_ptr)->SetUp();
          },
          &modality_ptrs))
    return false;

  // Init depth modalities
  if (!m3t::ConfigureObjects<m3t::DepthModality>(
          fs, "DepthModality", {"name", "body", "depth_camera", "depth_model"},
          [&](const auto &file_node, auto *depth_modality_ptr) {
            // Get objects required for depth modality constructor
            std::shared_ptr<m3t::Body> body_ptr;
            std::shared_ptr<m3t::DepthModel> depth_model_ptr;
            if (!m3t::GetObject(file_node, "body", "DepthModality", body_ptrs,
                                &body_ptr) ||
                !m3t::GetObject(file_node, "depth_model", "DepthModality",
                                depth_model_ptrs_, &depth_model_ptr))
              return false;

            // Construct depth modality
            *depth_modality_ptr = std::make_shared<m3t::DepthModality>(
                m3t::Name(file_node), body_ptr, depth_camera_ptr,
                depth_model_ptr);

            // Add additional objects
            if (use_silhouette_checking_)
              (*depth_modality_ptr)
                  ->UseSilhouetteChecking(silhouette_renderer_depth_ptr);

            // Set parameters
            depth_modality_setter_(*depth_modality_ptr);
            (*depth_modality_ptr)->set_n_unoccluded_iterations(0);
            (*depth_modality_ptr)
                ->set_reference_surface_area(max_model_surface_area_);
            return (*depth_modality_ptr)->SetUp();
          },
          &modality_ptrs))
    return false;

  // Init links
  std::vector<std::shared_ptr<m3t::Link>> link_ptrs;
  if (!m3t::ConfigureLinks(configfile_path, fs, body_ptrs, modality_ptrs,
                           &link_ptrs))
    return false;
  for (auto &link_ptr : link_ptrs)
    if (!link_ptr->SetUp()) return false;

  // Init constraints
  std::vector<std::shared_ptr<m3t::Constraint>> contraint_ptrs;
  if (!m3t::ConfigureConstraints(configfile_path, fs, link_ptrs,
                                 &contraint_ptrs))
    return false;
  for (auto &constraint_ptr : contraint_ptrs)
    if (!constraint_ptr->SetUp()) return false;

  // Init optimizers
  std::vector<std::shared_ptr<m3t::Optimizer>> optimizer_ptrs;
  if (!m3t::ConfigureOptimizers(
          configfile_path, fs, link_ptrs, contraint_ptrs,
          std::vector<std::shared_ptr<m3t::SoftConstraint>>{}, &optimizer_ptrs))
    return false;
  for (auto &optimizer_ptr : optimizer_ptrs) {
    optimizer_setter_(optimizer_ptr);
    if (!optimizer_ptr->SetUp()) return false;
  }

  // Init tracker
  for (auto &optimizer_ptr : optimizer_ptrs)
    (*tracker_ptr)->AddOptimizer(optimizer_ptr);
  tracker_setter_(*tracker_ptr);
  return (*tracker_ptr)->SetUp(false);
}

bool RTBEvaluator::SetRunConfiguration(
    const RunConfiguration &run_configuration,
    const std::shared_ptr<m3t::Tracker> &tracker_ptr) const {
  std::filesystem::path depth_camera_directory{
      dataset_directory_ / run_configuration.object_name /
      run_configuration.difficulty_level / run_configuration.sequence_name /
      run_configuration.depth_name};
  std::filesystem::path color_camera_directory{
      dataset_directory_ / run_configuration.object_name /
      run_configuration.difficulty_level / run_configuration.sequence_name /
      "rgb"};

  for (const auto &camera_ptr : tracker_ptr->camera_ptrs()) {
    if (camera_ptr->name() == "color_camera") {
      std::dynamic_pointer_cast<m3t::LoaderColorCamera>(camera_ptr)
          ->set_load_directory(color_camera_directory);
      std::dynamic_pointer_cast<m3t::LoaderColorCamera>(camera_ptr)
          ->set_load_index(0);
    }
    if (camera_ptr->name() == "depth_camera") {
      std::dynamic_pointer_cast<m3t::LoaderDepthCamera>(camera_ptr)
          ->set_load_directory(depth_camera_directory);
      std::dynamic_pointer_cast<m3t::LoaderDepthCamera>(camera_ptr)
          ->set_load_index(0);
    }
    if (!camera_ptr->SetUp()) return false;
  }

  if (save_images_) {
    auto save_directory_sequence{save_directory_ / run_configuration.title};
    std::filesystem::create_directories(save_directory_sequence);
    for (const auto &viewer_ptr : tracker_ptr->viewer_ptrs())
      viewer_ptr->StartSavingImages(save_directory_sequence);
  }
  return true;
}

bool RTBEvaluator::LoadPoses(
    const std::filesystem::path &path, int start_index,
    std::vector<std::vector<m3t::Transform3fA>> *body2world_poses_sequence,
    std::vector<float> *execution_times) const {
  // Open file
  cv::FileStorage fs;
  try {
    fs.open(cv::samples::findFile(path.string()), cv::FileStorage::READ);
  } catch (cv::Exception e) {
  }
  if (!fs.isOpened()) {
    std::cerr << "Could not open file " << path << std::endl;
    return false;
  }

  // Iterate all sequences
  body2world_poses_sequence->clear();
  if (execution_times) execution_times->clear();
  for (int i = start_index; true; ++i) {
    cv::FileNode fn_sequence{fs[std::to_string(i)]};
    if (fn_sequence.empty()) break;

    // Iterate all bodies
    std::vector<m3t::Transform3fA> body2world_poses(body_name2idx_map_.size());
    float execution_time;
    for (const auto &[body_name, idx] : body_name2idx_map_) {
      int body_id = stoi(body_name.substr(4, body_name.size()));

      // Find file node for body
      cv::FileNode fn_body;
      for (const auto &fn_obj : fn_sequence) {
        int obj_id;
        fn_obj["obj_id"] >> obj_id;
        if (obj_id == body_id) {
          fn_body = fn_obj;
          break;
        }
      }
      if (fn_body.empty()) {
        std::cerr << "obj_id " << body_id << " not found in sequence " << i
                  << std::endl;
        return false;
      }

      // Extract pose
      m3t::Transform3fA body2world_pose;
      for (int i = 0; i < 9; ++i)
        fn_body["cam_R_m2c"][i] >> body2world_pose.matrix()(i / 3, i % 3);
      for (int i = 0; i < 3; ++i)
        fn_body["cam_t_m2c"][i] >> body2world_pose.matrix()(i, 3);
      body2world_poses[idx] = body2world_pose;
      if (execution_times) fn_body["execution_time"] >> execution_time;
    }
    body2world_poses_sequence->push_back(std::move(body2world_poses));
    if (execution_times) execution_times->push_back(execution_time);
  }
  return true;
}

void RTBEvaluator::SetBodyAndJointPoses(
    const std::vector<m3t::Transform3fA> &body2world_poses,
    const std::shared_ptr<m3t::Tracker> &tracker_ptr) const {
  for (auto &optimizer_ptr : tracker_ptr->optimizer_ptrs()) {
    if (evaluation_mode_ == EvaluationMode::CONSTRAINED) {
      SetBodyAndJointPosesConstrained(body2world_poses,
                                      optimizer_ptr->root_link_ptr());
    } else {
      SetBodyAndJointPoses(body2world_poses, optimizer_ptr->root_link_ptr(),
                           nullptr);
    }
  }
}

void RTBEvaluator::SetBodyAndJointPoses(
    const std::vector<m3t::Transform3fA> &body2world_poses,
    const std::shared_ptr<m3t::Link> &link_ptr,
    const std::shared_ptr<m3t::Link> &parent_link_ptr) const {
  auto &body_ptr{link_ptr->body_ptr()};

  // Set body pose
  int idx = body_name2idx_map_.at(body_ptr->name());
  body_ptr->set_body2world_pose(body2world_poses[idx]);

  // Set joint pose
  if (parent_link_ptr) {
    auto &parent_body_ptr{parent_link_ptr->body_ptr()};
    link_ptr->set_joint2parent_pose(parent_body_ptr->world2body_pose() *
                                    body_ptr->body2world_pose() *
                                    link_ptr->body2joint_pose().inverse());
  }

  // Recursion
  for (const auto &child_link_ptr : link_ptr->child_link_ptrs())
    SetBodyAndJointPoses(body2world_poses, child_link_ptr, link_ptr);
}

void RTBEvaluator::SetBodyAndJointPosesConstrained(
    const std::vector<m3t::Transform3fA> &body2world_poses,
    const std::shared_ptr<m3t::Link> &root_link_ptr) const {
  for (const auto &link_ptr : root_link_ptr->child_link_ptrs()) {
    auto &body_ptr{link_ptr->body_ptr()};
    int idx = body_name2idx_map_.at(body_ptr->name());
    body_ptr->set_body2world_pose(body2world_poses[idx]);
    link_ptr->set_joint2parent_pose(body2world_poses[idx]);

    for (const auto &child_link_ptr : link_ptr->child_link_ptrs())
      SetBodyAndJointPoses(body2world_poses, child_link_ptr, link_ptr);
  }
}

void RTBEvaluator::ExecuteMeasuredTrackingCycle(
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
  if (visualize_tracking_ || save_images_)
    tracker_ptr->UpdateViewers(iteration);

  execution_times->complete_cycle =
      execution_times->calculate_correspondences +
      execution_times->calculate_gradient_and_hessian +
      execution_times->calculate_optimization +
      execution_times->calculate_results;
}

void RTBEvaluator::ExecuteViewingCycle(
    const std::shared_ptr<m3t::Tracker> &tracker_ptr, int iteration) const {
  tracker_ptr->UpdateCameras(false);
  if (visualize_tracking_ || save_images_)
    tracker_ptr->UpdateViewers(iteration);
}

void RTBEvaluator::CalculatePoseResults(
    const std::shared_ptr<m3t::Tracker> &tracker_ptr,
    const std::vector<m3t::Transform3fA> &gt_body2world_poses,
    Result *result) const {
  // Calcualte add and adds auc score
  float add_auc = 0.0f;
  float adds_auc = 0.0f;
  for (const auto &ids_combined_body : ids_combined_bodies_) {
    float add_error_combined_body = 0.0f;
    float adds_error_combined_body = 0.0f;
    for (int id : ids_combined_body) {
      std::string body_name{"body" + std::to_string(id)};
      const auto &body_ptr = *std::find_if(
          begin(tracker_ptr->body_ptrs()), end(tracker_ptr->body_ptrs()),
          [&](const auto &b) { return b->name() == body_name; });
      int idx = body_name2idx_map_.at(body_ptr->name());
      const auto &reduced_vertices{reduced_vertices_vector_[idx]};
      const auto &kdtree_index{kdtree_ptr_vector_[idx]->index};
      m3t::Transform3fA delta_pose{body_ptr->world2body_pose() *
                                   gt_body2world_poses[idx]};

      float add_error_body = 0.0f;
      float adds_error_body = 0.0f;
      size_t ret_index;
      float dist_sqrt;
      Eigen::Vector3f vertice_trans;
      for (const auto &vertice : reduced_vertices) {
        vertice_trans = delta_pose * vertice;
        add_error_body += (vertice - vertice_trans).norm();
        kdtree_index->knnSearch(vertice_trans.data(), 1, &ret_index,
                                &dist_sqrt);
        adds_error_body += std::sqrt(dist_sqrt);
      }
      add_error_combined_body += add_error_body / reduced_vertices.size();
      adds_error_combined_body += adds_error_body / reduced_vertices.size();
    }
    add_error_combined_body /= ids_combined_body.size();
    adds_error_combined_body /= ids_combined_body.size();
    add_auc +=
        1.0f - std::min(add_error_combined_body / error_threshold_, 1.0f);
    adds_auc +=
        1.0f - std::min(adds_error_combined_body / error_threshold_, 1.0f);
  }
  add_auc /= ids_combined_bodies_.size();
  adds_auc /= ids_combined_bodies_.size();
  result->add_auc = add_auc;
  result->adds_auc = adds_auc;

  // Calculate curve (tracking loss distribution)
  std::fill(begin(result->add_curve), end(result->add_curve), 1.0f);
  for (size_t i = 0; i < kNCurveValues; ++i) {
    if (add_auc < thresholds_[i]) break;
    result->add_curve[i] = 0.0f;
  }
  std::fill(begin(result->adds_curve), end(result->adds_curve), 1.0f);
  for (size_t i = 0; i < kNCurveValues; ++i) {
    if (adds_auc < thresholds_[i]) break;
    result->adds_curve[i] = 0.0f;
  }
}

RTBEvaluator::Result RTBEvaluator::CalculateAverageResult(
    const std::vector<std::string> &object_names,
    const std::vector<std::string> &difficulty_levels,
    const std::vector<std::string> &depth_names) const {
  size_t n_results = 0;
  std::vector<Result> result_sums;
  for (const auto &result : results_) {
    if (std::find(begin(object_names), end(object_names), result.object_name) !=
            end(object_names) &&
        std::find(begin(difficulty_levels), end(difficulty_levels),
                  result.difficulty_level) != end(difficulty_levels) &&
        std::find(begin(depth_names), end(depth_names), result.depth_name) !=
            end(depth_names)) {
      n_results += result.frame_results.size();
      result_sums.push_back(SumResults(result.frame_results));
    }
  }
  return DivideResult(SumResults(result_sums), n_results);
}

RTBEvaluator::Result RTBEvaluator::CalculateAverageResult(
    const std::vector<Result> &results) {
  return DivideResult(SumResults(results), results.size());
}

RTBEvaluator::Result RTBEvaluator::SumResults(
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

RTBEvaluator::Result RTBEvaluator::DivideResult(const Result &result,
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

void RTBEvaluator::VisualizeResult(const Result &result,
                                   const std::string &title) {
  std::cout << title << ": ";
  if (result.frame_index) std::cout << "frame " << result.frame_index << ": ";
  std::cout << "execution_time = " << result.execution_times.complete_cycle
            << " us, "
            << "add auc = " << result.add_auc
            << ", adds auc = " << result.adds_auc << std::endl;
}

float RTBEvaluator::ElapsedTime(
    const std::chrono::high_resolution_clock::time_point &begin_time) {
  auto end_time{std::chrono::high_resolution_clock::now()};
  return float(std::chrono::duration_cast<std::chrono::microseconds>(end_time -
                                                                     begin_time)
                   .count());
}
