// SPDX-License-Identifier: MIT
// Copyright (c) 2022 Manuel Stoiber, German Aerospace Center (DLR)

#include <icg/tracker.h>

namespace icg {

Tracker::Tracker(const std::string &name, int n_corr_iterations,
                 int n_update_iterations, bool synchronize_cameras,
                 const std::chrono::milliseconds &cycle_duration,
                 int visualization_time, int viewer_time)
    : name_{name},
      n_corr_iterations_{n_corr_iterations},
      n_update_iterations_{n_update_iterations},
      synchronize_cameras_{synchronize_cameras},
      cycle_duration_{cycle_duration},
      visualization_time_{visualization_time},
      viewer_time_{viewer_time} {}

Tracker::Tracker(const std::string &name,
                 const std::filesystem::path &metafile_path)
    : name_{name}, metafile_path_{metafile_path} {}

bool Tracker::SetUp(bool set_up_all_objects) {
  set_up_ = false;
  if (!metafile_path_.empty())
    if (!LoadMetaData()) return false;
  AssembleDerivedObjectPtrs();
  if (set_up_all_objects) {
    if (!SetUpAllObjects()) return false;
  } else {
    if (!AreAllObjectsSetUp()) return false;
  }
  set_up_ = true;
  return true;
}

bool Tracker::AddOptimizer(const std::shared_ptr<Optimizer> &optimizer_ptr) {
  set_up_ = false;
  if (!AddPtrIfNameNotExists(optimizer_ptr, &optimizer_ptrs_)) {
    std::cerr << "Optimizer " << optimizer_ptr->name() << " already exists"
              << std::endl;
    return false;
  }
  return true;
}

bool Tracker::DeleteOptimizer(const std::string &name) {
  set_up_ = false;
  if (!DeletePtrIfNameExists(name, &optimizer_ptrs_)) {
    std::cerr << "Optimizer " << name << " not found" << std::endl;
    return false;
  }
  return true;
}

void Tracker::ClearOptimizers() {
  set_up_ = false;
  optimizer_ptrs_.clear();
}

bool Tracker::AddDetector(const std::shared_ptr<Detector> &detector_ptr) {
  set_up_ = false;
  if (!AddPtrIfNameNotExists(detector_ptr, &detector_ptrs_)) {
    std::cerr << "Detector " << detector_ptr->name() << " already exists"
              << std::endl;
    return false;
  }
  return true;
}

bool Tracker::DeleteDetector(const std::string &name) {
  set_up_ = false;
  if (!DeletePtrIfNameExists(name, &detector_ptrs_)) {
    std::cerr << "Detector " << name << " not found" << std::endl;
    return false;
  }
  return true;
}

void Tracker::ClearDetectors() {
  set_up_ = false;
  detector_ptrs_.clear();
}

bool Tracker::AddRefiner(const std::shared_ptr<Refiner> &refiner_ptr) {
  set_up_ = false;
  if (!AddPtrIfNameNotExists(refiner_ptr, &refiner_ptrs_)) {
    std::cerr << "Refiner " << refiner_ptr->name() << " already exists"
              << std::endl;
    return false;
  }
  return true;
}

bool Tracker::DeleteRefiner(const std::string &name) {
  set_up_ = false;
  if (!DeletePtrIfNameExists(name, &refiner_ptrs_)) {
    std::cerr << "Refiner " << name << " not found" << std::endl;
    return false;
  }
  return true;
}

void Tracker::ClearRefiners() {
  set_up_ = false;
  refiner_ptrs_.clear();
}

bool Tracker::AddViewer(const std::shared_ptr<Viewer> &viewer_ptr) {
  set_up_ = false;
  if (!AddPtrIfNameNotExists(viewer_ptr, &viewer_ptrs_)) {
    std::cerr << "Viewer " << viewer_ptr->name() << " already exists"
              << std::endl;
    return false;
  }
  return true;
}

bool Tracker::DeleteViewer(const std::string &name) {
  set_up_ = false;
  if (!DeletePtrIfNameExists(name, &viewer_ptrs_)) {
    std::cerr << "Viewer " << name << " not found" << std::endl;
    return false;
  }
  return true;
}

void Tracker::ClearViewers() {
  set_up_ = false;
  viewer_ptrs_.clear();
}

bool Tracker::AddPublisher(const std::shared_ptr<Publisher> &publisher_ptr) {
  set_up_ = false;
  if (!AddPtrIfNameNotExists(publisher_ptr, &publisher_ptrs_)) {
    std::cerr << "Publisher " << publisher_ptr->name() << " already exists"
              << std::endl;
    return false;
  }
  return true;
}

bool Tracker::DeletePublisher(const std::string &name) {
  set_up_ = false;
  if (!DeletePtrIfNameExists(name, &publisher_ptrs_)) {
    std::cerr << "Publisher " << name << " not found" << std::endl;
    return false;
  }
  return true;
}

void Tracker::ClearPublishers() {
  set_up_ = false;
  publisher_ptrs_.clear();
}

void Tracker::set_name(const std::string &name) { name_ = name; }

void Tracker::set_metafile_path(const std::filesystem::path &metafile_path) {
  metafile_path_ = metafile_path;
  set_up_ = false;
}

void Tracker::set_n_corr_iterations(int n_corr_iterations) {
  n_corr_iterations_ = n_corr_iterations;
}

void Tracker::set_n_update_iterations(int n_update_iterations) {
  n_update_iterations_ = n_update_iterations;
}

void Tracker::set_synchronize_cameras(bool synchronize_cameras) {
  synchronize_cameras_ = synchronize_cameras;
}

void Tracker::set_cycle_duration(
    const std::chrono::milliseconds &cycle_duration) {
  cycle_duration_ = cycle_duration;
}

void Tracker::set_visualization_time(int visualization_time) {
  visualization_time_ = visualization_time;
}

void Tracker::set_viewer_time(int viewer_time) { viewer_time_ = viewer_time; }

bool Tracker::RunTrackerProcess(bool execute_detection, bool start_tracking) {
  if (!set_up_) {
    std::cerr << "Set up tracker " << name_ << " first" << std::endl;
    return false;
  }

  tracking_started_ = false;
  quit_tracker_process_ = false;
  execute_detection_ = execute_detection;
  start_tracking_ = start_tracking;
  for (int iteration = 0;; ++iteration) {
    auto begin{std::chrono::high_resolution_clock::now()};
    if (!UpdateCameras(execute_detection_)) return false;
    if (execute_detection_) {
      if (!ExecuteDetectionCycle(iteration)) return false;
      tracking_started_ = false;
      execute_detection_ = false;
    }
    if (start_tracking_) {
      if (!StartModalities(iteration)) return false;
      tracking_started_ = true;
      start_tracking_ = false;
    }
    if (tracking_started_) {
      if (!ExecuteTrackingCycle(iteration)) return false;
    }
    if (!UpdateViewers(iteration)) return false;
    if (quit_tracker_process_) return true;
    if (!synchronize_cameras_) WaitUntilCycleEnds(begin);
  }
  return true;
}

void Tracker::QuitTrackerProcess() { quit_tracker_process_ = true; }

void Tracker::ExecuteDetection(bool start_tracking) {
  execute_detection_ = true;
  start_tracking_ = start_tracking;
}

void Tracker::StartTracking() { start_tracking_ = true; }

void Tracker::StopTracking() { tracking_started_ = false; }

bool Tracker::ExecuteDetectionCycle(int iteration) {
  if (!DetectBodies()) return false;
  return RefinePoses();
}

bool Tracker::StartModalities(int iteration) {
  for (auto &start_modality_renderer_ptr : start_modality_renderer_ptrs_) {
    if (!start_modality_renderer_ptr->StartRendering()) return false;
  }
  for (auto &modality_ptr : modality_ptrs_) {
    if (!modality_ptr->StartModality(iteration, 0)) return false;
  }
  return true;
}

bool Tracker::ExecuteTrackingCycle(int iteration) {
  for (int corr_iteration = 0; corr_iteration < n_corr_iterations_;
       ++corr_iteration) {
    int corr_save_idx = iteration * n_corr_iterations_ + corr_iteration;
    if (!CalculateCorrespondences(iteration, corr_iteration)) return false;
    if (!VisualizeCorrespondences(corr_save_idx)) return false;
    for (int update_iteration = 0; update_iteration < n_update_iterations_;
         ++update_iteration) {
      int update_save_idx =
          corr_save_idx * n_update_iterations_ + update_iteration;
      if (!CalculateGradientAndHessian(iteration, corr_iteration,
                                       update_iteration))
        return false;
      if (!CalculateOptimization(iteration, corr_iteration, update_iteration))
        return false;
      if (!VisualizeOptimization(update_save_idx)) return false;
    }
  }
  if (!CalculateResults(iteration)) return false;
  if (!VisualizeResults(iteration)) return false;
  return UpdatePublishers(iteration);
}

bool Tracker::DetectBodies() {
  for (auto &detector_ptr : detector_ptrs_) {
    if (!detector_ptr->DetectBody()) return false;
  }
  return true;
}

bool Tracker::RefinePoses() {
  for (auto &refiner_ptr : refiner_ptrs_) {
    if (!refiner_ptr->RefinePoses()) return false;
  }
  return true;
}

bool Tracker::UpdateCameras(bool update_all_cameras) {
  if (update_all_cameras) {
    for (auto &camera_ptr : all_camera_ptrs_) {
      if (!camera_ptr->UpdateImage(synchronize_cameras_)) return false;
    }
  } else {
    for (auto &camera_ptr : main_camera_ptrs_) {
      if (!camera_ptr->UpdateImage(synchronize_cameras_)) return false;
    }
  }
  return true;
}

bool Tracker::CalculateCorrespondences(int iteration, int corr_iteration) {
  for (auto &correspondence_renderer_ptr : correspondence_renderer_ptrs_) {
    if (!correspondence_renderer_ptr->StartRendering()) return false;
  }
  for (auto &modality_ptr : modality_ptrs_) {
    if (!modality_ptr->CalculateCorrespondences(iteration, corr_iteration))
      return false;
  }
  return true;
}

bool Tracker::VisualizeCorrespondences(int save_idx) {
  bool imshow_correspondences = false;
  for (auto &modality_ptr : modality_ptrs_) {
    if (!modality_ptr->VisualizeCorrespondences(save_idx)) return false;
    if (modality_ptr->imshow_correspondence()) imshow_correspondences = true;
  }
  if (imshow_correspondences) {
    if (cv::waitKey(visualization_time_) == 'q') return false;
  }
  return true;
}

bool Tracker::CalculateGradientAndHessian(int iteration, int corr_iteration,
                                          int update_iteration) {
  for (auto &modality_ptr : modality_ptrs_) {
    if (!modality_ptr->CalculateGradientAndHessian(iteration, corr_iteration,
                                                   update_iteration))
      return false;
  }
  return true;
}

bool Tracker::CalculateOptimization(int iteration, int corr_iteration,
                                    int update_iteration) {
  for (auto &optimizer_ptr : optimizer_ptrs_) {
    if (!optimizer_ptr->CalculateOptimization(iteration, corr_iteration,
                                              update_iteration))
      return false;
  }
  return true;
}

bool Tracker::VisualizeOptimization(int save_idx) {
  bool imshow_pose_update = false;
  for (auto &modality_ptr : modality_ptrs_) {
    if (!modality_ptr->VisualizeOptimization(save_idx)) return false;
    if (modality_ptr->imshow_optimization()) imshow_pose_update = true;
  }
  if (imshow_pose_update) {
    if (cv::waitKey(visualization_time_) == 'q') return false;
  }
  return true;
}

bool Tracker::CalculateResults(int iteration) {
  for (auto &results_renderer_ptr : results_renderer_ptrs_) {
    if (!results_renderer_ptr->StartRendering()) return false;
  }
  for (auto &modality_ptr : modality_ptrs_) {
    if (!modality_ptr->CalculateResults(iteration)) return false;
  }
  return true;
}

bool Tracker::VisualizeResults(int save_idx) {
  bool imshow_result = false;
  for (auto &modality_ptr : modality_ptrs_) {
    if (!modality_ptr->VisualizeResults(save_idx)) return false;
    if (modality_ptr->imshow_result()) imshow_result = true;
  }
  if (imshow_result) {
    if (cv::waitKey(visualization_time_) == 'q') return false;
  }
  return true;
}

bool Tracker::UpdatePublishers(int iteration) {
  for (auto &publisher_ptr : publisher_ptrs_) {
    if (!publisher_ptr->UpdatePublisher(iteration)) return false;
  }
  return true;
}

bool Tracker::UpdateViewers(int iteration) {
  if (!viewer_ptrs_.empty()) {
    for (auto &viewer_ptr : viewer_ptrs_) {
      viewer_ptr->UpdateViewer(iteration);
    }
    char key = cv::waitKey(viewer_time_);
    if (key == 'd') {
      execute_detection_ = true;
    } else if (key == 'x') {
      execute_detection_ = true;
      start_tracking_ = true;
    } else if (key == 't') {
      start_tracking_ = true;
    } else if (key == 's') {
      tracking_started_ = false;
    } else if (key == 'q') {
      quit_tracker_process_ = true;
    }
  }
  return true;
}

const std::string &Tracker::name() const { return name_; }

const std::filesystem::path &Tracker::metafile_path() const {
  return metafile_path_;
}

const std::vector<std::shared_ptr<Optimizer>> &Tracker::optimizer_ptrs() const {
  return optimizer_ptrs_;
}

const std::vector<std::shared_ptr<Detector>> &Tracker::detector_ptrs() const {
  return detector_ptrs_;
}

const std::vector<std::shared_ptr<Refiner>> &Tracker::refiner_ptrs() const {
  return refiner_ptrs_;
}

const std::vector<std::shared_ptr<Viewer>> &Tracker::viewer_ptrs() const {
  return viewer_ptrs_;
}

const std::vector<std::shared_ptr<Publisher>> &Tracker::publisher_ptrs() const {
  return publisher_ptrs_;
}

const std::vector<std::shared_ptr<Modality>> &Tracker::modality_ptrs() const {
  return modality_ptrs_;
}

const std::vector<std::shared_ptr<Model>> &Tracker::model_ptrs() const {
  return model_ptrs_;
}

const std::vector<std::shared_ptr<Camera>> &Tracker::main_camera_ptrs() const {
  return main_camera_ptrs_;
}

const std::vector<std::shared_ptr<Camera>> &Tracker::all_camera_ptrs() const {
  return all_camera_ptrs_;
}

const std::vector<std::shared_ptr<RendererGeometry>>
    &Tracker::renderer_geometry_ptrs() const {
  return renderer_geometry_ptrs_;
}

const std::vector<std::shared_ptr<Body>> &Tracker::body_ptrs() const {
  return body_ptrs_;
}

const std::vector<std::shared_ptr<Renderer>>
    &Tracker::start_modality_renderer_ptrs() const {
  return start_modality_renderer_ptrs_;
}

const std::vector<std::shared_ptr<Renderer>>
    &Tracker::correspondence_renderer_ptrs() const {
  return correspondence_renderer_ptrs_;
}

const std::vector<std::shared_ptr<Renderer>> &Tracker::results_renderer_ptrs()
    const {
  return results_renderer_ptrs_;
}

int Tracker::n_corr_iterations() const { return n_corr_iterations_; }

int Tracker::n_update_iterations() const { return n_update_iterations_; }

bool Tracker::synchronize_cameras() const { return synchronize_cameras_; }

const std::chrono::milliseconds &Tracker::cycle_duration() const {
  return cycle_duration_;
}

int Tracker::visualization_time() const { return visualization_time_; }

int Tracker::viewer_time() const { return viewer_time_; }

bool Tracker::set_up() const { return set_up_; }

bool Tracker::LoadMetaData() {
  // Open file storage from yaml
  cv::FileStorage fs;
  if (!OpenYamlFileStorage(metafile_path_, &fs)) return false;

  // Read parameters from yaml
  int i_cycle_duration;
  ReadOptionalValueFromYaml(fs, "n_corr_iterations", &n_corr_iterations_);
  ReadOptionalValueFromYaml(fs, "n_update_iterations", &n_update_iterations_);
  ReadOptionalValueFromYaml(fs, "synchronize_cameras", &synchronize_cameras_);
  ReadOptionalValueFromYaml(fs, "cycle_duration", &i_cycle_duration);
  ReadOptionalValueFromYaml(fs, "visualization_time", &visualization_time_);
  ReadOptionalValueFromYaml(fs, "viewer_time", &viewer_time_);
  cycle_duration_ = std::chrono::milliseconds{i_cycle_duration};
  fs.release();
  return true;
}

void Tracker::WaitUntilCycleEnds(
    std::chrono::high_resolution_clock::time_point begin) {
  auto elapsed_time{std::chrono::duration_cast<std::chrono::milliseconds>(
      std::chrono::high_resolution_clock::now() - begin)};
  if (elapsed_time < cycle_duration_)
    std::this_thread::sleep_for(elapsed_time - cycle_duration_);
  else
    std::cerr << "Tracker too slow: elapsed time = " << elapsed_time.count()
              << " ms > " << cycle_duration_.count() << " ms" << std::endl;
}

void Tracker::AssembleDerivedObjectPtrs() {
  modality_ptrs_.clear();
  main_camera_ptrs_.clear();
  all_camera_ptrs_.clear();
  model_ptrs_.clear();
  start_modality_renderer_ptrs_.clear();
  results_renderer_ptrs_.clear();
  correspondence_renderer_ptrs_.clear();
  renderer_geometry_ptrs_.clear();
  body_ptrs_.clear();

  // Assemble objects from detectors
  for (auto &detector_ptr : detector_ptrs_) {
    AddPtrIfNameNotExists(detector_ptr->camera_ptr(), &all_camera_ptrs_);
    AddPtrsIfNameNotExists(detector_ptr->body_ptrs(), &body_ptrs_);
  }

  // Assemble required objects from refiner
  for (auto &refiner_ptr : refiner_ptrs_) {
    for (auto &optimizer_ptr : refiner_ptr->optimizer_ptrs()) {
      for (auto &modality_ptr : optimizer_ptr->modality_ptrs()) {
        AddPtrsIfNameNotExists(modality_ptr->camera_ptrs(), &all_camera_ptrs_);
      }
    }
  }

  // Assemble objects from viewers
  for (auto &viewer_ptr : viewer_ptrs_) {
    AddPtrIfNameNotExists(viewer_ptr->camera_ptr(), &main_camera_ptrs_);
    AddPtrIfNameNotExists(viewer_ptr->camera_ptr(), &all_camera_ptrs_);
    AddPtrIfNameNotExists(viewer_ptr->renderer_geometry_ptr(),
                          &renderer_geometry_ptrs_);
  }

  // Assemble objects from optimizer
  for (auto &optimizer_ptr : optimizer_ptrs_) {
    AddPtrsIfNameNotExists(optimizer_ptr->modality_ptrs(), &modality_ptrs_);
  }

  // Assemble objects from modalities
  for (auto &modality_ptr : modality_ptrs_) {
    AddPtrsIfNameNotExists(modality_ptr->camera_ptrs(), &main_camera_ptrs_);
    AddPtrsIfNameNotExists(modality_ptr->camera_ptrs(), &all_camera_ptrs_);

    AddPtrIfNameNotExists(modality_ptr->model_ptr(), &model_ptrs_);

    AddPtrsIfNameNotExists(modality_ptr->start_modality_renderer_ptrs(),
                           &start_modality_renderer_ptrs_);
    AddPtrsIfNameNotExists(modality_ptr->correspondence_renderer_ptrs(),
                           &correspondence_renderer_ptrs_);
    AddPtrsIfNameNotExists(modality_ptr->results_renderer_ptrs(),
                           &results_renderer_ptrs_);

    AddPtrIfNameNotExists(modality_ptr->body_ptr(), &body_ptrs_);
  }

  // Assemble objects from models
  for (auto &model_ptr : model_ptrs_) {
    AddPtrIfNameNotExists(model_ptr->body_ptr(), &body_ptrs_);
  }

  // Assemble objects from renderers
  for (auto &start_modality_renderer_ptr : start_modality_renderer_ptrs_) {
    AddPtrIfNameNotExists(start_modality_renderer_ptr->renderer_geometry_ptr(),
                          &renderer_geometry_ptrs_);
    AddPtrsIfNameNotExists(start_modality_renderer_ptr->referenced_body_ptrs(),
                           &body_ptrs_);
  }
  for (auto &correspondence_renderer_ptr : correspondence_renderer_ptrs_) {
    AddPtrIfNameNotExists(correspondence_renderer_ptr->renderer_geometry_ptr(),
                          &renderer_geometry_ptrs_);
    AddPtrsIfNameNotExists(correspondence_renderer_ptr->referenced_body_ptrs(),
                           &body_ptrs_);
  }
  for (auto results_renderer_ptr : results_renderer_ptrs_) {
    AddPtrIfNameNotExists(results_renderer_ptr->renderer_geometry_ptr(),
                          &renderer_geometry_ptrs_);
    AddPtrsIfNameNotExists(results_renderer_ptr->referenced_body_ptrs(),
                           &body_ptrs_);
  }

  // Assemble objects from renderer geometry
  for (auto &renderer_geometry_ptr : renderer_geometry_ptrs_) {
    AddPtrsIfNameNotExists(renderer_geometry_ptr->body_ptrs(), &body_ptrs_);
  }
}

bool Tracker::SetUpAllObjects() {
  return SetUpObjectPtrs(&body_ptrs_) &&
         SetUpObjectPtrs(&renderer_geometry_ptrs_) &&
         SetUpObjectPtrs(&all_camera_ptrs_) &&
         SetUpObjectPtrs(&start_modality_renderer_ptrs_) &&
         SetUpObjectPtrs(&correspondence_renderer_ptrs_) &&
         SetUpObjectPtrs(&results_renderer_ptrs_) &&
         SetUpObjectPtrs(&model_ptrs_) && SetUpObjectPtrs(&modality_ptrs_) &&
         SetUpObjectPtrs(&optimizer_ptrs_) && SetUpObjectPtrs(&viewer_ptrs_) &&
         SetUpObjectPtrs(&refiner_ptrs_) && SetUpObjectPtrs(&detector_ptrs_) &&
         SetUpObjectPtrs(&publisher_ptrs_);
}

bool Tracker::AreAllObjectsSetUp() {
  return AreObjectPtrsSetUp(&body_ptrs_) &&
         AreObjectPtrsSetUp(&renderer_geometry_ptrs_) &&
         AreObjectPtrsSetUp(&all_camera_ptrs_) &&
         AreObjectPtrsSetUp(&start_modality_renderer_ptrs_) &&
         AreObjectPtrsSetUp(&correspondence_renderer_ptrs_) &&
         AreObjectPtrsSetUp(&results_renderer_ptrs_) &&
         AreObjectPtrsSetUp(&model_ptrs_) &&
         AreObjectPtrsSetUp(&modality_ptrs_) &&
         AreObjectPtrsSetUp(&optimizer_ptrs_) &&
         AreObjectPtrsSetUp(&viewer_ptrs_) &&
         AreObjectPtrsSetUp(&refiner_ptrs_) &&
         AreObjectPtrsSetUp(&detector_ptrs_) &&
         AreObjectPtrsSetUp(&publisher_ptrs_);
}

}  // namespace icg
