// SPDX-License-Identifier: MIT
// Copyright (c) 2023 Manuel Stoiber, German Aerospace Center (DLR)

#include <m3t/tracker.h>

namespace m3t {

Tracker::Tracker(const std::string &name, int n_corr_iterations,
                 int n_update_iterations, bool synchronize_cameras,
                 bool start_tracking_after_detection,
                 const std::chrono::milliseconds &cycle_duration,
                 int visualization_time, int viewer_time)
    : name_{name},
      n_corr_iterations_{n_corr_iterations},
      n_update_iterations_{n_update_iterations},
      synchronize_cameras_{synchronize_cameras},
      start_tracking_after_detection_{start_tracking_after_detection},
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
  InitInternallyUsedObjectPtrs();
  ExtractAllOptimizerNames();
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

bool Tracker::AddSubscriber(const std::shared_ptr<Subscriber> &subscriber_ptr) {
  set_up_ = false;
  if (!AddPtrIfNameNotExists(subscriber_ptr, &subscriber_ptrs_)) {
    std::cerr << "Subscriber " << subscriber_ptr->name() << " already exists"
              << std::endl;
    return false;
  }
  return true;
}

bool Tracker::DeleteSubscriber(const std::string &name) {
  set_up_ = false;
  if (!DeletePtrIfNameExists(name, &subscriber_ptrs_)) {
    std::cerr << "Subscriber " << name << " not found" << std::endl;
    return false;
  }
  return true;
}

void Tracker::ClearSubscribers() {
  set_up_ = false;
  subscriber_ptrs_.clear();
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

void Tracker::set_start_tracking_after_detection(
    bool start_tracking_after_detection) {
  start_tracking_after_detection_ = start_tracking_after_detection;
}

void Tracker::set_cycle_duration(
    const std::chrono::milliseconds &cycle_duration) {
  cycle_duration_ = cycle_duration;
}

void Tracker::set_visualization_time(int visualization_time) {
  visualization_time_ = visualization_time;
}

void Tracker::set_viewer_time(int viewer_time) { viewer_time_ = viewer_time; }

bool Tracker::RunTrackerProcess(bool execute_detection, bool start_tracking,
                                const std::set<std::string> *names_detecting,
                                const std::set<std::string> *names_starting) {
  if (!set_up_) {
    std::cerr << "Set up tracker " << name_ << " first" << std::endl;
    return false;
  }

  // Define and validate names
  names_detecting_ = {};
  names_starting_ = {};
  names_tracking_ = {};
  if (execute_detection) {
    names_detecting_ = names_all_;
    if (names_detecting) names_detecting_ = *names_detecting;
  }
  if (start_tracking) {
    names_starting_ = names_all_;
    if (names_starting) names_starting_ = *names_starting;
  }
  ValidateNames();
  AssambleInternallyUsedObjectPtrs();

  // Run tracker process
  quit_tracker_process_ = false;
  for (int iteration = 0;; ++iteration) {
    auto begin{std::chrono::high_resolution_clock::now()};
    if (!UpdateCameras(iteration)) return false;
    if (!UpdateSubscribers(iteration)) return false;
    if (!CalculateConsistentPoses()) return false;
    tracking_mutex_.lock();
    if (!ExecuteDetectingStep(iteration)) return false;
    if (!ExecuteStartingStep(iteration)) return false;
    if (!ExecuteTrackingStep(iteration)) return false;
    tracking_mutex_.unlock();
    if (!UpdatePublishers(iteration)) return false;
    if (!UpdateViewers(iteration)) return false;
    if (quit_tracker_process_) return true;
    if (!synchronize_cameras_) WaitUntilCycleEnds(begin);
  }
  return true;
}

void Tracker::QuitTrackerProcess() { quit_tracker_process_ = true; }

void Tracker::ExecuteDetection(bool start_tracking,
                               const std::set<std::string> *names_detecting,
                               const std::set<std::string> *names_starting) {
  const std::lock_guard<std::mutex> lock{tracking_mutex_};
  names_detecting_ = names_all_;
  if (names_detecting) names_detecting_ = *names_detecting;
  if (start_tracking) {
    names_starting_ = names_all_;
    if (names_starting) names_starting_ = *names_starting;
  }
  ValidateNames();
  AssambleInternallyUsedObjectPtrs();
}

void Tracker::StartTracking(const std::set<std::string> *names_starting) {
  const std::lock_guard<std::mutex> lock{tracking_mutex_};
  if (names_starting)
    names_starting_.insert(names_starting->begin(), names_starting->end());
  else
    names_starting_.insert(names_all_.begin(), names_all_.end());
  ValidateNames();
  AssambleInternallyUsedObjectPtrs();
}

void Tracker::StopTracking(const std::set<std::string> *names_stopping) {
  const std::lock_guard<std::mutex> lock{tracking_mutex_};
  if (names_stopping) {
    for (const auto &name_stopping : *names_stopping) {
      names_detecting_.erase(name_stopping);
      names_starting_.erase(name_stopping);
      names_tracking_.erase(name_stopping);
    }
  } else {
    names_detecting_.clear();
    names_starting_.clear();
    names_tracking_.clear();
  }
  AssambleInternallyUsedObjectPtrs();
}

bool Tracker::UpdateSubscribers(int iteration) {
  for (auto &subscriber_ptr : subscriber_ptrs_) {
    if (!subscriber_ptr->UpdateSubscriber(iteration)) return false;
  }
  return true;
}

bool Tracker::UpdateCameras(int iteration) {
  for (auto &camera_ptr : camera_ptrs_) {
    if (!camera_ptr->UpdateImage(synchronize_cameras_)) return false;
  }
  return true;
}

bool Tracker::ExecuteDetectingStep(int iteration) {
  if (names_detecting_.empty()) return true;
  MoveBackPoses(names_detecting_);
  std::set<std::string> names_detected;
  if (!DetectPoses(names_detecting_, &names_detected)) return false;
  if (!RefinePoses(names_detected)) return false;
  if (!CalculateConsistentPoses()) return false;
  if (start_tracking_after_detection_)
    names_starting_.insert(names_detected.begin(), names_detected.end());
  for (const auto &name_detected : names_detected)
    names_detecting_.erase(name_detected);
  AssambleInternallyUsedObjectPtrs();
  return true;
}

bool Tracker::ExecuteStartingStep(int iteration) {
  if (names_starting_.empty()) return true;
  if (!StartModalities(iteration)) return false;
  names_tracking_.insert(names_starting_.begin(), names_starting_.end());
  names_starting_.clear();
  AssambleInternallyUsedObjectPtrs();
  return true;
}

bool Tracker::ExecuteTrackingStep(int iteration) {
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
  return VisualizeResults(iteration);
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
      ExecuteDetection(false);
    } else if (key == 'x') {
      ExecuteDetection(true);
    } else if (key == 't') {
      StartTracking();
    } else if (key == 's') {
      StopTracking();
    } else if (key == 'q') {
      quit_tracker_process_ = true;
    }
  }
  return true;
}

void Tracker::MoveBackPoses(const std::set<std::string> &names) {
  const m3t::Transform3fA background_pose{
      m3t::Transform3fA{Eigen::Translation3f{0.0f, 0.0f, -10.0f}}};
  for (auto &optimizer_ptr : optimizer_ptrs_) {
    if (names.find(optimizer_ptr->name()) != names.end()) {
      for (auto &link_ptr : optimizer_ptr->ReferencedLinks()) {
        link_ptr->set_link2world_pose(background_pose);
        if (link_ptr->body_ptr())
          link_ptr->body_ptr()->set_body2world_pose(background_pose);
      }
    }
  }
}

bool Tracker::DetectPoses(const std::set<std::string> &names,
                          std::set<std::string> *detected_names) {
  for (auto &detector_ptr : detecting_detector_ptrs_) {
    if (!detector_ptr->DetectPoses(names, detected_names)) return false;
  }
  return true;
}

bool Tracker::RefinePoses(const std::set<std::string> &names) {
  for (auto &refiner_ptr : detecting_refiner_ptrs_) {
    if (!refiner_ptr->RefinePoses(names)) return false;
  }
  return true;
}

bool Tracker::CalculateConsistentPoses() {
  for (auto &optimizer_ptr : optimizer_ptrs_) {
    if (!optimizer_ptr->CalculateConsistentPoses()) return false;
  }
  return true;
}

bool Tracker::StartModalities(int iteration) {
  for (auto &start_modality_renderer_ptr :
       starting_start_modality_renderer_ptrs_) {
    if (!start_modality_renderer_ptr->StartRendering()) return false;
  }
  for (auto &color_histograms_ptr : starting_color_histograms_ptrs_) {
    if (!color_histograms_ptr->ClearMemory()) return false;
  }
  for (auto &modality_ptr : starting_modality_ptrs_) {
    if (!modality_ptr->StartModality(iteration, 0)) return false;
  }
  for (auto &color_histograms_ptr : starting_color_histograms_ptrs_) {
    if (!color_histograms_ptr->InitializeHistograms()) return false;
  }
  return true;
}

bool Tracker::CalculateCorrespondences(int iteration, int corr_iteration) {
  for (auto &correspondence_renderer_ptr :
       tracking_correspondence_renderer_ptrs_) {
    if (!correspondence_renderer_ptr->StartRendering()) return false;
  }
  for (auto &modality_ptr : tracking_modality_ptrs_) {
    if (!modality_ptr->CalculateCorrespondences(iteration, corr_iteration))
      return false;
  }
  return true;
}

bool Tracker::VisualizeCorrespondences(int save_idx) {
  bool imshow_correspondences = false;
  for (auto &modality_ptr : tracking_modality_ptrs_) {
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
  for (auto &modality_ptr : tracking_modality_ptrs_) {
    if (!modality_ptr->CalculateGradientAndHessian(iteration, corr_iteration,
                                                   update_iteration))
      return false;
  }
  return true;
}

bool Tracker::CalculateOptimization(int iteration, int corr_iteration,
                                    int update_iteration) {
  for (auto &optimizer_ptr : tracking_optimizer_ptrs_) {
    if (!optimizer_ptr->CalculateOptimization(iteration, corr_iteration,
                                              update_iteration))
      return false;
  }
  return true;
}

bool Tracker::VisualizeOptimization(int save_idx) {
  bool imshow_pose_update = false;
  for (auto &modality_ptr : tracking_modality_ptrs_) {
    if (!modality_ptr->VisualizeOptimization(save_idx)) return false;
    if (modality_ptr->imshow_optimization()) imshow_pose_update = true;
  }
  if (imshow_pose_update) {
    if (cv::waitKey(visualization_time_) == 'q') return false;
  }
  return true;
}

bool Tracker::CalculateResults(int iteration) {
  for (auto &results_renderer_ptr : tracking_results_renderer_ptrs_) {
    if (!results_renderer_ptr->StartRendering()) return false;
  }
  for (auto &color_histograms_ptr : tracking_color_histograms_ptrs_) {
    if (!color_histograms_ptr->ClearMemory()) return false;
  }
  for (auto &modality_ptr : tracking_modality_ptrs_) {
    if (!modality_ptr->CalculateResults(iteration)) return false;
  }
  for (auto &color_histograms_ptr : tracking_color_histograms_ptrs_) {
    if (!color_histograms_ptr->UpdateHistograms()) return false;
  }
  return true;
}

bool Tracker::VisualizeResults(int save_idx) {
  bool imshow_result = false;
  for (auto &modality_ptr : tracking_modality_ptrs_) {
    if (!modality_ptr->VisualizeResults(save_idx)) return false;
    if (modality_ptr->imshow_result()) imshow_result = true;
  }
  if (imshow_result) {
    if (cv::waitKey(visualization_time_) == 'q') return false;
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

const std::vector<std::shared_ptr<Subscriber>> &Tracker::subscriber_ptrs()
    const {
  return subscriber_ptrs_;
}

const std::vector<std::shared_ptr<Link>> &Tracker::link_ptrs() const {
  return link_ptrs_;
}

const std::vector<std::shared_ptr<Constraint>> &Tracker::constraint_ptrs()
    const {
  return constraint_ptrs_;
}

const std::vector<std::shared_ptr<SoftConstraint>>
    &Tracker::soft_constraint_ptrs() const {
  return soft_constraint_ptrs_;
}

const std::vector<std::shared_ptr<Modality>> &Tracker::modality_ptrs() const {
  return modality_ptrs_;
}

const std::vector<std::shared_ptr<Model>> &Tracker::model_ptrs() const {
  return model_ptrs_;
}

const std::vector<std::shared_ptr<Camera>> &Tracker::camera_ptrs() const {
  return camera_ptrs_;
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

const std::vector<std::shared_ptr<ColorHistograms>>
    &Tracker::color_histograms_ptrs() const {
  return color_histograms_ptrs_;
}

int Tracker::n_corr_iterations() const { return n_corr_iterations_; }

int Tracker::n_update_iterations() const { return n_update_iterations_; }

bool Tracker::synchronize_cameras() const { return synchronize_cameras_; }

bool Tracker::start_tracking_after_detection() const {
  return start_tracking_after_detection_;
}

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
  ReadOptionalValueFromYaml(fs, "start_tracking_after_detection",
                            &start_tracking_after_detection_);
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

void Tracker::ValidateNames() {
  // Validate names_detecting
  auto unvalidated_names_detecting{names_detecting_};
  names_detecting_.clear();
  for (const auto &unvalidated_name_detecting : unvalidated_names_detecting) {
    for (const auto &detector_ptr : detector_ptrs_) {
      for (const auto &optimizer_ptr : detector_ptr->optimizer_ptrs()) {
        if (unvalidated_name_detecting == optimizer_ptr->name())
          names_detecting_.insert(unvalidated_name_detecting);
      }
    }
  }

  // Validate names_starting
  auto unvalidated_names_starting_ = names_starting_;
  names_starting_.clear();
  for (const auto &unvalidated_name_starting : unvalidated_names_starting_) {
    for (const auto &optimizer_ptr : optimizer_ptrs_) {
      if (unvalidated_name_starting == optimizer_ptr->name())
        names_starting_.insert(unvalidated_name_starting);
    }
  }

  // Validate names_tracking
  for (const auto &name_detecting : names_detecting_)
    names_tracking_.erase(name_detecting);
  for (const auto &name_starting : names_starting_)
    names_tracking_.erase(name_starting);
}

void Tracker::ExtractAllOptimizerNames() {
  names_all_.clear();
  for (const auto &optimizer_ptr : optimizer_ptrs_)
    names_all_.insert(optimizer_ptr->name());
}

void Tracker::InitInternallyUsedObjectPtrs() {
  detecting_detector_ptrs_ = detector_ptrs_;
  detecting_refiner_ptrs_ = refiner_ptrs_;
  starting_modality_ptrs_ = modality_ptrs_;
  starting_start_modality_renderer_ptrs_ = start_modality_renderer_ptrs_;
  starting_color_histograms_ptrs_ = color_histograms_ptrs_;
  tracking_optimizer_ptrs_ = optimizer_ptrs_;
  tracking_modality_ptrs_ = modality_ptrs_;
  tracking_correspondence_renderer_ptrs_ = correspondence_renderer_ptrs_;
  tracking_results_renderer_ptrs_ = results_renderer_ptrs_;
  tracking_color_histograms_ptrs_ = color_histograms_ptrs_;
}

void Tracker::AssambleInternallyUsedObjectPtrs() {
  // Assamble objects used for detecting
  detecting_detector_ptrs_.clear();
  for (const auto &detector_ptr : detector_ptrs_) {
    for (const auto &optimizer_ptr : detector_ptr->optimizer_ptrs()) {
      if (names_detecting_.find(optimizer_ptr->name()) !=
          names_detecting_.end()) {
        AddPtrIfNameNotExists(detector_ptr, &detecting_detector_ptrs_);
      }
    }
  }
  detecting_refiner_ptrs_.clear();
  for (const auto &refiner_ptr : refiner_ptrs_) {
    for (const auto &optimizer_ptr : refiner_ptr->optimizer_ptrs()) {
      if (names_detecting_.find(optimizer_ptr->name()) !=
          names_detecting_.end()) {
        AddPtrIfNameNotExists(refiner_ptr, &detecting_refiner_ptrs_);
      }
    }
  }

  // Assamble objects used for starting
  starting_modality_ptrs_.clear();
  starting_start_modality_renderer_ptrs_.clear();
  starting_color_histograms_ptrs_.clear();
  for (const auto &optimizer_ptr : optimizer_ptrs_) {
    if (names_detecting_.find(optimizer_ptr->name()) ==
            names_detecting_.end() &&
        names_starting_.find(optimizer_ptr->name()) != names_starting_.end()) {
      for (const auto &link_ptr : optimizer_ptr->ReferencedLinks()) {
        for (const auto &modality_ptr : link_ptr->modality_ptrs()) {
          AddPtrIfNameNotExists(modality_ptr, &starting_modality_ptrs_);
          AddPtrsIfNameNotExists(modality_ptr->start_modality_renderer_ptrs(),
                                 &starting_start_modality_renderer_ptrs_);
          AddPtrIfNameNotExists(modality_ptr->color_histograms_ptr(),
                                &starting_color_histograms_ptrs_);
        }
      }
    }
  }

  // Assamble objects used for tracking
  tracking_optimizer_ptrs_.clear();
  tracking_modality_ptrs_.clear();
  tracking_correspondence_renderer_ptrs_.clear();
  tracking_results_renderer_ptrs_.clear();
  tracking_color_histograms_ptrs_.clear();
  for (const auto &optimizer_ptr : optimizer_ptrs_) {
    if (names_tracking_.find(optimizer_ptr->name()) != names_tracking_.end()) {
      for (const auto &link_ptr : optimizer_ptr->ReferencedLinks()) {
        for (const auto &modality_ptr : link_ptr->modality_ptrs()) {
          AddPtrIfNameNotExists(optimizer_ptr, &tracking_optimizer_ptrs_);
          AddPtrIfNameNotExists(modality_ptr, &tracking_modality_ptrs_);
          AddPtrsIfNameNotExists(modality_ptr->correspondence_renderer_ptrs(),
                                 &tracking_correspondence_renderer_ptrs_);
          AddPtrsIfNameNotExists(modality_ptr->results_renderer_ptrs(),
                                 &tracking_results_renderer_ptrs_);
          AddPtrIfNameNotExists(modality_ptr->color_histograms_ptr(),
                                &tracking_color_histograms_ptrs_);
        }
      }
    }
  }
}

void Tracker::AssembleDerivedObjectPtrs() {
  soft_constraint_ptrs_.clear();
  constraint_ptrs_.clear();
  link_ptrs_.clear();
  modality_ptrs_.clear();
  camera_ptrs_.clear();
  model_ptrs_.clear();
  start_modality_renderer_ptrs_.clear();
  results_renderer_ptrs_.clear();
  correspondence_renderer_ptrs_.clear();
  renderer_geometry_ptrs_.clear();
  color_histograms_ptrs_.clear();
  body_ptrs_.clear();

  // Assemble objects from detectors
  for (auto &detector_ptr : detector_ptrs_) {
    AddPtrIfNameNotExists(detector_ptr->camera_ptr(), &camera_ptrs_);
    AddPtrsIfNameNotExists(detector_ptr->optimizer_ptrs(), &optimizer_ptrs_);
  }

  // Assemble required objects from refiner
  for (auto &refiner_ptr : refiner_ptrs_) {
    for (auto &optimizer_ptr : refiner_ptr->optimizer_ptrs()) {
      for (auto &link_ptr : optimizer_ptr->ReferencedLinks()) {
        for (auto &modality_ptr : link_ptr->modality_ptrs()) {
          AddPtrsIfNameNotExists(modality_ptr->camera_ptrs(), &camera_ptrs_);
        }
      }
    }
  }

  // Assemble objects from viewers
  for (auto &viewer_ptr : viewer_ptrs_) {
    AddPtrIfNameNotExists(viewer_ptr->camera_ptr(), &camera_ptrs_);
    AddPtrIfNameNotExists(viewer_ptr->renderer_geometry_ptr(),
                          &renderer_geometry_ptrs_);
  }

  // Assemble objects from optimizers
  for (auto &optimizer_ptr : optimizer_ptrs_) {
    AddPtrsIfNameNotExists(optimizer_ptr->soft_constraint_ptrs(),
                           &soft_constraint_ptrs_);
    AddPtrsIfNameNotExists(optimizer_ptr->constraint_ptrs(), &constraint_ptrs_);
    AddPtrsIfNameNotExists(optimizer_ptr->ReferencedLinks(), &link_ptrs_);
  }

  // Assemble objects from links
  for (auto &link_ptr : link_ptrs_) {
    AddPtrsIfNameNotExists(link_ptr->modality_ptrs(), &modality_ptrs_);
    AddPtrIfNameNotExists(link_ptr->body_ptr(), &body_ptrs_);
  }

  // Assemble objects from modalities
  for (auto &modality_ptr : modality_ptrs_) {
    AddPtrsIfNameNotExists(modality_ptr->camera_ptrs(), &camera_ptrs_);

    AddPtrIfNameNotExists(modality_ptr->model_ptr(), &model_ptrs_);

    AddPtrsIfNameNotExists(modality_ptr->start_modality_renderer_ptrs(),
                           &start_modality_renderer_ptrs_);
    AddPtrsIfNameNotExists(modality_ptr->correspondence_renderer_ptrs(),
                           &correspondence_renderer_ptrs_);
    AddPtrsIfNameNotExists(modality_ptr->results_renderer_ptrs(),
                           &results_renderer_ptrs_);

    AddPtrIfNameNotExists(modality_ptr->color_histograms_ptr(),
                          &color_histograms_ptrs_);

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
         SetUpObjectPtrs(&color_histograms_ptrs_) &&
         SetUpObjectPtrs(&renderer_geometry_ptrs_) &&
         SetUpObjectPtrs(&camera_ptrs_) &&
         SetUpObjectPtrs(&start_modality_renderer_ptrs_) &&
         SetUpObjectPtrs(&correspondence_renderer_ptrs_) &&
         SetUpObjectPtrs(&results_renderer_ptrs_) &&
         SetUpObjectPtrs(&model_ptrs_) && SetUpObjectPtrs(&modality_ptrs_) &&
         SetUpObjectPtrs(&link_ptrs_) && SetUpObjectPtrs(&constraint_ptrs_) &&
         SetUpObjectPtrs(&soft_constraint_ptrs_) &&
         SetUpObjectPtrs(&optimizer_ptrs_) && SetUpObjectPtrs(&viewer_ptrs_) &&
         SetUpObjectPtrs(&refiner_ptrs_) && SetUpObjectPtrs(&detector_ptrs_) &&
         SetUpObjectPtrs(&publisher_ptrs_) &&
         SetUpObjectPtrs(&subscriber_ptrs_);
}

bool Tracker::AreAllObjectsSetUp() {
  return AreObjectPtrsSetUp(&body_ptrs_) &&
         AreObjectPtrsSetUp(&color_histograms_ptrs_) &&
         AreObjectPtrsSetUp(&renderer_geometry_ptrs_) &&
         AreObjectPtrsSetUp(&camera_ptrs_) &&
         AreObjectPtrsSetUp(&start_modality_renderer_ptrs_) &&
         AreObjectPtrsSetUp(&correspondence_renderer_ptrs_) &&
         AreObjectPtrsSetUp(&results_renderer_ptrs_) &&
         AreObjectPtrsSetUp(&model_ptrs_) &&
         AreObjectPtrsSetUp(&modality_ptrs_) &&
         AreObjectPtrsSetUp(&link_ptrs_) &&
         AreObjectPtrsSetUp(&constraint_ptrs_) &&
         AreObjectPtrsSetUp(&soft_constraint_ptrs_) &&
         AreObjectPtrsSetUp(&optimizer_ptrs_) &&
         AreObjectPtrsSetUp(&viewer_ptrs_) &&
         AreObjectPtrsSetUp(&refiner_ptrs_) &&
         AreObjectPtrsSetUp(&detector_ptrs_) &&
         AreObjectPtrsSetUp(&publisher_ptrs_) &&
         AreObjectPtrsSetUp(&subscriber_ptrs_);
}

}  // namespace m3t
