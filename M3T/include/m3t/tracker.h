// SPDX-License-Identifier: MIT
// Copyright (c) 2023 Manuel Stoiber, German Aerospace Center (DLR)

#ifndef M3T_INCLUDE_M3T_TRACKER_H_
#define M3T_INCLUDE_M3T_TRACKER_H_

#include <m3t/body.h>
#include <m3t/camera.h>
#include <m3t/color_histograms.h>
#include <m3t/common.h>
#include <m3t/constraint.h>
#include <m3t/detector.h>
#include <m3t/link.h>
#include <m3t/modality.h>
#include <m3t/optimizer.h>
#include <m3t/publisher.h>
#include <m3t/refiner.h>
#include <m3t/renderer_geometry.h>
#include <m3t/soft_constraint.h>
#include <m3t/subscriber.h>
#include <m3t/viewer.h>

#include <chrono>
#include <memory>
#include <set>
#include <string>
#include <thread>
#include <vector>

namespace m3t {

/**
 * \brief Class that coordinates \ref Refiner, \ref Publisher, \ref Subscriber,
 * \ref Detector, \ref Optimizer, \ref Modality, \ref Renderer, \ref Viewer, and
 * \ref Camera objects to track multiple \ref Body objects.
 *
 * \details Using the method `RunTrackerProcess()`, the tracker process is
 * started. With the parameters `execute_detection` and `start_tracking`, the
 * function allows to specify if detection and tracking should be started
 * automatically. If they are not started automatically, the user can press the
 * *D* key for the detection and the *T* key to start tracking. With *X*, both
 * the detection is executed and tracking is started. With *S* tracking stops
 * while *Q* quits tracking. In addition to pressing keys on the keyboard, the
 * user can call `QuitTrackerProcess()`, `ExecuteDetection()`,
 * `StartTracking()`, and `StopTracking()` from another thread to control the
 * tracker process. All methods allow to specify names of optimizers that should
 * be considered by the respective command. If no name is specified, all
 * optimizers are considered. During `SetUp()` all required objects like \ref
 * Link, \ref Constraint, \ref Modality, \ref Model, \ref Renderer, \ref
 * RendererGeometry, \ref ColorHistograms \ref Camera, and \ref Body objects are
 * derived from objects that are provided by the user. `SetUp()` calls the
 * `SetUp()` method of all referenced objects in the correct order.
 *
 * @param optimizer_ptrs referenced \ref Optimizer objects that are considered.
 * @param detector_ptrs referenced \ref Detector objects that are considered.
 * @param refiner_ptrs referenced \ref Refiner objects that are considered.
 * @param viewer_ptrs referenced \ref Viewer objects that are considered.
 * @param publisher_ptrs referenced \ref Publisher objects that are considered.
 * @param subscriber_ptrs referenced \ref Subscriber objects that are
 * considered.
 * @param n_corr_iterations number of times new correspondences are established.
 * @param n_update_iterations number of times the pose is updated for each
 * correspondence iteration.
 * @param synchronize_cameras when true, \ref Camera objects wait for new images
 * to arrive. Otherwise they do not wait. Instead, the tracker waits until the
 * `cycle_duration` has passed and starts a new cycle.
 * @param start_tracking_after_detection when true bodies are directly tracked
 * after a successful detection.
 * @param cycle_duration duration of a cycle if the flag `synchronize_cameras`
 * is set to false.
 * @param visualization_time time in milliseconds visualizations from \ref
 * Modality objects are shown. 0 corresponds to infinite time.
 * @param viewer_time time in milliseconds images from \ref Viewer objects are
 * shown. 0 corresponds to infinite time.
 */
class Tracker {
 public:
  // Constructor and setup method
  Tracker(const std::string &name, int n_corr_iterations = 5,
          int n_update_iterations = 2, bool synchronize_cameras = true,
          bool start_tracking_after_detection = false,
          const std::chrono::milliseconds &cycle_duration =
              std::chrono::milliseconds{33},
          int visualization_time = 0, int viewer_time = 1);
  Tracker(const std::string &name, const std::filesystem::path &metafile_path);
  bool SetUp(bool set_up_all_objects = true);

  // Configure optimizer, detector, refiner, viewer, publisher, and subscriber
  bool AddOptimizer(const std::shared_ptr<Optimizer> &optimizer_ptr);
  bool DeleteOptimizer(const std::string &name);
  void ClearOptimizers();
  bool AddDetector(const std::shared_ptr<Detector> &detector_ptr);
  bool DeleteDetector(const std::string &name);
  void ClearDetectors();
  bool AddRefiner(const std::shared_ptr<Refiner> &refiner_ptr);
  bool DeleteRefiner(const std::string &name);
  void ClearRefiners();
  bool AddViewer(const std::shared_ptr<Viewer> &viewer_ptr);
  bool DeleteViewer(const std::string &name);
  void ClearViewers();
  bool AddPublisher(const std::shared_ptr<Publisher> &publisher_ptr);
  bool DeletePublisher(const std::string &name);
  void ClearPublishers();
  bool AddSubscriber(const std::shared_ptr<Subscriber> &subscriber_ptr);
  bool DeleteSubscriber(const std::string &name);
  void ClearSubscribers();

  // Setters
  void set_name(const std::string &name);
  void set_metafile_path(const std::filesystem::path &metafile_path);
  void set_n_corr_iterations(int n_corr_iterations);
  void set_n_update_iterations(int n_update_iterations);
  void set_synchronize_cameras(bool synchronize_cameras);
  void set_start_tracking_after_detection(bool start_tracking_after_detection);
  void set_cycle_duration(const std::chrono::milliseconds &cycle_duration);
  void set_visualization_time(int visualization_time);
  void set_viewer_time(int viewer_time);

  // Main method
  bool RunTrackerProcess(bool execute_detection = false,
                         bool start_tracking = false,
                         const std::set<std::string> *names_detecting = nullptr,
                         const std::set<std::string> *names_starting = nullptr);
  void QuitTrackerProcess();
  void ExecuteDetection(bool start_tracking = false,
                        const std::set<std::string> *names_detecting = nullptr,
                        const std::set<std::string> *names_starting = nullptr);
  void StartTracking(const std::set<std::string> *names_starting = nullptr);
  void StopTracking(const std::set<std::string> *names_stopping = nullptr);

  // Methods of tracker process for advanced use
  bool UpdateSubscribers(int iteration);
  bool UpdateCameras(int iteration);
  bool ExecuteDetectingStep(int iteration);
  bool ExecuteStartingStep(int iteration);
  bool ExecuteTrackingStep(int iteration);
  bool UpdatePublishers(int iteration);
  bool UpdateViewers(int iteration);

  // Individual steps of detecting step for advanced use
  void MoveBackPoses(const std::set<std::string> &names);
  bool DetectPoses(const std::set<std::string> &names,
              std::set<std::string> *detected_names = nullptr);
  bool RefinePoses(const std::set<std::string> &names);
  bool CalculateConsistentPoses();

  // Individual steps of starting step for advanced use
  bool StartModalities(int iteration);

  // Individual steps of tracking step for advanced use
  bool CalculateCorrespondences(int iteration, int corr_iteration);
  bool VisualizeCorrespondences(int save_idx);
  bool CalculateGradientAndHessian(int iteration, int corr_iteration,
                                   int opt_iteration);
  bool CalculateOptimization(int iteration, int corr_iteration,
                             int opt_iteration);
  bool VisualizeOptimization(int save_idx);
  bool CalculateResults(int iteration);
  bool VisualizeResults(int save_idx);

  // Getters
  const std::string &name() const;
  const std::filesystem::path &metafile_path() const;
  const std::vector<std::shared_ptr<Optimizer>> &optimizer_ptrs() const;
  const std::vector<std::shared_ptr<Detector>> &detector_ptrs() const;
  const std::vector<std::shared_ptr<Refiner>> &refiner_ptrs() const;
  const std::vector<std::shared_ptr<Viewer>> &viewer_ptrs() const;
  const std::vector<std::shared_ptr<Publisher>> &publisher_ptrs() const;
  const std::vector<std::shared_ptr<Subscriber>> &subscriber_ptrs() const;
  const std::vector<std::shared_ptr<Link>> &link_ptrs() const;
  const std::vector<std::shared_ptr<Constraint>> &constraint_ptrs() const;
  const std::vector<std::shared_ptr<SoftConstraint>> &soft_constraint_ptrs()
      const;
  const std::vector<std::shared_ptr<Modality>> &modality_ptrs() const;
  const std::vector<std::shared_ptr<Model>> &model_ptrs() const;
  const std::vector<std::shared_ptr<Camera>> &camera_ptrs() const;
  const std::vector<std::shared_ptr<RendererGeometry>> &renderer_geometry_ptrs()
      const;
  const std::vector<std::shared_ptr<Body>> &body_ptrs() const;
  const std::vector<std::shared_ptr<Renderer>> &start_modality_renderer_ptrs()
      const;
  const std::vector<std::shared_ptr<Renderer>> &correspondence_renderer_ptrs()
      const;
  const std::vector<std::shared_ptr<Renderer>> &results_renderer_ptrs() const;
  const std::vector<std::shared_ptr<ColorHistograms>> &color_histograms_ptrs()
      const;
  int n_corr_iterations() const;
  int n_update_iterations() const;
  bool synchronize_cameras() const;
  bool start_tracking_after_detection() const;
  const std::chrono::milliseconds &cycle_duration() const;
  int visualization_time() const;
  int viewer_time() const;
  bool set_up() const;

 private:
  // Helper methods
  bool LoadMetaData();
  void WaitUntilCycleEnds(std::chrono::high_resolution_clock::time_point begin);
  void ValidateNames();
  void ExtractAllOptimizerNames();
  void InitInternallyUsedObjectPtrs();
  void AssambleInternallyUsedObjectPtrs();
  void AssembleDerivedObjectPtrs();
  bool SetUpAllObjects();
  bool AreAllObjectsSetUp();

  // Objects
  std::vector<std::shared_ptr<Optimizer>> optimizer_ptrs_{};
  std::vector<std::shared_ptr<Detector>> detector_ptrs_{};
  std::vector<std::shared_ptr<Refiner>> refiner_ptrs_{};
  std::vector<std::shared_ptr<Viewer>> viewer_ptrs_{};
  std::vector<std::shared_ptr<Publisher>> publisher_ptrs_{};
  std::vector<std::shared_ptr<Subscriber>> subscriber_ptrs_{};
  std::vector<std::shared_ptr<Constraint>> constraint_ptrs_{};
  std::vector<std::shared_ptr<SoftConstraint>> soft_constraint_ptrs_{};
  std::vector<std::shared_ptr<Link>> link_ptrs_{};
  std::vector<std::shared_ptr<Modality>> modality_ptrs_{};
  std::vector<std::shared_ptr<Model>> model_ptrs_{};
  std::vector<std::shared_ptr<Camera>> camera_ptrs_{};
  std::vector<std::shared_ptr<RendererGeometry>> renderer_geometry_ptrs_{};
  std::vector<std::shared_ptr<Body>> body_ptrs_{};
  std::vector<std::shared_ptr<Renderer>> start_modality_renderer_ptrs_{};
  std::vector<std::shared_ptr<Renderer>> correspondence_renderer_ptrs_{};
  std::vector<std::shared_ptr<Renderer>> results_renderer_ptrs_{};
  std::vector<std::shared_ptr<ColorHistograms>> color_histograms_ptrs_{};

  // Parameters
  std::string name_{};
  std::filesystem::path metafile_path_{};
  int n_corr_iterations_ = 5;
  int n_update_iterations_ = 2;
  bool synchronize_cameras_ = true;
  bool start_tracking_after_detection_ = false;
  std::chrono::milliseconds cycle_duration_{33};
  int visualization_time_ = 0;
  int viewer_time_ = 1;

  // Internally used objects
  std::vector<std::shared_ptr<Detector>> detecting_detector_ptrs_{};
  std::vector<std::shared_ptr<Refiner>> detecting_refiner_ptrs_{};
  std::vector<std::shared_ptr<Modality>> starting_modality_ptrs_{};
  std::vector<std::shared_ptr<Renderer>>
      starting_start_modality_renderer_ptrs_{};
  std::vector<std::shared_ptr<ColorHistograms>>
      starting_color_histograms_ptrs_{};
  std::vector<std::shared_ptr<Optimizer>> tracking_optimizer_ptrs_{};
  std::vector<std::shared_ptr<Modality>> tracking_modality_ptrs_{};
  std::vector<std::shared_ptr<Renderer>>
      tracking_correspondence_renderer_ptrs_{};
  std::vector<std::shared_ptr<Renderer>> tracking_results_renderer_ptrs_{};
  std::vector<std::shared_ptr<ColorHistograms>>
      tracking_color_histograms_ptrs_{};

  // State variables
  std::set<std::string> names_all_{};
  std::set<std::string> names_detecting_{};
  std::set<std::string> names_starting_{};
  std::set<std::string> names_tracking_{};
  bool quit_tracker_process_ = false;
  bool set_up_ = false;
  std::mutex tracking_mutex_{};
};

}  // namespace m3t

#endif  // M3T_INCLUDE_M3T_TRACKER_H_
