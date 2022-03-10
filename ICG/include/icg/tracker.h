// SPDX-License-Identifier: MIT
// Copyright (c) 2022 Manuel Stoiber, German Aerospace Center (DLR)

#ifndef ICG_INCLUDE_ICG_TRACKER_H_
#define ICG_INCLUDE_ICG_TRACKER_H_

#include <icg/body.h>
#include <icg/camera.h>
#include <icg/common.h>
#include <icg/detector.h>
#include <icg/modality.h>
#include <icg/optimizer.h>
#include <icg/publisher.h>
#include <icg/refiner.h>
#include <icg/renderer_geometry.h>
#include <icg/viewer.h>

#include <chrono>
#include <memory>
#include <string>
#include <thread>
#include <vector>

namespace icg {

/**
 * \brief Class that coordinates \ref Refiner, \ref Publisher, \ref Detector,
 * \ref Optimizer, \ref Modality, \ref Renderer, \ref Viewer, and \ref
 * Camera objects to track multiple \ref Body objects.
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
 * tracker process. During `SetUp()` all required objects like \ref Modality,
 * \ref Model, \ref Renderer, \ref RendererGeometry, \ref Camera, and \ref Body
 * objects are derived from objects that are provided by the user. `SetUp()`
 * calls the `SetUp()` method of all referenced objects in the correct order.
 *
 * @param optimizer_ptrs referenced \ref Optimizer objects that are considered.
 * @param detector_ptrs referenced \ref Detector objects that are considered.
 * @param refiner_ptrs referenced \ref Refiner objects that are considered.
 * @param viewer_ptrs referenced \ref Viewer objects that are considered.
 * @param publisher_ptrs referenced \ref Publisher objects that are considered.
 * @param n_corr_iterations number of times new correspondences are established.
 * @param n_update_iterations number of times the pose is updated for each
 * correspondence iteration.
 * @param synchronize_cameras when true, \ref Camera objects wait for new images
 * to arrive. Otherwise they do not wait. Instead, the tracker waits until the
 * `cycle_duration` has passed and starts a new cycle.
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
          const std::chrono::milliseconds &cycle_duration =
              std::chrono::milliseconds{33},
          int visualization_time = 0, int viewer_time = 1);
  Tracker(const std::string &name, const std::filesystem::path &metafile_path);
  bool SetUp(bool set_up_all_objects = true);

  // Configure optimizer, viewer, and publisher
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

  // Setters
  void set_name(const std::string &name);
  void set_metafile_path(const std::filesystem::path &metafile_path);
  void set_n_corr_iterations(int n_corr_iterations);
  void set_n_update_iterations(int n_update_iterations);
  void set_synchronize_cameras(bool synchronize_cameras);
  void set_cycle_duration(const std::chrono::milliseconds &cycle_duration);
  void set_visualization_time(int visualization_time);
  void set_viewer_time(int viewer_time);

  // Main method
  bool RunTrackerProcess(bool execute_detection = false,
                         bool start_tracking = false);
  void QuitTrackerProcess();
  void ExecuteDetection(bool start_tracking = false);
  void StartTracking();
  void StopTracking();

  // Methods for advanced use
  bool ExecuteDetectionCycle(int iteration);
  bool StartModalities(int iteration);
  bool ExecuteTrackingCycle(int iteration);

  // Individual steps of detection cycle for advanced use
  bool DetectBodies();
  bool RefinePoses();

  // Individual steps of tracking cycle for advanced use
  bool UpdateCameras(bool update_all_cameras);
  bool CalculateCorrespondences(int iteration, int corr_iteration);
  bool VisualizeCorrespondences(int save_idx);
  bool CalculateGradientAndHessian(int iteration, int corr_iteration,
                                   int opt_iteration);
  bool CalculateOptimization(int iteration, int corr_iteration,
                             int opt_iteration);
  bool VisualizeOptimization(int save_idx);
  bool CalculateResults(int iteration);
  bool VisualizeResults(int save_idx);
  bool UpdatePublishers(int iteration);
  bool UpdateViewers(int iteration);

  // Getters
  const std::string &name() const;
  const std::filesystem::path &metafile_path() const;
  const std::vector<std::shared_ptr<Optimizer>> &optimizer_ptrs() const;
  const std::vector<std::shared_ptr<Detector>> &detector_ptrs() const;
  const std::vector<std::shared_ptr<Refiner>> &refiner_ptrs() const;
  const std::vector<std::shared_ptr<Viewer>> &viewer_ptrs() const;
  const std::vector<std::shared_ptr<Publisher>> &publisher_ptrs() const;
  const std::vector<std::shared_ptr<Modality>> &modality_ptrs() const;
  const std::vector<std::shared_ptr<Model>> &model_ptrs() const;
  const std::vector<std::shared_ptr<Camera>> &main_camera_ptrs() const;
  const std::vector<std::shared_ptr<Camera>> &all_camera_ptrs() const;
  const std::vector<std::shared_ptr<RendererGeometry>> &renderer_geometry_ptrs()
      const;
  const std::vector<std::shared_ptr<Body>> &body_ptrs() const;
  const std::vector<std::shared_ptr<Renderer>> &start_modality_renderer_ptrs()
      const;
  const std::vector<std::shared_ptr<Renderer>> &correspondence_renderer_ptrs()
      const;
  const std::vector<std::shared_ptr<Renderer>> &results_renderer_ptrs() const;
  int n_corr_iterations() const;
  int n_update_iterations() const;
  bool synchronize_cameras() const;
  const std::chrono::milliseconds &cycle_duration() const;
  int visualization_time() const;
  int viewer_time() const;
  bool set_up() const;

 private:
  // Helper methods
  bool LoadMetaData();
  void WaitUntilCycleEnds(std::chrono::high_resolution_clock::time_point begin);
  void AssembleDerivedObjectPtrs();
  bool SetUpAllObjects();
  bool AreAllObjectsSetUp();

  // Objects
  std::vector<std::shared_ptr<Optimizer>> optimizer_ptrs_{};
  std::vector<std::shared_ptr<Detector>> detector_ptrs_{};
  std::vector<std::shared_ptr<Refiner>> refiner_ptrs_{};
  std::vector<std::shared_ptr<Viewer>> viewer_ptrs_{};
  std::vector<std::shared_ptr<Publisher>> publisher_ptrs_{};
  std::vector<std::shared_ptr<Modality>> modality_ptrs_{};
  std::vector<std::shared_ptr<Model>> model_ptrs_{};
  std::vector<std::shared_ptr<Camera>> main_camera_ptrs_{};  // For tracking
  std::vector<std::shared_ptr<Camera>> all_camera_ptrs_{};  // With detector cam
  std::vector<std::shared_ptr<RendererGeometry>> renderer_geometry_ptrs_{};
  std::vector<std::shared_ptr<Body>> body_ptrs_{};
  std::vector<std::shared_ptr<Renderer>> start_modality_renderer_ptrs_{};
  std::vector<std::shared_ptr<Renderer>> correspondence_renderer_ptrs_{};
  std::vector<std::shared_ptr<Renderer>> results_renderer_ptrs_{};

  // Parameters
  std::string name_{};
  std::filesystem::path metafile_path_{};
  int n_corr_iterations_ = 5;
  int n_update_iterations_ = 2;
  bool synchronize_cameras_ = true;
  std::chrono::milliseconds cycle_duration_{33};
  int visualization_time_ = 0;
  int viewer_time_ = 1;

  // State variables
  bool execute_detection_ = false;
  bool start_tracking_ = false;
  bool tracking_started_ = false;
  bool quit_tracker_process_ = false;
  bool set_up_ = false;
};

}  // namespace icg

#endif  // ICG_INCLUDE_ICG_TRACKER_H_
