// SPDX-License-Identifier: MIT
// Copyright (c) 2020 Manuel Stoiber, German Aerospace Center (DLR)

#ifndef OBJECT_TRACKING_INCLUDE_RBGT_TRACKER_H_
#define OBJECT_TRACKING_INCLUDE_RBGT_TRACKER_H_

#include <rbgt/body.h>
#include <rbgt/camera.h>
#include <rbgt/common.h>
#include <rbgt/occlusion_mask_renderer.h>
#include <rbgt/region_modality.h>
#include <rbgt/renderer_geometry.h>
#include <rbgt/viewer.h>

#include <memory>
#include <vector>

namespace rbgt {

// Class that holds multiple region modalities and viewers for visualization. It
// coordinates all objects and referenced objects for continuous tracking. Also,
// it defines how many correspondence iterations and pose update iterations are
// executed and implements functionality that defines how long visualizations
// are shown.
class Tracker {
 public:
  // Constructor and setter methods
  void AddRegionModality(std::shared_ptr<RegionModality> region_modality_ptr);
  void AddViewer(std::shared_ptr<Viewer> viewer_ptr);
  void set_n_corr_iterations(int n_corr_iterations);
  void set_n_update_iterations(int n_update_iterations);
  void set_visualization_time(int visualization_time);
  void set_viewer_time(int viewer_time);

  // Main method
  void StartTracker(bool start_tracking);

  // Methods for advanced use
  void SetUpObjects();
  bool ExecuteViewingCycle(int iteration);
  bool ExecuteTrackingCycle(int iteration);

  // Individual steps of tracking cycle for advanced use
  bool StartRegionModalities();
  bool CalculateBeforeCameraUpdate();
  bool UpdateCameras();
  bool StartOcclusionMaskRendering();
  bool CalculateCorrespondences(int corr_iteration);
  bool VisualizeCorrespondences(int save_idx);
  bool CalculatePoseUpdate();
  bool VisualizePoseUpdate(int save_idx);
  bool VisualizeResults(int save_idx);
  bool UpdateViewers(int save_idx);

  // Getters
  std::vector<std::shared_ptr<RegionModality>> region_modality_ptrs() const;
  std::vector<std::shared_ptr<Viewer>> viewer_ptrs() const;
  int n_corr_iterations() const;
  int n_update_iterations() const;
  int visualization_time() const;
  int viewer_time() const;

 private:
  // Objects
  std::vector<std::shared_ptr<RegionModality>> region_modality_ptrs_;
  std::vector<std::shared_ptr<Viewer>> viewer_ptrs_;
  std::vector<std::shared_ptr<Camera>> camera_ptrs_;
  std::vector<std::shared_ptr<OcclusionMaskRenderer>>
      occlusion_mask_renderer_ptrs_;

  // Parameters
  int n_corr_iterations_ = 7;
  int n_update_iterations_ = 2;
  int visualization_time_ = 0;
  int viewer_time_ = 1;

  // State variables
  bool start_tracking_ = false;
  bool tracking_started_ = false;
};

template <typename T>
void AddPtrIfNameNotExists(T &&ptr, std::vector<T> *ptrs) {
  if (std::none_of(begin(*ptrs), end(*ptrs),
                   [&ptr](const T &p) { return p->name() == ptr->name(); })) {
    ptrs->push_back(std::move(ptr));
  }
}

}  // namespace rbgt

#endif  // OBJECT_TRACKING_INCLUDE_RBGT_TRACKER_H_
