// SPDX-License-Identifier: MIT
// Copyright (c) 2020 Manuel Stoiber, German Aerospace Center (DLR)

#include <rbgt/tracker.h>

namespace rbgt {

void Tracker::AddRegionModality(
    std::shared_ptr<RegionModality> region_modality_ptr) {
  region_modality_ptrs_.push_back(std::move(region_modality_ptr));
}

void Tracker::AddViewer(std::shared_ptr<Viewer> viewer_ptr) {
  viewer_ptrs_.push_back(std::move(viewer_ptr));
}

void Tracker::set_n_corr_iterations(int n_corr_iterations) {
  n_corr_iterations_ = n_corr_iterations;
}

void Tracker::set_n_update_iterations(int n_update_iterations) {
  n_update_iterations_ = n_update_iterations;
}

void Tracker::set_visualization_time(int visualization_time) {
  visualization_time_ = visualization_time;
}

void Tracker::set_viewer_time(int viewer_time) { viewer_time_ = viewer_time; }

void Tracker::StartTracker(bool start_tracking) {
  start_tracking_ = start_tracking;
  SetUpObjects();

  for (int iteration = 0;; ++iteration) {
    if (start_tracking_) {
      if (!StartRegionModalities()) return;
      tracking_started_ = true;
      start_tracking_ = false;
    }
    if (tracking_started_) {
      if (!ExecuteTrackingCycle(iteration)) return;
    } else {
      if (!ExecuteViewingCycle(iteration)) return;
    }
  }
}

void Tracker::SetUpObjects() {
  camera_ptrs_.clear();
  occlusion_mask_renderer_ptrs_.clear();
  for (auto &region_modality_ptr : region_modality_ptrs_) {
    if (region_modality_ptr->camera_ptr())
      AddPtrIfNameNotExists(region_modality_ptr->camera_ptr(), &camera_ptrs_);
    if (region_modality_ptr->occlusion_mask_renderer_ptr())
      AddPtrIfNameNotExists(region_modality_ptr->occlusion_mask_renderer_ptr(),
                            &occlusion_mask_renderer_ptrs_);
  }
  for (auto &viewer_ptr : viewer_ptrs_) {
    if (viewer_ptr->camera_ptr())
      AddPtrIfNameNotExists(viewer_ptr->camera_ptr(), &camera_ptrs_);
  }
}

bool Tracker::ExecuteViewingCycle(int iteration) {
  if (!UpdateCameras()) return false;
  return UpdateViewers(iteration);
}

bool Tracker::ExecuteTrackingCycle(int iteration) {
  if (!CalculateBeforeCameraUpdate()) return false;
  if (!UpdateCameras()) return false;
  for (int corr_iteration = 0; corr_iteration < n_corr_iterations_;
       ++corr_iteration) {
    int corr_save_idx = iteration * n_corr_iterations_ + corr_iteration;
    if (!StartOcclusionMaskRendering()) return false;
    if (!CalculateCorrespondences(corr_iteration)) return false;
    if (!VisualizeCorrespondences(corr_save_idx)) return false;
    for (int update_iteration = 0; update_iteration < n_update_iterations_;
         ++update_iteration) {
      int update_save_idx =
          corr_save_idx * n_update_iterations_ + update_iteration;
      if (!CalculatePoseUpdate()) return false;
      if (!VisualizePoseUpdate(update_save_idx)) return false;
    }
  }
  if (!VisualizeResults(iteration)) return false;
  if (!UpdateViewers(iteration)) return false;
  return true;
}

bool Tracker::StartRegionModalities() {
  for (auto &region_modality_ptr : region_modality_ptrs_) {
    if (!region_modality_ptr->StartModality()) return false;
  }
  return true;
}

bool Tracker::CalculateBeforeCameraUpdate() {
  for (auto &region_modality_ptr : region_modality_ptrs_) {
    if (!region_modality_ptr->CalculateBeforeCameraUpdate()) return false;
  }
  return true;
}

bool Tracker::UpdateCameras() {
  for (auto &camera_ptr : camera_ptrs_) {
    if (!camera_ptr->UpdateImage()) return false;
  }
  return true;
}

bool Tracker::StartOcclusionMaskRendering() {
  for (auto &occlusion_renderer_ptr : occlusion_mask_renderer_ptrs_) {
    if (!occlusion_renderer_ptr->StartRendering()) return false;
  }
  return true;
}

bool Tracker::CalculateCorrespondences(int corr_iteration) {
  for (auto &region_modality_ptr : region_modality_ptrs_) {
    if (!region_modality_ptr->CalculateCorrespondences(corr_iteration))
      return false;
  }
  return true;
}

bool Tracker::VisualizeCorrespondences(int save_idx) {
  bool imshow_correspondences = false;
  for (auto &region_modality_ptr : region_modality_ptrs_) {
    if (!region_modality_ptr->VisualizeCorrespondences(save_idx)) return false;
    if (region_modality_ptr->imshow_correspondence())
      imshow_correspondences = true;
  }
  if (imshow_correspondences) {
    if (cv::waitKey(visualization_time_) == 'q') return false;
  }
  return true;
}

bool Tracker::CalculatePoseUpdate() {
  for (auto &region_modality_ptr : region_modality_ptrs_) {
    if (!region_modality_ptr->CalculatePoseUpdate()) return false;
  }
  return true;
}

bool Tracker::VisualizePoseUpdate(int save_idx) {
  bool imshow_pose_update = false;
  for (auto &region_modality_ptr : region_modality_ptrs_) {
    if (!region_modality_ptr->VisualizePoseUpdate(save_idx)) return false;
    if (region_modality_ptr->imshow_pose_update()) imshow_pose_update = true;
  }
  if (imshow_pose_update) {
    if (cv::waitKey(visualization_time_) == 'q') return false;
  }
  return true;
}

bool Tracker::VisualizeResults(int save_idx) {
  bool imshow_result = false;
  for (auto &region_modality_ptr : region_modality_ptrs_) {
    if (!region_modality_ptr->VisualizeResults(save_idx)) return false;
    if (region_modality_ptr->imshow_result()) imshow_result = true;
  }
  if (imshow_result) {
    if (cv::waitKey(visualization_time_) == 'q') return false;
  }
  return true;
}

bool Tracker::UpdateViewers(int iteration) {
  if (!viewer_ptrs_.empty()) {
    for (auto &viewer_ptr : viewer_ptrs_) {
      viewer_ptr->UpdateViewer(iteration);
    }
    char key = cv::waitKey(viewer_time_);
    if (key == 't' && !tracking_started_)
      start_tracking_ = true;
    else if (key == 'q')
      return false;
  }
  return true;
}

std::vector<std::shared_ptr<RegionModality>> Tracker::region_modality_ptrs()
    const {
  return region_modality_ptrs_;
}

std::vector<std::shared_ptr<Viewer>> Tracker::viewer_ptrs() const {
  return viewer_ptrs_;
}

int Tracker::n_corr_iterations() const { return n_corr_iterations_; }

int Tracker::n_update_iterations() const { return n_update_iterations_; }

int Tracker::visualization_time() const { return visualization_time_; }

int Tracker::viewer_time() const { return viewer_time_; }

}  // namespace rbgt
