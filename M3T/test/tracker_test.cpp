// SPDX-License-Identifier: MIT
// Copyright (c) 2023 Manuel Stoiber, German Aerospace Center (DLR)

#include <gtest/gtest.h>
#include <m3t/body.h>
#include <m3t/camera.h>
#include <m3t/depth_modality.h>
#include <m3t/generator.h>
#include <m3t/link.h>
#include <m3t/normal_viewer.h>
#include <m3t/optimizer.h>
#include <m3t/refiner.h>
#include <m3t/region_modality.h>
#include <m3t/renderer_geometry.h>
#include <m3t/static_detector.h>
#include <m3t/tracker.h>

#include "common_test.h"

const std::filesystem::path tracker_test_directory{data_directory /
                                                   "tracker_test"};

class TrackerTest : public testing::Test {
 protected:
  void SetUp() override {
    auto body_ptr{TriangleBodyPtr()};
    auto color_camera_ptr{ColorCameraPtr()};
    auto depth_camera_ptr{DepthCameraPtr()};
    auto region_modality_ptr{std::make_shared<m3t::RegionModality>(
        "triangle_region_modality", body_ptr, color_camera_ptr,
        TriangleRegionModelPtr())};
    region_modality_ptr->MeasureOcclusions(depth_camera_ptr);
    auto depth_modality_ptr{std::make_shared<m3t::DepthModality>(
        "triangle_depth_modality", body_ptr, depth_camera_ptr,
        TriangleDepthModelPtr())};
    depth_modality_ptr->MeasureOcclusions();
    auto link_ptr{std::make_shared<m3t::Link>("link", body_ptr)};
    link_ptr->AddModality(region_modality_ptr);
    link_ptr->AddModality(depth_modality_ptr);

    optimizer_ptr_ =
        std::make_shared<m3t::Optimizer>("triangle_optimizer", link_ptr);
    auto renderer_geometry_ptr{
        std::make_shared<m3t::RendererGeometry>("renderer_geometry")};
    renderer_geometry_ptr->AddBody(body_ptr);
    viewer_ptr_ = std::make_shared<m3t::NormalColorViewer>(
        "normal_viewer", color_camera_ptr, renderer_geometry_ptr);
    detector_ptr_ = std::make_shared<m3t::StaticDetector>(
        "triangle_detector", body_directory / "triangle_static_detector.yaml",
        optimizer_ptr_);
    refiner_ptr_ = std::make_shared<m3t::Refiner>("refiner");
    refiner_ptr_->AddOptimizer(optimizer_ptr_);

    tracker_ptr_ = std::make_shared<m3t::Tracker>(
        name_, n_corr_iterations_, n_update_iterations_, synchronize_cameras_,
        start_tracking_after_detection_, cycle_duration_, visualization_time_,
        viewer_time_);

    names_.insert(optimizer_ptr_->name());
  }

  bool TestTrackerParameters(const m3t::Tracker &tracker) {
    return tracker.name() == name_ &&
           tracker.n_corr_iterations() == n_corr_iterations_ &&
           tracker.n_update_iterations() == n_update_iterations_ &&
           tracker.synchronize_cameras() == synchronize_cameras_ &&
           tracker.start_tracking_after_detection() ==
               start_tracking_after_detection_ &&
           tracker.cycle_duration() == cycle_duration_ &&
           tracker.visualization_time() == visualization_time_ &&
           tracker.viewer_time() == viewer_time_;
  }

  std::shared_ptr<m3t::Tracker> tracker_ptr_;
  std::string name_{"tracker"};
  std::filesystem::path metafile_path_{tracker_test_directory / "tracker.yaml"};
  std::filesystem::path configfile_path_{tracker_test_directory /
                                         "tracker_config.yaml"};
  std::shared_ptr<m3t::Optimizer> optimizer_ptr_;
  std::shared_ptr<m3t::NormalColorViewer> viewer_ptr_;
  std::shared_ptr<m3t::StaticDetector> detector_ptr_;
  std::shared_ptr<m3t::Refiner> refiner_ptr_;
  std::set<std::string> names_{};
  int n_corr_iterations_ = 7;
  int n_update_iterations_ = 2;
  bool synchronize_cameras_ = false;
  bool start_tracking_after_detection_ = false;
  std::chrono::milliseconds cycle_duration_{30};
  int visualization_time_ = 1;
  int viewer_time_ = 1;
};

TEST_F(TrackerTest, SetUpFromData) {
  ASSERT_TRUE(tracker_ptr_->SetUp());
  ASSERT_TRUE(tracker_ptr_->set_up());
  ASSERT_TRUE(TestTrackerParameters(*tracker_ptr_));
}

TEST_F(TrackerTest, SetUpFromMetaFile) {
  m3t::Tracker tracker{name_, metafile_path_};
  ASSERT_TRUE(tracker.SetUp());
  ASSERT_TRUE(tracker.set_up());
  ASSERT_TRUE(TestTrackerParameters(tracker));
}

TEST_F(TrackerTest, AddDeleteClearObjects) {
  ASSERT_TRUE(tracker_ptr_->SetUp());
  ASSERT_TRUE(tracker_ptr_->set_up());
  ASSERT_TRUE(tracker_ptr_->AddOptimizer(optimizer_ptr_));
  ASSERT_FALSE(tracker_ptr_->set_up());
  ASSERT_TRUE(tracker_ptr_->SetUp());
  ASSERT_TRUE(tracker_ptr_->set_up());
  ASSERT_TRUE(tracker_ptr_->DeleteOptimizer(optimizer_ptr_->name()));
  ASSERT_FALSE(tracker_ptr_->set_up());
  ASSERT_TRUE(tracker_ptr_->SetUp());
  ASSERT_TRUE(tracker_ptr_->set_up());
  tracker_ptr_->ClearOptimizers();
  ASSERT_FALSE(tracker_ptr_->set_up());
  ASSERT_TRUE(tracker_ptr_->SetUp());
  ASSERT_TRUE(tracker_ptr_->set_up());
  ASSERT_TRUE(tracker_ptr_->AddViewer(viewer_ptr_));
  ASSERT_FALSE(tracker_ptr_->set_up());
  ASSERT_TRUE(tracker_ptr_->SetUp());
  ASSERT_TRUE(tracker_ptr_->set_up());
  ASSERT_TRUE(tracker_ptr_->DeleteViewer(viewer_ptr_->name()));
  ASSERT_FALSE(tracker_ptr_->set_up());
  ASSERT_TRUE(tracker_ptr_->SetUp());
  ASSERT_TRUE(tracker_ptr_->set_up());
  tracker_ptr_->ClearViewers();
  ASSERT_FALSE(tracker_ptr_->set_up());
  ASSERT_TRUE(tracker_ptr_->SetUp());
  ASSERT_TRUE(tracker_ptr_->set_up());
  ASSERT_TRUE(tracker_ptr_->AddDetector(detector_ptr_));
  ASSERT_FALSE(tracker_ptr_->set_up());
  ASSERT_TRUE(tracker_ptr_->SetUp());
  ASSERT_TRUE(tracker_ptr_->set_up());
  ASSERT_TRUE(tracker_ptr_->DeleteDetector(detector_ptr_->name()));
  ASSERT_FALSE(tracker_ptr_->set_up());
  ASSERT_TRUE(tracker_ptr_->SetUp());
  ASSERT_TRUE(tracker_ptr_->set_up());
  tracker_ptr_->ClearDetectors();
  ASSERT_FALSE(tracker_ptr_->set_up());
  ASSERT_TRUE(tracker_ptr_->SetUp());
  ASSERT_TRUE(tracker_ptr_->set_up());
  ASSERT_TRUE(tracker_ptr_->AddRefiner(refiner_ptr_));
  ASSERT_FALSE(tracker_ptr_->set_up());
  ASSERT_TRUE(tracker_ptr_->SetUp());
  ASSERT_TRUE(tracker_ptr_->set_up());
  ASSERT_TRUE(tracker_ptr_->DeleteRefiner(refiner_ptr_->name()));
  ASSERT_FALSE(tracker_ptr_->set_up());
  ASSERT_TRUE(tracker_ptr_->SetUp());
  ASSERT_TRUE(tracker_ptr_->set_up());
  tracker_ptr_->ClearRefiners();
  ASSERT_FALSE(tracker_ptr_->set_up());
  ASSERT_TRUE(tracker_ptr_->SetUp());
  ASSERT_TRUE(tracker_ptr_->set_up());
  ASSERT_TRUE(TestTrackerParameters(*tracker_ptr_));
}

TEST_F(TrackerTest, TestWithoutSetUp) {
  ASSERT_FALSE(tracker_ptr_->RunTrackerProcess(true, true));
}

TEST_F(TrackerTest, OptimizePoseMatrix) {
  std::filesystem::create_directory(temp_directory);
  viewer_ptr_->set_display_images(false);
  viewer_ptr_->StartSavingImages(temp_directory);
  ASSERT_TRUE(tracker_ptr_->AddViewer(viewer_ptr_));
  ASSERT_TRUE(tracker_ptr_->AddOptimizer(optimizer_ptr_));
  ASSERT_TRUE(tracker_ptr_->SetUp());
  ASSERT_TRUE(tracker_ptr_->StartModalities(0));
  ASSERT_TRUE(tracker_ptr_->ExecuteTrackingStep(0));
  ASSERT_TRUE(tracker_ptr_->UpdateViewers(0));
  auto pose_matrix{tracker_ptr_->body_ptrs()[0]->body2world_pose().matrix()};
  ASSERT_TRUE(SaveOrSkipMatrix(kGenerateGroundTruth, tracker_test_directory,
                               "triangle_pose.txt", pose_matrix));
  ASSERT_TRUE(CompareToLoadedMatrix(tracker_test_directory, "triangle_pose.txt",
                                    pose_matrix, 1.0e-5f));
}

TEST_F(TrackerTest, OptimizePoseMatrixGeneratorSetUp) {
  std::filesystem::create_directory(temp_directory);
  std::shared_ptr<m3t::Tracker> tracker_ptr;
  ASSERT_TRUE(GenerateConfiguredTracker(configfile_path_, &tracker_ptr));
  ASSERT_TRUE(tracker_ptr->SetUp());
  ASSERT_TRUE(tracker_ptr->DetectPoses(names_));
  ASSERT_TRUE(tracker_ptr->StartModalities(0));
  ASSERT_TRUE(tracker_ptr->ExecuteTrackingStep(0));
  ASSERT_TRUE(tracker_ptr_->UpdateViewers(0));
  auto pose_matrix{tracker_ptr->body_ptrs()[0]->body2world_pose().matrix()};
  ASSERT_TRUE(SaveOrSkipMatrix(kGenerateGroundTruth, tracker_test_directory,
                               "triangle_pose.txt", pose_matrix));
  ASSERT_TRUE(CompareToLoadedMatrix(tracker_test_directory, "triangle_pose.txt",
                                    pose_matrix, 1.0e-5f));
}
