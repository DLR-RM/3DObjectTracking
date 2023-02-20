// SPDX-License-Identifier: MIT
// Copyright (c) 2023 Manuel Stoiber, German Aerospace Center (DLR)

#include <gtest/gtest.h>
#include <m3t/body.h>
#include <m3t/camera.h>
#include <m3t/depth_modality.h>
#include <m3t/link.h>
#include <m3t/optimizer.h>
#include <m3t/refiner.h>
#include <m3t/region_modality.h>

#include "common_test.h"

const std::filesystem::path refiner_test_directory{data_directory /
                                                   "refiner_test"};

class RefinerTest : public testing::Test {
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
    refiner_ptr_ = std::make_shared<m3t::Refiner>(
        name_, n_corr_iterations_, n_update_iterations_, visualization_time_);

    names_.insert(optimizer_ptr_->name());
  }

  bool TestRefinerParameters(const m3t::Refiner &refiner) {
    return refiner.name() == name_ &&
           refiner.n_corr_iterations() == n_corr_iterations_ &&
           refiner.n_update_iterations() == n_update_iterations_ &&
           refiner.visualization_time() == visualization_time_;
  }

  std::shared_ptr<m3t::Refiner> refiner_ptr_;
  std::string name_{"refiner"};
  std::filesystem::path metafile_path_{refiner_test_directory / "refiner.yaml"};
  std::shared_ptr<m3t::Optimizer> optimizer_ptr_;
  int n_corr_iterations_ = 7;
  int n_update_iterations_ = 3;
  int visualization_time_ = 2;
  std::set<std::string> names_{};
};

TEST_F(RefinerTest, SetUpFromData) {
  ASSERT_TRUE(refiner_ptr_->SetUp());
  ASSERT_TRUE(refiner_ptr_->set_up());
  ASSERT_TRUE(TestRefinerParameters(*refiner_ptr_));
}

TEST_F(RefinerTest, SetUpFromMetaFile) {
  m3t::Refiner refiner{name_, metafile_path_};
  ASSERT_TRUE(refiner.SetUp());
  ASSERT_TRUE(refiner.set_up());
  ASSERT_TRUE(TestRefinerParameters(refiner));
}

TEST_F(RefinerTest, AddDeleteClearObjects) {
  ASSERT_TRUE(refiner_ptr_->SetUp());
  ASSERT_TRUE(refiner_ptr_->set_up());
  ASSERT_TRUE(refiner_ptr_->AddOptimizer(optimizer_ptr_));
  ASSERT_FALSE(refiner_ptr_->set_up());
  ASSERT_TRUE(refiner_ptr_->SetUp());
  ASSERT_TRUE(refiner_ptr_->set_up());
  ASSERT_TRUE(refiner_ptr_->DeleteOptimizer(optimizer_ptr_->name()));
  ASSERT_FALSE(refiner_ptr_->set_up());
  ASSERT_TRUE(refiner_ptr_->SetUp());
  ASSERT_TRUE(refiner_ptr_->set_up());
  refiner_ptr_->ClearOptimizers();
  ASSERT_FALSE(refiner_ptr_->set_up());
  ASSERT_TRUE(refiner_ptr_->SetUp());
  ASSERT_TRUE(refiner_ptr_->set_up());
  ASSERT_TRUE(TestRefinerParameters(*refiner_ptr_));
}

TEST_F(RefinerTest, TestWithoutSetUp) {
  ASSERT_FALSE(refiner_ptr_->RefinePoses(names_));
}

TEST_F(RefinerTest, OptimizePoseMatrix) {
  ASSERT_TRUE(refiner_ptr_->AddOptimizer(optimizer_ptr_));
  ASSERT_TRUE(refiner_ptr_->SetUp());
  ASSERT_TRUE(refiner_ptr_->RefinePoses(names_));
  auto pose_matrix{refiner_ptr_->body_ptrs()[0]->body2world_pose().matrix()};
  ASSERT_TRUE(SaveOrSkipMatrix(kGenerateGroundTruth, refiner_test_directory,
                               "triangle_pose.txt", pose_matrix));
  ASSERT_TRUE(CompareToLoadedMatrix(refiner_test_directory, "triangle_pose.txt",
                                    pose_matrix, 1.0e-5f));
}
