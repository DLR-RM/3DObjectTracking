// SPDX-License-Identifier: MIT
// Copyright (c) 2023 Manuel Stoiber, German Aerospace Center (DLR)

#include <gtest/gtest.h>
#include <m3t/body.h>
#include <m3t/camera.h>
#include <m3t/depth_modality.h>
#include <m3t/optimizer.h>
#include <m3t/region_modality.h>

#include "common_test.h"

const std::filesystem::path optimizer_test_directory{data_directory /
                                                     "optimizer_test"};

class OptimizerTest : public testing::Test {
 protected:
  void SetUp() override {
    auto body_ptr{TriangleBodyPtr()};
    auto region_modality_ptr{std::make_shared<m3t::RegionModality>(
        "triangle_region_modality", body_ptr, ColorCameraPtr(),
        TriangleRegionModelPtr())};
    region_modality_ptr->SetUp();
    region_modality_ptr->StartModality(0, 0);
    region_modality_ptr->CalculateCorrespondences(0, 0);
    region_modality_ptr->CalculateGradientAndHessian(0, 0, 0);
    auto depth_modality_ptr{std::make_shared<m3t::DepthModality>(
        "triangle_depth_modality", body_ptr, DepthCameraPtr(),
        TriangleDepthModelPtr())};
    depth_modality_ptr->SetUp();
    depth_modality_ptr->CalculateCorrespondences(0, 0);
    depth_modality_ptr->CalculateGradientAndHessian(0, 0, 0);
    link_ptr_ = std::make_shared<m3t::Link>("link", body_ptr);
    link_ptr_->AddModality(region_modality_ptr);
    link_ptr_->AddModality(depth_modality_ptr);
    link_ptr_->SetUp();

    optimizer_ptr_ = std::make_shared<m3t::Optimizer>(
        name_, link_ptr_, tikhonov_parameter_rotation_,
        tikhonov_parameter_translation_);
  }

  bool TestOptimizerParameters(const m3t::Optimizer &optimizer) {
    return optimizer.name() == name_ &&
           optimizer.tikhonov_parameter_rotation() ==
               tikhonov_parameter_rotation_ &&
           optimizer.tikhonov_parameter_translation() ==
               tikhonov_parameter_translation_;
  }

  std::shared_ptr<m3t::Optimizer> optimizer_ptr_;
  std::string name_{"optimizer"};
  std::filesystem::path metafile_path_{optimizer_test_directory /
                                       "optimizer.yaml"};
  std::shared_ptr<m3t::Link> link_ptr_;
  float tikhonov_parameter_rotation_ = 5000.0f;
  float tikhonov_parameter_translation_ = 500000.0f;
};

TEST_F(OptimizerTest, SetUpFromData) {
  ASSERT_TRUE(optimizer_ptr_->SetUp());
  ASSERT_TRUE(optimizer_ptr_->set_up());
  ASSERT_TRUE(TestOptimizerParameters(*optimizer_ptr_));
}

TEST_F(OptimizerTest, SetUpFromMetaFile) {
  m3t::Optimizer optimizer{name_, metafile_path_, link_ptr_};
  ASSERT_TRUE(optimizer.SetUp());
  ASSERT_TRUE(optimizer.set_up());
  ASSERT_TRUE(TestOptimizerParameters(optimizer));
}

TEST_F(OptimizerTest, ParameterChanges) {
  ASSERT_TRUE(optimizer_ptr_->SetUp());
  ASSERT_TRUE(optimizer_ptr_->set_up());
  optimizer_ptr_->set_tikhonov_parameter_rotation(tikhonov_parameter_rotation_);
  ASSERT_FALSE(optimizer_ptr_->set_up());
  ASSERT_TRUE(optimizer_ptr_->SetUp());
  ASSERT_TRUE(optimizer_ptr_->set_up());
  optimizer_ptr_->set_tikhonov_parameter_translation(
      tikhonov_parameter_translation_);
  ASSERT_FALSE(optimizer_ptr_->set_up());
  ASSERT_TRUE(optimizer_ptr_->SetUp());
  ASSERT_TRUE(optimizer_ptr_->set_up());
  ASSERT_TRUE(TestOptimizerParameters(*optimizer_ptr_));
}

TEST_F(OptimizerTest, TestWithoutSetUp) {
  ASSERT_FALSE(optimizer_ptr_->CalculateOptimization(0, 0, 0));
}

TEST_F(OptimizerTest, TestWithoutSetUpLink) {
  link_ptr_->ClearModalities();
  ASSERT_FALSE(optimizer_ptr_->SetUp());
}

TEST_F(OptimizerTest, Optimize) {
  ASSERT_TRUE(optimizer_ptr_->SetUp());
  ASSERT_TRUE(optimizer_ptr_->CalculateOptimization(0, 0, 0));
  auto pose_matrix{optimizer_ptr_->root_link_ptr()->link2world_pose().matrix()};
  ASSERT_TRUE(SaveOrSkipMatrix(kGenerateGroundTruth, optimizer_test_directory,
                               "triangle_pose.txt", pose_matrix));
  ASSERT_TRUE(CompareToLoadedMatrix(optimizer_test_directory,
                                    "triangle_pose.txt", pose_matrix, 1.0e-5f));
}
