// SPDX-License-Identifier: MIT
// Copyright (c) 2023 Manuel Stoiber, German Aerospace Center (DLR)

#include <gtest/gtest.h>
#include <m3t/body.h>
#include <m3t/camera.h>
#include <m3t/manual_detector.h>
#include <m3t/static_detector.h>

#include "common_test.h"

const std::filesystem::path detector_test_directory{data_directory /
                                                    "detector_test"};

class StaticDetectorTest : public testing::Test {
 protected:
  void SetUp() override {
    body_ptr_ = TriangleBodyPtr();
    body_ptr_->set_body2world_pose(m3t::Transform3fA::Identity());
    link_ptr_ = std::make_shared<m3t::Link>("triangle_link", body_ptr_);
    link_ptr_->set_link2world_pose(m3t::Transform3fA::Identity());
    link_ptr_->SetUp();
    optimizer_ptr_ =
        std::make_shared<m3t::Optimizer>("triangle_optimizer", link_ptr_);
    optimizer_ptr_->SetUp();
    link2world_pose_.matrix() << 0.607674f, 0.786584f, -0.10962f, -0.081876f,
        0.408914f, -0.428214f, -0.805868f, -0.00546736f, -0.680823f, 0.444881f,
        -0.58186f, 0.618302f, 0.0f, 0.0f, 0.0f, 1.0f;
    names_.insert(optimizer_ptr_->name());
    detector_ptr_ = std::make_shared<m3t::StaticDetector>(name_, optimizer_ptr_,
                                                          link2world_pose_);
  }

  bool TestDetectorParameters(const m3t::StaticDetector &detector) {
    return detector.name() == name_ &&
           detector.link2world_pose().matrix() == link2world_pose_.matrix();
  }

  std::shared_ptr<m3t::StaticDetector> detector_ptr_;
  std::string name_{"detector"};
  std::filesystem::path metafile_path_{body_directory /
                                       "triangle_static_detector.yaml"};
  std::shared_ptr<m3t::Body> body_ptr_;
  std::shared_ptr<m3t::Link> link_ptr_;
  std::shared_ptr<m3t::Optimizer> optimizer_ptr_;
  m3t::Transform3fA link2world_pose_{};
  std::set<std::string> names_{};
};

TEST_F(StaticDetectorTest, SetUpFromData) {
  ASSERT_TRUE(detector_ptr_->SetUp());
  ASSERT_TRUE(detector_ptr_->set_up());
  ASSERT_TRUE(TestDetectorParameters(*detector_ptr_));
}

TEST_F(StaticDetectorTest, SetUpFromMetaFile) {
  m3t::StaticDetector detector{name_, metafile_path_, optimizer_ptr_};
  ASSERT_TRUE(detector.SetUp());
  ASSERT_TRUE(detector.set_up());
  ASSERT_TRUE(TestDetectorParameters(detector));
}

TEST_F(StaticDetectorTest, TestWithoutSetUp) {
  ASSERT_FALSE(detector_ptr_->DetectPoses(names_));
}

TEST_F(StaticDetectorTest, TestWithoutSetUpOptimizer) {
  detector_ptr_->set_optimizer_ptr(
      std::make_shared<m3t::Optimizer>("triangle_optimizer", link_ptr_));
  ASSERT_FALSE(detector_ptr_->SetUp());
}

TEST_F(StaticDetectorTest, DetectPose) {
  ASSERT_TRUE(detector_ptr_->SetUp());
  ASSERT_TRUE(detector_ptr_->DetectPoses(names_));
  auto pose_matrix{body_ptr_->body2world_pose().matrix()};
  ASSERT_TRUE(SaveOrSkipMatrix(kGenerateGroundTruth, detector_test_directory,
                               "detector_triangle_pose.txt", pose_matrix));
  ASSERT_TRUE(CompareToLoadedMatrix(detector_test_directory,
                                    "detector_triangle_pose.txt", pose_matrix,
                                    0.0f));
}

class ManualDetectorTest : public testing::Test {
 protected:
  void SetUp() override {
    body_ptr_ = TriangleBodyPtr();
    link_ptr_ = std::make_shared<m3t::Link>("triangle_link", body_ptr_);
    link_ptr_->SetUp();
    optimizer_ptr_ =
        std::make_shared<m3t::Optimizer>("triangle_optimizer", link_ptr_);
    optimizer_ptr_->SetUp();
    camera_ptr_ = ColorCameraPtr();
    names_.insert(optimizer_ptr_->name());
    detector_ptr_ = std::make_shared<m3t::ManualDetector>(
        name_, optimizer_ptr_, camera_ptr_, reference_points_,
        detector_image_path_);
  }

  bool TestDetectorParameters(const m3t::ManualDetector &detector) {
    return detector.name() == name_ &&
           detector.reference_points() == reference_points_ &&
           m3t::Equivalent(detector.detector_image_path(),
                           detector_image_path_);
  }

  std::shared_ptr<m3t::ManualDetector> detector_ptr_;
  std::string name_{"detector"};
  std::filesystem::path metafile_path_{body_directory /
                                       "triangle_manual_detector.yaml"};
  std::shared_ptr<m3t::Body> body_ptr_;
  std::shared_ptr<m3t::Link> link_ptr_;
  std::shared_ptr<m3t::Optimizer> optimizer_ptr_;
  std::shared_ptr<m3t::LoaderColorCamera> camera_ptr_;
  std::vector<cv::Point3f> reference_points_{
      cv::Point3f{-0.0332f, 0.0f, 0.0f}, cv::Point3f{0.0192f, -0.0332f, 0.0f},
      cv::Point3f{0.0192f, 0.0332f, 0.0f}, cv::Point3f{0.0f, 0.0f, 0.0f}};
  std::filesystem::path detector_image_path_{body_directory /
                                             "triangle_manual_detector.png"};
  std::set<std::string> names_{};
};

TEST_F(ManualDetectorTest, SetUpFromData) {
  ASSERT_TRUE(detector_ptr_->SetUp());
  ASSERT_TRUE(detector_ptr_->set_up());
  ASSERT_TRUE(TestDetectorParameters(*detector_ptr_));
}

TEST_F(ManualDetectorTest, SetUpFromMetaFile) {
  m3t::ManualDetector detector{name_, metafile_path_, optimizer_ptr_,
                               camera_ptr_};
  ASSERT_TRUE(detector.SetUp());
  ASSERT_TRUE(detector.set_up());
  ASSERT_TRUE(TestDetectorParameters(detector));
}

TEST_F(ManualDetectorTest, TestWithoutSetUp) {
  ASSERT_FALSE(detector_ptr_->DetectPoses(names_));
}

TEST_F(ManualDetectorTest, TestWithoutSetUpBody) {
  detector_ptr_->set_optimizer_ptr(
      std::make_shared<m3t::Optimizer>("triangle_optimizer", link_ptr_));
  ASSERT_FALSE(detector_ptr_->SetUp());
}

TEST_F(ManualDetectorTest, TestWithoutSetUpCamera) {
  detector_ptr_->set_color_camera_ptr(ColorCameraPtrNoSetUp());
  ASSERT_FALSE(detector_ptr_->SetUp());
}
