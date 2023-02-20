// SPDX-License-Identifier: MIT
// Copyright (c) 2023 Manuel Stoiber, German Aerospace Center (DLR)

#include <gtest/gtest.h>
#include <m3t/body.h>

#include "common_test.h"

class BodyTest : public testing::Test {
 protected:
  void SetUp() override {
    body_ptr_ = std::make_shared<m3t::Body>(
        name_, geometry_path_, geometry_unit_in_meter_,
        geometry_counterclockwise_, geometry_enable_culling_,
        geometry2body_pose_);
  }

  bool TestBodyParameters(const m3t::Body &body) {
    return body.name() == name_ &&
           m3t::Equivalent(body.geometry_path(), geometry_path_) &&
           body.geometry_unit_in_meter() == geometry_unit_in_meter_ &&
           body.geometry_counterclockwise() == geometry_counterclockwise_ &&
           body.geometry_enable_culling() == geometry_enable_culling_ &&
           body.geometry2body_pose().matrix() == geometry2body_pose_.matrix() &&
           body.maximum_body_diameter() == maximum_body_diameter_;
  }

  std::shared_ptr<m3t::Body> body_ptr_;
  std::string name_{"triangle"};
  std::filesystem::path metafile_path_{body_directory / "triangle.yaml"};
  std::filesystem::path default_metafile_path_{body_directory /
                                               "default_body.yaml"};
  std::filesystem::path geometry_path_{body_directory / "triangle.obj"};
  float geometry_unit_in_meter_ = 1.0f;
  bool geometry_counterclockwise_ = true;
  bool geometry_enable_culling_ = true;
  m3t::Transform3fA geometry2body_pose_{
      Eigen::Translation3f{0.0f, 0.0f, -0.006f}};
  uchar body_id_ = 150;
  uchar region_id_ = 200;
  float maximum_body_diameter_ = 0.0776427314f;
  m3t::Transform3fA pose_{Eigen::Translation3f{-0.2f, 0.1f, 0.4f}};
};

TEST_F(BodyTest, SetUpFromData) {
  ASSERT_TRUE(body_ptr_->SetUp());
  ASSERT_TRUE(body_ptr_->set_up());
  ASSERT_TRUE(TestBodyParameters(*body_ptr_));
}

TEST_F(BodyTest, SetUpFromMetaFile) {
  m3t::Body body{name_, metafile_path_};
  ASSERT_TRUE(body.SetUp());
  ASSERT_TRUE(body.set_up());
  ASSERT_TRUE(TestBodyParameters(body));
}

TEST_F(BodyTest, SetUpFromDefaultMetaFile) {
  m3t::Body body{name_, default_metafile_path_};
  ASSERT_TRUE(body.SetUp());
  ASSERT_TRUE(body.set_up());
  ASSERT_TRUE(TestBodyParameters(body));
}

TEST_F(BodyTest, ParameterChanges) {
  ASSERT_TRUE(body_ptr_->SetUp());
  ASSERT_TRUE(body_ptr_->set_up());
  body_ptr_->set_metafile_path(metafile_path_);
  ASSERT_FALSE(body_ptr_->set_up());
  ASSERT_TRUE(body_ptr_->SetUp());
  ASSERT_TRUE(body_ptr_->set_up());
  body_ptr_->set_geometry_path(geometry_path_);
  ASSERT_FALSE(body_ptr_->set_up());
  ASSERT_TRUE(body_ptr_->SetUp());
  ASSERT_TRUE(body_ptr_->set_up());
  body_ptr_->set_geometry_unit_in_meter(geometry_unit_in_meter_);
  ASSERT_FALSE(body_ptr_->set_up());
  ASSERT_TRUE(body_ptr_->SetUp());
  ASSERT_TRUE(body_ptr_->set_up());
  body_ptr_->set_geometry2body_pose(geometry2body_pose_);
  ASSERT_FALSE(body_ptr_->set_up());
  ASSERT_TRUE(body_ptr_->SetUp());
  ASSERT_TRUE(body_ptr_->set_up());
  ASSERT_TRUE(TestBodyParameters(*body_ptr_));
}

TEST_F(BodyTest, TestID) {
  ASSERT_FALSE(body_ptr_->body_id() == body_id_);
  ASSERT_FALSE(body_ptr_->region_id() == region_id_);
  body_ptr_->set_body_id(body_id_);
  body_ptr_->set_region_id(region_id_);
  ASSERT_TRUE(body_ptr_->body_id() == body_id_);
  ASSERT_TRUE(body_ptr_->region_id() == region_id_);
}

TEST_F(BodyTest, TestPose) {
  body_ptr_->set_world2body_pose(pose_);
  ASSERT_TRUE(body_ptr_->world2body_pose().matrix() == pose_.matrix());
  ASSERT_TRUE(body_ptr_->body2world_pose().matrix() ==
              pose_.inverse().matrix());
  body_ptr_->set_body2world_pose(pose_);
  ASSERT_TRUE(body_ptr_->body2world_pose().matrix() == pose_.matrix());
  ASSERT_TRUE(body_ptr_->world2body_pose().matrix() ==
              pose_.inverse().matrix());
}
