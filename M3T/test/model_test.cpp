// SPDX-License-Identifier: MIT
// Copyright (c) 2023 Manuel Stoiber and Anne Elisabeth Reichert,
// German Aerospace Center (DLR)

#include <gtest/gtest.h>
#include <m3t/body.h>
#include <m3t/depth_model.h>
#include <m3t/region_model.h>

#include "common_test.h"

const std::filesystem::path model_test_directory{data_directory / "model_test"};

class RegionModelTest : public testing::Test {
 protected:
  void SetUp() override {
    body_ptr_ = SchaumaBodyPtr();
    associated_body_ptr_ = TriangleBodyPtr();
    associated_body_ptr_->set_metafile_path(std::filesystem::path(""));
    model_ptr_ = std::make_shared<m3t::RegionModel>(
        name_, body_ptr_, model_path_, sphere_radius_, n_divides_, n_points_,
        max_radius_depth_offset_, stride_depth_offset_, use_random_seed_,
        image_size_);
  }

  bool TestModelParameters(const m3t::RegionModel &model) {
    return model.name() == name_ &&
           m3t::Equivalent(model.model_path(), model_path_) &&
           model.sphere_radius() == sphere_radius_ &&
           model.n_divides() == n_divides_ && model.n_points() == n_points_ &&
           model.max_radius_depth_offset() == max_radius_depth_offset_ &&
           model.stride_depth_offset() == stride_depth_offset_ &&
           model.use_random_seed() == use_random_seed_ &&
           model.image_size() == image_size_;
  }

  static bool CompareViewData(const m3t::RegionModel::View &view1,
                              const m3t::RegionModel::View &view2,
                              float max_error) {
    if (view1.orientation != view2.orientation) return false;
    if (view1.contour_length != view2.contour_length) return false;
    for (size_t i = 0; i < view1.data_points.size(); ++i) {
      if (abs((view1.data_points[i].center_f_body -
               view2.data_points[i].center_f_body)
                  .maxCoeff()) > max_error ||
          abs((view1.data_points[i].normal_f_body -
               view2.data_points[i].normal_f_body)
                  .maxCoeff()) > max_error ||
          abs(view1.data_points[i].foreground_distance -
              view2.data_points[i].foreground_distance) > max_error ||
          abs(view1.data_points[i].background_distance -
              view2.data_points[i].background_distance) > max_error ||
          !std::equal(
              begin(view1.data_points[i].depth_offsets),
              end(view1.data_points[i].depth_offsets),
              begin(view2.data_points[i].depth_offsets),
              [&](float o1, float o2) { return abs(o1 - o2) <= max_error; }))
        return false;
    }
    return true;
  }

  std::shared_ptr<m3t::RegionModel> model_ptr_;
  std::string name_{"region_model"};
  std::filesystem::path metafile_path_{model_test_directory /
                                       "region_model.yaml"};
  std::filesystem::path default_metafile_path_{model_test_directory /
                                               "default_model.yaml"};
  std::shared_ptr<m3t::Body> body_ptr_;
  std::shared_ptr<m3t::Body> associated_body_ptr_;
  std::filesystem::path model_path_{model_test_directory / "region_model.bin"};
  float sphere_radius_ = 0.4f;
  int n_divides_ = 2;
  int n_points_ = 10;
  float max_radius_depth_offset_ = 0.05f;
  float stride_depth_offset_ = 0.002f;
  bool use_random_seed_ = false;
  int image_size_ = 500;
  m3t::Transform3fA body2camera_pose_{Eigen::Translation3f{-0.2f, 0.1f, 0.4f}};
};

TEST_F(RegionModelTest, SetUpFromData) {
  ASSERT_TRUE(model_ptr_->SetUp());
  ASSERT_TRUE(model_ptr_->set_up());
  ASSERT_TRUE(TestModelParameters(*model_ptr_));
}

TEST_F(RegionModelTest, SetUpFromMetaFile) {
  m3t::RegionModel model{name_, metafile_path_, body_ptr_};
  ASSERT_TRUE(model.SetUp());
  ASSERT_TRUE(model.set_up());
  ASSERT_TRUE(TestModelParameters(model));
}

TEST_F(RegionModelTest, SetUpFromDefaultMetaFile) {
  m3t::RegionModel model{name_, default_metafile_path_, body_ptr_};
  ASSERT_TRUE(model.SetUp());
  ASSERT_TRUE(model.set_up());
  ASSERT_TRUE(TestModelParameters(model));
}

TEST_F(RegionModelTest, ParameterChanges) {
  ASSERT_TRUE(model_ptr_->SetUp());
  ASSERT_TRUE(model_ptr_->set_up());
  model_ptr_->set_body_ptr(body_ptr_);
  ASSERT_FALSE(model_ptr_->set_up());
  ASSERT_TRUE(model_ptr_->SetUp());
  ASSERT_TRUE(model_ptr_->set_up());
  model_ptr_->set_model_path(model_path_);
  ASSERT_FALSE(model_ptr_->set_up());
  ASSERT_TRUE(model_ptr_->SetUp());
  ASSERT_TRUE(model_ptr_->set_up());
  model_ptr_->set_sphere_radius(sphere_radius_);
  ASSERT_FALSE(model_ptr_->set_up());
  ASSERT_TRUE(model_ptr_->SetUp());
  ASSERT_TRUE(model_ptr_->set_up());
  model_ptr_->set_n_divides(n_divides_);
  ASSERT_FALSE(model_ptr_->set_up());
  ASSERT_TRUE(model_ptr_->SetUp());
  ASSERT_TRUE(model_ptr_->set_up());
  model_ptr_->set_n_points(n_points_);
  ASSERT_FALSE(model_ptr_->set_up());
  ASSERT_TRUE(model_ptr_->SetUp());
  ASSERT_TRUE(model_ptr_->set_up());
  model_ptr_->set_max_radius_depth_offset(max_radius_depth_offset_);
  ASSERT_FALSE(model_ptr_->set_up());
  ASSERT_TRUE(model_ptr_->SetUp());
  ASSERT_TRUE(model_ptr_->set_up());
  model_ptr_->set_stride_depth_offset(stride_depth_offset_);
  ASSERT_FALSE(model_ptr_->set_up());
  ASSERT_TRUE(model_ptr_->SetUp());
  ASSERT_TRUE(model_ptr_->set_up());
  model_ptr_->set_use_random_seed(use_random_seed_);
  ASSERT_FALSE(model_ptr_->set_up());
  ASSERT_TRUE(model_ptr_->SetUp());
  ASSERT_TRUE(model_ptr_->set_up());
  model_ptr_->set_image_size(image_size_);
  ASSERT_FALSE(model_ptr_->set_up());
  ASSERT_TRUE(model_ptr_->SetUp());
  ASSERT_TRUE(model_ptr_->set_up());
  ASSERT_TRUE(TestModelParameters(*model_ptr_));
}

TEST_F(RegionModelTest, TestWithoutSetUpBody) {
  model_ptr_->set_body_ptr(SchaumaBodyPtrNoSetUp());
  ASSERT_FALSE(model_ptr_->SetUp());
}

TEST_F(RegionModelTest, TestWithWrongDepthOffsetVariables) {
  model_ptr_->set_max_radius_depth_offset(0.1f);
  ASSERT_FALSE(model_ptr_->SetUp());
}

TEST_F(RegionModelTest, TestWithoutSetUp) {
  const m3t::RegionModel::View *view;
  ASSERT_FALSE(model_ptr_->GetClosestView(body2camera_pose_, &view));
}

TEST_F(RegionModelTest, GetClosestView) {
  const m3t::RegionModel::View *view;
  ASSERT_TRUE(model_ptr_->SetUp());
  ASSERT_TRUE(model_ptr_->GetClosestView(body2camera_pose_, &view));
}

TEST_F(RegionModelTest, GenerateAndLoadModel) {
  // Generate model
  const m3t::RegionModel::View *view;
  std::filesystem::path temp_model_path{temp_directory / "temp.bin"};
  std::filesystem::create_directory(temp_directory);
  std::filesystem::remove(temp_model_path);
  model_ptr_->set_model_path(temp_model_path);
  ASSERT_TRUE(model_ptr_->SetUp());
  ASSERT_TRUE(model_ptr_->GetClosestView(body2camera_pose_, &view));

  // Load model
  const m3t::RegionModel::View *loaded_view;
  auto loaded_model_ptr{std::make_shared<m3t::RegionModel>(*model_ptr_)};
  ASSERT_TRUE(loaded_model_ptr->SetUp());
  ASSERT_TRUE(
      loaded_model_ptr->GetClosestView(body2camera_pose_, &loaded_view));

  // Compare views
  ASSERT_TRUE(CompareViewData(*view, *loaded_view, 0.0f));
}

TEST_F(RegionModelTest, IncludeAssociatedBodyPtrs) {
  std::filesystem::path temp_model_path{temp_directory / "temp.bin"};
  std::filesystem::create_directory(temp_directory);
  std::filesystem::remove(temp_model_path);
  model_ptr_->set_model_path(temp_model_path);
  auto copied_associated_body_ptr{
      std::make_shared<m3t::Body>(*associated_body_ptr_)};
  std::string associated_body_ptr_name = "triangle_2";
  copied_associated_body_ptr->set_name(associated_body_ptr_name);

  // Add associated body_ptrs in RegionModel
  model_ptr_->AddAssociatedBody(associated_body_ptr_, false, true);
  model_ptr_->AddAssociatedBody(associated_body_ptr_, true, true);
  ASSERT_TRUE(model_ptr_->associated_body_ptrs().size() == 1);
  model_ptr_->AddAssociatedBody(copied_associated_body_ptr, true, true);
  ASSERT_TRUE(model_ptr_->associated_body_ptrs().size() == 2);
  ASSERT_TRUE(model_ptr_->SetUp());

  // Remove associated body_ptrs from RegionModel
  model_ptr_->DeleteAssociatedBody(associated_body_ptr_name);
  ASSERT_TRUE(model_ptr_->associated_body_ptrs().size() == 1);
  ASSERT_TRUE(model_ptr_->movable_same_region_body_ptrs().empty());
  model_ptr_->ClearAssociatedBodies();
  ASSERT_TRUE(model_ptr_->associated_body_ptrs().empty());
  ASSERT_TRUE(model_ptr_->fixed_same_region_body_ptrs().empty());
}

TEST_F(RegionModelTest, ValidateFullyOccludedBody) {
  const m3t::RegionModel::View *view;
  std::filesystem::path temp_model_path{temp_directory / "temp.bin"};
  std::filesystem::create_directory(temp_directory);
  std::filesystem::remove(temp_model_path);
  model_ptr_->set_model_path(temp_model_path);
  model_ptr_->AddAssociatedBody(associated_body_ptr_, true, false);
  // Minimize main_body, movable body completely occludes body in view
  body_ptr_->set_metafile_path(std::filesystem::path(""));
  body_ptr_->set_geometry_unit_in_meter(0.001f);
  body_ptr_->SetUp();

  // Check contour length
  ASSERT_TRUE(model_ptr_->SetUp());
  ASSERT_TRUE(model_ptr_->GetClosestView(body2camera_pose_, &view));
  ASSERT_TRUE(view->contour_length == 0.0f);
}

TEST_F(RegionModelTest, ValidationRuleMovableBody) {
  const m3t::RegionModel::View *view;
  std::filesystem::path temp_model_path{temp_directory / "temp.bin"};
  std::filesystem::create_directory(temp_directory);
  std::filesystem::remove(temp_model_path);
  model_ptr_->set_model_path(temp_model_path);
  model_ptr_->set_n_divides(0);
  associated_body_ptr_->set_geometry2body_pose(
      associated_body_ptr_->geometry2body_pose() *
      Eigen::Translation3f(0, 0.05, -0.01));
  model_ptr_->AddAssociatedBody(associated_body_ptr_, true, false);

  // Generate view
  m3t::Transform3fA camera2body_pose;
  camera2body_pose.matrix() << -0.525731, -0.262866, 0.809017, -0.323607, 0,
      0.951056, 0.309017, -0.123607, -0.850651, 0.16246, -0.5, 0.2, 0, 0, 0, 1;
  ASSERT_TRUE(model_ptr_->SetUp());
  ASSERT_TRUE(model_ptr_->GetClosestView(camera2body_pose.inverse(), &view));

  // Load correct model
  const m3t::RegionModel::View *loaded_view;
  auto loaded_model_ptr{std::make_shared<m3t::RegionModel>(*model_ptr_)};
  loaded_model_ptr->set_model_path(model_test_directory /
                                   "multi_region_model_movable.bin");
  ASSERT_TRUE(loaded_model_ptr->SetUp());
  ASSERT_TRUE(loaded_model_ptr->GetClosestView(camera2body_pose.inverse(),
                                               &loaded_view));

  // Compare views
  ASSERT_TRUE(RegionModelTest::CompareViewData(*view, *loaded_view, 0.00001f));
}

TEST_F(RegionModelTest, ValidationRuleFixedBody) {
  const m3t::RegionModel::View *view;
  std::filesystem::path temp_model_path{temp_directory / "temp.bin"};
  std::filesystem::create_directory(temp_directory);
  std::filesystem::remove(temp_model_path);
  model_ptr_->set_model_path(temp_model_path);
  model_ptr_->set_n_divides(0);
  model_ptr_->AddAssociatedBody(associated_body_ptr_, false, false);

  // Generate view
  m3t::Transform3fA camera2body_pose;
  camera2body_pose.matrix() << 0, -0.273266, 0.961938, -0.384775, 0, 0.961938,
      0.273266, -0.109307, -1, 0, 0, 0, 0, 0, 0, 1;
  ASSERT_TRUE(model_ptr_->SetUp());
  ASSERT_TRUE(model_ptr_->GetClosestView(camera2body_pose.inverse(), &view));

  // Load correct model
  const m3t::RegionModel::View *loaded_view;
  auto loaded_model_ptr{std::make_shared<m3t::RegionModel>(*model_ptr_)};
  loaded_model_ptr->set_model_path(model_test_directory /
                                   "multi_region_model_fixed.bin");
  ASSERT_TRUE(loaded_model_ptr->SetUp());
  ASSERT_TRUE(loaded_model_ptr->GetClosestView(camera2body_pose.inverse(),
                                               &loaded_view));

  // Compare views
  ASSERT_TRUE(RegionModelTest::CompareViewData(*view, *loaded_view, 0.00001f));
}

TEST_F(RegionModelTest, ValidationRuleSameRegionBody) {
  const m3t::RegionModel::View *view;
  std::filesystem::path temp_model_path{temp_directory / "temp.bin"};
  std::filesystem::create_directory(temp_directory);
  std::filesystem::remove(temp_model_path);
  model_ptr_->set_model_path(temp_model_path);
  model_ptr_->set_n_divides(0);
  model_ptr_->AddAssociatedBody(associated_body_ptr_, false, true);

  // Generate view
  m3t::Transform3fA camera2body_pose;
  camera2body_pose.matrix() << 0.810147, -0.403436, 0.425325, -0.17013, 0,
      0.72553, 0.688191, -0.275276, -0.586227, -0.557535, 0.587785, -0.235114,
      0, 0, 0, 1;
  ASSERT_TRUE(model_ptr_->SetUp());
  ASSERT_TRUE(model_ptr_->GetClosestView(camera2body_pose.inverse(), &view));

  // Load correct model
  const m3t::RegionModel::View *loaded_view;
  auto loaded_model_ptr{std::make_shared<m3t::RegionModel>(*model_ptr_)};
  loaded_model_ptr->set_model_path(model_test_directory /
                                   "multi_region_model_same.bin");
  ASSERT_TRUE(loaded_model_ptr->SetUp());
  ASSERT_TRUE(loaded_model_ptr->GetClosestView(camera2body_pose.inverse(),
                                               &loaded_view));

  // Compare views
  ASSERT_TRUE(RegionModelTest::CompareViewData(*view, *loaded_view, 0.00002f));
}

class DepthModelTest : public testing::Test {
 protected:
  void SetUp() override {
    body_ptr_ = SchaumaBodyPtr();
    occlusion_body_ptr_ = TriangleBodyPtr();
    occlusion_body_ptr_->set_metafile_path(std::filesystem::path(""));
    model_ptr_ = std::make_shared<m3t::DepthModel>(
        name_, body_ptr_, model_path_, sphere_radius_, n_divides_, n_points_,
        max_radius_depth_offset_, stride_depth_offset_, use_random_seed_,
        image_size_);
  }

  bool TestModelParameters(const m3t::DepthModel &model) {
    return model.name() == name_ &&
           m3t::Equivalent(model.model_path(), model_path_) &&
           model.sphere_radius() == sphere_radius_ &&
           model.n_divides() == n_divides_ && model.n_points() == n_points_ &&
           model.max_radius_depth_offset() == max_radius_depth_offset_ &&
           model.stride_depth_offset() == stride_depth_offset_ &&
           model.use_random_seed() == use_random_seed_ &&
           model.image_size() == image_size_;
  }

  static bool CompareViewData(const m3t::DepthModel::View &view1,
                              const m3t::DepthModel::View &view2,
                              float max_error) {
    if (view1.orientation != view2.orientation) return false;
    if (view1.surface_area != view2.surface_area) return false;
    for (size_t i = 0; i < view1.data_points.size(); ++i) {
      if (abs((view1.data_points[i].center_f_body -
               view2.data_points[i].center_f_body)
                  .maxCoeff()) > max_error ||
          abs((view1.data_points[i].normal_f_body -
               view2.data_points[i].normal_f_body)
                  .maxCoeff()) > max_error ||
          !std::equal(
              begin(view1.data_points[i].depth_offsets),
              end(view1.data_points[i].depth_offsets),
              begin(view2.data_points[i].depth_offsets),
              [&](float o1, float o2) { return abs(o1 - o2) <= max_error; }))
        return false;
    }
    return true;
  }

  std::shared_ptr<m3t::DepthModel> model_ptr_;
  std::string name_{"depth_model"};
  std::filesystem::path metafile_path_{model_test_directory /
                                       "depth_model.yaml"};
  std::filesystem::path default_metafile_path_{model_test_directory /
                                               "default_model.yaml"};
  std::shared_ptr<m3t::Body> body_ptr_;
  std::shared_ptr<m3t::Body> occlusion_body_ptr_;
  std::filesystem::path model_path_{model_test_directory / "depth_model.bin"};
  float sphere_radius_ = 0.4f;
  int n_divides_ = 2;
  int n_points_ = 10;
  float max_radius_depth_offset_ = 0.05f;
  float stride_depth_offset_ = 0.002f;
  bool use_random_seed_ = false;
  int image_size_ = 500;
  m3t::Transform3fA body2camera_pose_{Eigen::Translation3f{-0.2f, 0.1f, 0.4f}};
};

TEST_F(DepthModelTest, SetUpFromData) {
  ASSERT_TRUE(model_ptr_->SetUp());
  ASSERT_TRUE(model_ptr_->set_up());
  ASSERT_TRUE(TestModelParameters(*model_ptr_));
}

TEST_F(DepthModelTest, SetUpFromMetaFile) {
  m3t::DepthModel model{name_, metafile_path_, body_ptr_};
  ASSERT_TRUE(model.SetUp());
  ASSERT_TRUE(model.set_up());
  ASSERT_TRUE(TestModelParameters(model));
}

TEST_F(DepthModelTest, SetUpFromDefaultMetaFile) {
  m3t::DepthModel model{name_, default_metafile_path_, body_ptr_};
  ASSERT_TRUE(model.SetUp());
  ASSERT_TRUE(model.set_up());
  ASSERT_TRUE(TestModelParameters(model));
}

TEST_F(DepthModelTest, ParameterChanges) {
  ASSERT_TRUE(model_ptr_->SetUp());
  ASSERT_TRUE(model_ptr_->set_up());
  model_ptr_->set_body_ptr(body_ptr_);
  ASSERT_FALSE(model_ptr_->set_up());
  ASSERT_TRUE(model_ptr_->SetUp());
  ASSERT_TRUE(model_ptr_->set_up());
  model_ptr_->set_model_path(model_path_);
  ASSERT_FALSE(model_ptr_->set_up());
  ASSERT_TRUE(model_ptr_->SetUp());
  ASSERT_TRUE(model_ptr_->set_up());
  model_ptr_->set_sphere_radius(sphere_radius_);
  ASSERT_FALSE(model_ptr_->set_up());
  ASSERT_TRUE(model_ptr_->SetUp());
  ASSERT_TRUE(model_ptr_->set_up());
  model_ptr_->set_n_divides(n_divides_);
  ASSERT_FALSE(model_ptr_->set_up());
  ASSERT_TRUE(model_ptr_->SetUp());
  ASSERT_TRUE(model_ptr_->set_up());
  model_ptr_->set_n_points(n_points_);
  ASSERT_FALSE(model_ptr_->set_up());
  ASSERT_TRUE(model_ptr_->SetUp());
  ASSERT_TRUE(model_ptr_->set_up());
  model_ptr_->set_max_radius_depth_offset(max_radius_depth_offset_);
  ASSERT_FALSE(model_ptr_->set_up());
  ASSERT_TRUE(model_ptr_->SetUp());
  ASSERT_TRUE(model_ptr_->set_up());
  model_ptr_->set_stride_depth_offset(stride_depth_offset_);
  ASSERT_FALSE(model_ptr_->set_up());
  ASSERT_TRUE(model_ptr_->SetUp());
  ASSERT_TRUE(model_ptr_->set_up());
  model_ptr_->set_use_random_seed(use_random_seed_);
  ASSERT_FALSE(model_ptr_->set_up());
  ASSERT_TRUE(model_ptr_->SetUp());
  ASSERT_TRUE(model_ptr_->set_up());
  model_ptr_->set_image_size(image_size_);
  ASSERT_FALSE(model_ptr_->set_up());
  ASSERT_TRUE(model_ptr_->SetUp());
  ASSERT_TRUE(model_ptr_->set_up());
  ASSERT_TRUE(TestModelParameters(*model_ptr_));
}

TEST_F(DepthModelTest, TestWithoutSetUpBody) {
  model_ptr_->set_body_ptr(SchaumaBodyPtrNoSetUp());
  ASSERT_FALSE(model_ptr_->SetUp());
}

TEST_F(DepthModelTest, TestWithWrongDepthOffsetVariables) {
  model_ptr_->set_max_radius_depth_offset(0.1f);
  ASSERT_FALSE(model_ptr_->SetUp());
}

TEST_F(DepthModelTest, TestWithoutSetUp) {
  const m3t::DepthModel::View *view;
  ASSERT_FALSE(model_ptr_->GetClosestView(body2camera_pose_, &view));
}

TEST_F(DepthModelTest, GetClosestView) {
  const m3t::DepthModel::View *view;
  ASSERT_TRUE(model_ptr_->SetUp());
  ASSERT_TRUE(model_ptr_->GetClosestView(body2camera_pose_, &view));
}

TEST_F(DepthModelTest, GenerateAndLoadModel) {
  // Generate model
  const m3t::DepthModel::View *view;
  std::filesystem::path temp_model_path{temp_directory / "temp.bin"};
  std::filesystem::create_directory(temp_directory);
  std::filesystem::remove(temp_model_path);
  model_ptr_->set_model_path(temp_model_path);
  ASSERT_TRUE(model_ptr_->SetUp());
  ASSERT_TRUE(model_ptr_->GetClosestView(body2camera_pose_, &view));

  // Load model
  const m3t::DepthModel::View *loaded_view;
  auto loaded_model_ptr{std::make_shared<m3t::DepthModel>(*model_ptr_)};
  ASSERT_TRUE(loaded_model_ptr->SetUp());
  ASSERT_TRUE(
      loaded_model_ptr->GetClosestView(body2camera_pose_, &loaded_view));

  // Compare views
  ASSERT_TRUE(CompareViewData(*view, *loaded_view, 0.0f));
}

TEST_F(DepthModelTest, ValidateFullyOccludedBody) {
  const m3t::DepthModel::View *view;
  std::filesystem::path temp_model_path{temp_directory / "temp.bin"};
  std::filesystem::create_directory(temp_directory);
  std::filesystem::remove(temp_model_path);
  model_ptr_->set_model_path(temp_model_path);
  model_ptr_->AddOcclusionBody(occlusion_body_ptr_);
  // Minimize main_body, movable body completely occludes body in view
  body_ptr_->set_metafile_path(std::filesystem::path(""));
  body_ptr_->set_geometry_unit_in_meter(0.001f);
  body_ptr_->SetUp();

  // Check contour length
  ASSERT_TRUE(model_ptr_->SetUp());
  ASSERT_TRUE(model_ptr_->GetClosestView(body2camera_pose_, &view));
  ASSERT_TRUE(view->surface_area == 0.0f);
}

TEST_F(DepthModelTest, ValidationRuleOcclusionBody) {
  const m3t::DepthModel::View *view;
  std::filesystem::path temp_model_path{temp_directory / "temp.bin"};
  std::filesystem::create_directory(temp_directory);
  std::filesystem::remove(temp_model_path);
  model_ptr_->set_model_path(temp_model_path);
  model_ptr_->set_n_divides(0);
  occlusion_body_ptr_->set_geometry2body_pose(
      occlusion_body_ptr_->geometry2body_pose() *
      Eigen::Translation3f(0, 0.05, -0.01));
  model_ptr_->AddOcclusionBody(occlusion_body_ptr_);

  // Generate view
  m3t::Transform3fA camera2body_pose;
  camera2body_pose.matrix() << -0.525731, -0.262866, 0.809017, -0.323607, 0,
      0.951056, 0.309017, -0.123607, -0.850651, 0.16246, -0.5, 0.2, 0, 0, 0, 1;
  ASSERT_TRUE(model_ptr_->SetUp());
  ASSERT_TRUE(model_ptr_->GetClosestView(camera2body_pose.inverse(), &view));

  // Load correct model
  const m3t::DepthModel::View *loaded_view;
  auto loaded_model_ptr{std::make_shared<m3t::DepthModel>(*model_ptr_)};
  loaded_model_ptr->set_model_path(model_test_directory /
                                   "depth_model_occlusion.bin");
  ASSERT_TRUE(loaded_model_ptr->SetUp());
  ASSERT_TRUE(loaded_model_ptr->GetClosestView(camera2body_pose.inverse(),
                                               &loaded_view));

  // Compare views
  ASSERT_TRUE(DepthModelTest::CompareViewData(*view, *loaded_view, 0.00001f));
}
