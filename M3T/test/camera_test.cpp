// SPDX-License-Identifier: MIT
// Copyright (c) 2023 Manuel Stoiber, German Aerospace Center (DLR)

#include <gtest/gtest.h>
#include <m3t/loader_camera.h>

#include "common_test.h"

class LoaderColorCameraTest : public testing::Test {
 protected:
  void SetUp() override {
    camera_ptr_ = std::make_shared<m3t::LoaderColorCamera>(
        name_, load_directory_, intrinsics_, image_name_pre_, load_index_,
        n_leading_zeros_, image_name_post_, load_image_type_);
  }

  bool TestCameraParameters(const m3t::LoaderColorCamera &camera) {
    return camera.name() == name_ &&
           m3t::Equivalent(camera.load_directory(), load_directory_) &&
           camera.intrinsics().fu == intrinsics_.fu &&
           camera.intrinsics().fv == intrinsics_.fv &&
           camera.image_name_pre() == image_name_pre_ &&
           camera.n_leading_zeros() == n_leading_zeros_ &&
           camera.image_name_post() == image_name_post_ &&
           camera.load_image_type() == load_image_type_;
  }

  std::shared_ptr<m3t::LoaderColorCamera> camera_ptr_;
  std::string name_{"color_camera"};
  std::filesystem::path load_directory_{sequence_directory};
  std::filesystem::path metafile_path_{sequence_directory /
                                       "color_camera.yaml"};
  m3t::Intrinsics intrinsics_{698.128f, 698.617f, 478.459f, 274.426f, 960, 540};
  std::string image_name_pre_{"color_camera_image_"};
  int load_index_ = 200;
  int n_leading_zeros_ = 0;
  std::string image_name_post_ = "";
  std::string load_image_type_ = "png";
  m3t::Transform3fA pose_{Eigen::Translation3f{-0.2f, 0.1f, 0.4f}};
};

TEST_F(LoaderColorCameraTest, SetUpFromData) {
  ASSERT_TRUE(camera_ptr_->SetUp());
  ASSERT_TRUE(camera_ptr_->set_up());
  ASSERT_TRUE(TestCameraParameters(*camera_ptr_));
}

TEST_F(LoaderColorCameraTest, SetUpFromMetaFile) {
  m3t::LoaderColorCamera camera{name_, metafile_path_};
  ASSERT_TRUE(camera.SetUp());
  ASSERT_TRUE(camera.set_up());
  ASSERT_TRUE(TestCameraParameters(camera));
}

TEST_F(LoaderColorCameraTest, ParameterChanges) {
  ASSERT_TRUE(camera_ptr_->SetUp());
  ASSERT_TRUE(camera_ptr_->set_up());
  camera_ptr_->StartSavingImages(temp_directory, 123, "bmp");
  ASSERT_TRUE(camera_ptr_->save_images());
  ASSERT_FALSE(camera_ptr_->set_up());
  camera_ptr_->StopSavingImages();
  camera_ptr_->set_load_index(load_index_);
  ASSERT_TRUE(camera_ptr_->SetUp());
  ASSERT_TRUE(camera_ptr_->set_up());
  camera_ptr_->set_load_directory(load_directory_);
  ASSERT_FALSE(camera_ptr_->set_up());
  camera_ptr_->set_load_index(load_index_);
  ASSERT_TRUE(camera_ptr_->SetUp());
  ASSERT_TRUE(camera_ptr_->set_up());
  camera_ptr_->set_metafile_path(metafile_path_);
  ASSERT_FALSE(camera_ptr_->set_up());
  camera_ptr_->set_load_index(load_index_);
  ASSERT_TRUE(camera_ptr_->SetUp());
  ASSERT_TRUE(camera_ptr_->set_up());
  camera_ptr_->set_intrinsics(intrinsics_);
  ASSERT_FALSE(camera_ptr_->set_up());
  camera_ptr_->set_load_index(load_index_);
  ASSERT_TRUE(camera_ptr_->SetUp());
  ASSERT_TRUE(camera_ptr_->set_up());
  camera_ptr_->set_image_name_pre(image_name_pre_);
  ASSERT_FALSE(camera_ptr_->set_up());
  camera_ptr_->set_load_index(load_index_);
  ASSERT_TRUE(camera_ptr_->SetUp());
  ASSERT_TRUE(camera_ptr_->set_up());
  camera_ptr_->set_load_index(load_index_);
  ASSERT_FALSE(camera_ptr_->set_up());
  camera_ptr_->set_load_index(load_index_);
  ASSERT_TRUE(camera_ptr_->SetUp());
  ASSERT_TRUE(camera_ptr_->set_up());
  camera_ptr_->set_n_leading_zeros(n_leading_zeros_);
  ASSERT_FALSE(camera_ptr_->set_up());
  camera_ptr_->set_load_index(load_index_);
  ASSERT_TRUE(camera_ptr_->SetUp());
  ASSERT_TRUE(camera_ptr_->set_up());
  camera_ptr_->set_image_name_post(image_name_post_);
  ASSERT_FALSE(camera_ptr_->set_up());
  camera_ptr_->set_load_index(load_index_);
  ASSERT_TRUE(camera_ptr_->SetUp());
  ASSERT_TRUE(camera_ptr_->set_up());
  camera_ptr_->set_load_image_type(load_image_type_);
  ASSERT_TRUE(TestCameraParameters(*camera_ptr_));
}

TEST_F(LoaderColorCameraTest, TestPose) {
  camera_ptr_->set_world2camera_pose(pose_);
  ASSERT_TRUE(camera_ptr_->world2camera_pose().matrix() == pose_.matrix());
  ASSERT_TRUE(camera_ptr_->camera2world_pose().matrix() ==
              pose_.inverse().matrix());
  camera_ptr_->set_camera2world_pose(pose_);
  ASSERT_TRUE(camera_ptr_->camera2world_pose().matrix() == pose_.matrix());
  ASSERT_TRUE(camera_ptr_->world2camera_pose().matrix() ==
              pose_.inverse().matrix());
}

TEST_F(LoaderColorCameraTest, TestWithoutSetUp) {
  ASSERT_FALSE(camera_ptr_->UpdateImage(true));
}

TEST_F(LoaderColorCameraTest, UpdateImage) {
  ASSERT_TRUE(camera_ptr_->SetUp());
  ASSERT_TRUE(camera_ptr_->UpdateImage(true));
  ASSERT_TRUE(camera_ptr_->load_index() == load_index_ + 2);
  ASSERT_TRUE(CompareToLoadedImage(sequence_directory,
                                   "color_camera_image_201.png",
                                   camera_ptr_->image(), 0, 0));
}

TEST_F(LoaderColorCameraTest, SaveAndLoadImage) {
  std::filesystem::create_directory(temp_directory);
  camera_ptr_->StartSavingImages(temp_directory, 123, "bmp");
  ASSERT_TRUE(camera_ptr_->SetUp());
  ASSERT_TRUE(CompareToLoadedImage(temp_directory, "color_camera_image_123.bmp",
                                   camera_ptr_->image(), 0, 0));
  m3t::LoaderColorCamera temp_camera{"temp_camera", metafile_path_};
  ASSERT_TRUE(temp_camera.SetUp());
  ASSERT_TRUE(CompareImages(camera_ptr_->image(), temp_camera.image(), 0, 0));
}

class LoaderDepthCameraTest : public testing::Test {
 protected:
  void SetUp() override {
    camera_ptr_ = std::make_shared<m3t::LoaderDepthCamera>(
        name_, load_directory_, intrinsics_, depth_scale_, image_name_pre_,
        load_index_, n_leading_zeros_, image_name_post_, load_image_type_);
  }

  bool TestCameraParameters(const m3t::LoaderDepthCamera &camera) {
    return camera.name() == name_ &&
           m3t::Equivalent(camera.load_directory(), load_directory_) &&
           camera.intrinsics().fu == intrinsics_.fu &&
           camera.intrinsics().fv == intrinsics_.fv &&
           camera.depth_scale() == depth_scale_ &&
           camera.image_name_pre() == image_name_pre_ &&
           camera.n_leading_zeros() == n_leading_zeros_ &&
           camera.image_name_post() == image_name_post_ &&
           camera.load_image_type() == load_image_type_;
  }

  std::shared_ptr<m3t::LoaderDepthCamera> camera_ptr_;
  std::string name_{"depth_camera"};
  std::filesystem::path load_directory_{sequence_directory};
  std::filesystem::path metafile_path_{sequence_directory /
                                       "depth_camera.yaml"};
  m3t::Intrinsics intrinsics_{425.773f, 425.773f, 427.202f, 237.662f, 848, 480};
  float depth_scale_ = 0.001f;
  std::string image_name_pre_{"depth_camera_image_"};
  int load_index_ = 200;
  int n_leading_zeros_ = 0;
  std::string image_name_post_ = "";
  std::string load_image_type_ = "png";
  m3t::Transform3fA pose_{Eigen::Translation3f{-0.2f, 0.1f, 0.4f}};
};

TEST_F(LoaderDepthCameraTest, SetUpFromData) {
  ASSERT_TRUE(TestCameraParameters(*camera_ptr_));
  ASSERT_TRUE(camera_ptr_->SetUp());
  ASSERT_TRUE(camera_ptr_->set_up());
  ASSERT_TRUE(TestCameraParameters(*camera_ptr_));
}

TEST_F(LoaderDepthCameraTest, SetUpFromMetaFile) {
  m3t::LoaderDepthCamera camera{name_, metafile_path_};
  ASSERT_TRUE(camera.SetUp());
  ASSERT_TRUE(camera.set_up());
  ASSERT_TRUE(TestCameraParameters(camera));
}

TEST_F(LoaderDepthCameraTest, ParameterChanges) {
  ASSERT_TRUE(camera_ptr_->SetUp());
  ASSERT_TRUE(camera_ptr_->set_up());
  camera_ptr_->StartSavingImages(temp_directory, 123, "bmp");
  ASSERT_TRUE(camera_ptr_->save_images());
  ASSERT_FALSE(camera_ptr_->set_up());
  camera_ptr_->StopSavingImages();
  camera_ptr_->set_load_index(load_index_);
  ASSERT_TRUE(camera_ptr_->SetUp());
  ASSERT_TRUE(camera_ptr_->set_up());
  camera_ptr_->set_load_directory(load_directory_);
  ASSERT_FALSE(camera_ptr_->set_up());
  camera_ptr_->set_load_index(load_index_);
  ASSERT_TRUE(camera_ptr_->SetUp());
  ASSERT_TRUE(camera_ptr_->set_up());
  camera_ptr_->set_metafile_path(metafile_path_);
  ASSERT_FALSE(camera_ptr_->set_up());
  camera_ptr_->set_load_index(load_index_);
  ASSERT_TRUE(camera_ptr_->SetUp());
  ASSERT_TRUE(camera_ptr_->set_up());
  camera_ptr_->set_intrinsics(intrinsics_);
  ASSERT_FALSE(camera_ptr_->set_up());
  camera_ptr_->set_load_index(load_index_);
  ASSERT_TRUE(camera_ptr_->SetUp());
  ASSERT_TRUE(camera_ptr_->set_up());
  camera_ptr_->set_depth_scale(depth_scale_);
  ASSERT_FALSE(camera_ptr_->set_up());
  camera_ptr_->set_load_index(load_index_);
  ASSERT_TRUE(camera_ptr_->SetUp());
  ASSERT_TRUE(camera_ptr_->set_up());
  camera_ptr_->set_image_name_pre(image_name_pre_);
  ASSERT_FALSE(camera_ptr_->set_up());
  camera_ptr_->set_load_index(load_index_);
  ASSERT_TRUE(camera_ptr_->SetUp());
  ASSERT_TRUE(camera_ptr_->set_up());
  camera_ptr_->set_load_index(load_index_);
  ASSERT_FALSE(camera_ptr_->set_up());
  camera_ptr_->set_load_index(load_index_);
  ASSERT_TRUE(camera_ptr_->SetUp());
  ASSERT_TRUE(camera_ptr_->set_up());
  camera_ptr_->set_n_leading_zeros(n_leading_zeros_);
  ASSERT_FALSE(camera_ptr_->set_up());
  camera_ptr_->set_load_index(load_index_);
  ASSERT_TRUE(camera_ptr_->SetUp());
  ASSERT_TRUE(camera_ptr_->set_up());
  camera_ptr_->set_image_name_post(image_name_post_);
  ASSERT_FALSE(camera_ptr_->set_up());
  camera_ptr_->set_load_index(load_index_);
  ASSERT_TRUE(camera_ptr_->SetUp());
  ASSERT_TRUE(camera_ptr_->set_up());
  camera_ptr_->set_load_image_type(load_image_type_);
  ASSERT_TRUE(TestCameraParameters(*camera_ptr_));
}

TEST_F(LoaderDepthCameraTest, TestPose) {
  camera_ptr_->set_world2camera_pose(pose_);
  ASSERT_TRUE(camera_ptr_->world2camera_pose().matrix() == pose_.matrix());
  ASSERT_TRUE(camera_ptr_->camera2world_pose().matrix() ==
              pose_.inverse().matrix());
  camera_ptr_->set_camera2world_pose(pose_);
  ASSERT_TRUE(camera_ptr_->camera2world_pose().matrix() == pose_.matrix());
  ASSERT_TRUE(camera_ptr_->world2camera_pose().matrix() ==
              pose_.inverse().matrix());
}

TEST_F(LoaderDepthCameraTest, TestWithoutSetUp) {
  ASSERT_FALSE(camera_ptr_->UpdateImage(true));
}

TEST_F(LoaderDepthCameraTest, UpdateImage) {
  ASSERT_TRUE(camera_ptr_->SetUp());
  ASSERT_TRUE(camera_ptr_->UpdateImage(true));
  ASSERT_TRUE(camera_ptr_->load_index() == load_index_ + 2);
  ASSERT_TRUE(CompareToLoadedImage(sequence_directory,
                                   "depth_camera_image_201.png",
                                   camera_ptr_->image(), 0, 0));
}

TEST_F(LoaderDepthCameraTest, SaveAndLoadImage) {
  std::filesystem::create_directory(temp_directory);
  camera_ptr_->StartSavingImages(temp_directory, 123, "png");
  ASSERT_TRUE(camera_ptr_->SetUp());
  ASSERT_TRUE(CompareToLoadedImage(temp_directory, "depth_camera_image_123.png",
                                   camera_ptr_->image(), 0, 0));
  m3t::LoaderDepthCamera temp_camera{"temp_camera", metafile_path_};
  ASSERT_TRUE(temp_camera.SetUp());
  ASSERT_TRUE(CompareImages(camera_ptr_->image(), temp_camera.image(), 0, 0));
}
