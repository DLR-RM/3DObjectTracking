// SPDX-License-Identifier: MIT
// Copyright (c) 2023 Manuel Stoiber, German Aerospace Center (DLR)

#include <gtest/gtest.h>
#include <m3t/basic_depth_renderer.h>
#include <m3t/body.h>
#include <m3t/camera.h>
#include <m3t/normal_renderer.h>
#include <m3t/renderer_geometry.h>
#include <m3t/silhouette_renderer.h>

#include "common_test.h"

const std::filesystem::path renderer_test_directory{data_directory /
                                                    "renderer_test"};

class FullSilhouetteRendererTest : public testing::Test {
 protected:
  void SetUp() override {
    auto triangle_body_ptr{TriangleBodyPtr()};
    auto schauma_body_ptr{SchaumaBodyPtr()};
    renderer_geometry_ptr_ =
        std::make_shared<m3t::RendererGeometry>("renderer_geometry");
    renderer_geometry_ptr_->SetUp();
    renderer_geometry_ptr_->AddBody(triangle_body_ptr);
    renderer_geometry_ptr_->AddBody(schauma_body_ptr);

    renderer_ptr_ = std::make_shared<m3t::FullSilhouetteRenderer>(
        name_, renderer_geometry_ptr_, world2camera_pose_, intrinsics_,
        id_type_, z_min_, z_max_);
  }

  bool TestRendererParameters(const m3t::FullSilhouetteRenderer &renderer) {
    return renderer.name() == name_ &&
           renderer.world2camera_pose().matrix() ==
               world2camera_pose_.matrix() &&
           renderer.intrinsics().fu == intrinsics_.fu &&
           renderer.intrinsics().fv == intrinsics_.fv &&
           renderer.id_type() == id_type_ && renderer.z_min() == z_min_ &&
           renderer.z_max() == z_max_;
  }

  std::shared_ptr<m3t::RendererGeometry> renderer_geometry_ptr_;
  std::shared_ptr<m3t::FullSilhouetteRenderer> renderer_ptr_;
  std::string name_{"renderer"};
  std::filesystem::path metafile_path_{renderer_test_directory /
                                       "full_silhouette_renderer.yaml"};
  m3t::Transform3fA world2camera_pose_{Eigen::Translation3f{0.01f, 0.0f, 0.0f}};
  m3t::Intrinsics intrinsics_{698.128f, 698.617f, 478.459f, 274.426f, 640, 480};
  m3t::IDType id_type_ = m3t::IDType::BODY;
  float z_min_ = 0.1f;
  float z_max_ = 2.0f;
};

TEST_F(FullSilhouetteRendererTest, SetUpFromData) {
  ASSERT_TRUE(TestRendererParameters(*renderer_ptr_));
  ASSERT_TRUE(renderer_ptr_->SetUp());
  ASSERT_TRUE(renderer_ptr_->set_up());
  ASSERT_TRUE(TestRendererParameters(*renderer_ptr_));
}

TEST_F(FullSilhouetteRendererTest, SetUpFromCamera) {
  auto camera_ptr{ColorCameraPtr()};
  camera_ptr->set_world2camera_pose(world2camera_pose_);
  m3t::FullSilhouetteRenderer renderer{
      name_, renderer_geometry_ptr_, camera_ptr, id_type_, z_min_, z_max_};
  ASSERT_TRUE(renderer.SetUp());
  ASSERT_TRUE(renderer.set_up());
  ASSERT_TRUE(TestRendererParameters(renderer));
}

TEST_F(FullSilhouetteRendererTest, SetUpFromMetaFile) {
  auto camera_ptr{ColorCameraPtr()};
  camera_ptr->set_world2camera_pose(world2camera_pose_);
  m3t::FullSilhouetteRenderer renderer{name_, metafile_path_,
                                       renderer_geometry_ptr_, camera_ptr};
  ASSERT_TRUE(renderer.SetUp());
  ASSERT_TRUE(renderer.set_up());
  ASSERT_TRUE(TestRendererParameters(renderer));
}

TEST_F(FullSilhouetteRendererTest, ParameterChanges) {
  ASSERT_TRUE(renderer_ptr_->SetUp());
  ASSERT_TRUE(renderer_ptr_->set_up());
  renderer_ptr_->set_camera_ptr(nullptr);
  ASSERT_FALSE(renderer_ptr_->set_up());
  ASSERT_TRUE(renderer_ptr_->SetUp());
  ASSERT_TRUE(renderer_ptr_->set_up());
  renderer_ptr_->set_renderer_geometry_ptr(renderer_geometry_ptr_);
  ASSERT_FALSE(renderer_ptr_->set_up());
  ASSERT_TRUE(renderer_ptr_->SetUp());
  ASSERT_TRUE(renderer_ptr_->set_up());
  renderer_ptr_->set_intrinsics(intrinsics_);
  ASSERT_FALSE(renderer_ptr_->set_up());
  ASSERT_TRUE(renderer_ptr_->SetUp());
  ASSERT_TRUE(renderer_ptr_->set_up());
  renderer_ptr_->set_z_min(z_min_);
  ASSERT_FALSE(renderer_ptr_->set_up());
  ASSERT_TRUE(renderer_ptr_->SetUp());
  ASSERT_TRUE(renderer_ptr_->set_up());
  renderer_ptr_->set_z_max(z_max_);
  ASSERT_FALSE(renderer_ptr_->set_up());
  ASSERT_TRUE(renderer_ptr_->SetUp());
  ASSERT_TRUE(renderer_ptr_->set_up());
  ASSERT_TRUE(TestRendererParameters(*renderer_ptr_));
}

TEST_F(FullSilhouetteRendererTest, TestWithoutSetUpRendererGeometry) {
  renderer_ptr_->set_renderer_geometry_ptr(
      std::make_shared<m3t::RendererGeometry>("rg"));
  ASSERT_FALSE(renderer_ptr_->SetUp());
}

TEST_F(FullSilhouetteRendererTest, TestWithoutSetUpCamera) {
  renderer_ptr_->set_camera_ptr(ColorCameraPtrNoSetUp());
  ASSERT_FALSE(renderer_ptr_->SetUp());
}

TEST_F(FullSilhouetteRendererTest, TestWithoutSetUp) {
  ASSERT_FALSE(renderer_ptr_->StartRendering());
  ASSERT_FALSE(renderer_ptr_->FetchSilhouetteImage());
  ASSERT_FALSE(renderer_ptr_->FetchDepthImage());
}

TEST_F(FullSilhouetteRendererTest, TestWithoutRendering) {
  renderer_ptr_->SetUp();
  ASSERT_FALSE(renderer_ptr_->FetchSilhouetteImage());
  ASSERT_FALSE(renderer_ptr_->FetchDepthImage());
}

TEST_F(FullSilhouetteRendererTest, TestSilhouetteImage) {
  ASSERT_TRUE(renderer_ptr_->SetUp());
  ASSERT_TRUE(renderer_ptr_->StartRendering());
  ASSERT_TRUE(renderer_ptr_->FetchSilhouetteImage());
  SaveOrSkipImage(kGenerateGroundTruth, renderer_test_directory,
                  "silhouette_image.png", renderer_ptr_->silhouette_image());
  ASSERT_TRUE(CompareToLoadedImage(renderer_test_directory,
                                   "silhouette_image.png",
                                   renderer_ptr_->silhouette_image(), 0, 10));
}

TEST_F(FullSilhouetteRendererTest, TestDepthImage) {
  ASSERT_TRUE(renderer_ptr_->SetUp());
  ASSERT_TRUE(renderer_ptr_->StartRendering());
  ASSERT_TRUE(renderer_ptr_->FetchDepthImage());
  SaveOrSkipImage(kGenerateGroundTruth, renderer_test_directory,
                  "depth_image.png", renderer_ptr_->depth_image());
  ASSERT_TRUE(CompareToLoadedImage(renderer_test_directory, "depth_image.png",
                                   renderer_ptr_->depth_image(), 0, 10));
}

class FocusedSilhouetteRendererTest : public testing::Test {
 protected:
  void SetUp() override {
    triangle_body_ptr_ = TriangleBodyPtr();
    auto schauma_body_ptr{SchaumaBodyPtr()};
    renderer_geometry_ptr_ =
        std::make_shared<m3t::RendererGeometry>("renderer_geometry");
    renderer_geometry_ptr_->SetUp();
    renderer_geometry_ptr_->AddBody(triangle_body_ptr_);
    renderer_geometry_ptr_->AddBody(schauma_body_ptr);

    renderer_ptr_ = std::make_shared<m3t::FocusedSilhouetteRenderer>(
        name_, renderer_geometry_ptr_, world2camera_pose_, intrinsics_,
        id_type_, image_size_, z_min_, z_max_);
    renderer_ptr_->AddReferencedBody(triangle_body_ptr_);
  }

  bool TestRendererParameters(const m3t::FocusedSilhouetteRenderer &renderer) {
    return renderer.name() == name_ &&
           renderer.world2camera_pose().matrix() ==
               world2camera_pose_.matrix() &&
           renderer.intrinsics().fu == intrinsics_.fu &&
           renderer.intrinsics().fv == intrinsics_.fv &&
           renderer.id_type() == id_type_ &&
           renderer.image_size() == image_size_ && renderer.z_min() == z_min_ &&
           renderer.z_max() == z_max_;
  }

  std::shared_ptr<m3t::Body> triangle_body_ptr_;
  std::shared_ptr<m3t::RendererGeometry> renderer_geometry_ptr_;
  std::shared_ptr<m3t::FocusedSilhouetteRenderer> renderer_ptr_;
  std::string name_{"renderer"};
  std::filesystem::path metafile_path_{renderer_test_directory /
                                       "focused_silhouette_renderer.yaml"};
  m3t::Transform3fA world2camera_pose_{Eigen::Translation3f{0.01f, 0.0f, 0.0f}};
  m3t::Intrinsics intrinsics_{698.128f, 698.617f, 478.459f, 274.426f, 640, 480};
  m3t::IDType id_type_ = m3t::IDType::BODY;
  int image_size_ = 200;
  float z_min_ = 0.1f;
  float z_max_ = 2.0f;
};

TEST_F(FocusedSilhouetteRendererTest, SetUpFromData) {
  ASSERT_TRUE(TestRendererParameters(*renderer_ptr_));
  ASSERT_TRUE(renderer_ptr_->SetUp());
  ASSERT_TRUE(renderer_ptr_->set_up());
  ASSERT_TRUE(TestRendererParameters(*renderer_ptr_));
}

TEST_F(FocusedSilhouetteRendererTest, SetUpFromCamera) {
  auto camera_ptr{ColorCameraPtr()};
  camera_ptr->set_world2camera_pose(world2camera_pose_);
  m3t::FocusedSilhouetteRenderer renderer{
      name_, renderer_geometry_ptr_, camera_ptr, id_type_, image_size_, z_min_,
      z_max_};
  renderer.AddReferencedBody(triangle_body_ptr_);
  ASSERT_TRUE(renderer.SetUp());
  ASSERT_TRUE(renderer.set_up());
  ASSERT_TRUE(TestRendererParameters(renderer));
}

TEST_F(FocusedSilhouetteRendererTest, SetUpFromMetaFile) {
  auto camera_ptr{ColorCameraPtr()};
  camera_ptr->set_world2camera_pose(world2camera_pose_);
  m3t::FocusedSilhouetteRenderer renderer{name_, metafile_path_,
                                          renderer_geometry_ptr_, camera_ptr};
  renderer.AddReferencedBody(triangle_body_ptr_);
  ASSERT_TRUE(renderer.SetUp());
  ASSERT_TRUE(renderer.set_up());
  ASSERT_TRUE(TestRendererParameters(renderer));
}

TEST_F(FocusedSilhouetteRendererTest, ParameterChanges) {
  ASSERT_TRUE(renderer_ptr_->SetUp());
  ASSERT_TRUE(renderer_ptr_->set_up());
  renderer_ptr_->set_camera_ptr(nullptr);
  ASSERT_FALSE(renderer_ptr_->set_up());
  ASSERT_TRUE(renderer_ptr_->SetUp());
  ASSERT_TRUE(renderer_ptr_->set_up());
  renderer_ptr_->set_renderer_geometry_ptr(renderer_geometry_ptr_);
  ASSERT_FALSE(renderer_ptr_->set_up());
  ASSERT_TRUE(renderer_ptr_->SetUp());
  ASSERT_TRUE(renderer_ptr_->set_up());
  renderer_ptr_->set_intrinsics(intrinsics_);
  ASSERT_FALSE(renderer_ptr_->set_up());
  ASSERT_TRUE(renderer_ptr_->SetUp());
  ASSERT_TRUE(renderer_ptr_->set_up());
  renderer_ptr_->set_z_min(z_min_);
  ASSERT_FALSE(renderer_ptr_->set_up());
  ASSERT_TRUE(renderer_ptr_->SetUp());
  ASSERT_TRUE(renderer_ptr_->set_up());
  renderer_ptr_->set_image_size(image_size_);
  ASSERT_FALSE(renderer_ptr_->set_up());
  ASSERT_TRUE(renderer_ptr_->SetUp());
  ASSERT_TRUE(renderer_ptr_->set_up());
  renderer_ptr_->set_z_max(z_max_);
  ASSERT_FALSE(renderer_ptr_->set_up());
  ASSERT_TRUE(renderer_ptr_->SetUp());
  ASSERT_TRUE(renderer_ptr_->set_up());
  ASSERT_TRUE(TestRendererParameters(*renderer_ptr_));
}

TEST_F(FocusedSilhouetteRendererTest, AddDeleteClearReferencedBodies) {
  ASSERT_TRUE(renderer_ptr_->SetUp());
  ASSERT_TRUE(renderer_ptr_->set_up());
  renderer_ptr_->ClearReferencedBodies();
  ASSERT_FALSE(renderer_ptr_->set_up());
  ASSERT_FALSE(renderer_ptr_->SetUp());
  ASSERT_TRUE(renderer_ptr_->AddReferencedBody(triangle_body_ptr_));
  ASSERT_FALSE(renderer_ptr_->AddReferencedBody(triangle_body_ptr_));
  ASSERT_FALSE(renderer_ptr_->set_up());
  ASSERT_TRUE(renderer_ptr_->SetUp());
  ASSERT_TRUE(renderer_ptr_->set_up());
  ASSERT_TRUE(renderer_ptr_->DeleteReferencedBody(triangle_body_ptr_->name()));
  ASSERT_FALSE(renderer_ptr_->DeleteReferencedBody(triangle_body_ptr_->name()));
  ASSERT_FALSE(renderer_ptr_->set_up());
  ASSERT_FALSE(renderer_ptr_->SetUp());
  ASSERT_TRUE(renderer_ptr_->referenced_body_ptrs().empty());
}

TEST_F(FocusedSilhouetteRendererTest, TestWithoutSetUpRendererGeometry) {
  renderer_ptr_->set_renderer_geometry_ptr(
      std::make_shared<m3t::RendererGeometry>("rg"));
  ASSERT_FALSE(renderer_ptr_->SetUp());
}

TEST_F(FocusedSilhouetteRendererTest, TestWithoutSetUpCamera) {
  renderer_ptr_->set_camera_ptr(ColorCameraPtrNoSetUp());
  ASSERT_FALSE(renderer_ptr_->SetUp());
}

TEST_F(FocusedSilhouetteRendererTest, TestWithoutSetUpReferencedBody) {
  renderer_ptr_->ClearReferencedBodies();
  renderer_ptr_->AddReferencedBody(TriangleBodyPtrNoSetUp());
  ASSERT_FALSE(renderer_ptr_->SetUp());
}

TEST_F(FocusedSilhouetteRendererTest, TestWithoutSetUp) {
  ASSERT_FALSE(renderer_ptr_->StartRendering());
  ASSERT_FALSE(renderer_ptr_->FetchSilhouetteImage());
  ASSERT_FALSE(renderer_ptr_->FetchDepthImage());
}

TEST_F(FocusedSilhouetteRendererTest, TestWithoutRendering) {
  renderer_ptr_->SetUp();
  ASSERT_FALSE(renderer_ptr_->FetchSilhouetteImage());
  ASSERT_FALSE(renderer_ptr_->FetchDepthImage());
}

TEST_F(FocusedSilhouetteRendererTest, TestSilhouetteImage) {
  ASSERT_TRUE(renderer_ptr_->SetUp());
  ASSERT_TRUE(renderer_ptr_->StartRendering());
  ASSERT_TRUE(renderer_ptr_->FetchSilhouetteImage());
  SaveOrSkipImage(kGenerateGroundTruth, renderer_test_directory,
                  "focused_silhouette_image.png",
                  renderer_ptr_->focused_silhouette_image());
  ASSERT_TRUE(CompareToLoadedImage(
      renderer_test_directory, "focused_silhouette_image.png",
      renderer_ptr_->focused_silhouette_image(), 0, 10));
}

TEST_F(FocusedSilhouetteRendererTest, TestDepthImage) {
  ASSERT_TRUE(renderer_ptr_->SetUp());
  ASSERT_TRUE(renderer_ptr_->StartRendering());
  ASSERT_TRUE(renderer_ptr_->FetchDepthImage());
  SaveOrSkipImage(kGenerateGroundTruth, renderer_test_directory,
                  "focused_depth_image.png",
                  renderer_ptr_->focused_depth_image());
  ASSERT_TRUE(
      CompareToLoadedImage(renderer_test_directory, "focused_depth_image.png",
                           renderer_ptr_->focused_depth_image(), 0, 10));
}

class FullNormalRendererTest : public testing::Test {
 protected:
  void SetUp() override {
    auto triangle_body_ptr{TriangleBodyPtr()};
    auto schauma_body_ptr{SchaumaBodyPtr()};
    renderer_geometry_ptr_ =
        std::make_shared<m3t::RendererGeometry>("renderer_geometry");
    renderer_geometry_ptr_->SetUp();
    renderer_geometry_ptr_->AddBody(triangle_body_ptr);
    renderer_geometry_ptr_->AddBody(schauma_body_ptr);

    renderer_ptr_ = std::make_shared<m3t::FullNormalRenderer>(
        name_, renderer_geometry_ptr_, world2camera_pose_, intrinsics_, z_min_,
        z_max_);
  }

  bool TestRendererParameters(const m3t::FullNormalRenderer &renderer) {
    return renderer.name() == name_ &&
           renderer.world2camera_pose().matrix() ==
               world2camera_pose_.matrix() &&
           renderer.intrinsics().fu == intrinsics_.fu &&
           renderer.intrinsics().fv == intrinsics_.fv &&
           renderer.z_min() == z_min_ && renderer.z_max() == z_max_;
  }

  std::shared_ptr<m3t::RendererGeometry> renderer_geometry_ptr_;
  std::shared_ptr<m3t::FullNormalRenderer> renderer_ptr_;
  std::string name_{"renderer"};
  std::filesystem::path metafile_path_{renderer_test_directory /
                                       "full_normal_renderer.yaml"};
  m3t::Transform3fA world2camera_pose_{Eigen::Translation3f{0.01f, 0.0f, 0.0f}};
  m3t::Intrinsics intrinsics_{698.128f, 698.617f, 478.459f, 274.426f, 640, 480};
  float z_min_ = 0.1f;
  float z_max_ = 2.0f;
};

TEST_F(FullNormalRendererTest, SetUpFromData) {
  ASSERT_TRUE(TestRendererParameters(*renderer_ptr_));
  ASSERT_TRUE(renderer_ptr_->SetUp());
  ASSERT_TRUE(renderer_ptr_->set_up());
  ASSERT_TRUE(TestRendererParameters(*renderer_ptr_));
}

TEST_F(FullNormalRendererTest, SetUpFromCamera) {
  auto camera_ptr{ColorCameraPtr()};
  camera_ptr->set_world2camera_pose(world2camera_pose_);
  m3t::FullNormalRenderer renderer{name_, renderer_geometry_ptr_, camera_ptr,
                                   z_min_, z_max_};
  ASSERT_TRUE(renderer.SetUp());
  ASSERT_TRUE(renderer.set_up());
  ASSERT_TRUE(TestRendererParameters(renderer));
}

TEST_F(FullNormalRendererTest, SetUpFromMetaFile) {
  auto camera_ptr{ColorCameraPtr()};
  camera_ptr->set_world2camera_pose(world2camera_pose_);
  m3t::FullNormalRenderer renderer{name_, metafile_path_,
                                   renderer_geometry_ptr_, camera_ptr};
  ASSERT_TRUE(renderer.SetUp());
  ASSERT_TRUE(renderer.set_up());
  ASSERT_TRUE(TestRendererParameters(renderer));
}

TEST_F(FullNormalRendererTest, ParameterChanges) {
  ASSERT_TRUE(renderer_ptr_->SetUp());
  ASSERT_TRUE(renderer_ptr_->set_up());
  renderer_ptr_->set_camera_ptr(nullptr);
  ASSERT_FALSE(renderer_ptr_->set_up());
  ASSERT_TRUE(renderer_ptr_->SetUp());
  ASSERT_TRUE(renderer_ptr_->set_up());
  renderer_ptr_->set_renderer_geometry_ptr(renderer_geometry_ptr_);
  ASSERT_FALSE(renderer_ptr_->set_up());
  ASSERT_TRUE(renderer_ptr_->SetUp());
  ASSERT_TRUE(renderer_ptr_->set_up());
  renderer_ptr_->set_intrinsics(intrinsics_);
  ASSERT_FALSE(renderer_ptr_->set_up());
  ASSERT_TRUE(renderer_ptr_->SetUp());
  ASSERT_TRUE(renderer_ptr_->set_up());
  renderer_ptr_->set_z_min(z_min_);
  ASSERT_FALSE(renderer_ptr_->set_up());
  ASSERT_TRUE(renderer_ptr_->SetUp());
  ASSERT_TRUE(renderer_ptr_->set_up());
  renderer_ptr_->set_z_max(z_max_);
  ASSERT_FALSE(renderer_ptr_->set_up());
  ASSERT_TRUE(renderer_ptr_->SetUp());
  ASSERT_TRUE(renderer_ptr_->set_up());
  ASSERT_TRUE(TestRendererParameters(*renderer_ptr_));
}

TEST_F(FullNormalRendererTest, TestWithoutSetUpRendererGeometry) {
  renderer_ptr_->set_renderer_geometry_ptr(
      std::make_shared<m3t::RendererGeometry>("rg"));
  ASSERT_FALSE(renderer_ptr_->SetUp());
}

TEST_F(FullNormalRendererTest, TestWithoutSetUpCamera) {
  renderer_ptr_->set_camera_ptr(ColorCameraPtrNoSetUp());
  ASSERT_FALSE(renderer_ptr_->SetUp());
}

TEST_F(FullNormalRendererTest, TestWithoutSetUp) {
  ASSERT_FALSE(renderer_ptr_->StartRendering());
  ASSERT_FALSE(renderer_ptr_->FetchNormalImage());
  ASSERT_FALSE(renderer_ptr_->FetchDepthImage());
}

TEST_F(FullNormalRendererTest, TestWithoutRendering) {
  renderer_ptr_->SetUp();
  ASSERT_FALSE(renderer_ptr_->FetchNormalImage());
  ASSERT_FALSE(renderer_ptr_->FetchDepthImage());
}

TEST_F(FullNormalRendererTest, TestNormalImage) {
  ASSERT_TRUE(renderer_ptr_->SetUp());
  ASSERT_TRUE(renderer_ptr_->StartRendering());
  ASSERT_TRUE(renderer_ptr_->FetchNormalImage());
  SaveOrSkipImage(kGenerateGroundTruth, renderer_test_directory,
                  "normal_image.png", renderer_ptr_->normal_image());
  ASSERT_TRUE(CompareToLoadedImage(renderer_test_directory, "normal_image.png",
                                   renderer_ptr_->normal_image(), 0, 10));
}

TEST_F(FullNormalRendererTest, TestDepthImage) {
  ASSERT_TRUE(renderer_ptr_->SetUp());
  ASSERT_TRUE(renderer_ptr_->StartRendering());
  ASSERT_TRUE(renderer_ptr_->FetchDepthImage());
  SaveOrSkipImage(kGenerateGroundTruth, renderer_test_directory,
                  "depth_image.png", renderer_ptr_->depth_image());
  ASSERT_TRUE(CompareToLoadedImage(renderer_test_directory, "depth_image.png",
                                   renderer_ptr_->depth_image(), 0, 10));
}

class FocusedNormalRendererTest : public testing::Test {
 protected:
  void SetUp() override {
    triangle_body_ptr_ = TriangleBodyPtr();
    auto schauma_body_ptr{SchaumaBodyPtr()};
    renderer_geometry_ptr_ =
        std::make_shared<m3t::RendererGeometry>("renderer_geometry");
    renderer_geometry_ptr_->SetUp();
    renderer_geometry_ptr_->AddBody(triangle_body_ptr_);
    renderer_geometry_ptr_->AddBody(schauma_body_ptr);

    renderer_ptr_ = std::make_shared<m3t::FocusedNormalRenderer>(
        name_, renderer_geometry_ptr_, world2camera_pose_, intrinsics_,
        image_size_, z_min_, z_max_);
    renderer_ptr_->AddReferencedBody(triangle_body_ptr_);
  }

  bool TestRendererParameters(const m3t::FocusedNormalRenderer &renderer) {
    return renderer.name() == name_ &&
           renderer.world2camera_pose().matrix() ==
               world2camera_pose_.matrix() &&
           renderer.intrinsics().fu == intrinsics_.fu &&
           renderer.intrinsics().fv == intrinsics_.fv &&
           renderer.image_size() == image_size_ && renderer.z_min() == z_min_ &&
           renderer.z_max() == z_max_;
  }

  std::shared_ptr<m3t::Body> triangle_body_ptr_;
  std::shared_ptr<m3t::RendererGeometry> renderer_geometry_ptr_;
  std::shared_ptr<m3t::FocusedNormalRenderer> renderer_ptr_;
  std::string name_{"renderer"};
  std::filesystem::path metafile_path_{renderer_test_directory /
                                       "focused_normal_renderer.yaml"};
  m3t::Transform3fA world2camera_pose_{Eigen::Translation3f{0.01f, 0.0f, 0.0f}};
  m3t::Intrinsics intrinsics_{698.128f, 698.617f, 478.459f, 274.426f, 640, 480};
  int image_size_ = 200;
  float z_min_ = 0.1f;
  float z_max_ = 2.0f;
};

TEST_F(FocusedNormalRendererTest, SetUpFromData) {
  ASSERT_TRUE(TestRendererParameters(*renderer_ptr_));
  ASSERT_TRUE(renderer_ptr_->SetUp());
  ASSERT_TRUE(renderer_ptr_->set_up());
  ASSERT_TRUE(TestRendererParameters(*renderer_ptr_));
}

TEST_F(FocusedNormalRendererTest, SetUpFromCamera) {
  auto camera_ptr{ColorCameraPtr()};
  camera_ptr->set_world2camera_pose(world2camera_pose_);
  m3t::FocusedNormalRenderer renderer{
      name_, renderer_geometry_ptr_, camera_ptr, image_size_, z_min_, z_max_};
  renderer.AddReferencedBody(triangle_body_ptr_);
  ASSERT_TRUE(renderer.SetUp());
  ASSERT_TRUE(renderer.set_up());
  ASSERT_TRUE(TestRendererParameters(renderer));
}

TEST_F(FocusedNormalRendererTest, SetUpFromMetaFile) {
  auto camera_ptr{ColorCameraPtr()};
  camera_ptr->set_world2camera_pose(world2camera_pose_);
  m3t::FocusedNormalRenderer renderer{name_, metafile_path_,
                                      renderer_geometry_ptr_, camera_ptr};
  renderer.AddReferencedBody(triangle_body_ptr_);
  ASSERT_TRUE(renderer.SetUp());
  ASSERT_TRUE(renderer.set_up());
  ASSERT_TRUE(TestRendererParameters(renderer));
}

TEST_F(FocusedNormalRendererTest, ParameterChanges) {
  ASSERT_TRUE(renderer_ptr_->SetUp());
  ASSERT_TRUE(renderer_ptr_->set_up());
  renderer_ptr_->set_camera_ptr(nullptr);
  ASSERT_FALSE(renderer_ptr_->set_up());
  ASSERT_TRUE(renderer_ptr_->SetUp());
  ASSERT_TRUE(renderer_ptr_->set_up());
  renderer_ptr_->set_renderer_geometry_ptr(renderer_geometry_ptr_);
  ASSERT_FALSE(renderer_ptr_->set_up());
  ASSERT_TRUE(renderer_ptr_->SetUp());
  ASSERT_TRUE(renderer_ptr_->set_up());
  renderer_ptr_->set_intrinsics(intrinsics_);
  ASSERT_FALSE(renderer_ptr_->set_up());
  ASSERT_TRUE(renderer_ptr_->SetUp());
  ASSERT_TRUE(renderer_ptr_->set_up());
  renderer_ptr_->set_z_min(z_min_);
  ASSERT_FALSE(renderer_ptr_->set_up());
  ASSERT_TRUE(renderer_ptr_->SetUp());
  ASSERT_TRUE(renderer_ptr_->set_up());
  renderer_ptr_->set_image_size(image_size_);
  ASSERT_FALSE(renderer_ptr_->set_up());
  ASSERT_TRUE(renderer_ptr_->SetUp());
  ASSERT_TRUE(renderer_ptr_->set_up());
  renderer_ptr_->set_z_max(z_max_);
  ASSERT_FALSE(renderer_ptr_->set_up());
  ASSERT_TRUE(renderer_ptr_->SetUp());
  ASSERT_TRUE(renderer_ptr_->set_up());
  ASSERT_TRUE(TestRendererParameters(*renderer_ptr_));
}

TEST_F(FocusedNormalRendererTest, AddDeleteClearReferencedBodies) {
  ASSERT_TRUE(renderer_ptr_->SetUp());
  ASSERT_TRUE(renderer_ptr_->set_up());
  renderer_ptr_->ClearReferencedBodies();
  ASSERT_FALSE(renderer_ptr_->set_up());
  ASSERT_FALSE(renderer_ptr_->SetUp());
  ASSERT_TRUE(renderer_ptr_->AddReferencedBody(triangle_body_ptr_));
  ASSERT_FALSE(renderer_ptr_->AddReferencedBody(triangle_body_ptr_));
  ASSERT_FALSE(renderer_ptr_->set_up());
  ASSERT_TRUE(renderer_ptr_->SetUp());
  ASSERT_TRUE(renderer_ptr_->set_up());
  ASSERT_TRUE(renderer_ptr_->DeleteReferencedBody(triangle_body_ptr_->name()));
  ASSERT_FALSE(renderer_ptr_->DeleteReferencedBody(triangle_body_ptr_->name()));
  ASSERT_FALSE(renderer_ptr_->set_up());
  ASSERT_FALSE(renderer_ptr_->SetUp());
  ASSERT_TRUE(renderer_ptr_->referenced_body_ptrs().empty());
}

TEST_F(FocusedNormalRendererTest, TestWithoutSetUpRendererGeometry) {
  renderer_ptr_->set_renderer_geometry_ptr(
      std::make_shared<m3t::RendererGeometry>("rg"));
  ASSERT_FALSE(renderer_ptr_->SetUp());
}

TEST_F(FocusedNormalRendererTest, TestWithoutSetUpCamera) {
  renderer_ptr_->set_camera_ptr(ColorCameraPtrNoSetUp());
  ASSERT_FALSE(renderer_ptr_->SetUp());
}

TEST_F(FocusedNormalRendererTest, TestWithoutSetUpReferencedBody) {
  renderer_ptr_->ClearReferencedBodies();
  renderer_ptr_->AddReferencedBody(TriangleBodyPtrNoSetUp());
  ASSERT_FALSE(renderer_ptr_->SetUp());
}

TEST_F(FocusedNormalRendererTest, TestWithoutSetUp) {
  ASSERT_FALSE(renderer_ptr_->StartRendering());
  ASSERT_FALSE(renderer_ptr_->FetchNormalImage());
  ASSERT_FALSE(renderer_ptr_->FetchDepthImage());
}

TEST_F(FocusedNormalRendererTest, TestWithoutRendering) {
  renderer_ptr_->SetUp();
  ASSERT_FALSE(renderer_ptr_->FetchNormalImage());
  ASSERT_FALSE(renderer_ptr_->FetchDepthImage());
}

TEST_F(FocusedNormalRendererTest, TestNormalImage) {
  ASSERT_TRUE(renderer_ptr_->SetUp());
  ASSERT_TRUE(renderer_ptr_->StartRendering());
  ASSERT_TRUE(renderer_ptr_->FetchNormalImage());
  SaveOrSkipImage(kGenerateGroundTruth, renderer_test_directory,
                  "focused_normal_image.png",
                  renderer_ptr_->focused_normal_image());
  ASSERT_TRUE(
      CompareToLoadedImage(renderer_test_directory, "focused_normal_image.png",
                           renderer_ptr_->focused_normal_image(), 0, 10));
}

TEST_F(FocusedNormalRendererTest, TestDepthImage) {
  ASSERT_TRUE(renderer_ptr_->SetUp());
  ASSERT_TRUE(renderer_ptr_->StartRendering());
  ASSERT_TRUE(renderer_ptr_->FetchDepthImage());
  SaveOrSkipImage(kGenerateGroundTruth, renderer_test_directory,
                  "focused_depth_image.png",
                  renderer_ptr_->focused_depth_image());
  ASSERT_TRUE(
      CompareToLoadedImage(renderer_test_directory, "focused_depth_image.png",
                           renderer_ptr_->focused_depth_image(), 0, 10));
}

class FullBasicDepthRendererTest : public testing::Test {
 protected:
  void SetUp() override {
    triangle_body_ptr_ = TriangleBodyPtr();
    auto schauma_body_ptr{SchaumaBodyPtr()};
    renderer_geometry_ptr_ =
        std::make_shared<m3t::RendererGeometry>("renderer_geometry");
    renderer_geometry_ptr_->SetUp();
    renderer_geometry_ptr_->AddBody(triangle_body_ptr_);
    renderer_geometry_ptr_->AddBody(schauma_body_ptr);

    renderer_ptr_ = std::make_shared<m3t::FullBasicDepthRenderer>(
        name_, renderer_geometry_ptr_, world2camera_pose_, intrinsics_, z_min_,
        z_max_);
  }

  bool TestRendererParameters(const m3t::FullBasicDepthRenderer &renderer) {
    return renderer.name() == name_ &&
           renderer.world2camera_pose().matrix() ==
               world2camera_pose_.matrix() &&
           renderer.intrinsics().fu == intrinsics_.fu &&
           renderer.intrinsics().fv == intrinsics_.fv &&
           renderer.z_min() == z_min_ && renderer.z_max() == z_max_;
  }

  std::shared_ptr<m3t::Body> triangle_body_ptr_;
  std::shared_ptr<m3t::RendererGeometry> renderer_geometry_ptr_;
  std::shared_ptr<m3t::FullBasicDepthRenderer> renderer_ptr_;
  std::string name_{"renderer"};
  std::filesystem::path metafile_path_{renderer_test_directory /
                                       "full_basic_depth_renderer.yaml"};
  m3t::Transform3fA world2camera_pose_{Eigen::Translation3f{0.01f, 0.0f, 0.0f}};
  m3t::Intrinsics intrinsics_{698.128f, 698.617f, 478.459f, 274.426f, 640, 480};
  float z_min_ = 0.1f;
  float z_max_ = 2.0f;
};

TEST_F(FullBasicDepthRendererTest, SetUpFromData) {
  ASSERT_TRUE(TestRendererParameters(*renderer_ptr_));
  ASSERT_TRUE(renderer_ptr_->SetUp());
  ASSERT_TRUE(renderer_ptr_->set_up());
  ASSERT_TRUE(TestRendererParameters(*renderer_ptr_));
}

TEST_F(FullBasicDepthRendererTest, SetUpFromCamera) {
  auto camera_ptr{ColorCameraPtr()};
  camera_ptr->set_world2camera_pose(world2camera_pose_);
  m3t::FullBasicDepthRenderer renderer{name_, renderer_geometry_ptr_,
                                       camera_ptr, z_min_, z_max_};
  ASSERT_TRUE(renderer.SetUp());
  ASSERT_TRUE(renderer.set_up());
  ASSERT_TRUE(TestRendererParameters(renderer));
}

TEST_F(FullBasicDepthRendererTest, SetUpFromMetaFile) {
  auto camera_ptr{ColorCameraPtr()};
  camera_ptr->set_world2camera_pose(world2camera_pose_);
  m3t::FullBasicDepthRenderer renderer{name_, metafile_path_,
                                       renderer_geometry_ptr_, camera_ptr};
  ASSERT_TRUE(renderer.SetUp());
  ASSERT_TRUE(renderer.set_up());
  ASSERT_TRUE(TestRendererParameters(renderer));
}

TEST_F(FullBasicDepthRendererTest, ParameterChanges) {
  ASSERT_TRUE(renderer_ptr_->SetUp());
  ASSERT_TRUE(renderer_ptr_->set_up());
  renderer_ptr_->set_camera_ptr(nullptr);
  ASSERT_FALSE(renderer_ptr_->set_up());
  ASSERT_TRUE(renderer_ptr_->SetUp());
  ASSERT_TRUE(renderer_ptr_->set_up());
  renderer_ptr_->set_renderer_geometry_ptr(renderer_geometry_ptr_);
  ASSERT_FALSE(renderer_ptr_->set_up());
  ASSERT_TRUE(renderer_ptr_->SetUp());
  ASSERT_TRUE(renderer_ptr_->set_up());
  renderer_ptr_->set_intrinsics(intrinsics_);
  ASSERT_FALSE(renderer_ptr_->set_up());
  ASSERT_TRUE(renderer_ptr_->SetUp());
  ASSERT_TRUE(renderer_ptr_->set_up());
  renderer_ptr_->set_z_min(z_min_);
  ASSERT_FALSE(renderer_ptr_->set_up());
  ASSERT_TRUE(renderer_ptr_->SetUp());
  ASSERT_TRUE(renderer_ptr_->set_up());
  renderer_ptr_->set_z_max(z_max_);
  ASSERT_FALSE(renderer_ptr_->set_up());
  ASSERT_TRUE(renderer_ptr_->SetUp());
  ASSERT_TRUE(renderer_ptr_->set_up());
  ASSERT_TRUE(TestRendererParameters(*renderer_ptr_));
}

TEST_F(FullBasicDepthRendererTest, TestWithoutSetUpRendererGeometry) {
  renderer_ptr_->set_renderer_geometry_ptr(
      std::make_shared<m3t::RendererGeometry>("rg"));
  ASSERT_FALSE(renderer_ptr_->SetUp());
}

TEST_F(FullBasicDepthRendererTest, TestWithoutSetUpCamera) {
  renderer_ptr_->set_camera_ptr(ColorCameraPtrNoSetUp());
  ASSERT_FALSE(renderer_ptr_->SetUp());
}

TEST_F(FullBasicDepthRendererTest, TestWithoutSetUp) {
  ASSERT_FALSE(renderer_ptr_->StartRendering());
  ASSERT_FALSE(renderer_ptr_->FetchDepthImage());
}

TEST_F(FullBasicDepthRendererTest, TestWithoutRendering) {
  renderer_ptr_->SetUp();
  ASSERT_FALSE(renderer_ptr_->FetchDepthImage());
}

TEST_F(FullBasicDepthRendererTest, TestDepthImage) {
  ASSERT_TRUE(renderer_ptr_->SetUp());
  ASSERT_TRUE(renderer_ptr_->StartRendering());
  ASSERT_TRUE(renderer_ptr_->FetchDepthImage());
  SaveOrSkipImage(kGenerateGroundTruth, renderer_test_directory,
                  "depth_image.png", renderer_ptr_->depth_image());
  ASSERT_TRUE(CompareToLoadedImage(renderer_test_directory, "depth_image.png",
                                   renderer_ptr_->depth_image(), 0, 10));
}

class FocusedBasicDepthRendererTest : public testing::Test {
 protected:
  void SetUp() override {
    triangle_body_ptr_ = TriangleBodyPtr();
    auto schauma_body_ptr{SchaumaBodyPtr()};
    renderer_geometry_ptr_ =
        std::make_shared<m3t::RendererGeometry>("renderer_geometry");
    renderer_geometry_ptr_->SetUp();
    renderer_geometry_ptr_->AddBody(triangle_body_ptr_);
    renderer_geometry_ptr_->AddBody(schauma_body_ptr);

    renderer_ptr_ = std::make_shared<m3t::FocusedBasicDepthRenderer>(
        name_, renderer_geometry_ptr_, world2camera_pose_, intrinsics_,
        image_size_, z_min_, z_max_);
    renderer_ptr_->AddReferencedBody(triangle_body_ptr_);
  }

  bool TestRendererParameters(const m3t::FocusedBasicDepthRenderer &renderer) {
    return renderer.name() == name_ &&
           renderer.world2camera_pose().matrix() ==
               world2camera_pose_.matrix() &&
           renderer.intrinsics().fu == intrinsics_.fu &&
           renderer.intrinsics().fv == intrinsics_.fv &&
           renderer.image_size() == image_size_ && renderer.z_min() == z_min_ &&
           renderer.z_max() == z_max_;
  }

  std::shared_ptr<m3t::Body> triangle_body_ptr_;
  std::shared_ptr<m3t::RendererGeometry> renderer_geometry_ptr_;
  std::shared_ptr<m3t::FocusedBasicDepthRenderer> renderer_ptr_;
  std::string name_{"renderer"};
  std::filesystem::path metafile_path_{renderer_test_directory /
                                       "focused_basic_depth_renderer.yaml"};
  m3t::Transform3fA world2camera_pose_{Eigen::Translation3f{0.01f, 0.0f, 0.0f}};
  m3t::Intrinsics intrinsics_{698.128f, 698.617f, 478.459f, 274.426f, 640, 480};
  int image_size_ = 200;
  float z_min_ = 0.1f;
  float z_max_ = 2.0f;
};

TEST_F(FocusedBasicDepthRendererTest, SetUpFromData) {
  ASSERT_TRUE(TestRendererParameters(*renderer_ptr_));
  ASSERT_TRUE(renderer_ptr_->SetUp());
  ASSERT_TRUE(renderer_ptr_->set_up());
  ASSERT_TRUE(TestRendererParameters(*renderer_ptr_));
}

TEST_F(FocusedBasicDepthRendererTest, SetUpFromCamera) {
  auto camera_ptr{ColorCameraPtr()};
  camera_ptr->set_world2camera_pose(world2camera_pose_);
  m3t::FocusedBasicDepthRenderer renderer{
      name_, renderer_geometry_ptr_, camera_ptr, image_size_, z_min_, z_max_};
  renderer.AddReferencedBody(triangle_body_ptr_);
  ASSERT_TRUE(renderer.SetUp());
  ASSERT_TRUE(renderer.set_up());
  ASSERT_TRUE(TestRendererParameters(renderer));
}

TEST_F(FocusedBasicDepthRendererTest, SetUpFromMetaFile) {
  auto camera_ptr{ColorCameraPtr()};
  camera_ptr->set_world2camera_pose(world2camera_pose_);
  m3t::FocusedBasicDepthRenderer renderer{name_, metafile_path_,
                                          renderer_geometry_ptr_, camera_ptr};
  renderer.AddReferencedBody(triangle_body_ptr_);
  ASSERT_TRUE(renderer.SetUp());
  ASSERT_TRUE(renderer.set_up());
  ASSERT_TRUE(TestRendererParameters(renderer));
}

TEST_F(FocusedBasicDepthRendererTest, ParameterChanges) {
  ASSERT_TRUE(renderer_ptr_->SetUp());
  ASSERT_TRUE(renderer_ptr_->set_up());
  renderer_ptr_->set_camera_ptr(nullptr);
  ASSERT_FALSE(renderer_ptr_->set_up());
  ASSERT_TRUE(renderer_ptr_->SetUp());
  ASSERT_TRUE(renderer_ptr_->set_up());
  renderer_ptr_->set_renderer_geometry_ptr(renderer_geometry_ptr_);
  ASSERT_FALSE(renderer_ptr_->set_up());
  ASSERT_TRUE(renderer_ptr_->SetUp());
  ASSERT_TRUE(renderer_ptr_->set_up());
  renderer_ptr_->set_intrinsics(intrinsics_);
  ASSERT_FALSE(renderer_ptr_->set_up());
  ASSERT_TRUE(renderer_ptr_->SetUp());
  ASSERT_TRUE(renderer_ptr_->set_up());
  renderer_ptr_->set_image_size(image_size_);
  ASSERT_FALSE(renderer_ptr_->set_up());
  ASSERT_TRUE(renderer_ptr_->SetUp());
  ASSERT_TRUE(renderer_ptr_->set_up());
  renderer_ptr_->set_z_min(z_min_);
  ASSERT_FALSE(renderer_ptr_->set_up());
  ASSERT_TRUE(renderer_ptr_->SetUp());
  ASSERT_TRUE(renderer_ptr_->set_up());
  renderer_ptr_->set_z_max(z_max_);
  ASSERT_FALSE(renderer_ptr_->set_up());
  ASSERT_TRUE(renderer_ptr_->SetUp());
  ASSERT_TRUE(renderer_ptr_->set_up());
  ASSERT_TRUE(TestRendererParameters(*renderer_ptr_));
}

TEST_F(FocusedBasicDepthRendererTest, AddDeleteClearReferencedBodies) {
  ASSERT_TRUE(renderer_ptr_->SetUp());
  ASSERT_TRUE(renderer_ptr_->set_up());
  renderer_ptr_->ClearReferencedBodies();
  ASSERT_FALSE(renderer_ptr_->set_up());
  ASSERT_FALSE(renderer_ptr_->SetUp());
  ASSERT_TRUE(renderer_ptr_->AddReferencedBody(triangle_body_ptr_));
  ASSERT_FALSE(renderer_ptr_->AddReferencedBody(triangle_body_ptr_));
  ASSERT_FALSE(renderer_ptr_->set_up());
  ASSERT_TRUE(renderer_ptr_->SetUp());
  ASSERT_TRUE(renderer_ptr_->set_up());
  ASSERT_TRUE(renderer_ptr_->DeleteReferencedBody(triangle_body_ptr_->name()));
  ASSERT_FALSE(renderer_ptr_->DeleteReferencedBody(triangle_body_ptr_->name()));
  ASSERT_FALSE(renderer_ptr_->set_up());
  ASSERT_FALSE(renderer_ptr_->SetUp());
  ASSERT_TRUE(renderer_ptr_->referenced_body_ptrs().empty());
}

TEST_F(FocusedBasicDepthRendererTest, TestWithoutSetUpRendererGeometry) {
  renderer_ptr_->set_renderer_geometry_ptr(
      std::make_shared<m3t::RendererGeometry>("rg"));
  ASSERT_FALSE(renderer_ptr_->SetUp());
}

TEST_F(FocusedBasicDepthRendererTest, TestWithoutSetUpCamera) {
  renderer_ptr_->set_camera_ptr(ColorCameraPtrNoSetUp());
  ASSERT_FALSE(renderer_ptr_->SetUp());
}

TEST_F(FocusedBasicDepthRendererTest, TestWithoutSetUpReferencedBody) {
  renderer_ptr_->ClearReferencedBodies();
  renderer_ptr_->AddReferencedBody(TriangleBodyPtrNoSetUp());
  ASSERT_FALSE(renderer_ptr_->SetUp());
}

TEST_F(FocusedBasicDepthRendererTest, TestWithoutSetUp) {
  ASSERT_FALSE(renderer_ptr_->StartRendering());
  ASSERT_FALSE(renderer_ptr_->FetchDepthImage());
}

TEST_F(FocusedBasicDepthRendererTest, TestWithoutRendering) {
  renderer_ptr_->SetUp();
  ASSERT_FALSE(renderer_ptr_->FetchDepthImage());
}

TEST_F(FocusedBasicDepthRendererTest, TestDepthImage) {
  ASSERT_TRUE(renderer_ptr_->SetUp());
  ASSERT_TRUE(renderer_ptr_->StartRendering());
  ASSERT_TRUE(renderer_ptr_->FetchDepthImage());
  SaveOrSkipImage(kGenerateGroundTruth, renderer_test_directory,
                  "focused_depth_image.png",
                  renderer_ptr_->focused_depth_image());
  ASSERT_TRUE(
      CompareToLoadedImage(renderer_test_directory, "focused_depth_image.png",
                           renderer_ptr_->focused_depth_image(), 0, 10));
}
