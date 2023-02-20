// SPDX-License-Identifier: MIT
// Copyright (c) 2023 Manuel Stoiber, German Aerospace Center (DLR)

#include <gtest/gtest.h>
#include <m3t/body.h>
#include <m3t/camera.h>
#include <m3t/image_viewer.h>
#include <m3t/normal_viewer.h>
#include <m3t/renderer_geometry.h>

#include "common_test.h"

const std::filesystem::path viewer_test_directory{data_directory /
                                                  "viewer_test"};

class ImageColorViewerTest : public testing::Test {
 protected:
  void SetUp() override {
    camera_ptr_ = ColorCameraPtr();
    viewer_ptr_ = std::make_shared<m3t::ImageColorViewer>(name_, camera_ptr_);
  }

  bool TestViewerParameters(const m3t::ImageColorViewer &viewer) {
    return viewer.name() == name_ &&
           viewer.display_images() == display_images_ &&
           viewer.save_images() == save_images_;
  }

  std::shared_ptr<m3t::ImageColorViewer> viewer_ptr_;
  std::string name_{"viewer"};
  std::filesystem::path metafile_path_{viewer_test_directory /
                                       "image_color_viewer.yaml"};
  std::shared_ptr<m3t::ColorCamera> camera_ptr_;
  bool display_images_ = true;
  bool save_images_ = false;
};

TEST_F(ImageColorViewerTest, SetUpFromData) {
  ASSERT_TRUE(TestViewerParameters(*viewer_ptr_));
  ASSERT_TRUE(viewer_ptr_->SetUp());
  ASSERT_TRUE(viewer_ptr_->set_up());
  ASSERT_TRUE(TestViewerParameters(*viewer_ptr_));
}

TEST_F(ImageColorViewerTest, SetUpFromMetaFile) {
  m3t::ImageColorViewer viewer{name_, metafile_path_, camera_ptr_};
  ASSERT_TRUE(viewer.SetUp());
  ASSERT_TRUE(viewer.set_up());
  ASSERT_TRUE(TestViewerParameters(viewer));
}

TEST_F(ImageColorViewerTest, ParameterChanges) {
  ASSERT_TRUE(viewer_ptr_->SetUp());
  ASSERT_TRUE(viewer_ptr_->set_up());
  viewer_ptr_->StartSavingImages(temp_directory, "bmp");
  ASSERT_TRUE(viewer_ptr_->save_images());
  ASSERT_TRUE(viewer_ptr_->set_up());
  viewer_ptr_->StopSavingImages();
  ASSERT_FALSE(viewer_ptr_->save_images());
  ASSERT_TRUE(viewer_ptr_->set_up());
  viewer_ptr_->set_color_camera_ptr(camera_ptr_);
  ASSERT_FALSE(viewer_ptr_->set_up());
  ASSERT_TRUE(viewer_ptr_->SetUp());
  ASSERT_TRUE(viewer_ptr_->set_up());
  ASSERT_TRUE(TestViewerParameters(*viewer_ptr_));
}

TEST_F(ImageColorViewerTest, TestWithoutSetUpCamera) {
  viewer_ptr_->set_color_camera_ptr(ColorCameraPtrNoSetUp());
  ASSERT_FALSE(viewer_ptr_->SetUp());
}

TEST_F(ImageColorViewerTest, TestWithoutSetUp) {
  ASSERT_FALSE(viewer_ptr_->UpdateViewer(0));
}

TEST_F(ImageColorViewerTest, UpdateAndSaveImage) {
  std::filesystem::create_directory(temp_directory);
  viewer_ptr_->set_display_images(false);
  viewer_ptr_->StartSavingImages(temp_directory, "png");
  ASSERT_TRUE(viewer_ptr_->SetUp());
  ASSERT_TRUE(viewer_ptr_->UpdateViewer(123));
  ASSERT_TRUE(CompareLoadedImages(sequence_directory,
                                  "color_camera_image_200.png", temp_directory,
                                  "viewer_image_123.png", 0, 0));
}

class ImageDepthViewerTest : public testing::Test {
 protected:
  void SetUp() override {
    camera_ptr_ = DepthCameraPtr();
    viewer_ptr_ = std::make_shared<m3t::ImageDepthViewer>(
        name_, camera_ptr_, min_depth_, max_depth_);
  }

  bool TestViewerParameters(const m3t::ImageDepthViewer &viewer) {
    return viewer.name() == name_ && viewer.min_depth() == min_depth_ &&
           viewer.max_depth() == max_depth_ &&
           viewer.display_images() == display_images_ &&
           viewer.save_images() == save_images_;
  }

  std::shared_ptr<m3t::ImageDepthViewer> viewer_ptr_;
  std::string name_{"viewer"};
  std::filesystem::path metafile_path_{viewer_test_directory /
                                       "image_depth_viewer.yaml"};
  std::shared_ptr<m3t::DepthCamera> camera_ptr_;
  float min_depth_ = 0.3f;
  float max_depth_ = 1.3f;
  bool display_images_ = true;
  bool save_images_ = false;
};

TEST_F(ImageDepthViewerTest, SetUpFromData) {
  ASSERT_TRUE(TestViewerParameters(*viewer_ptr_));
  ASSERT_TRUE(viewer_ptr_->SetUp());
  ASSERT_TRUE(viewer_ptr_->set_up());
  ASSERT_TRUE(TestViewerParameters(*viewer_ptr_));
}

TEST_F(ImageDepthViewerTest, SetUpFromMetaFile) {
  m3t::ImageDepthViewer viewer{name_, metafile_path_, camera_ptr_};
  ASSERT_TRUE(viewer.SetUp());
  ASSERT_TRUE(viewer.set_up());
  ASSERT_TRUE(TestViewerParameters(viewer));
}

TEST_F(ImageDepthViewerTest, ParameterChanges) {
  ASSERT_TRUE(viewer_ptr_->SetUp());
  ASSERT_TRUE(viewer_ptr_->set_up());
  viewer_ptr_->StartSavingImages(temp_directory, "bmp");
  ASSERT_TRUE(viewer_ptr_->save_images());
  ASSERT_TRUE(viewer_ptr_->set_up());
  viewer_ptr_->StopSavingImages();
  ASSERT_FALSE(viewer_ptr_->save_images());
  ASSERT_TRUE(viewer_ptr_->set_up());
  viewer_ptr_->set_depth_camera_ptr(camera_ptr_);
  ASSERT_FALSE(viewer_ptr_->set_up());
  ASSERT_TRUE(viewer_ptr_->SetUp());
  ASSERT_TRUE(viewer_ptr_->set_up());
  ASSERT_TRUE(TestViewerParameters(*viewer_ptr_));
}

TEST_F(ImageDepthViewerTest, TestWithoutSetUpCamera) {
  viewer_ptr_->set_depth_camera_ptr(DepthCameraPtrNoSetUp());
  ASSERT_FALSE(viewer_ptr_->SetUp());
}

TEST_F(ImageDepthViewerTest, TestWithoutSetUp) {
  ASSERT_FALSE(viewer_ptr_->UpdateViewer(0));
}

TEST_F(ImageDepthViewerTest, UpdateAndSaveImage) {
  std::filesystem::create_directory(temp_directory);
  viewer_ptr_->set_display_images(false);
  viewer_ptr_->StartSavingImages(temp_directory, "png");
  ASSERT_TRUE(viewer_ptr_->SetUp());
  ASSERT_TRUE(viewer_ptr_->UpdateViewer(123));
  ASSERT_TRUE(CopyOrSkipImage(kGenerateGroundTruth, viewer_test_directory,
                              "depth_viewer_image.png", temp_directory,
                              "viewer_image_123.png"));
  ASSERT_TRUE(CompareLoadedImages(viewer_test_directory,
                                  "depth_viewer_image.png", temp_directory,
                                  "viewer_image_123.png", 0, 0));
}

class NormalColorViewerTest : public testing::Test {
 protected:
  void SetUp() override {
    auto triangle_body_ptr{TriangleBodyPtr()};
    auto schauma_body_ptr{SchaumaBodyPtr()};
    renderer_geometry_ptr_ =
        std::make_shared<m3t::RendererGeometry>("renderer_geometry");
    renderer_geometry_ptr_->SetUp();
    renderer_geometry_ptr_->AddBody(triangle_body_ptr);
    renderer_geometry_ptr_->AddBody(schauma_body_ptr);

    camera_ptr_ = ColorCameraPtr();

    viewer_ptr_ = std::make_shared<m3t::NormalColorViewer>(
        name_, camera_ptr_, renderer_geometry_ptr_, opacity_);
  }

  bool TestViewerParameters(const m3t::NormalColorViewer &viewer) {
    return viewer.name() == name_ && viewer.opacity() == opacity_ &&
           viewer.display_images() == display_images_ &&
           viewer.save_images() == save_images_;
  }

  std::shared_ptr<m3t::NormalColorViewer> viewer_ptr_;
  std::string name_{"viewer"};
  std::filesystem::path metafile_path_{viewer_test_directory /
                                       "normal_color_viewer.yaml"};
  std::shared_ptr<m3t::ColorCamera> camera_ptr_;
  std::shared_ptr<m3t::RendererGeometry> renderer_geometry_ptr_;
  float opacity_ = 0.7f;
  bool display_images_ = true;
  bool save_images_ = false;
};

TEST_F(NormalColorViewerTest, SetUpFromData) {
  ASSERT_TRUE(TestViewerParameters(*viewer_ptr_));
  ASSERT_TRUE(viewer_ptr_->SetUp());
  ASSERT_TRUE(viewer_ptr_->set_up());
  ASSERT_TRUE(TestViewerParameters(*viewer_ptr_));
}

TEST_F(NormalColorViewerTest, SetUpFromMetaFile) {
  m3t::NormalColorViewer viewer{name_, metafile_path_, camera_ptr_,
                                renderer_geometry_ptr_};
  ASSERT_TRUE(viewer.SetUp());
  ASSERT_TRUE(viewer.set_up());
  ASSERT_TRUE(TestViewerParameters(viewer));
}

TEST_F(NormalColorViewerTest, ParameterChanges) {
  ASSERT_TRUE(viewer_ptr_->SetUp());
  ASSERT_TRUE(viewer_ptr_->set_up());
  viewer_ptr_->StartSavingImages(temp_directory, "bmp");
  ASSERT_TRUE(viewer_ptr_->save_images());
  ASSERT_TRUE(viewer_ptr_->set_up());
  viewer_ptr_->StopSavingImages();
  ASSERT_FALSE(viewer_ptr_->save_images());
  ASSERT_TRUE(viewer_ptr_->set_up());
  viewer_ptr_->set_color_camera_ptr(camera_ptr_);
  ASSERT_FALSE(viewer_ptr_->set_up());
  ASSERT_TRUE(viewer_ptr_->SetUp());
  ASSERT_TRUE(viewer_ptr_->set_up());
  viewer_ptr_->set_renderer_geometry_ptr(renderer_geometry_ptr_);
  ASSERT_FALSE(viewer_ptr_->set_up());
  ASSERT_TRUE(viewer_ptr_->SetUp());
  ASSERT_TRUE(viewer_ptr_->set_up());
  ASSERT_TRUE(TestViewerParameters(*viewer_ptr_));
}

TEST_F(NormalColorViewerTest, TestWithoutSetUpRendererGeometry) {
  viewer_ptr_->set_renderer_geometry_ptr(
      std::make_shared<m3t::RendererGeometry>("rg"));
  ASSERT_FALSE(viewer_ptr_->SetUp());
}

TEST_F(NormalColorViewerTest, TestWithoutSetUpCamera) {
  viewer_ptr_->set_color_camera_ptr(ColorCameraPtrNoSetUp());
  ASSERT_FALSE(viewer_ptr_->SetUp());
}

TEST_F(NormalColorViewerTest, TestWithoutSetUp) {
  ASSERT_FALSE(viewer_ptr_->UpdateViewer(0));
}

TEST_F(NormalColorViewerTest, UpdateAndSaveImage) {
  std::filesystem::create_directory(temp_directory);
  viewer_ptr_->set_display_images(false);
  viewer_ptr_->StartSavingImages(temp_directory, "png");
  ASSERT_TRUE(viewer_ptr_->SetUp());
  ASSERT_TRUE(viewer_ptr_->UpdateViewer(123));
  ASSERT_TRUE(CopyOrSkipImage(kGenerateGroundTruth, viewer_test_directory,
                              "normal_color_viewer_image.png", temp_directory,
                              "viewer_image_123.png"));
  ASSERT_TRUE(CompareLoadedImages(
      viewer_test_directory, "normal_color_viewer_image.png", temp_directory,
      "viewer_image_123.png", 0, 0));
}

class NormalDepthViewerTest : public testing::Test {
 protected:
  void SetUp() override {
    auto triangle_body_ptr{TriangleBodyPtr()};
    auto schauma_body_ptr{SchaumaBodyPtr()};
    renderer_geometry_ptr_ =
        std::make_shared<m3t::RendererGeometry>("renderer_geometry");
    renderer_geometry_ptr_->SetUp();
    renderer_geometry_ptr_->AddBody(triangle_body_ptr);
    renderer_geometry_ptr_->AddBody(schauma_body_ptr);

    camera_ptr_ = DepthCameraPtr();

    viewer_ptr_ = std::make_shared<m3t::NormalDepthViewer>(
        name_, camera_ptr_, renderer_geometry_ptr_, min_depth_, max_depth_,
        opacity_);
  }

  bool TestViewerParameters(const m3t::NormalDepthViewer &viewer) {
    return viewer.name() == name_ && viewer.min_depth() == min_depth_ &&
           viewer.max_depth() == max_depth_ && viewer.opacity() == opacity_ &&
           viewer.display_images() == display_images_ &&
           viewer.save_images() == save_images_;
  }

  std::shared_ptr<m3t::NormalDepthViewer> viewer_ptr_;
  std::string name_{"viewer"};
  std::filesystem::path metafile_path_{viewer_test_directory /
                                       "normal_depth_viewer.yaml"};
  std::shared_ptr<m3t::DepthCamera> camera_ptr_;
  std::shared_ptr<m3t::RendererGeometry> renderer_geometry_ptr_;
  float min_depth_ = 0.3f;
  float max_depth_ = 1.3f;
  float opacity_ = 0.7f;
  bool display_images_ = true;
  bool save_images_ = false;
};

TEST_F(NormalDepthViewerTest, SetUpFromData) {
  ASSERT_TRUE(TestViewerParameters(*viewer_ptr_));
  ASSERT_TRUE(viewer_ptr_->SetUp());
  ASSERT_TRUE(viewer_ptr_->set_up());
  ASSERT_TRUE(TestViewerParameters(*viewer_ptr_));
}

TEST_F(NormalDepthViewerTest, SetUpFromMetaFile) {
  m3t::NormalDepthViewer viewer{name_, metafile_path_, camera_ptr_,
                                renderer_geometry_ptr_};
  ASSERT_TRUE(viewer.SetUp());
  ASSERT_TRUE(viewer.set_up());
  ASSERT_TRUE(TestViewerParameters(viewer));
}

TEST_F(NormalDepthViewerTest, ParameterChanges) {
  ASSERT_TRUE(viewer_ptr_->SetUp());
  ASSERT_TRUE(viewer_ptr_->set_up());
  viewer_ptr_->StartSavingImages(temp_directory, "bmp");
  ASSERT_TRUE(viewer_ptr_->save_images());
  ASSERT_TRUE(viewer_ptr_->set_up());
  viewer_ptr_->StopSavingImages();
  ASSERT_FALSE(viewer_ptr_->save_images());
  ASSERT_TRUE(viewer_ptr_->set_up());
  viewer_ptr_->set_depth_camera_ptr(camera_ptr_);
  ASSERT_FALSE(viewer_ptr_->set_up());
  ASSERT_TRUE(viewer_ptr_->SetUp());
  ASSERT_TRUE(viewer_ptr_->set_up());
  viewer_ptr_->set_renderer_geometry_ptr(renderer_geometry_ptr_);
  ASSERT_FALSE(viewer_ptr_->set_up());
  ASSERT_TRUE(viewer_ptr_->SetUp());
  ASSERT_TRUE(viewer_ptr_->set_up());
  ASSERT_TRUE(TestViewerParameters(*viewer_ptr_));
}

TEST_F(NormalDepthViewerTest, TestWithoutSetUpRendererGeometry) {
  viewer_ptr_->set_renderer_geometry_ptr(
      std::make_shared<m3t::RendererGeometry>("rg"));
  ASSERT_FALSE(viewer_ptr_->SetUp());
}

TEST_F(NormalDepthViewerTest, TestWithoutSetUpCamera) {
  viewer_ptr_->set_depth_camera_ptr(DepthCameraPtrNoSetUp());
  ASSERT_FALSE(viewer_ptr_->SetUp());
}

TEST_F(NormalDepthViewerTest, TestWithoutSetUp) {
  ASSERT_FALSE(viewer_ptr_->UpdateViewer(0));
}

TEST_F(NormalDepthViewerTest, UpdateAndSaveImage) {
  std::filesystem::create_directory(temp_directory);
  viewer_ptr_->set_display_images(false);
  viewer_ptr_->StartSavingImages(temp_directory, "png");
  ASSERT_TRUE(viewer_ptr_->SetUp());
  ASSERT_TRUE(viewer_ptr_->UpdateViewer(123));
  ASSERT_TRUE(CopyOrSkipImage(kGenerateGroundTruth, viewer_test_directory,
                              "normal_depth_viewer_image.png", temp_directory,
                              "viewer_image_123.png"));
  ASSERT_TRUE(CompareLoadedImages(
      viewer_test_directory, "normal_depth_viewer_image.png", temp_directory,
      "viewer_image_123.png", 0, 0));
}
