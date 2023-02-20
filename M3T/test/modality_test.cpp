// SPDX-License-Identifier: MIT
// Copyright (c) 2023 Manuel Stoiber, German Aerospace Center (DLR)

#include <gtest/gtest.h>
#include <m3t/body.h>
#include <m3t/camera.h>
#include <m3t/depth_modality.h>
#include <m3t/depth_model.h>
#include <m3t/region_modality.h>
#include <m3t/region_model.h>
#include <m3t/texture_modality.h>

#include "common_test.h"

const std::filesystem::path modality_test_directory{data_directory /
                                                    "modality_test"};

class RegionModalityTest : public testing::Test {
 protected:
  void SetUp() override {
    model_ptr_ = TriangleRegionModelPtr();
    body_ptr_ = model_ptr_->body_ptr();
    color_camera_ptr_ = ColorCameraPtr();
    depth_camera_ptr_ = DepthCameraPtr();
    depth_renderer_ptr_ = FocusedBasicDepthRendererPtr(
        body_ptr_, SchaumaBodyPtr(), color_camera_ptr_);
    silhouette_renderer_ptr_ = FocusedSilhouetteRendererPtr(
        body_ptr_, SchaumaBodyPtr(), color_camera_ptr_);
    silhouette_renderer_ptr_->set_id_type(m3t::IDType::REGION);
    color_histograms_ptr_ = ColorHistogramsPtr();
    modality_ptr_ = std::make_shared<m3t::RegionModality>(
        name_, body_ptr_, color_camera_ptr_, model_ptr_);
  }

  bool TestModalityParameters(const m3t::RegionModality &modality) {
    return modality.name() == name_ &&
           modality.function_amplitude() == function_amplitude_ &&
           modality.function_slope() == function_slope_ &&
           modality.function_length() == function_length_ &&
           modality.distribution_length() == distribution_length_ &&
           modality.n_histogram_bins() == n_histogram_bins_ &&
           modality.model_occlusions() == model_occlusions_ &&
           modality.measure_occlusions() == measure_occlusions_;
  }

  std::shared_ptr<m3t::RegionModality> modality_ptr_;
  std::string name_{"region_modality"};
  std::filesystem::path metafile_path_{modality_test_directory /
                                       "region_modality.yaml"};
  std::shared_ptr<m3t::Body> body_ptr_;
  std::shared_ptr<m3t::ColorCamera> color_camera_ptr_;
  std::shared_ptr<m3t::DepthCamera> depth_camera_ptr_;
  std::shared_ptr<m3t::RegionModel> model_ptr_;
  std::shared_ptr<m3t::FocusedDepthRenderer> depth_renderer_ptr_;
  std::shared_ptr<m3t::FocusedSilhouetteRenderer> silhouette_renderer_ptr_;
  std::shared_ptr<m3t::ColorHistograms> color_histograms_ptr_;
  float function_amplitude_ = 0.43f;
  float function_slope_ = 0.5f;
  int function_length_ = 8;
  int distribution_length_ = 12;
  int n_histogram_bins_ = 16;
  bool model_occlusions_ = false;
  bool measure_occlusions_ = false;
};

TEST_F(RegionModalityTest, SetUpFromData) {
  ASSERT_TRUE(modality_ptr_->SetUp());
  ASSERT_TRUE(modality_ptr_->set_up());
  ASSERT_TRUE(TestModalityParameters(*modality_ptr_));
}

TEST_F(RegionModalityTest, SetUpFromMetaFile) {
  m3t::RegionModality modality{name_, metafile_path_, body_ptr_,
                               color_camera_ptr_, model_ptr_};
  ASSERT_TRUE(modality.SetUp());
  ASSERT_TRUE(modality.set_up());
  ASSERT_TRUE(TestModalityParameters(modality));
}

TEST_F(RegionModalityTest, ParameterChanges) {
  ASSERT_TRUE(modality_ptr_->SetUp());
  ASSERT_TRUE(modality_ptr_->set_up());
  modality_ptr_->set_body_ptr(body_ptr_);
  ASSERT_FALSE(modality_ptr_->set_up());
  ASSERT_TRUE(modality_ptr_->SetUp());
  ASSERT_TRUE(modality_ptr_->set_up());
  modality_ptr_->set_color_camera_ptr(color_camera_ptr_);
  ASSERT_FALSE(modality_ptr_->set_up());
  ASSERT_TRUE(modality_ptr_->SetUp());
  ASSERT_TRUE(modality_ptr_->set_up());
  modality_ptr_->set_region_model_ptr(model_ptr_);
  ASSERT_FALSE(modality_ptr_->set_up());
  ASSERT_TRUE(modality_ptr_->SetUp());
  ASSERT_TRUE(modality_ptr_->set_up());
  modality_ptr_->set_function_amplitude(function_amplitude_);
  ASSERT_FALSE(modality_ptr_->set_up());
  ASSERT_TRUE(modality_ptr_->SetUp());
  ASSERT_TRUE(modality_ptr_->set_up());
  modality_ptr_->set_function_slope(function_slope_);
  ASSERT_FALSE(modality_ptr_->set_up());
  ASSERT_TRUE(modality_ptr_->SetUp());
  ASSERT_TRUE(modality_ptr_->set_up());
  modality_ptr_->set_function_length(function_length_);
  ASSERT_FALSE(modality_ptr_->set_up());
  ASSERT_TRUE(modality_ptr_->SetUp());
  ASSERT_TRUE(modality_ptr_->set_up());
  modality_ptr_->set_distribution_length(distribution_length_);
  ASSERT_FALSE(modality_ptr_->set_up());
  ASSERT_TRUE(modality_ptr_->SetUp());
  ASSERT_TRUE(modality_ptr_->set_up());
  modality_ptr_->set_n_histogram_bins(n_histogram_bins_);
  ASSERT_FALSE(modality_ptr_->set_up());
  ASSERT_TRUE(modality_ptr_->SetUp());
  ASSERT_TRUE(modality_ptr_->set_up());
  modality_ptr_->UseRegionChecking(silhouette_renderer_ptr_);
  ASSERT_FALSE(modality_ptr_->set_up());
  ASSERT_TRUE(modality_ptr_->SetUp());
  ASSERT_TRUE(modality_ptr_->set_up());
  modality_ptr_->DoNotUseRegionChecking();
  ASSERT_FALSE(modality_ptr_->set_up());
  ASSERT_TRUE(modality_ptr_->SetUp());
  ASSERT_TRUE(modality_ptr_->set_up());
  modality_ptr_->ModelOcclusions(depth_renderer_ptr_);
  ASSERT_FALSE(modality_ptr_->set_up());
  ASSERT_TRUE(modality_ptr_->SetUp());
  ASSERT_TRUE(modality_ptr_->set_up());
  modality_ptr_->DoNotModelOcclusions();
  ASSERT_FALSE(modality_ptr_->set_up());
  ASSERT_TRUE(modality_ptr_->SetUp());
  ASSERT_TRUE(modality_ptr_->set_up());
  modality_ptr_->MeasureOcclusions(depth_camera_ptr_);
  ASSERT_FALSE(modality_ptr_->set_up());
  ASSERT_TRUE(modality_ptr_->SetUp());
  ASSERT_TRUE(modality_ptr_->set_up());
  modality_ptr_->DoNotMeasureOcclusions();
  ASSERT_FALSE(modality_ptr_->set_up());
  ASSERT_TRUE(modality_ptr_->SetUp());
  ASSERT_TRUE(modality_ptr_->set_up());
  modality_ptr_->UseSharedColorHistograms(color_histograms_ptr_);
  ASSERT_FALSE(modality_ptr_->set_up());
  ASSERT_TRUE(modality_ptr_->SetUp());
  ASSERT_TRUE(modality_ptr_->set_up());
  modality_ptr_->DoNotUseSharedColorHistograms();
  ASSERT_FALSE(modality_ptr_->set_up());
  ASSERT_TRUE(modality_ptr_->SetUp());
  ASSERT_TRUE(modality_ptr_->set_up());
  ASSERT_TRUE(TestModalityParameters(*modality_ptr_));
}

TEST_F(RegionModalityTest, TestWithoutSetUp) {
  ASSERT_FALSE(modality_ptr_->StartModality(0, 0));
  ASSERT_FALSE(modality_ptr_->CalculateCorrespondences(0, 0));
  ASSERT_FALSE(modality_ptr_->VisualizeCorrespondences(0));
  ASSERT_FALSE(modality_ptr_->CalculateGradientAndHessian(0, 0, 0));
  ASSERT_FALSE(modality_ptr_->VisualizeOptimization(0));
  ASSERT_FALSE(modality_ptr_->CalculateResults(0));
  ASSERT_FALSE(modality_ptr_->VisualizeResults(0));
}

TEST_F(RegionModalityTest, TestWithoutSetUpBody) {
  modality_ptr_->set_body_ptr(TriangleBodyPtrNoSetUp());
  ASSERT_FALSE(modality_ptr_->SetUp());
}

TEST_F(RegionModalityTest, TestWithoutSetUpRegionModel) {
  modality_ptr_->set_region_model_ptr(TriangleRegionModelPtrNoSetUp());
  ASSERT_FALSE(modality_ptr_->SetUp());
}

TEST_F(RegionModalityTest, TestWithoutSetUpCamera) {
  modality_ptr_->set_color_camera_ptr(ColorCameraPtrNoSetUp());
  ASSERT_FALSE(modality_ptr_->SetUp());
}

TEST_F(RegionModalityTest, CalculateCorrespondences) {
  std::filesystem::create_directory(temp_directory);
  modality_ptr_->set_visualize_lines_correspondence(true);
  modality_ptr_->set_display_visualization(false);
  modality_ptr_->StartSavingVisualizations(temp_directory);
  ASSERT_TRUE(modality_ptr_->SetUp());
  ASSERT_TRUE(modality_ptr_->StartModality(0, 0));
  ASSERT_TRUE(modality_ptr_->CalculateCorrespondences(0, 0));
  ASSERT_TRUE(modality_ptr_->VisualizeCorrespondences(0));
  ASSERT_TRUE(CopyOrSkipImage(kGenerateGroundTruth, modality_test_directory,
                              "region_modality.png", temp_directory,
                              "region_modality_lines_correspondence_0.png"));
  ASSERT_TRUE(CompareLoadedImages(
      modality_test_directory, "region_modality.png", temp_directory,
      "region_modality_lines_correspondence_0.png", 0, 0));
}

TEST_F(RegionModalityTest, CalculateCorrespondencesRegionChecking) {
  std::filesystem::create_directory(temp_directory);
  silhouette_renderer_ptr_->StartRendering();
  modality_ptr_->UseRegionChecking(silhouette_renderer_ptr_);
  modality_ptr_->set_visualize_lines_correspondence(true);
  modality_ptr_->set_visualize_points_silhouette_rendering_correspondence(true);
  modality_ptr_->set_display_visualization(false);
  modality_ptr_->StartSavingVisualizations(temp_directory);
  ASSERT_TRUE(modality_ptr_->SetUp());
  ASSERT_TRUE(modality_ptr_->StartModality(0, 0));
  ASSERT_TRUE(modality_ptr_->CalculateCorrespondences(0, 0));
  ASSERT_TRUE(modality_ptr_->VisualizeCorrespondences(0));
  ASSERT_TRUE(CopyOrSkipImage(kGenerateGroundTruth, modality_test_directory,
                              "region_modality_region_checking.png",
                              temp_directory,
                              "region_modality_lines_correspondence_0.png"));
  ASSERT_TRUE(CompareLoadedImages(
      modality_test_directory, "region_modality_region_checking.png",
      temp_directory, "region_modality_lines_correspondence_0.png", 0, 0));
  ASSERT_TRUE(CopyOrSkipImage(
      kGenerateGroundTruth, modality_test_directory,
      "region_modality_silhouette_region_checking.png", temp_directory,
      "region_modality_silhouette_rendering_correspondence_0.png"));
  ASSERT_TRUE(CompareLoadedImages(
      modality_test_directory, "region_modality_silhouette_region_checking.png",
      temp_directory,
      "region_modality_silhouette_rendering_correspondence_0.png", 0, 0));
}

TEST_F(RegionModalityTest, CalculateCorrespondencesMeasuredOcclusions) {
  std::filesystem::create_directory(temp_directory);
  modality_ptr_->MeasureOcclusions(depth_camera_ptr_);
  modality_ptr_->set_n_unoccluded_iterations(0);
  modality_ptr_->set_visualize_lines_correspondence(true);
  modality_ptr_->set_visualize_points_depth_image_correspondence(true);
  modality_ptr_->set_display_visualization(false);
  modality_ptr_->StartSavingVisualizations(temp_directory);
  ASSERT_TRUE(modality_ptr_->SetUp());
  ASSERT_TRUE(modality_ptr_->StartModality(0, 0));
  ASSERT_TRUE(modality_ptr_->CalculateCorrespondences(0, 0));
  ASSERT_TRUE(modality_ptr_->VisualizeCorrespondences(0));
  ASSERT_TRUE(CopyOrSkipImage(kGenerateGroundTruth, modality_test_directory,
                              "region_modality_measured_occlusions.png",
                              temp_directory,
                              "region_modality_lines_correspondence_0.png"));
  ASSERT_TRUE(CompareLoadedImages(
      modality_test_directory, "region_modality_measured_occlusions.png",
      temp_directory, "region_modality_lines_correspondence_0.png", 0, 0));
  ASSERT_TRUE(CopyOrSkipImage(
      kGenerateGroundTruth, modality_test_directory,
      "region_modality_depth_measured_occlusions.png", temp_directory,
      "region_modality_depth_image_correspondence_0.png"));
  ASSERT_TRUE(CompareLoadedImages(
      modality_test_directory, "region_modality_depth_measured_occlusions.png",
      temp_directory, "region_modality_depth_image_correspondence_0.png", 0,
      0));
}

TEST_F(RegionModalityTest, CalculateCorrespondencesModeledOcclusions) {
  std::filesystem::create_directory(temp_directory);
  depth_renderer_ptr_->StartRendering();
  modality_ptr_->ModelOcclusions(depth_renderer_ptr_);
  modality_ptr_->set_n_unoccluded_iterations(0);
  modality_ptr_->set_visualize_lines_correspondence(true);
  modality_ptr_->set_visualize_points_depth_rendering_correspondence(true);
  modality_ptr_->set_display_visualization(false);
  modality_ptr_->StartSavingVisualizations(temp_directory);
  ASSERT_TRUE(modality_ptr_->SetUp());
  ASSERT_TRUE(modality_ptr_->StartModality(0, 0));
  ASSERT_TRUE(modality_ptr_->CalculateCorrespondences(0, 0));
  ASSERT_TRUE(modality_ptr_->VisualizeCorrespondences(0));
  ASSERT_TRUE(CopyOrSkipImage(kGenerateGroundTruth, modality_test_directory,
                              "region_modality_modeled_occlusions.png",
                              temp_directory,
                              "region_modality_lines_correspondence_0.png"));
  ASSERT_TRUE(CompareLoadedImages(
      modality_test_directory, "region_modality_modeled_occlusions.png",
      temp_directory, "region_modality_lines_correspondence_0.png", 0, 0));
  ASSERT_TRUE(CopyOrSkipImage(
      kGenerateGroundTruth, modality_test_directory,
      "region_modality_depth_modeled_occlusions.png", temp_directory,
      "region_modality_depth_rendering_correspondence_0.png"));
  ASSERT_TRUE(CompareLoadedImages(
      modality_test_directory, "region_modality_depth_modeled_occlusions.png",
      temp_directory, "region_modality_depth_rendering_correspondence_0.png", 0,
      0));
}

TEST_F(RegionModalityTest, CalculateGlobalGradientAndHessian) {
  ASSERT_TRUE(modality_ptr_->SetUp());
  ASSERT_TRUE(modality_ptr_->StartModality(0, 0));
  ASSERT_TRUE(modality_ptr_->CalculateCorrespondences(0, 0));
  ASSERT_TRUE(modality_ptr_->CalculateGradientAndHessian(0, 0, 0));
  ASSERT_TRUE(SaveOrSkipMatrix(kGenerateGroundTruth, modality_test_directory,
                               "region_modality_global_hessian.txt",
                               modality_ptr_->hessian()));
  ASSERT_TRUE(CompareToLoadedMatrix(modality_test_directory,
                                    "region_modality_global_hessian.txt",
                                    modality_ptr_->hessian(), 1.0e-3f));
  ASSERT_TRUE(SaveOrSkipMatrix(kGenerateGroundTruth, modality_test_directory,
                               "region_modality_global_gradient.txt",
                               modality_ptr_->gradient()));
  ASSERT_TRUE(CompareToLoadedMatrix(modality_test_directory,
                                    "region_modality_global_gradient.txt",
                                    modality_ptr_->gradient(), 1.0e-3f));
}

TEST_F(RegionModalityTest, CalculateLocalGradientAndHessian) {
  ASSERT_TRUE(modality_ptr_->SetUp());
  ASSERT_TRUE(modality_ptr_->StartModality(0, 0));
  ASSERT_TRUE(modality_ptr_->CalculateCorrespondences(0, 0));
  ASSERT_TRUE(modality_ptr_->CalculateGradientAndHessian(0, 0, 1));
  ASSERT_TRUE(SaveOrSkipMatrix(kGenerateGroundTruth, modality_test_directory,
                               "region_modality_local_hessian.txt",
                               modality_ptr_->hessian()));
  ASSERT_TRUE(CompareToLoadedMatrix(modality_test_directory,
                                    "region_modality_local_hessian.txt",
                                    modality_ptr_->hessian(), 1.0e-3f));
  ASSERT_TRUE(SaveOrSkipMatrix(kGenerateGroundTruth, modality_test_directory,
                               "region_modality_local_gradient.txt",
                               modality_ptr_->gradient()));
  ASSERT_TRUE(CompareToLoadedMatrix(modality_test_directory,
                                    "region_modality_local_gradient.txt",
                                    modality_ptr_->gradient(), 1.0e-3f));
}

class DepthModalityTest : public testing::Test {
 protected:
  void SetUp() override {
    model_ptr_ = TriangleDepthModelPtr();
    body_ptr_ = model_ptr_->body_ptr();
    depth_camera_ptr_ = DepthCameraPtr();
    depth_renderer_ptr_ = FocusedBasicDepthRendererPtr(
        body_ptr_, SchaumaBodyPtr(), depth_camera_ptr_);
    silhouette_renderer_ptr_ = FocusedSilhouetteRendererPtr(
        body_ptr_, SchaumaBodyPtr(), depth_camera_ptr_);
    silhouette_renderer_ptr_->set_id_type(m3t::IDType::BODY);
    modality_ptr_ = std::make_shared<m3t::DepthModality>(
        name_, body_ptr_, depth_camera_ptr_, model_ptr_);
  }

  bool TestModalityParameters(const m3t::DepthModality &modality) {
    return modality.name() == name_ &&
           modality.model_occlusions() == model_occlusions_ &&
           modality.measure_occlusions() == measure_occlusions_;
  }

  std::shared_ptr<m3t::DepthModality> modality_ptr_;
  std::string name_{"depth_modality"};
  std::filesystem::path metafile_path_{modality_test_directory /
                                       "depth_modality.yaml"};
  std::shared_ptr<m3t::Body> body_ptr_;
  std::shared_ptr<m3t::DepthCamera> depth_camera_ptr_;
  std::shared_ptr<m3t::DepthModel> model_ptr_;
  std::shared_ptr<m3t::FocusedDepthRenderer> depth_renderer_ptr_;
  std::shared_ptr<m3t::FocusedSilhouetteRenderer> silhouette_renderer_ptr_;
  bool model_occlusions_ = false;
  bool measure_occlusions_ = false;
};

TEST_F(DepthModalityTest, SetUpFromData) {
  ASSERT_TRUE(modality_ptr_->SetUp());
  ASSERT_TRUE(modality_ptr_->set_up());
  ASSERT_TRUE(TestModalityParameters(*modality_ptr_));
}

TEST_F(DepthModalityTest, SetUpFromMetaFile) {
  m3t::DepthModality modality{name_, metafile_path_, body_ptr_,
                              depth_camera_ptr_, model_ptr_};
  ASSERT_TRUE(modality.SetUp());
  ASSERT_TRUE(modality.set_up());
  ASSERT_TRUE(TestModalityParameters(modality));
}

TEST_F(DepthModalityTest, ParameterChanges) {
  ASSERT_TRUE(modality_ptr_->SetUp());
  ASSERT_TRUE(modality_ptr_->set_up());
  modality_ptr_->set_body_ptr(body_ptr_);
  ASSERT_FALSE(modality_ptr_->set_up());
  ASSERT_TRUE(modality_ptr_->SetUp());
  ASSERT_TRUE(modality_ptr_->set_up());
  modality_ptr_->set_depth_camera_ptr(depth_camera_ptr_);
  ASSERT_FALSE(modality_ptr_->set_up());
  ASSERT_TRUE(modality_ptr_->SetUp());
  ASSERT_TRUE(modality_ptr_->set_up());
  modality_ptr_->set_depth_model_ptr(model_ptr_);
  ASSERT_FALSE(modality_ptr_->set_up());
  ASSERT_TRUE(modality_ptr_->SetUp());
  ASSERT_TRUE(modality_ptr_->set_up());
  modality_ptr_->UseSilhouetteChecking(silhouette_renderer_ptr_);
  ASSERT_FALSE(modality_ptr_->set_up());
  ASSERT_TRUE(modality_ptr_->SetUp());
  ASSERT_TRUE(modality_ptr_->set_up());
  modality_ptr_->DoNotUseSilhouetteChecking();
  ASSERT_FALSE(modality_ptr_->set_up());
  ASSERT_TRUE(modality_ptr_->SetUp());
  ASSERT_TRUE(modality_ptr_->set_up());
  modality_ptr_->ModelOcclusions(depth_renderer_ptr_);
  ASSERT_FALSE(modality_ptr_->set_up());
  ASSERT_TRUE(modality_ptr_->SetUp());
  ASSERT_TRUE(modality_ptr_->set_up());
  modality_ptr_->DoNotModelOcclusions();
  ASSERT_FALSE(modality_ptr_->set_up());
  ASSERT_TRUE(modality_ptr_->SetUp());
  ASSERT_TRUE(modality_ptr_->set_up());
  modality_ptr_->MeasureOcclusions();
  ASSERT_FALSE(modality_ptr_->set_up());
  ASSERT_TRUE(modality_ptr_->SetUp());
  ASSERT_TRUE(modality_ptr_->set_up());
  modality_ptr_->DoNotMeasureOcclusions();
  ASSERT_FALSE(modality_ptr_->set_up());
  ASSERT_TRUE(modality_ptr_->SetUp());
  ASSERT_TRUE(modality_ptr_->set_up());
  ASSERT_TRUE(TestModalityParameters(*modality_ptr_));
}

TEST_F(DepthModalityTest, TestWithoutSetUp) {
  ASSERT_FALSE(modality_ptr_->StartModality(0, 0));
  ASSERT_FALSE(modality_ptr_->CalculateCorrespondences(0, 0));
  ASSERT_FALSE(modality_ptr_->VisualizeCorrespondences(0));
  ASSERT_FALSE(modality_ptr_->CalculateGradientAndHessian(0, 0, 0));
  ASSERT_FALSE(modality_ptr_->VisualizeOptimization(0));
  ASSERT_FALSE(modality_ptr_->CalculateResults(0));
  ASSERT_FALSE(modality_ptr_->VisualizeResults(0));
}

TEST_F(DepthModalityTest, TestWithoutSetUpBody) {
  modality_ptr_->set_body_ptr(TriangleBodyPtrNoSetUp());
  ASSERT_FALSE(modality_ptr_->SetUp());
}

TEST_F(DepthModalityTest, TestWithoutSetUpDepthModel) {
  modality_ptr_->set_depth_model_ptr(TriangleDepthModelPtrNoSetUp());
  ASSERT_FALSE(modality_ptr_->SetUp());
}

TEST_F(DepthModalityTest, TestWithoutSetUpCamera) {
  modality_ptr_->set_depth_camera_ptr(DepthCameraPtrNoSetUp());
  ASSERT_FALSE(modality_ptr_->SetUp());
}

TEST_F(DepthModalityTest, CalculateCorrespondences) {
  std::filesystem::create_directory(temp_directory);
  modality_ptr_->set_visualize_points_correspondence(true);
  modality_ptr_->set_visualize_correspondences_correspondence(true);
  modality_ptr_->set_display_visualization(false);
  modality_ptr_->StartSavingVisualizations(temp_directory);
  ASSERT_TRUE(modality_ptr_->SetUp());
  ASSERT_TRUE(modality_ptr_->CalculateCorrespondences(0, 0));
  ASSERT_TRUE(modality_ptr_->VisualizeCorrespondences(0));
  ASSERT_TRUE(
      CopyOrSkipImage(kGenerateGroundTruth, modality_test_directory,
                      "depth_modality_correspondences.png", temp_directory,
                      "depth_modality_correspondences_correspondence_0.png"));
  ASSERT_TRUE(CompareLoadedImages(
      modality_test_directory, "depth_modality_correspondences.png",
      temp_directory, "depth_modality_correspondences_correspondence_0.png", 0,
      0));
  ASSERT_TRUE(CopyOrSkipImage(
      kGenerateGroundTruth, modality_test_directory, "depth_modality.png",
      temp_directory, "depth_modality_depth_image_correspondence_0.png"));
  ASSERT_TRUE(CompareLoadedImages(
      modality_test_directory, "depth_modality.png", temp_directory,
      "depth_modality_depth_image_correspondence_0.png", 0, 0));
}

TEST_F(DepthModalityTest, CalculateCorrespondencesSilhouetteChecking) {
  std::filesystem::create_directory(temp_directory);
  silhouette_renderer_ptr_->StartRendering();
  modality_ptr_->UseSilhouetteChecking(silhouette_renderer_ptr_);
  modality_ptr_->set_visualize_points_correspondence(true);
  modality_ptr_->set_visualize_points_silhouette_rendering_correspondence(true);
  modality_ptr_->set_display_visualization(false);
  modality_ptr_->StartSavingVisualizations(temp_directory);
  ASSERT_TRUE(modality_ptr_->SetUp());
  ASSERT_TRUE(modality_ptr_->CalculateCorrespondences(0, 0));
  ASSERT_TRUE(modality_ptr_->VisualizeCorrespondences(0));
  ASSERT_TRUE(
      CopyOrSkipImage(kGenerateGroundTruth, modality_test_directory,
                      "depth_modality_silhouette_checking.png", temp_directory,
                      "depth_modality_depth_image_correspondence_0.png"));
  ASSERT_TRUE(CompareLoadedImages(
      modality_test_directory, "depth_modality_silhouette_checking.png",
      temp_directory, "depth_modality_depth_image_correspondence_0.png", 0, 0));
  ASSERT_TRUE(CopyOrSkipImage(
      kGenerateGroundTruth, modality_test_directory,
      "depth_modality_silhouette_silhouette_checking.png", temp_directory,
      "depth_modality_silhouette_rendering_correspondence_0.png"));
  ASSERT_TRUE(CompareLoadedImages(
      modality_test_directory,
      "depth_modality_silhouette_silhouette_checking.png", temp_directory,
      "depth_modality_silhouette_rendering_correspondence_0.png", 0, 0));
}

TEST_F(DepthModalityTest, CalculateCorrespondencesMeasuredOcclusions) {
  std::filesystem::create_directory(temp_directory);
  modality_ptr_->MeasureOcclusions();
  modality_ptr_->set_n_unoccluded_iterations(0);
  modality_ptr_->set_visualize_points_correspondence(true);
  modality_ptr_->set_display_visualization(false);
  modality_ptr_->StartSavingVisualizations(temp_directory);
  ASSERT_TRUE(modality_ptr_->SetUp());
  ASSERT_TRUE(modality_ptr_->CalculateCorrespondences(0, 0));
  ASSERT_TRUE(modality_ptr_->VisualizeCorrespondences(0));
  ASSERT_TRUE(
      CopyOrSkipImage(kGenerateGroundTruth, modality_test_directory,
                      "depth_modality_measured_occlusions.png", temp_directory,
                      "depth_modality_depth_image_correspondence_0.png"));
  ASSERT_TRUE(CompareLoadedImages(
      modality_test_directory, "depth_modality_measured_occlusions.png",
      temp_directory, "depth_modality_depth_image_correspondence_0.png", 0, 0));
}

TEST_F(DepthModalityTest, CalculateCorrespondencesModeledOcclusions) {
  std::filesystem::create_directory(temp_directory);
  depth_renderer_ptr_->StartRendering();
  modality_ptr_->ModelOcclusions(depth_renderer_ptr_);
  modality_ptr_->set_n_unoccluded_iterations(0);
  modality_ptr_->set_visualize_points_correspondence(true);
  modality_ptr_->set_visualize_points_depth_rendering_correspondence(true);
  modality_ptr_->set_display_visualization(false);
  modality_ptr_->StartSavingVisualizations(temp_directory);
  ASSERT_TRUE(modality_ptr_->SetUp());
  ASSERT_TRUE(modality_ptr_->CalculateCorrespondences(0, 0));
  ASSERT_TRUE(modality_ptr_->VisualizeCorrespondences(0));
  ASSERT_TRUE(
      CopyOrSkipImage(kGenerateGroundTruth, modality_test_directory,
                      "depth_modality_modeled_occlusions.png", temp_directory,
                      "depth_modality_depth_image_correspondence_0.png"));
  ASSERT_TRUE(CompareLoadedImages(
      modality_test_directory, "depth_modality_modeled_occlusions.png",
      temp_directory, "depth_modality_depth_image_correspondence_0.png", 0, 0));
  ASSERT_TRUE(CopyOrSkipImage(
      kGenerateGroundTruth, modality_test_directory,
      "depth_modality_depth_modeled_occlusions.png", temp_directory,
      "depth_modality_depth_rendering_correspondence_0.png"));
  ASSERT_TRUE(CompareLoadedImages(
      modality_test_directory, "depth_modality_depth_modeled_occlusions.png",
      temp_directory, "depth_modality_depth_rendering_correspondence_0.png", 0,
      0));
}

TEST_F(DepthModalityTest, CalculateGradientAndHessian) {
  ASSERT_TRUE(modality_ptr_->SetUp());
  ASSERT_TRUE(modality_ptr_->CalculateCorrespondences(0, 0));
  ASSERT_TRUE(modality_ptr_->CalculateGradientAndHessian(0, 0, 0));
  ASSERT_TRUE(SaveOrSkipMatrix(kGenerateGroundTruth, modality_test_directory,
                               "depth_modality_hessian.txt",
                               modality_ptr_->hessian()));
  ASSERT_TRUE(CompareToLoadedMatrix(modality_test_directory,
                                    "depth_modality_hessian.txt",
                                    modality_ptr_->hessian(), 1.0e-3f));
  ASSERT_TRUE(SaveOrSkipMatrix(kGenerateGroundTruth, modality_test_directory,
                               "depth_modality_gradient.txt",
                               modality_ptr_->gradient()));
  ASSERT_TRUE(CompareToLoadedMatrix(modality_test_directory,
                                    "depth_modality_gradient.txt",
                                    modality_ptr_->gradient(), 1.0e-3f));
}

class TextureModalityTest : public testing::Test {
 protected:
  void SetUp() override {
    body_ptr_ = TriangleBodyPtr();
    color_camera_ptr_ = ColorCameraPtr();
    depth_camera_ptr_ = DepthCameraPtr();
    silhouette_renderer_ptr_ = FocusedSilhouetteRendererPtr(
        body_ptr_, SchaumaBodyPtr(), color_camera_ptr_);
    modality_ptr_ = std::make_shared<m3t::TextureModality>(
        name_, body_ptr_, color_camera_ptr_, silhouette_renderer_ptr_);
    modality_ptr_->set_orb_n_features(orb_n_features_);
  }

  bool TestModalityParameters(const m3t::TextureModality &modality) {
    return modality.name() == name_ &&
           modality.descriptor_type() == descriptor_type &&
           modality.focused_image_size() == focused_image_size_ &&
           modality.descriptor_distance_threshold() ==
               descriptor_distance_threshold_ &&
           modality.tukey_norm_constant() == tukey_norm_constant_ &&
           modality.orb_n_features() == orb_n_features_ &&
           modality.orb_scale_factor() == orb_scale_factor_ &&
           modality.orb_n_levels() == orb_n_levels_ &&
           modality.measure_occlusions() == measure_occlusions_;
  }

  std::shared_ptr<m3t::TextureModality> modality_ptr_;
  std::string name_{"texture_modality"};
  std::filesystem::path metafile_path_{modality_test_directory /
                                       "texture_modality.yaml"};
  std::shared_ptr<m3t::Body> body_ptr_;
  std::shared_ptr<m3t::ColorCamera> color_camera_ptr_;
  std::shared_ptr<m3t::DepthCamera> depth_camera_ptr_;
  std::shared_ptr<m3t::FocusedSilhouetteRenderer> silhouette_renderer_ptr_;
  m3t::TextureModality::DescriptorType descriptor_type =
      m3t::TextureModality::DescriptorType::ORB;
  int focused_image_size_ = 200;
  float descriptor_distance_threshold_ = 0.7f;
  float tukey_norm_constant_ = 20.0f;
  int orb_n_features_ = 300;
  float orb_scale_factor_ = 1.2f;
  int orb_n_levels_ = 3;
  bool measure_occlusions_ = false;
};

TEST_F(TextureModalityTest, SetUpFromData) {
  ASSERT_TRUE(modality_ptr_->SetUp());
  ASSERT_TRUE(modality_ptr_->set_up());
  ASSERT_TRUE(TestModalityParameters(*modality_ptr_));
}

TEST_F(TextureModalityTest, SetUpFromMetaFile) {
  m3t::TextureModality modality{name_, metafile_path_, body_ptr_,
                                color_camera_ptr_, silhouette_renderer_ptr_};
  ASSERT_TRUE(modality.SetUp());
  ASSERT_TRUE(modality.set_up());
  ASSERT_TRUE(TestModalityParameters(modality));
}

TEST_F(TextureModalityTest, ParameterChanges) {
  ASSERT_TRUE(modality_ptr_->SetUp());
  ASSERT_TRUE(modality_ptr_->set_up());
  modality_ptr_->set_body_ptr(body_ptr_);
  ASSERT_FALSE(modality_ptr_->set_up());
  ASSERT_TRUE(modality_ptr_->SetUp());
  ASSERT_TRUE(modality_ptr_->set_up());
  modality_ptr_->set_color_camera_ptr(color_camera_ptr_);
  ASSERT_FALSE(modality_ptr_->set_up());
  ASSERT_TRUE(modality_ptr_->SetUp());
  ASSERT_TRUE(modality_ptr_->set_up());
  modality_ptr_->set_silhouette_renderer_ptr(silhouette_renderer_ptr_);
  ASSERT_FALSE(modality_ptr_->set_up());
  ASSERT_TRUE(modality_ptr_->SetUp());
  ASSERT_TRUE(modality_ptr_->set_up());
  modality_ptr_->set_descriptor_type(descriptor_type);
  ASSERT_FALSE(modality_ptr_->set_up());
  ASSERT_TRUE(modality_ptr_->SetUp());
  ASSERT_TRUE(modality_ptr_->set_up());
  modality_ptr_->set_orb_n_features(orb_n_features_);
  ASSERT_FALSE(modality_ptr_->set_up());
  ASSERT_TRUE(modality_ptr_->SetUp());
  ASSERT_TRUE(modality_ptr_->set_up());
  modality_ptr_->set_orb_scale_factor(orb_scale_factor_);
  ASSERT_FALSE(modality_ptr_->set_up());
  ASSERT_TRUE(modality_ptr_->SetUp());
  ASSERT_TRUE(modality_ptr_->set_up());
  modality_ptr_->set_orb_n_levels(orb_n_levels_);
  ASSERT_FALSE(modality_ptr_->set_up());
  ASSERT_TRUE(modality_ptr_->SetUp());
  ASSERT_TRUE(modality_ptr_->set_up());
  modality_ptr_->MeasureOcclusions(depth_camera_ptr_);
  ASSERT_FALSE(modality_ptr_->set_up());
  ASSERT_TRUE(modality_ptr_->SetUp());
  ASSERT_TRUE(modality_ptr_->set_up());
  modality_ptr_->DoNotMeasureOcclusions();
  ASSERT_FALSE(modality_ptr_->set_up());
  ASSERT_TRUE(modality_ptr_->SetUp());
  ASSERT_TRUE(modality_ptr_->set_up());
  ASSERT_TRUE(TestModalityParameters(*modality_ptr_));
}

TEST_F(TextureModalityTest, TestWithoutSetUp) {
  ASSERT_FALSE(modality_ptr_->StartModality(0, 0));
  ASSERT_FALSE(modality_ptr_->CalculateCorrespondences(0, 0));
  ASSERT_FALSE(modality_ptr_->VisualizeCorrespondences(0));
  ASSERT_FALSE(modality_ptr_->CalculateGradientAndHessian(0, 0, 0));
  ASSERT_FALSE(modality_ptr_->VisualizeOptimization(0));
  ASSERT_FALSE(modality_ptr_->CalculateResults(0));
  ASSERT_FALSE(modality_ptr_->VisualizeResults(0));
}

TEST_F(TextureModalityTest, TestWithoutSetUpBody) {
  modality_ptr_->set_body_ptr(TriangleBodyPtrNoSetUp());
  ASSERT_FALSE(modality_ptr_->SetUp());
}

TEST_F(TextureModalityTest, TestWithoutSetUpSilhouetteRenderer) {
  modality_ptr_->set_silhouette_renderer_ptr(
      FocusedSilhouetteRendererPtrNoSetUp(body_ptr_, SchaumaBodyPtr(),
                                          color_camera_ptr_));
  ASSERT_FALSE(modality_ptr_->SetUp());
}

TEST_F(TextureModalityTest, TestWithoutSetUpCamera) {
  modality_ptr_->set_color_camera_ptr(ColorCameraPtrNoSetUp());
  ASSERT_FALSE(modality_ptr_->SetUp());
}

TEST_F(TextureModalityTest, Reconstruct3DPoints) {
  std::filesystem::create_directory(temp_directory);
  silhouette_renderer_ptr_->StartRendering();
  modality_ptr_->set_visualize_points_silhouette_rendering_result(true);
  modality_ptr_->set_display_visualization(false);
  modality_ptr_->StartSavingVisualizations(temp_directory);
  ASSERT_TRUE(modality_ptr_->SetUp());
  ASSERT_TRUE(modality_ptr_->StartModality(0, 0));
  ASSERT_TRUE(modality_ptr_->VisualizeResults(0));
  ASSERT_TRUE(CopyOrSkipImage(
      kGenerateGroundTruth, modality_test_directory, "texture_modality.png",
      temp_directory, "texture_modality_silhouette_rendering_result_0.png"));
  ASSERT_TRUE(CompareLoadedImages(
      modality_test_directory, "texture_modality.png", temp_directory,
      "texture_modality_silhouette_rendering_result_0.png", 0, 0));
}

TEST_F(TextureModalityTest, Reconstruct3DPointsMeasuredOcclusions) {
  std::filesystem::create_directory(temp_directory);
  silhouette_renderer_ptr_->StartRendering();
  modality_ptr_->MeasureOcclusions(depth_camera_ptr_);
  modality_ptr_->set_visualize_points_depth_image_result(true);
  modality_ptr_->set_display_visualization(false);
  modality_ptr_->StartSavingVisualizations(temp_directory);
  ASSERT_TRUE(modality_ptr_->SetUp());
  ASSERT_TRUE(modality_ptr_->StartModality(0, 0));
  ASSERT_TRUE(modality_ptr_->VisualizeResults(0));
  ASSERT_TRUE(CopyOrSkipImage(kGenerateGroundTruth, modality_test_directory,
                              "texture_modality_measured_occlusions.png",
                              temp_directory,
                              "texture_modality_depth_image_result_0.png"));
  ASSERT_TRUE(CompareLoadedImages(
      modality_test_directory, "texture_modality_measured_occlusions.png",
      temp_directory, "texture_modality_depth_image_result_0.png", 0, 0));
}

TEST_F(TextureModalityTest, Reconstruct3DPointsModelledOcclusions) {
  std::filesystem::create_directory(temp_directory);
  silhouette_renderer_ptr_->StartRendering();
  modality_ptr_->ModelOcclusions(silhouette_renderer_ptr_);
  modality_ptr_->set_visualize_points_depth_rendering_result(true);
  modality_ptr_->set_display_visualization(false);
  modality_ptr_->StartSavingVisualizations(temp_directory);
  ASSERT_TRUE(modality_ptr_->SetUp());
  ASSERT_TRUE(modality_ptr_->StartModality(0, 0));
  ASSERT_TRUE(modality_ptr_->VisualizeResults(0));
  ASSERT_TRUE(CopyOrSkipImage(kGenerateGroundTruth, modality_test_directory,
                              "texture_modality_modeled_occlusions.png",
                              temp_directory,
                              "texture_modality_depth_rendering_result_0.png"));
  ASSERT_TRUE(CompareLoadedImages(
      modality_test_directory, "texture_modality_modeled_occlusions.png",
      temp_directory, "texture_modality_depth_rendering_result_0.png", 0, 0));
}

TEST_F(TextureModalityTest, CalculateCorrespondences) {
  std::filesystem::create_directory(temp_directory);
  silhouette_renderer_ptr_->StartRendering();
  modality_ptr_->set_visualize_correspondences_correspondence(true);
  modality_ptr_->set_display_visualization(false);
  modality_ptr_->StartSavingVisualizations(temp_directory);
  ASSERT_TRUE(modality_ptr_->SetUp());
  ASSERT_TRUE(modality_ptr_->StartModality(0, 0));
  ASSERT_TRUE(color_camera_ptr_->UpdateImage(false));
  ASSERT_TRUE(modality_ptr_->CalculateCorrespondences(0, 0));
  ASSERT_TRUE(modality_ptr_->VisualizeCorrespondences(0));
  ASSERT_TRUE(
      CopyOrSkipImage(kGenerateGroundTruth, modality_test_directory,
                      "texture_modality_correspondence.png", temp_directory,
                      "texture_modality_correspondences_correspondence_0.png"));
  ASSERT_TRUE(CompareLoadedImages(
      modality_test_directory, "texture_modality_correspondence.png",
      temp_directory, "texture_modality_correspondences_correspondence_0.png",
      0, 0));
}

TEST_F(TextureModalityTest, CalculateGradientAndHessian) {
  silhouette_renderer_ptr_->StartRendering();
  ASSERT_TRUE(modality_ptr_->SetUp());
  ASSERT_TRUE(modality_ptr_->StartModality(0, 0));
  ASSERT_TRUE(color_camera_ptr_->UpdateImage(false));
  ASSERT_TRUE(modality_ptr_->CalculateCorrespondences(0, 0));
  ASSERT_TRUE(modality_ptr_->CalculateGradientAndHessian(0, 0, 0));
  ASSERT_TRUE(SaveOrSkipMatrix(kGenerateGroundTruth, modality_test_directory,
                               "texture_modality_hessian.txt",
                               modality_ptr_->hessian()));
  ASSERT_TRUE(CompareToLoadedMatrix(modality_test_directory,
                                    "texture_modality_hessian.txt",
                                    modality_ptr_->hessian(), 1.0e-3f));
  ASSERT_TRUE(SaveOrSkipMatrix(kGenerateGroundTruth, modality_test_directory,
                               "texture_modality_gradient.txt",
                               modality_ptr_->gradient()));
  ASSERT_TRUE(CompareToLoadedMatrix(modality_test_directory,
                                    "texture_modality_gradient.txt",
                                    modality_ptr_->gradient(), 1.0e-3f));
}
