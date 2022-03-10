// SPDX-License-Identifier: MIT
// Copyright (c) 2022 Manuel Stoiber, German Aerospace Center (DLR)

#include <filesystem/filesystem.h>
#include <icg/azure_kinect_camera.h>
#include <icg/basic_depth_renderer.h>
#include <icg/body.h>
#include <icg/common.h>
#include <icg/depth_modality.h>
#include <icg/depth_model.h>
#include <icg/normal_viewer.h>
#include <icg/region_modality.h>
#include <icg/region_model.h>
#include <icg/renderer_geometry.h>
#include <icg/static_detector.h>
#include <icg/tracker.h>

#include <Eigen/Geometry>
#include <memory>
#include <string>

int main(int argc, char *argv[]) {
  if (argc < 3) {
    std::cerr << "Not enough arguments: Provide directory and body_names";
    return 0;
  }
  const std::filesystem::path directory{argv[1]};
  std::vector<std::string> body_names;
  for (int i = 2; i < argc; ++i) {
    body_names.push_back(std::string{argv[i]});
  }

  constexpr bool kUseDepthViewer = true;
  constexpr bool kMeasureOcclusions = true;
  constexpr bool kModelOcclusions = false;
  constexpr bool kVisualizePoseResult = false;
  constexpr bool kSaveImages = false;
  const std::filesystem::path save_directory{""};

  // Set up tracker and renderer geometry
  auto tracker_ptr{std::make_shared<icg::Tracker>("tracker")};
  auto renderer_geometry_ptr{
      std::make_shared<icg::RendererGeometry>("renderer geometry")};

  // Set up cameras
  auto color_camera_ptr{
      std::make_shared<icg::AzureKinectColorCamera>("azure_kinect_color")};
  auto depth_camera_ptr{
      std::make_shared<icg::AzureKinectDepthCamera>("azure_kinect_depth")};

  // Set up viewers
  auto color_viewer_ptr{std::make_shared<icg::NormalColorViewer>(
      "color_viewer", color_camera_ptr, renderer_geometry_ptr)};
  if (kSaveImages) color_viewer_ptr->StartSavingImages(save_directory, "bmp");
  tracker_ptr->AddViewer(color_viewer_ptr);
  if (kUseDepthViewer) {
    auto depth_viewer_ptr{std::make_shared<icg::NormalDepthViewer>(
        "depth_viewer", depth_camera_ptr, renderer_geometry_ptr, 0.3f, 1.0f)};
    if (kSaveImages) depth_viewer_ptr->StartSavingImages(save_directory, "bmp");
    tracker_ptr->AddViewer(depth_viewer_ptr);
  }

  // Set up depth renderer
  auto color_depth_renderer_ptr{
      std::make_shared<icg::FocusedBasicDepthRenderer>(
          "color_depth_renderer", renderer_geometry_ptr, color_camera_ptr)};
  auto depth_depth_renderer_ptr{
      std::make_shared<icg::FocusedBasicDepthRenderer>(
          "depth_depth_renderer", renderer_geometry_ptr, depth_camera_ptr)};

  for (const auto body_name : body_names) {
    // Set up body
    std::filesystem::path metafile_path{directory / (body_name + ".yaml")};
    auto body_ptr{std::make_shared<icg::Body>(body_name, metafile_path)};
    renderer_geometry_ptr->AddBody(body_ptr);
    color_depth_renderer_ptr->AddReferencedBody(body_ptr);
    depth_depth_renderer_ptr->AddReferencedBody(body_ptr);

    // Set up detector
    std::filesystem::path detector_path{directory /
                                        (body_name + "_detector.yaml")};
    auto detector_ptr{std::make_shared<icg::StaticDetector>(
        body_name + "_detector", detector_path, body_ptr)};
    tracker_ptr->AddDetector(detector_ptr);

    // Set up models
    auto region_model_ptr{std::make_shared<icg::RegionModel>(
        body_name + "_region_model", body_ptr,
        directory / (body_name + "_region_model.bin"))};
    auto depth_model_ptr{std::make_shared<icg::DepthModel>(
        body_name + "_depth_model", body_ptr,
        directory / (body_name + "_depth_model.bin"))};

    // Set up modalities
    auto region_modality_ptr{std::make_shared<icg::RegionModality>(
        body_name + "_region_modality", body_ptr, color_camera_ptr,
        region_model_ptr)};
    auto depth_modality_ptr{std::make_shared<icg::DepthModality>(
        body_name + "_depth_modality", body_ptr, depth_camera_ptr,
        depth_model_ptr)};
    if (kVisualizePoseResult) {
      region_modality_ptr->set_visualize_pose_result(true);
    }
    if (kMeasureOcclusions) {
      region_modality_ptr->MeasureOcclusions(depth_camera_ptr);
      depth_modality_ptr->MeasureOcclusions();
    }
    if (kModelOcclusions) {
      region_modality_ptr->ModelOcclusions(color_depth_renderer_ptr);
      depth_modality_ptr->ModelOcclusions(depth_depth_renderer_ptr);
    }

    // Set up optimizer
    auto body1_optimizer_ptr{
        std::make_shared<icg::Optimizer>(body_name + "_optimizer")};
    body1_optimizer_ptr->AddModality(region_modality_ptr);
    body1_optimizer_ptr->AddModality(depth_modality_ptr);
    tracker_ptr->AddOptimizer(body1_optimizer_ptr);
  }

  // Start tracking
  if (!tracker_ptr->SetUp()) return 0;
  if (!tracker_ptr->RunTrackerProcess(true, false)) return 0;
  return 0;
}
