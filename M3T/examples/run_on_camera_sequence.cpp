// SPDX-License-Identifier: MIT
// Copyright (c) 2023 Manuel Stoiber, German Aerospace Center (DLR)

#include <filesystem/filesystem.h>
#include <m3t/azure_kinect_camera.h>
#include <m3t/basic_depth_renderer.h>
#include <m3t/body.h>
#include <m3t/common.h>
#include <m3t/depth_modality.h>
#include <m3t/depth_model.h>
#include <m3t/link.h>
#include <m3t/normal_viewer.h>
#include <m3t/region_modality.h>
#include <m3t/region_model.h>
#include <m3t/renderer_geometry.h>
#include <m3t/static_detector.h>
#include <m3t/texture_modality.h>
#include <m3t/tracker.h>

#include <Eigen/Geometry>
#include <memory>
#include <string>

int main(int argc, char *argv[]) {
  if (argc < 3) {
    std::cerr << "Not enough arguments: Provide directory and body_names";
    return -1;
  }
  const std::filesystem::path directory{argv[1]};
  std::vector<std::string> body_names;
  for (int i = 2; i < argc; ++i) {
    body_names.push_back(std::string{argv[i]});
  }

  constexpr bool kUseDepthViewer = true;
  constexpr bool kUseRegionModality = true;
  constexpr bool kUseTextureModality = false;
  constexpr bool kUseDepthModality = true;
  constexpr bool kMeasureOcclusions = true;
  constexpr bool kModelOcclusions = false;
  constexpr bool kVisualizePoseResult = false;
  constexpr bool kSaveImages = false;
  const std::filesystem::path save_directory{""};

  // Set up tracker and renderer geometry
  auto tracker_ptr{std::make_shared<m3t::Tracker>("tracker")};
  auto renderer_geometry_ptr{
      std::make_shared<m3t::RendererGeometry>("renderer geometry")};

  // Set up cameras
  auto color_camera_ptr{
      std::make_shared<m3t::AzureKinectColorCamera>("azure_kinect_color")};
  auto depth_camera_ptr{
      std::make_shared<m3t::AzureKinectDepthCamera>("azure_kinect_depth")};

  // Set up viewers
  auto color_viewer_ptr{std::make_shared<m3t::NormalColorViewer>(
      "color_viewer", color_camera_ptr, renderer_geometry_ptr)};
  if (kSaveImages) color_viewer_ptr->StartSavingImages(save_directory, "bmp");
  tracker_ptr->AddViewer(color_viewer_ptr);
  if (kUseDepthViewer) {
    auto depth_viewer_ptr{std::make_shared<m3t::NormalDepthViewer>(
        "depth_viewer", depth_camera_ptr, renderer_geometry_ptr, 0.3f, 1.0f)};
    if (kSaveImages) depth_viewer_ptr->StartSavingImages(save_directory, "bmp");
    tracker_ptr->AddViewer(depth_viewer_ptr);
  }

  // Set up depth renderer
  auto color_depth_renderer_ptr{
      std::make_shared<m3t::FocusedBasicDepthRenderer>(
          "color_depth_renderer", renderer_geometry_ptr, color_camera_ptr)};
  auto depth_depth_renderer_ptr{
      std::make_shared<m3t::FocusedBasicDepthRenderer>(
          "depth_depth_renderer", renderer_geometry_ptr, depth_camera_ptr)};

  // Set up silhouette renderer
  auto color_silhouette_renderer_ptr{
      std::make_shared<m3t::FocusedSilhouetteRenderer>(
          "color_silhouette_renderer", renderer_geometry_ptr,
          color_camera_ptr)};

  for (const auto body_name : body_names) {
    // Set up body
    std::filesystem::path metafile_path{directory / (body_name + ".yaml")};
    auto body_ptr{std::make_shared<m3t::Body>(body_name, metafile_path)};
    renderer_geometry_ptr->AddBody(body_ptr);
    color_depth_renderer_ptr->AddReferencedBody(body_ptr);
    depth_depth_renderer_ptr->AddReferencedBody(body_ptr);
    color_silhouette_renderer_ptr->AddReferencedBody(body_ptr);

    // Set up models
    auto region_model_ptr{std::make_shared<m3t::RegionModel>(
        body_name + "_region_model", body_ptr,
        directory / (body_name + "_region_model.bin"))};
    auto depth_model_ptr{std::make_shared<m3t::DepthModel>(
        body_name + "_depth_model", body_ptr,
        directory / (body_name + "_depth_model.bin"))};

    // Set up modalities
    auto region_modality_ptr{std::make_shared<m3t::RegionModality>(
        body_name + "_region_modality", body_ptr, color_camera_ptr,
        region_model_ptr)};
    auto texture_modality_ptr{std::make_shared<m3t::TextureModality>(
        body_name + "_texture_modality", body_ptr, color_camera_ptr,
        color_silhouette_renderer_ptr)};
    auto depth_modality_ptr{std::make_shared<m3t::DepthModality>(
        body_name + "_depth_modality", body_ptr, depth_camera_ptr,
        depth_model_ptr)};
    if (kVisualizePoseResult) {
      region_modality_ptr->set_visualize_pose_result(true);
    }
    if (kMeasureOcclusions) {
      region_modality_ptr->MeasureOcclusions(depth_camera_ptr);
      texture_modality_ptr->MeasureOcclusions(depth_camera_ptr);
      depth_modality_ptr->MeasureOcclusions();
    }
    if (kModelOcclusions) {
      region_modality_ptr->ModelOcclusions(color_depth_renderer_ptr);
      texture_modality_ptr->ModelOcclusions(color_depth_renderer_ptr);
      depth_modality_ptr->ModelOcclusions(depth_depth_renderer_ptr);
    }

    // Set up link
    auto link_ptr{std::make_shared<m3t::Link>(body_name + "_link", body_ptr)};
    if (kUseRegionModality) link_ptr->AddModality(region_modality_ptr);
    if (kUseTextureModality) link_ptr->AddModality(texture_modality_ptr);
    if (kUseDepthModality) link_ptr->AddModality(depth_modality_ptr);

    // Set up optimizer
    auto optimizer_ptr{
        std::make_shared<m3t::Optimizer>(body_name + "_optimizer", link_ptr)};
    tracker_ptr->AddOptimizer(optimizer_ptr);

    // Set up detector
    std::filesystem::path detector_path{directory /
                                        (body_name + "_detector.yaml")};
    auto detector_ptr{std::make_shared<m3t::StaticDetector>(
        body_name + "_detector", detector_path, optimizer_ptr)};
    tracker_ptr->AddDetector(detector_ptr);
  }

  // Start tracking
  if (!tracker_ptr->SetUp()) return -1;
  if (!tracker_ptr->RunTrackerProcess(true, false)) return -1;
  return 0;
}
