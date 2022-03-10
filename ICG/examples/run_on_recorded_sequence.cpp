// SPDX-License-Identifier: MIT
// Copyright (c) 2022 Manuel Stoiber, German Aerospace Center (DLR)

#include <filesystem/filesystem.h>
#include <icg/basic_depth_renderer.h>
#include <icg/body.h>
#include <icg/common.h>
#include <icg/loader_camera.h>
#include <icg/manual_detector.h>
#include <icg/normal_viewer.h>
#include <icg/region_modality.h>
#include <icg/renderer_geometry.h>
#include <icg/tracker.h>

#include <Eigen/Geometry>
#include <memory>

// Example script for the detector usage on the data provided in data/sequence
// with the object triangle
int main(int argc, char *argv[]) {
  if (argc != 5) {
    std::cerr << "Not enough arguments: Provide camera metafile, body "
                 "metafile, detector metafile, temp directory";
    return 0;
  }
  const std::filesystem::path color_camera_metafile_path{argv[1]};
  const std::filesystem::path body_metafile_path{argv[2]};
  const std::filesystem::path detector_metafile_path{argv[3]};
  const std::filesystem::path temp_directory{argv[4]};

  // Set up tracker and renderer geometry
  auto tracker_ptr{std::make_shared<icg::Tracker>("tracker")};
  auto renderer_geometry_ptr{
      std::make_shared<icg::RendererGeometry>("renderer_geometry")};

  // Set up camera
  auto camera_ptr{std::make_shared<icg::LoaderColorCamera>(
      "color_camera", color_camera_metafile_path)};

  // Set up viewers
  auto viewer_ptr{std::make_shared<icg::NormalColorViewer>(
      "viewer", camera_ptr, renderer_geometry_ptr)};
  tracker_ptr->AddViewer(viewer_ptr);

  // Set up body triangle
  auto body_ptr{std::make_shared<icg::Body>("triangle", body_metafile_path)};
  renderer_geometry_ptr->AddBody(body_ptr);

  // Set up detector
  auto body_detector{std::make_shared<icg::ManualDetector>(
      "body_detector", detector_metafile_path, body_ptr, camera_ptr)};
  tracker_ptr->AddDetector(body_detector);

  // Set up region model body
  auto body_region_model_ptr{std::make_shared<icg::RegionModel>(
      "body_region_model", body_ptr, temp_directory / "body_region_model.bin")};

  // Set up region modality body
  auto body_region_modality_ptr{std::make_shared<icg::RegionModality>(
      "body_region_modality", body_ptr, camera_ptr, body_region_model_ptr)};

  // Set up optimizer body
  auto body_optimizer_ptr{std::make_shared<icg::Optimizer>("body_optimizer")};
  body_optimizer_ptr->AddModality(body_region_modality_ptr);
  tracker_ptr->AddOptimizer(body_optimizer_ptr);

  // Start tracking
  if (!tracker_ptr->SetUp()) return 0;
  if (!tracker_ptr->RunTrackerProcess(false, false)) return 0;
  return 0;
}
