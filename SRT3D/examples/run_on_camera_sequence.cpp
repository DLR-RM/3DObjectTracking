// SPDX-License-Identifier: MIT
// Copyright (c) 2021 Manuel Stoiber, German Aerospace Center (DLR)

#include <srt3d/azure_kinect_camera.h>
#include <srt3d/body.h>
#include <srt3d/common.h>
#include <srt3d/normal_viewer.h>
#include <srt3d/occlusion_renderer.h>
#include <srt3d/region_modality.h>
#include <srt3d/renderer_geometry.h>
#include <srt3d/tracker.h>

#include <Eigen/Geometry>
#include <filesystem>
#include <memory>
#include <string>

int main() {
  // Change accordingly
  const std::filesystem::path model_directory{
      "/your/directory/to/save/and/load/models/"};

  constexpr bool kSaveViewerImage = false;
  const std::filesystem::path viewer_save_directory{
      "/your/directory/to/save/viewer/images/"};

  // Set up tracker and renderer geometry
  auto tracker_ptr{std::make_shared<srt3d::Tracker>("tracker")};
  auto renderer_geometry_ptr{
      std::make_shared<srt3d::RendererGeometry>("renderer geometry")};

  // Set up camera
  auto camera_ptr{std::make_shared<srt3d::AzureKinectCamera>("azure_kinect")};

  // Set up viewers
  auto viewer_ptr{std::make_shared<srt3d::NormalViewer>("viewer", camera_ptr,
                                                        renderer_geometry_ptr)};
  if (kSaveViewerImage) viewer_ptr->StartSavingImages(viewer_save_directory);
  tracker_ptr->AddViewer(viewer_ptr);

  // Set up body1 (change accordingly)
  const std::filesystem::path body1_geometry_path{"/your/path/to/body1.obj"};
  srt3d::Transform3fA body1_geometry2body_pose{
      Eigen::Translation3f(0.0f, 0.0f, 0.0f)};
  srt3d::Transform3fA body1_world2body_pose;
  body1_world2body_pose.matrix() << -0.845644f, -0.271396f, 0.459818f,
      -0.0707055f, -0.533926f, 0.43576f, -0.724728f, 0.254323f, -0.00367445f,
      -0.858296f, -0.513344f, 0.279658f, 0.0f, 0.0f, 0.0f, 1.0f;
  auto body1_ptr{std::make_shared<srt3d::Body>("body1", body1_geometry_path,
                                               1.0f, true, true, 0.1f,
                                               body1_geometry2body_pose, 1)};
  body1_ptr->set_world2body_pose(body1_world2body_pose);
  renderer_geometry_ptr->AddBody(body1_ptr);

  // Set up model body 1
  auto body1_model_ptr{std::make_shared<srt3d::Model>(
      "body1_model", body1_ptr, model_directory, "body1_model.bin")};

  // Set up region modality body 1
  auto body1_region_modality_ptr{std::make_shared<srt3d::RegionModality>(
      "body1_region_modality", body1_ptr, body1_model_ptr, camera_ptr)};
  tracker_ptr->AddRegionModality(body1_region_modality_ptr);

  // Set up body2 (change accordingly)
  const std::filesystem::path body2_geometry_path{"/your/path/to/body2.obj"};
  srt3d::Transform3fA body2_geometry2body_pose{
      Eigen::Translation3f(0.0f, 0.0f, 0.0f)};
  srt3d::Transform3fA body2_world2body_pose;
  body2_world2body_pose.matrix() << -0.999495f, -0.0298023f, -0.0120959f,
      0.108489f, -0.0319917f, 0.88242f, 0.469534f, -0.155606f, -0.00332271f,
      0.469629f, -0.882873f, 0.416361f, 0.0f, 0.0f, 0.0f, 1.0f;
  auto body2_ptr{std::make_shared<srt3d::Body>("body2", body2_geometry_path,
                                               1.0f, true, true, 0.1f,
                                               body2_geometry2body_pose, 2)};
  body2_ptr->set_world2body_pose(body2_world2body_pose);
  renderer_geometry_ptr->AddBody(body2_ptr);

  // Set up model body 2
  const std::string body2_model_name{"body2_model"};
  auto body2_model_ptr{std::make_shared<srt3d::Model>(
      "body2_model", body2_ptr, model_directory, "body2_model.bin")};

  // Set up region modality body 2
  auto body2_region_modality_ptr{std::make_shared<srt3d::RegionModality>(
      "body2_region_modality", body2_ptr, body2_model_ptr, camera_ptr)};
  tracker_ptr->AddRegionModality(body2_region_modality_ptr);

  // Set up occlusion renderer
  auto occlusion_renderer_ptr{std::make_shared<srt3d::OcclusionRenderer>(
      "occlusion_renderer", renderer_geometry_ptr, camera_ptr)};
  body1_region_modality_ptr->UseOcclusionHandling(occlusion_renderer_ptr);
  body2_region_modality_ptr->UseOcclusionHandling(occlusion_renderer_ptr);

  // Start tracking
  tracker_ptr->SetUpTracker();
  tracker_ptr->StartTracker(false);
  return 0;
}
