// SPDX-License-Identifier: MIT
// Copyright (c) 2020 Manuel Stoiber, German Aerospace Center (DLR)

#include <rbgt/body.h>
#include <rbgt/common.h>
#include <rbgt/azure_kinect_camera.h>
#include <rbgt/normal_image_viewer.h>
#include <rbgt/occlusion_mask_renderer.h>
#include <rbgt/region_modality.h>
#include <rbgt/renderer_geometry.h>
#include <rbgt/tracker.h>

#include <Eigen/Geometry>
#include <memory>
#include <string>

int main() {
  // Change accordingly
  const std::string model_path{"/your/path/to/save/and/load/models/"};

  constexpr bool kSaveViewerImage = false;
  const std::string viewer_save_path{"/your/path/to/save/viewer/images/"};

  // Set up tracker and renderer geometry
  auto tracker_ptr{std::make_shared<rbgt::Tracker>()};
  auto renderer_geometry_ptr{std::make_shared<rbgt::RendererGeometry>()};

  // Set up camera
  auto camera_ptr{std::make_shared<rbgt::AzureKinectCamera>()};
  camera_ptr->Init("azure kinect");

  // Set up viewers
  auto viewer_ptr{std::make_shared<rbgt::NormalImageViewer>()};
  viewer_ptr->Init("viewer", renderer_geometry_ptr, camera_ptr);
  if (kSaveViewerImage) viewer_ptr->StartSavingImages(viewer_save_path);
  tracker_ptr->AddViewer(viewer_ptr);

  // Set up body1 (change accordingly)
  const std::string body1_geometry_path{"/your/path/to/body1.obj"};
  rbgt::Transform3fA body1_geometry2body_pose{
      Eigen::Translation3f(0.0f, 0.0f, 0.0f)};
  rbgt::Transform3fA body1_world2body_pose;
  body1_world2body_pose.matrix() << -0.845644f, -0.271396f, 0.459818f,
      -0.0707055f, -0.533926f, 0.43576f, -0.724728f, 0.254323f, -0.00367445f,
      -0.858296f, -0.513344f, 0.279658f, 0.0f, 0.0f, 0.0f, 1.0f;
  auto body1_ptr{std::make_shared<rbgt::Body>("body1", body1_geometry_path,
                                              1.0f, true, true, 0.1f,
                                              body1_geometry2body_pose)};
  body1_ptr->set_world2body_pose(body1_world2body_pose);
  body1_ptr->set_occlusion_mask_id(1);
  renderer_geometry_ptr->AddBody(body1_ptr);

  // Set up model body 1
  const std::string body1_model_name{"body1_model"};
  auto body1_model_ptr{std::make_shared<rbgt::Model>(body1_model_name)};
  if (!body1_model_ptr->LoadModel(model_path, body1_model_name)) {
    body1_model_ptr->GenerateModel(*body1_ptr, 0.8f, 4, 200);
    body1_model_ptr->SaveModel(model_path, body1_model_name);
  }

  // Set up region modality body 1
  auto body1_region_modality_ptr{std::make_shared<rbgt::RegionModality>()};
  body1_region_modality_ptr->Init("body1_region_modality", body1_ptr,
                                  body1_model_ptr, camera_ptr);
  tracker_ptr->AddRegionModality(body1_region_modality_ptr);

  // Set up body2 (change accordingly)
  const std::string body2_geometry_path{"/your/path/to/body2.obj"};
  rbgt::Transform3fA body2_geometry2body_pose{
      Eigen::Translation3f(0.0f, 0.0f, 0.0f)};
  rbgt::Transform3fA body2_world2body_pose;
  body2_world2body_pose.matrix() << -0.999495f, -0.0298023f, -0.0120959f,
      0.108489f, -0.0319917f, 0.88242f, 0.469534f, -0.155606f, -0.00332271f,
      0.469629f, -0.882873f, 0.416361f, 0.0f, 0.0f, 0.0f, 1.0f;
  auto body2_ptr{std::make_shared<rbgt::Body>("body2", body2_geometry_path,
                                              1.0f, true, true, 0.1f,
                                              body2_geometry2body_pose)};
  body2_ptr->set_world2body_pose(body2_world2body_pose);
  body2_ptr->set_occlusion_mask_id(2);
  renderer_geometry_ptr->AddBody(body2_ptr);

  // Set up model body 2
  const std::string body2_model_name{"body2_model"};
  auto body2_model_ptr{std::make_shared<rbgt::Model>(body2_model_name)};
  if (!body2_model_ptr->LoadModel(model_path, body2_model_name)) {
    body2_model_ptr->GenerateModel(*body2_ptr, 0.8f, 4, 200);
    body2_model_ptr->SaveModel(model_path, body2_model_name);
  }

  // Set up region modality body 2
  auto body2_region_modality_ptr{std::make_shared<rbgt::RegionModality>()};
  body2_region_modality_ptr->Init("body2_region_modality", body2_ptr,
                                  body2_model_ptr, camera_ptr);
  tracker_ptr->AddRegionModality(body2_region_modality_ptr);

  // Set up occlusion mask renderer
  auto occlusion_mask_renderer_ptr{
      std::make_shared<rbgt::OcclusionMaskRenderer>()};
  occlusion_mask_renderer_ptr->InitFromCamera(
      "occlusion_mask_renderer", renderer_geometry_ptr, *camera_ptr);
  body1_region_modality_ptr->UseOcclusionHandling(occlusion_mask_renderer_ptr);
  body2_region_modality_ptr->UseOcclusionHandling(occlusion_mask_renderer_ptr);

  // Start tracking
  tracker_ptr->StartTracker(false);
  return 0;
}
