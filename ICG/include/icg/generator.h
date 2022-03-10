// SPDX-License-Identifier: MIT
// Copyright (c) 2022 Manuel Stoiber, German Aerospace Center (DLR)

#ifndef ICG_INCLUDE_ICG_GENERATOR_H_
#define ICG_INCLUDE_ICG_GENERATOR_H_

#ifdef USE_REALSENSE
#include <icg/realsense_camera.h>
#endif
#ifdef USE_AZURE_KINECT
#include <icg/azure_kinect_camera.h>
#endif
#include <icg/basic_depth_renderer.h>
#include <icg/body.h>
#include <icg/camera.h>
#include <icg/common.h>
#include <icg/depth_modality.h>
#include <icg/depth_model.h>
#include <icg/detector.h>
#include <icg/image_viewer.h>
#include <icg/loader_camera.h>
#include <icg/manual_detector.h>
#include <icg/modality.h>
#include <icg/normal_viewer.h>
#include <icg/optimizer.h>
#include <icg/publisher.h>
#include <icg/refiner.h>
#include <icg/region_modality.h>
#include <icg/region_model.h>
#include <icg/renderer_geometry.h>
#include <icg/static_detector.h>
#include <icg/tracker.h>
#include <icg/viewer.h>

#include <string>
#include <vector>

namespace icg {

// Generator
bool GenerateConfiguredTracker(const std::filesystem::path& configfile_path,
                               std::shared_ptr<Tracker>* tracker_ptr);

}  // namespace icg

#endif  // ICG_INCLUDE_ICG_GENERATOR_H_
