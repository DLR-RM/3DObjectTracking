// SPDX-License-Identifier: MIT
// Copyright (c) 2022 Manuel Stoiber, German Aerospace Center (DLR)

#include <icg/generator.h>

namespace icg {

std::string Name(const cv::FileNode& file_node) {
  return file_node["name"].string();
}

std::filesystem::path MetafilePath(
    const cv::FileNode& file_node,
    const std::filesystem::path& configfile_path) {
  auto path{std::filesystem::path{file_node["metafile_path"].string()}};
  if (path.is_relative())
    return configfile_path.parent_path() / path;
  else
    return path;
}

bool MetafilePathEmpty(const cv::FileNode& file_node) {
  return file_node["metafile_path"].empty();
}

template <typename T>
bool GetObject(const cv::FileNode& file_node, const std::string& parameter_name,
               const std::string& class_name,
               const std::vector<std::shared_ptr<T>>& object_ptrs,
               std::shared_ptr<T>* object_ptr) {
  auto object_name_file_node{file_node[parameter_name]};
  std::string object_name = object_name_file_node.string();
  if (!GetPtrIfNameExists(object_name, object_ptrs, object_ptr)) {
    std::cerr << "Object " << object_name << " in " << parameter_name << ", "
              << class_name << " does not exist.";
    return false;
  }
  return true;
}

template <typename T, typename F>
bool AddObject(const cv::FileNode& file_node, const std::string& parameter_name,
               const std::string& class_name,
               const std::vector<std::shared_ptr<T>>& object_ptrs,
               const F& AddFunction) {
  auto object_name_file_node{file_node[parameter_name]};
  std::string object_name = object_name_file_node.string();
  std::shared_ptr<T> object_ptr;
  if (!GetPtrIfNameExists(object_name, object_ptrs, &object_ptr)) {
    std::cerr << "Object " << object_name << " in " << parameter_name << ", "
              << class_name << " does not exist.";
    return false;
  }
  return AddFunction(object_ptr);
}

template <typename T, typename F>
bool AddObjects(const cv::FileNode& file_node,
                const std::string& parameter_name,
                const std::string& class_name,
                const std::vector<std::shared_ptr<T>>& object_ptrs,
                const F& AddFunction) {
  auto object_name_file_nodes{file_node[parameter_name]};
  for (const auto& object_name_file_node : object_name_file_nodes) {
    std::string object_name = object_name_file_node.string();
    std::shared_ptr<T> object_ptr;
    if (!GetPtrIfNameExists(object_name, object_ptrs, &object_ptr)) {
      std::cerr << "Object " << object_name << " in " << parameter_name << ", "
                << class_name << " does not exist.";
      return false;
    }
    if (!AddFunction(object_ptr)) return false;
  }
  return true;
}

template <typename T1, typename T2, typename F>
bool ConfigureObjects(const cv::FileStorage& file_storage,
                      const std::string& class_name,
                      const std::vector<std::string>& parameter_names,
                      const F& GeneratorFunction,
                      std::vector<std::shared_ptr<T2>>* object_ptrs) {
  if (file_storage[class_name].empty()) return true;
  cv::FileNode file_nodes = file_storage[class_name];
  for (const cv::FileNode& file_node : file_nodes) {
    for (const auto& parameter_name : parameter_names) {
      if (file_node[parameter_name].empty()) {
        std::cerr << "Required parameter \"" << parameter_name
                  << "\" was not configured for " << class_name;
        return false;
      }
    }
    std::shared_ptr<T1> object_ptr;
    if (!GeneratorFunction(file_node, &object_ptr)) return false;
    object_ptrs->push_back(object_ptr);
  }
  return true;
}

template <typename T1, typename T2>
bool ConfigureObjectsMetafileRequired(
    const std::filesystem::path& configfile_path,
    const cv::FileStorage& file_storage, const std::string& class_name,
    std::vector<std::shared_ptr<T2>>* object_ptrs) {
  return ConfigureObjects<T1>(
      file_storage, class_name, {"name", "metafile_path"},
      [&](const auto& file_node, auto* object_ptr) {
        *object_ptr = std::make_shared<T1>(
            Name(file_node), MetafilePath(file_node, configfile_path));
        return true;
      },
      object_ptrs);
}

template <typename T1, typename T2>
bool ConfigureObjectsMetafileOptional(
    const std::filesystem::path& configfile_path,
    const cv::FileStorage& file_storage, const std::string& class_name,
    std::vector<std::shared_ptr<T2>>* object_ptrs) {
  return ConfigureObjects<T1>(
      file_storage, class_name, {"name"},
      [&](const auto& file_node, auto* object_ptr) {
        if (MetafilePathEmpty(file_node))
          *object_ptr = std::make_shared<T1>(Name(file_node));
        else
          *object_ptr = std::make_shared<T1>(
              Name(file_node), MetafilePath(file_node, configfile_path));
        return true;
      },
      object_ptrs);
}

template <typename T1, typename T2>
bool ConfigureObjectsMetafileAndBodyRequired(
    const std::filesystem::path& configfile_path,
    const cv::FileStorage& file_storage, const std::string& class_name,
    const std::vector<std::shared_ptr<Body>>& body_ptrs,
    std::vector<std::shared_ptr<T2>>* object_ptrs) {
  return ConfigureObjects<T1>(
      file_storage, class_name, {"name", "metafile_path", "body"},
      [&](const auto& file_node, auto* object_ptr) {
        std::shared_ptr<Body> body_ptr;
        if (!GetObject(file_node, "body", class_name, body_ptrs, &body_ptr))
          return false;
        *object_ptr = std::make_shared<T1>(
            Name(file_node), MetafilePath(file_node, configfile_path),
            body_ptr);
        return true;
      },
      object_ptrs);
}

bool ConfigureRendererGeometries(
    const cv::FileStorage& file_storage,
    const std::vector<std::shared_ptr<Body>>& body_ptrs,
    std::vector<std::shared_ptr<RendererGeometry>>* renderer_geometry_ptrs) {
  return ConfigureObjects<RendererGeometry>(
      file_storage, "RendererGeometry", {"name", "bodies"},
      [&](const auto& file_node, auto* renderer_geometry_ptr) {
        *renderer_geometry_ptr =
            std::make_shared<RendererGeometry>(Name(file_node));
        return AddObjects(file_node, "bodies", "RendererGeometry", body_ptrs,
                          [&](const auto& body_ptr) {
                            return (*renderer_geometry_ptr)->AddBody(body_ptr);
                          });
      },
      renderer_geometry_ptrs);
}

template <typename T1, typename T2>
bool ConfigureFocusedRenderers(
    const std::filesystem::path& configfile_path,
    const cv::FileStorage& file_storage, const std::string& class_name,
    const std::vector<std::shared_ptr<RendererGeometry>>&
        renderer_geometry_ptrs,
    const std::vector<std::shared_ptr<Camera>>& camera_ptrs,
    const std::vector<std::shared_ptr<Body>>& body_ptrs,
    std::vector<std::shared_ptr<T2>>* renderer_ptrs) {
  return ConfigureObjects<T1>(
      file_storage, class_name,
      {"name", "renderer_geometry", "camera", "referenced_bodies"},
      [&](const auto& file_node, auto* renderer_ptr) {
        // Get objects required for renderer constructor
        std::shared_ptr<RendererGeometry> renderer_geometry_ptr;
        std::shared_ptr<Camera> camera_ptr;
        if (!GetObject(file_node, "renderer_geometry", class_name,
                       renderer_geometry_ptrs, &renderer_geometry_ptr) ||
            !GetObject(file_node, "camera", class_name, camera_ptrs,
                       &camera_ptr))
          return false;

        // Construct renderer
        if (MetafilePathEmpty(file_node))
          *renderer_ptr = std::make_shared<T1>(
              Name(file_node), renderer_geometry_ptr, camera_ptr);
        else
          *renderer_ptr = std::make_shared<T1>(
              Name(file_node), MetafilePath(file_node, configfile_path),
              renderer_geometry_ptr, camera_ptr);

        // Add additional objects
        return AddObjects(file_node, "referenced_bodies", class_name, body_ptrs,
                          [&](const auto& body_ptr) {
                            return (*renderer_ptr)->AddReferencedBody(body_ptr);
                          });
      },
      renderer_ptrs);
}

bool ConfigureRegionModels(
    const std::filesystem::path& configfile_path,
    const cv::FileStorage& file_storage,
    const std::vector<std::shared_ptr<Body>>& body_ptrs,
    std::vector<std::shared_ptr<RegionModel>>* region_model_ptrs) {
  std::string class_name{"RegionModel"};
  return ConfigureObjects<RegionModel>(
      file_storage, class_name, {"name", "metafile_path", "body"},
      [&](const auto& file_node, auto* region_model_ptr) {
        // Get objects required for region model constructor
        std::shared_ptr<Body> body_ptr;
        if (!GetObject(file_node, "body", class_name, body_ptrs, &body_ptr))
          return false;

        // Construct region model
        *region_model_ptr = std::make_shared<RegionModel>(
            Name(file_node), MetafilePath(file_node, configfile_path),
            body_ptr);
        return true;
      },
      region_model_ptrs);
}

bool ConfigureRegionModalities(
    const std::filesystem::path& configfile_path,
    const cv::FileStorage& file_storage,
    const std::vector<std::shared_ptr<Body>>& body_ptrs,
    const std::vector<std::shared_ptr<ColorCamera>>& color_camera_ptrs,
    const std::vector<std::shared_ptr<RegionModel>>& region_model_ptrs,
    const std::vector<std::shared_ptr<DepthCamera>>& depth_camera_ptrs,
    const std::vector<std::shared_ptr<FocusedDepthRenderer>>&
        focused_depth_renderer_ptrs,
    std::vector<std::shared_ptr<Modality>>* modality_ptrs) {
  std::string class_name{"RegionModality"};
  return ConfigureObjects<RegionModality>(
      file_storage, class_name,
      {"name", "body", "color_camera", "region_model"},
      [&](const auto& file_node, auto* region_modality_ptr) {
        // Get objects required for region modality constructor
        std::shared_ptr<Body> body_ptr;
        std::shared_ptr<ColorCamera> color_camera_ptr;
        std::shared_ptr<RegionModel> region_model_ptr;
        if (!GetObject(file_node, "body", class_name, body_ptrs, &body_ptr) ||
            !GetObject(file_node, "color_camera", class_name, color_camera_ptrs,
                       &color_camera_ptr) ||
            !GetObject(file_node, "region_model", class_name, region_model_ptrs,
                       &region_model_ptr))
          return false;

        // Construct region modality
        if (MetafilePathEmpty(file_node))
          *region_modality_ptr = std::make_shared<RegionModality>(
              Name(file_node), body_ptr, color_camera_ptr, region_model_ptr);
        else
          *region_modality_ptr = std::make_shared<RegionModality>(
              Name(file_node), MetafilePath(file_node, configfile_path),
              body_ptr, color_camera_ptr, region_model_ptr);

        // Add additional objects
        if (!file_node["measure_occlusions"].empty()) {
          if (!AddObject(
                  file_node["measure_occlusions"], "depth_camera", class_name,
                  depth_camera_ptrs, [&](const auto& depth_camera_ptr) {
                    (*region_modality_ptr)->MeasureOcclusions(depth_camera_ptr);
                    return true;
                  }))
            return false;
        }
        if (!file_node["model_occlusions"].empty()) {
          if (!AddObject(file_node["model_occlusions"],
                         "focused_depth_renderer", class_name,
                         focused_depth_renderer_ptrs,
                         [&](const auto& focused_depth_renderer_ptr) {
                           (*region_modality_ptr)
                               ->ModelOcclusions(focused_depth_renderer_ptr);
                           return true;
                         }))
            return false;
        }
        return true;
      },
      modality_ptrs);
}

bool ConfigureDepthModalities(
    const std::filesystem::path& configfile_path,
    const cv::FileStorage& file_storage,
    const std::vector<std::shared_ptr<Body>>& body_ptrs,
    const std::vector<std::shared_ptr<DepthCamera>>& depth_camera_ptrs,
    const std::vector<std::shared_ptr<DepthModel>>& depth_model_ptrs,
    const std::vector<std::shared_ptr<FocusedDepthRenderer>>&
        focused_depth_renderer_ptrs,
    std::vector<std::shared_ptr<Modality>>* modality_ptrs) {
  std::string class_name{"DepthModality"};
  return ConfigureObjects<DepthModality>(
      file_storage, class_name, {"name", "body", "depth_camera", "depth_model"},
      [&](const auto& file_node, auto* depth_modality_ptr) {
        // Get objects required for depth modality constructor
        std::shared_ptr<Body> body_ptr;
        std::shared_ptr<DepthCamera> depth_camera_ptr;
        std::shared_ptr<DepthModel> depth_model_ptr;
        if (!GetObject(file_node, "body", class_name, body_ptrs, &body_ptr) ||
            !GetObject(file_node, "depth_camera", class_name, depth_camera_ptrs,
                       &depth_camera_ptr) ||
            !GetObject(file_node, "depth_model", class_name, depth_model_ptrs,
                       &depth_model_ptr))
          return false;

        // Construct depth modality
        if (MetafilePathEmpty(file_node))
          *depth_modality_ptr = std::make_shared<DepthModality>(
              Name(file_node), body_ptr, depth_camera_ptr, depth_model_ptr);
        else
          *depth_modality_ptr = std::make_shared<DepthModality>(
              Name(file_node), MetafilePath(file_node, configfile_path),
              body_ptr, depth_camera_ptr, depth_model_ptr);

        // Add additional objects
        if (!file_node["model_occlusions"].empty()) {
          if (!AddObject(file_node["model_occlusions"],
                         "focused_depth_renderer", class_name,
                         focused_depth_renderer_ptrs,
                         [&](const auto& focused_depth_renderer_ptr) {
                           (*depth_modality_ptr)
                               ->ModelOcclusions(focused_depth_renderer_ptr);
                           return true;
                         }))
            return false;
        }
        return true;
      },
      modality_ptrs);
}

bool ConfigureOptimizers(
    const std::filesystem::path& configfile_path,
    const cv::FileStorage& file_storage,
    const std::vector<std::shared_ptr<Modality>>& modality_ptrs,
    std::vector<std::shared_ptr<Optimizer>>* optimizer_ptrs) {
  std::string class_name{"Optimizer"};
  return ConfigureObjects<Optimizer>(
      file_storage, class_name, {"name", "modalities"},
      [&](const auto& file_node, auto* optimizer_ptr) {
        if (MetafilePathEmpty(file_node))
          *optimizer_ptr = std::make_shared<Optimizer>(Name(file_node));
        else
          *optimizer_ptr = std::make_shared<Optimizer>(
              Name(file_node), MetafilePath(file_node, configfile_path));
        return AddObjects(file_node, "modalities", class_name, modality_ptrs,
                          [&](const auto& modality_ptr) {
                            return (*optimizer_ptr)->AddModality(modality_ptr);
                          });
      },
      optimizer_ptrs);
}

template <typename T1, typename T2>
bool ConfigureImageViewers(const std::filesystem::path& configfile_path,
                           const cv::FileStorage& file_storage,
                           const std::string& class_name,
                           const std::string& camera_parameter_name,
                           const std::vector<std::shared_ptr<T2>>& camera_ptrs,
                           std::vector<std::shared_ptr<Viewer>>* viewer_ptrs) {
  return ConfigureObjects<T1>(
      file_storage, class_name, {"name", camera_parameter_name},
      [&](const auto& file_node, auto* viewer_ptr) {
        std::shared_ptr<T2> camera_ptr;
        if (!GetObject(file_node, camera_parameter_name, class_name,
                       camera_ptrs, &camera_ptr))
          return false;
        if (MetafilePathEmpty(file_node))
          *viewer_ptr = std::make_shared<T1>(Name(file_node), camera_ptr);
        else
          *viewer_ptr = std::make_shared<T1>(
              Name(file_node), MetafilePath(file_node, configfile_path),
              camera_ptr);
        return true;
      },
      viewer_ptrs);
}

template <typename T1, typename T2>
bool ConfigureNormalViewers(
    const std::filesystem::path& configfile_path,
    const cv::FileStorage& file_storage, const std::string& class_name,
    const std::string& camera_parameter_name,
    const std::vector<std::shared_ptr<T2>>& camera_ptrs,
    const std::vector<std::shared_ptr<RendererGeometry>>&
        renderer_geometry_ptrs,
    std::vector<std::shared_ptr<Viewer>>* viewer_ptrs) {
  return ConfigureObjects<T1>(
      file_storage, class_name,
      {"name", camera_parameter_name, "renderer_geometry"},
      [&](const auto& file_node, auto* viewer_ptr) {
        std::shared_ptr<T2> camera_ptr;
        std::shared_ptr<RendererGeometry> renderer_geometry_ptr;
        if (!GetObject(file_node, camera_parameter_name, class_name,
                       camera_ptrs, &camera_ptr) ||
            !GetObject(file_node, "renderer_geometry", class_name,
                       renderer_geometry_ptrs, &renderer_geometry_ptr))
          return false;
        if (MetafilePathEmpty(file_node))
          *viewer_ptr = std::make_shared<T1>(Name(file_node), camera_ptr,
                                             renderer_geometry_ptr);
        else
          *viewer_ptr = std::make_shared<T1>(
              Name(file_node), MetafilePath(file_node, configfile_path),
              camera_ptr, renderer_geometry_ptr);
        return true;
      },
      viewer_ptrs);
}

bool ConfigureManualDetectors(
    const std::filesystem::path& configfile_path,
    const cv::FileStorage& file_storage,
    const std::vector<std::shared_ptr<Body>>& body_ptrs,
    const std::vector<std::shared_ptr<ColorCamera>>& color_camera_ptrs,
    std::vector<std::shared_ptr<Detector>>* detector_ptrs) {
  std::string class_name{"ManualDetector"};
  return ConfigureObjects<ManualDetector>(
      file_storage, class_name,
      {"name", "metafile_path", "body", "color_camera"},
      [&](const auto& file_node, auto* detector_ptr) {
        std::shared_ptr<Body> body_ptr;
        std::shared_ptr<ColorCamera> color_camera_ptr;
        if (!GetObject(file_node, "body", class_name, body_ptrs, &body_ptr) ||
            !GetObject(file_node, "color_camera", class_name, color_camera_ptrs,
                       &color_camera_ptr))
          return false;
        *detector_ptr = std::make_shared<ManualDetector>(
            Name(file_node), MetafilePath(file_node, configfile_path), body_ptr,
            color_camera_ptr);
        return true;
      },
      detector_ptrs);
}

bool ConfigureRefiners(
    const std::filesystem::path& configfile_path,
    const cv::FileStorage& file_storage,
    const std::vector<std::shared_ptr<Optimizer>>& optimizer_ptrs,
    std::vector<std::shared_ptr<Refiner>>* refiner_ptrs) {
  std::string class_name{"Refiner"};
  return ConfigureObjects<Refiner>(
      file_storage, class_name, {"name", "optimizers"},
      [&](const auto& file_node, auto* refiner_ptr) {
        if (MetafilePathEmpty(file_node))
          *refiner_ptr = std::make_shared<Refiner>(Name(file_node));
        else
          *refiner_ptr = std::make_shared<Refiner>(
              Name(file_node), MetafilePath(file_node, configfile_path));
        return AddObjects(file_node, "optimizers", class_name, optimizer_ptrs,
                          [&](const auto& optimizer_ptr) {
                            return (*refiner_ptr)->AddOptimizer(optimizer_ptr);
                          });
      },
      refiner_ptrs);
}

bool ConfigureTrackers(
    const std::filesystem::path& configfile_path,
    const cv::FileStorage& file_storage,
    const std::vector<std::shared_ptr<Optimizer>>& optimizer_ptrs,
    const std::vector<std::shared_ptr<Detector>>& detector_ptrs,
    const std::vector<std::shared_ptr<Refiner>>& refiner_ptrs,
    const std::vector<std::shared_ptr<Viewer>>& viewer_ptrs,
    std::vector<std::shared_ptr<Tracker>>* tracker_ptrs) {
  std::string class_name{"Tracker"};
  return ConfigureObjects<Tracker>(
      file_storage, class_name, {"name", "optimizers"},
      [&](const auto& file_node, auto* tracker_ptr) {
        if (MetafilePathEmpty(file_node))
          *tracker_ptr = std::make_shared<Tracker>(Name(file_node));
        else
          *tracker_ptr = std::make_shared<Tracker>(
              Name(file_node), MetafilePath(file_node, configfile_path));
        if (!AddObjects(file_node, "optimizers", class_name, optimizer_ptrs,
                        [&](const auto& optimizer_ptr) {
                          return (*tracker_ptr)->AddOptimizer(optimizer_ptr);
                        }))
          return false;
        if (!file_node["detectors"].empty()) {
          if (!AddObjects(file_node, "detectors", class_name, detector_ptrs,
                          [&](const auto& detector_ptr) {
                            return (*tracker_ptr)->AddDetector(detector_ptr);
                          }))
            return false;
        }
        if (!file_node["refiners"].empty()) {
          if (!AddObjects(file_node, "refiners", class_name, refiner_ptrs,
                          [&](const auto& refiner_ptr) {
                            return (*tracker_ptr)->AddRefiner(refiner_ptr);
                          }))
            return false;
        }
        if (!file_node["viewers"].empty()) {
          if (!AddObjects(file_node, "viewers", class_name, viewer_ptrs,
                          [&](const auto& viewer_ptr) {
                            return (*tracker_ptr)->AddViewer(viewer_ptr);
                          }))
            return false;
        }
        return true;
      },
      tracker_ptrs);
}

bool GenerateConfiguredTracker(const std::filesystem::path& configfile_path,
                               std::shared_ptr<Tracker>* tracker_ptr) {
  // Open file storage from yaml
  cv::FileStorage fs;
  if (!OpenYamlFileStorage(configfile_path, &fs)) return false;

  // Configure bodies
  std::vector<std::shared_ptr<Body>> body_ptrs;
  if (!ConfigureObjectsMetafileRequired<Body>(configfile_path, fs, "Body",
                                              &body_ptrs))
    return false;

  // Configure renderer_geometries
  std::vector<std::shared_ptr<RendererGeometry>> renderer_geometry_ptrs;
  if (!ConfigureRendererGeometries(fs, body_ptrs, &renderer_geometry_ptrs))
    return false;

  // Configure color cameras
  std::vector<std::shared_ptr<ColorCamera>> color_camera_ptrs;
  if (!ConfigureObjectsMetafileRequired<LoaderColorCamera>(
          configfile_path, fs, "LoaderColorCamera", &color_camera_ptrs))
    return false;
#ifdef USE_REALSENSE
  if (!ConfigureObjectsMetafileOptional<RealSenseColorCamera>(
          configfile_path, fs, "RealSenseColorCamera", &color_camera_ptrs))
    return false;
#endif
#ifdef USE_AZURE_KINECT
  if (!ConfigureObjectsMetafileOptional<AzureKinectColorCamera>(
          configfile_path, fs, "AzureKinectColorCamera", &color_camera_ptrs))
    return false;
#endif

  // Configure depth cameras
  std::vector<std::shared_ptr<DepthCamera>> depth_camera_ptrs;
  if (!ConfigureObjectsMetafileRequired<LoaderDepthCamera>(
          configfile_path, fs, "LoaderDepthCamera", &depth_camera_ptrs))
    return false;
#ifdef USE_REALSENSE
  if (!ConfigureObjectsMetafileOptional<RealSenseDepthCamera>(
          configfile_path, fs, "RealSenseDepthCamera", &depth_camera_ptrs))
    return false;
#endif
#ifdef USE_AZURE_KINECT
  if (!ConfigureObjectsMetafileOptional<AzureKinectDepthCamera>(
          configfile_path, fs, "AzureKinectDepthCamera", &depth_camera_ptrs))
    return false;
#endif

  // Create combined camera vector
  std::vector<std::shared_ptr<Camera>> camera_ptrs;
  std::copy(begin(color_camera_ptrs), end(color_camera_ptrs),
            std::back_inserter(camera_ptrs));
  std::copy(begin(depth_camera_ptrs), end(depth_camera_ptrs),
            std::back_inserter(camera_ptrs));

  // Configure basic depth renderers
  std::vector<std::shared_ptr<FocusedDepthRenderer>>
      focused_depth_renderer_ptrs;
  if (!ConfigureFocusedRenderers<FocusedBasicDepthRenderer>(
          configfile_path, fs, "FocusedBasicDepthRenderer",
          renderer_geometry_ptrs, camera_ptrs, body_ptrs,
          &focused_depth_renderer_ptrs))
    return false;

  // Configure region models
  std::vector<std::shared_ptr<RegionModel>> region_model_ptrs;
  if (!ConfigureRegionModels(configfile_path, fs, body_ptrs,
                             &region_model_ptrs))
    return false;

  // Configure depth models
  std::vector<std::shared_ptr<DepthModel>> depth_model_ptrs;
  if (!ConfigureObjectsMetafileAndBodyRequired<DepthModel>(
          configfile_path, fs, "DepthModel", body_ptrs, &depth_model_ptrs))
    return false;

  // Configure modalities
  std::vector<std::shared_ptr<Modality>> modality_ptrs;
  if (!ConfigureRegionModalities(
          configfile_path, fs, body_ptrs, color_camera_ptrs, region_model_ptrs,
          depth_camera_ptrs, focused_depth_renderer_ptrs, &modality_ptrs) ||
      !ConfigureDepthModalities(configfile_path, fs, body_ptrs,
                                depth_camera_ptrs, depth_model_ptrs,
                                focused_depth_renderer_ptrs, &modality_ptrs))
    return false;

  // Configure optimizers
  std::vector<std::shared_ptr<Optimizer>> optimizer_ptrs;
  if (!ConfigureOptimizers(configfile_path, fs, modality_ptrs, &optimizer_ptrs))
    return false;

  // Configure viewers
  std::vector<std::shared_ptr<Viewer>> viewer_ptrs;
  if (!ConfigureImageViewers<ImageColorViewer>(
          configfile_path, fs, "ImageColorViewer", "color_camera",
          color_camera_ptrs, &viewer_ptrs) ||
      !ConfigureImageViewers<ImageDepthViewer>(
          configfile_path, fs, "ImageDepthViewer", "depth_camera",
          depth_camera_ptrs, &viewer_ptrs) ||
      !ConfigureNormalViewers<NormalColorViewer>(
          configfile_path, fs, "NormalColorViewer", "color_camera",
          color_camera_ptrs, renderer_geometry_ptrs, &viewer_ptrs) ||
      !ConfigureNormalViewers<NormalDepthViewer>(
          configfile_path, fs, "NormalDepthViewer", "depth_camera",
          depth_camera_ptrs, renderer_geometry_ptrs, &viewer_ptrs))
    return false;

  // Configure detectors
  std::vector<std::shared_ptr<Detector>> detector_ptrs;
  if (!ConfigureObjectsMetafileAndBodyRequired<StaticDetector>(
          configfile_path, fs, "StaticDetector", body_ptrs, &detector_ptrs) ||
      !ConfigureManualDetectors(configfile_path, fs, body_ptrs,
                                color_camera_ptrs, &detector_ptrs))
    return false;

  // Configure refiners
  std::vector<std::shared_ptr<Refiner>> refiner_ptrs;
  if (!ConfigureRefiners(configfile_path, fs, optimizer_ptrs, &refiner_ptrs))
    return false;

  // Configure trackers
  std::vector<std::shared_ptr<Tracker>> tracker_ptrs;
  if (!ConfigureTrackers(configfile_path, fs, optimizer_ptrs, detector_ptrs,
                         refiner_ptrs, viewer_ptrs, &tracker_ptrs))
    return false;

  // Check if exactly one tracker was configured
  if (tracker_ptrs.size() < 1) {
    std::cerr << "No tracker was configured in " << configfile_path
              << std::endl;
    return false;
  } else if (tracker_ptrs.size() > 1) {
    std::cerr << "More than one tracker was configured in " << configfile_path
              << std::endl;
    return false;
  }

  // Return configured tracker
  (*tracker_ptr) = tracker_ptrs[0];
  return true;
}

}  // namespace icg
