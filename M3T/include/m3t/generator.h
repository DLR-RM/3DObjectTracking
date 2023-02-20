// SPDX-License-Identifier: MIT
// Copyright (c) 2023 Manuel Stoiber, German Aerospace Center (DLR)

#ifndef M3T_INCLUDE_M3T_GENERATOR_H_
#define M3T_INCLUDE_M3T_GENERATOR_H_

#ifdef USE_REALSENSE
#include <m3t/realsense_camera.h>
#endif
#ifdef USE_AZURE_KINECT
#include <m3t/azure_kinect_camera.h>
#endif
#include <m3t/basic_depth_renderer.h>
#include <m3t/body.h>
#include <m3t/camera.h>
#include <m3t/color_histograms.h>
#include <m3t/common.h>
#include <m3t/constraint.h>
#include <m3t/depth_modality.h>
#include <m3t/depth_model.h>
#include <m3t/detector.h>
#include <m3t/image_viewer.h>
#include <m3t/link.h>
#include <m3t/loader_camera.h>
#include <m3t/manual_detector.h>
#include <m3t/modality.h>
#include <m3t/normal_viewer.h>
#include <m3t/optimizer.h>
#include <m3t/publisher.h>
#include <m3t/refiner.h>
#include <m3t/region_modality.h>
#include <m3t/region_model.h>
#include <m3t/renderer_geometry.h>
#include <m3t/soft_constraint.h>
#include <m3t/static_detector.h>
#include <m3t/subscriber.h>
#include <m3t/texture_modality.h>
#include <m3t/tracker.h>
#include <m3t/viewer.h>

#include <string>
#include <vector>

namespace m3t {

inline std::string Name(const cv::FileNode& file_node) {
  return file_node["name"].string();
}

inline std::filesystem::path MetafilePath(
    const cv::FileNode& file_node,
    const std::filesystem::path& configfile_path) {
  auto path{std::filesystem::path{file_node["metafile_path"].string()}};
  if (path.is_relative())
    return configfile_path.parent_path() / path;
  else
    return path;
}

inline bool MetafilePathEmpty(const cv::FileNode& file_node) {
  return file_node["metafile_path"].empty();
}

template <typename T>
inline bool GetObject(const cv::FileNode& file_node,
                      const std::string& parameter_name,
                      const std::string& class_name,
                      const std::vector<std::shared_ptr<T>>& object_ptrs,
                      std::shared_ptr<T>* object_ptr) {
  const auto& object_name_file_node{file_node[parameter_name]};
  std::string object_name = object_name_file_node.string();
  if (!GetPtrIfNameExists(object_name, object_ptrs, object_ptr)) {
    std::cerr << "Object " << object_name << " in " << parameter_name << ", "
              << class_name << " does not exist.";
    return false;
  }
  return true;
}

template <typename T>
inline bool GetObjects(const cv::FileNode& file_node,
                       const std::string& parameter_name,
                       const std::string& class_name,
                       const std::vector<std::shared_ptr<T>>& all_object_ptrs,
                       std::vector<std::shared_ptr<T>>* object_ptrs) {
  const auto& object_name_file_nodes{file_node[parameter_name]};
  for (const auto& object_name_file_node : object_name_file_nodes) {
    std::string object_name = object_name_file_node.string();
    std::shared_ptr<T> object_ptr;
    if (!GetPtrIfNameExists(object_name, all_object_ptrs, &object_ptr)) {
      std::cerr << "Object " << object_name << " in " << parameter_name << ", "
                << class_name << " does not exist.";
      return false;
    }
    object_ptrs->push_back(object_ptr);
  }
  return true;
}

template <typename T, typename F>
inline bool AddObject(const cv::FileNode& file_node,
                      const std::string& parameter_name,
                      const std::string& class_name,
                      const std::vector<std::shared_ptr<T>>& object_ptrs,
                      const F& AddFunction) {
  const auto& object_name_file_node{file_node[parameter_name]};
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
inline bool AddObjects(const cv::FileNode& file_node,
                       const std::string& parameter_name,
                       const std::string& class_name,
                       const std::vector<std::shared_ptr<T>>& object_ptrs,
                       const F& AddFunction) {
  const auto& object_name_file_nodes{file_node[parameter_name]};
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
inline bool ConfigureObjects(const cv::FileStorage& file_storage,
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
inline bool ConfigureObjectsMetafileRequired(
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
inline bool ConfigureObjectsMetafileOptional(
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
inline bool ConfigureObjectsMetafileAndBodyRequired(
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

inline bool ConfigureRendererGeometries(
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
inline bool ConfigureFocusedRenderers(
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

inline bool ConfigureRegionModels(
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

        // Add additional objects
        if (!file_node["fixed_bodies"].empty()) {
          if (!AddObjects(file_node, "fixed_bodies", class_name, body_ptrs,
                          [&](const auto& fixed_body_ptr) {
                            (*region_model_ptr)
                                ->AddAssociatedBody(fixed_body_ptr, false,
                                                    false);
                            return true;
                          }))
            return false;
        }
        if (!file_node["movable_bodies"].empty()) {
          if (!AddObjects(file_node, "movable_bodies", class_name, body_ptrs,
                          [&](const auto& movable_body_ptr) {
                            (*region_model_ptr)
                                ->AddAssociatedBody(movable_body_ptr, true,
                                                    false);
                            return true;
                          }))
            return false;
        }
        if (!file_node["fixed_same_region_bodies"].empty()) {
          if (!AddObjects(file_node, "fixed_same_region_bodies", class_name,
                          body_ptrs,
                          [&](const auto& fixed_same_region_body_ptr) {
                            (*region_model_ptr)
                                ->AddAssociatedBody(fixed_same_region_body_ptr,
                                                    false, true);
                            return true;
                          }))
            return false;
        }
        if (!file_node["movable_same_region_bodies"].empty()) {
          if (!AddObjects(file_node, "movable_same_region_bodies", class_name,
                          body_ptrs,
                          [&](const auto& movable_same_region_body_ptr) {
                            (*region_model_ptr)
                                ->AddAssociatedBody(
                                    movable_same_region_body_ptr, true, true);
                            return true;
                          }))
            return false;
        }
        return true;
      },
      region_model_ptrs);
}

inline bool ConfigureDepthModels(
    const std::filesystem::path& configfile_path,
    const cv::FileStorage& file_storage,
    const std::vector<std::shared_ptr<Body>>& body_ptrs,
    std::vector<std::shared_ptr<DepthModel>>* depth_model_ptrs) {
  std::string class_name{"DepthModel"};
  return ConfigureObjects<DepthModel>(
      file_storage, class_name, {"name", "metafile_path", "body"},
      [&](const auto& file_node, auto* depth_model_ptr) {
        // Get objects required for depth model constructor
        std::shared_ptr<Body> body_ptr;
        if (!GetObject(file_node, "body", class_name, body_ptrs, &body_ptr))
          return false;

        // Construct depth model
        *depth_model_ptr = std::make_shared<DepthModel>(
            Name(file_node), MetafilePath(file_node, configfile_path),
            body_ptr);

        // Add additional objects
        if (!file_node["occlusion_bodies"].empty()) {
          if (!AddObjects(
                  file_node, "occlusion_bodies", class_name, body_ptrs,
                  [&](const auto& occlusion_body_ptr) {
                    (*depth_model_ptr)->AddOcclusionBody(occlusion_body_ptr);
                    return true;
                  }))
            return false;
        }
        return true;
      },
      depth_model_ptrs);
}

inline bool ConfigureRegionModalities(
    const std::filesystem::path& configfile_path,
    const cv::FileStorage& file_storage,
    const std::vector<std::shared_ptr<Body>>& body_ptrs,
    const std::vector<std::shared_ptr<ColorCamera>>& color_camera_ptrs,
    const std::vector<std::shared_ptr<RegionModel>>& region_model_ptrs,
    const std::vector<std::shared_ptr<DepthCamera>>& depth_camera_ptrs,
    const std::vector<std::shared_ptr<FocusedDepthRenderer>>&
        focused_depth_renderer_ptrs,
    const std::vector<std::shared_ptr<FocusedSilhouetteRenderer>>&
        focused_silhouette_renderer_ptrs,
    const std::vector<std::shared_ptr<ColorHistograms>>& color_histograms_ptrs,
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
        if (!file_node["use_region_checking"].empty()) {
          if (!AddObject(file_node["use_region_checking"],
                         "focused_silhouette_renderer", class_name,
                         focused_silhouette_renderer_ptrs,
                         [&](const auto& focused_silhouette_renderer_ptr) {
                           (*region_modality_ptr)
                               ->UseRegionChecking(
                                   focused_silhouette_renderer_ptr);
                           return true;
                         }))
            return false;
        }
        if (!file_node["use_shared_color_histograms"].empty()) {
          if (!AddObject(file_node["use_shared_color_histograms"],
                         "color_histograms", class_name, color_histograms_ptrs,
                         [&](const auto& color_histograms_ptr) {
                           (*region_modality_ptr)
                               ->UseSharedColorHistograms(color_histograms_ptr);
                           return true;
                         }))
            return false;
        }
        return true;
      },
      modality_ptrs);
}

inline bool ConfigureDepthModalities(
    const std::filesystem::path& configfile_path,
    const cv::FileStorage& file_storage,
    const std::vector<std::shared_ptr<Body>>& body_ptrs,
    const std::vector<std::shared_ptr<DepthCamera>>& depth_camera_ptrs,
    const std::vector<std::shared_ptr<DepthModel>>& depth_model_ptrs,
    const std::vector<std::shared_ptr<FocusedDepthRenderer>>&
        focused_depth_renderer_ptrs,
    const std::vector<std::shared_ptr<FocusedSilhouetteRenderer>>&
        focused_silhouette_renderer_ptrs,
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
        if (!file_node["use_silhouette_checking"].empty()) {
          if (!AddObject(file_node["use_silhouette_checking"],
                         "focused_silhouette_renderer", class_name,
                         focused_silhouette_renderer_ptrs,
                         [&](const auto& focused_silhouette_renderer_ptr) {
                           (*depth_modality_ptr)
                               ->UseSilhouetteChecking(
                                   focused_silhouette_renderer_ptr);
                           return true;
                         }))
            return false;
        }
        return true;
      },
      modality_ptrs);
}

inline bool ConfigureTextureModalities(
    const std::filesystem::path& configfile_path,
    const cv::FileStorage& file_storage,
    const std::vector<std::shared_ptr<Body>>& body_ptrs,
    const std::vector<std::shared_ptr<ColorCamera>>& color_camera_ptrs,
    const std::vector<std::shared_ptr<FocusedSilhouetteRenderer>>&
        focused_silhouette_renderer_ptrs,
    const std::vector<std::shared_ptr<DepthCamera>>& depth_camera_ptrs,
    const std::vector<std::shared_ptr<FocusedDepthRenderer>>&
        focused_depth_renderer_ptrs,
    std::vector<std::shared_ptr<Modality>>* modality_ptrs) {
  std::string class_name{"TextureModality"};
  return ConfigureObjects<TextureModality>(
      file_storage, class_name,
      {"name", "body", "color_camera", "focused_silhouette_renderer"},
      [&](const auto& file_node, auto* texture_modality_ptr) {
        // Get objects required for texture modality constructor
        std::shared_ptr<Body> body_ptr;
        std::shared_ptr<ColorCamera> color_camera_ptr;
        std::shared_ptr<FocusedSilhouetteRenderer>
            focused_silhouette_renderer_ptr;
        if (!GetObject(file_node, "body", class_name, body_ptrs, &body_ptr) ||
            !GetObject(file_node, "color_camera", class_name, color_camera_ptrs,
                       &color_camera_ptr) ||
            !GetObject(file_node, "focused_silhouette_renderer", class_name,
                       focused_silhouette_renderer_ptrs,
                       &focused_silhouette_renderer_ptr))
          return false;

        // Construct texture modality
        if (MetafilePathEmpty(file_node))
          *texture_modality_ptr = std::make_shared<TextureModality>(
              Name(file_node), body_ptr, color_camera_ptr,
              focused_silhouette_renderer_ptr);
        else
          *texture_modality_ptr = std::make_shared<TextureModality>(
              Name(file_node), MetafilePath(file_node, configfile_path),
              body_ptr, color_camera_ptr, focused_silhouette_renderer_ptr);

        // Add additional objects
        if (!file_node["measure_occlusions"].empty()) {
          if (!AddObject(file_node["measure_occlusions"], "depth_camera",
                         class_name, depth_camera_ptrs,
                         [&](const auto& depth_camera_ptr) {
                           (*texture_modality_ptr)
                               ->MeasureOcclusions(depth_camera_ptr);
                           return true;
                         }))
            return false;
        }
        if (!file_node["model_occlusions"].empty()) {
          if (!AddObject(file_node["model_occlusions"],
                         "focused_depth_renderer", class_name,
                         focused_depth_renderer_ptrs,
                         [&](const auto& focused_depth_renderer_ptr) {
                           (*texture_modality_ptr)
                               ->ModelOcclusions(focused_depth_renderer_ptr);
                           return true;
                         }))
            return false;
        }
        return true;
      },
      modality_ptrs);
}

inline bool ConfigureLinks(
    const std::filesystem::path& configfile_path,
    const cv::FileStorage& file_storage,
    const std::vector<std::shared_ptr<Body>>& body_ptrs,
    const std::vector<std::shared_ptr<Modality>>& modality_ptrs,
    std::vector<std::shared_ptr<Link>>* link_ptrs) {
  std::string class_name{"Link"};
  if (!ConfigureObjects<Link>(
          file_storage, class_name, {"name"},
          [&](const auto& file_node, auto* link_ptr) {
            // Construct link
            if (MetafilePathEmpty(file_node))
              *link_ptr = std::make_shared<Link>(Name(file_node));
            else
              *link_ptr = std::make_shared<Link>(
                  Name(file_node), MetafilePath(file_node, configfile_path));

            // Add additional objects
            if (!file_node["body"].empty()) {
              if (!AddObject(file_node, "body", class_name, body_ptrs,
                             [&](const auto& body_ptr) {
                               (*link_ptr)->set_body_ptr(body_ptr);
                               return true;
                             }))
                return false;
            }
            if (!file_node["modalities"].empty()) {
              if (!AddObjects(file_node, "modalities", class_name,
                              modality_ptrs, [&](const auto& modality_ptr) {
                                return (*link_ptr)->AddModality(modality_ptr);
                              }))
                return false;
            }
            return true;
          },
          link_ptrs))
    return false;

  // Configure child links
  if (file_storage[class_name].empty()) return true;
  const cv::FileNode file_nodes = file_storage[class_name];
  for (const cv::FileNode& file_node : file_nodes) {
    std::shared_ptr<Link> link_ptr;
    if (!GetObject(file_node, "name", class_name, *link_ptrs, &link_ptr))
      return false;
    if (!file_node["child_links"].empty()) {
      if (!AddObjects(file_node, "child_links", class_name, *link_ptrs,
                      [&](const auto& child_link_ptr) {
                        return link_ptr->AddChildLink(child_link_ptr);
                      }))
        return false;
    }
  }
  return true;
}

inline bool ConfigureConstraints(
    const std::filesystem::path& configfile_path,
    const cv::FileStorage& file_storage,
    std::vector<std::shared_ptr<Link>>& link_ptrs,
    std::vector<std::shared_ptr<Constraint>>* constraint_ptrs) {
  std::string class_name{"Constraint"};
  return ConfigureObjects<Constraint>(
      file_storage, class_name, {"name", "link1", "link2"},
      [&](const auto& file_node, auto* constraint_ptr) {
        // Get objects required for constraint constructor
        std::shared_ptr<Link> link1_ptr;
        std::shared_ptr<Link> link2_ptr;
        if (!GetObject(file_node, "link1", class_name, link_ptrs, &link1_ptr) ||
            !GetObject(file_node, "link2", class_name, link_ptrs, &link2_ptr))
          return false;

        // Construct constraint
        if (MetafilePathEmpty(file_node))
          *constraint_ptr = std::make_shared<Constraint>(Name(file_node),
                                                         link1_ptr, link2_ptr);
        else
          *constraint_ptr = std::make_shared<Constraint>(
              Name(file_node), MetafilePath(file_node, configfile_path),
              link1_ptr, link2_ptr);
        return true;
      },
      constraint_ptrs);
}

inline bool ConfigureSoftConstraints(
    const std::filesystem::path& configfile_path,
    const cv::FileStorage& file_storage,
    std::vector<std::shared_ptr<Link>>& link_ptrs,
    std::vector<std::shared_ptr<SoftConstraint>>* soft_constraint_ptrs) {
  std::string class_name{"SoftConstraint"};
  return ConfigureObjects<SoftConstraint>(
      file_storage, class_name, {"name", "link1", "link2"},
      [&](const auto& file_node, auto* soft_constraint_ptr) {
        // Get objects required for soft constraint constructor
        std::shared_ptr<Link> link1_ptr;
        std::shared_ptr<Link> link2_ptr;
        if (!GetObject(file_node, "link1", class_name, link_ptrs, &link1_ptr) ||
            !GetObject(file_node, "link2", class_name, link_ptrs, &link2_ptr))
          return false;

        // Construct soft constraint
        if (MetafilePathEmpty(file_node))
          *soft_constraint_ptr = std::make_shared<SoftConstraint>(
              Name(file_node), link1_ptr, link2_ptr);
        else
          *soft_constraint_ptr = std::make_shared<SoftConstraint>(
              Name(file_node), MetafilePath(file_node, configfile_path),
              link1_ptr, link2_ptr);
        return true;
      },
      soft_constraint_ptrs);
}

inline bool ConfigureOptimizers(
    const std::filesystem::path& configfile_path,
    const cv::FileStorage& file_storage,
    const std::vector<std::shared_ptr<Link>>& link_ptrs,
    const std::vector<std::shared_ptr<Constraint>>& constraint_ptrs,
    const std::vector<std::shared_ptr<SoftConstraint>>& soft_constraint_ptrs,
    std::vector<std::shared_ptr<Optimizer>>* optimizer_ptrs) {
  std::string class_name{"Optimizer"};
  return ConfigureObjects<Optimizer>(
      file_storage, class_name, {"name", "root_link"},
      [&](const auto& file_node, auto* optimizer_ptr) {
        // Get objects required for optimizer constructor
        std::shared_ptr<Link> link_ptr;
        if (!GetObject(file_node, "root_link", class_name, link_ptrs,
                       &link_ptr))
          return false;

        // Construct optimizer
        if (MetafilePathEmpty(file_node))
          *optimizer_ptr =
              std::make_shared<Optimizer>(Name(file_node), link_ptr);
        else
          *optimizer_ptr = std::make_shared<Optimizer>(
              Name(file_node), MetafilePath(file_node, configfile_path),
              link_ptr);

        // Add additional objects
        if (!file_node["constraints"].empty()) {
          if (!AddObjects(file_node, "constraints", class_name, constraint_ptrs,
                          [&](const auto& constraint_ptr) {
                            (*optimizer_ptr)->AddConstraint(constraint_ptr);
                            return true;
                          }))
            return false;
        }
        if (!file_node["soft_constraints"].empty()) {
          if (!AddObjects(
                  file_node, "soft_constraints", class_name,
                  soft_constraint_ptrs, [&](const auto& soft_constraint_ptr) {
                    (*optimizer_ptr)->AddSoftConstraint(soft_constraint_ptr);
                    return true;
                  }))
            return false;
        }

        return true;
      },
      optimizer_ptrs);
}

template <typename T1, typename T2>
inline bool ConfigureImageViewers(
    const std::filesystem::path& configfile_path,
    const cv::FileStorage& file_storage, const std::string& class_name,
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
inline bool ConfigureNormalViewers(
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

inline bool ConfigureStaticDetectors(
    const std::filesystem::path& configfile_path,
    const cv::FileStorage& file_storage,
    const std::vector<std::shared_ptr<Optimizer>>& optimizer_ptrs,
    std::vector<std::shared_ptr<Detector>>* detector_ptrs) {
  std::string class_name{"StaticDetector"};
  return ConfigureObjects<StaticDetector>(
      file_storage, class_name, {"name", "metafile_path", "optimizer"},
      [&](const auto& file_node, auto* detector_ptr) {
        std::shared_ptr<Optimizer> optimizer_ptr;
        if (!GetObject(file_node, "optimizer", class_name, optimizer_ptrs,
                       &optimizer_ptr))
          return false;
        *detector_ptr = std::make_shared<StaticDetector>(
            Name(file_node), MetafilePath(file_node, configfile_path),
            optimizer_ptr);
        return true;
      },
      detector_ptrs);
}

inline bool ConfigureManualDetectors(
    const std::filesystem::path& configfile_path,
    const cv::FileStorage& file_storage,
    const std::vector<std::shared_ptr<Optimizer>>& optimizer_ptrs,
    const std::vector<std::shared_ptr<ColorCamera>>& color_camera_ptrs,
    std::vector<std::shared_ptr<Detector>>* detector_ptrs) {
  std::string class_name{"ManualDetector"};
  return ConfigureObjects<ManualDetector>(
      file_storage, class_name,
      {"name", "metafile_path", "optimizer", "color_camera"},
      [&](const auto& file_node, auto* detector_ptr) {
        std::shared_ptr<Optimizer> optimizer_ptr;
        std::shared_ptr<ColorCamera> color_camera_ptr;
        if (!GetObject(file_node, "optimizer", class_name, optimizer_ptrs,
                       &optimizer_ptr) ||
            !GetObject(file_node, "color_camera", class_name, color_camera_ptrs,
                       &color_camera_ptr))
          return false;
        *detector_ptr = std::make_shared<ManualDetector>(
            Name(file_node), MetafilePath(file_node, configfile_path),
            optimizer_ptr, color_camera_ptr);
        return true;
      },
      detector_ptrs);
}

inline bool ConfigureRefiners(
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

inline bool ConfigureTrackers(
    const std::filesystem::path& configfile_path,
    const cv::FileStorage& file_storage,
    const std::vector<std::shared_ptr<Optimizer>>& optimizer_ptrs,
    const std::vector<std::shared_ptr<Detector>>& detector_ptrs,
    const std::vector<std::shared_ptr<Publisher>>& publisher_ptrs,
    const std::vector<std::shared_ptr<Subscriber>>& subscriber_ptrs,
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
        if (!file_node["publishers"].empty()) {
          if (!AddObjects(file_node, "publishers", class_name, publisher_ptrs,
                          [&](const auto& publisher_ptr) {
                            return (*tracker_ptr)->AddPublisher(publisher_ptr);
                          }))
            return false;
        }
        if (!file_node["subscribers"].empty()) {
          if (!AddObjects(
                  file_node, "subscribers", class_name, subscriber_ptrs,
                  [&](const auto& subscriber_ptr) {
                    return (*tracker_ptr)->AddSubscriber(subscriber_ptr);
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

inline bool GenerateConfiguredTracker(
    const std::filesystem::path& configfile_path,
    std::shared_ptr<Tracker>* tracker_ptr) {
  // Open file storage from yaml
  cv::FileStorage fs;
  if (!OpenYamlFileStorage(configfile_path, &fs)) return false;

  // Configure bodies
  std::vector<std::shared_ptr<Body>> body_ptrs;
  if (!ConfigureObjectsMetafileRequired<Body>(configfile_path, fs, "Body",
                                              &body_ptrs))
    return false;

  // Configure color histograms
  std::vector<std::shared_ptr<ColorHistograms>> color_histograms_ptrs;
  if (!ConfigureObjectsMetafileOptional<ColorHistograms>(
          configfile_path, fs, "ColorHistograms", &color_histograms_ptrs))
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

  // Configure silhouette renderers
  std::vector<std::shared_ptr<FocusedSilhouetteRenderer>>
      focused_silhouette_renderer_ptrs;
  if (!ConfigureFocusedRenderers<FocusedSilhouetteRenderer>(
          configfile_path, fs, "FocusedSilhouetteRenderer",
          renderer_geometry_ptrs, camera_ptrs, body_ptrs,
          &focused_silhouette_renderer_ptrs))
    return false;
  std::copy(begin(focused_silhouette_renderer_ptrs),
            end(focused_silhouette_renderer_ptrs),
            std::back_inserter(focused_depth_renderer_ptrs));

  // Configure region models
  std::vector<std::shared_ptr<RegionModel>> region_model_ptrs;
  if (!ConfigureRegionModels(configfile_path, fs, body_ptrs,
                             &region_model_ptrs))
    return false;

  // Configure depth models
  std::vector<std::shared_ptr<DepthModel>> depth_model_ptrs;
  if (!ConfigureDepthModels(configfile_path, fs, body_ptrs, &depth_model_ptrs))
    return false;

  // Configure modalities
  std::vector<std::shared_ptr<Modality>> modality_ptrs;
  if (!ConfigureRegionModalities(configfile_path, fs, body_ptrs,
                                 color_camera_ptrs, region_model_ptrs,
                                 depth_camera_ptrs, focused_depth_renderer_ptrs,
                                 focused_silhouette_renderer_ptrs,
                                 color_histograms_ptrs, &modality_ptrs) ||
      !ConfigureDepthModalities(
          configfile_path, fs, body_ptrs, depth_camera_ptrs, depth_model_ptrs,
          focused_depth_renderer_ptrs, focused_silhouette_renderer_ptrs,
          &modality_ptrs) ||
      !ConfigureTextureModalities(
          configfile_path, fs, body_ptrs, color_camera_ptrs,
          focused_silhouette_renderer_ptrs, depth_camera_ptrs,
          focused_depth_renderer_ptrs, &modality_ptrs))
    return false;

  // Configure links
  std::vector<std::shared_ptr<Link>> link_ptrs;
  if (!ConfigureLinks(configfile_path, fs, body_ptrs, modality_ptrs,
                      &link_ptrs))
    return false;

  // Configure constraints
  std::vector<std::shared_ptr<Constraint>> constraint_ptrs;
  if (!ConfigureConstraints(configfile_path, fs, link_ptrs, &constraint_ptrs))
    return false;

  // Configure soft constraints
  std::vector<std::shared_ptr<SoftConstraint>> soft_constraint_ptrs;
  if (!ConfigureSoftConstraints(configfile_path, fs, link_ptrs,
                                &soft_constraint_ptrs))
    return false;

  // Configure optimizers
  std::vector<std::shared_ptr<Optimizer>> optimizer_ptrs;
  if (!ConfigureOptimizers(configfile_path, fs, link_ptrs, constraint_ptrs,
                           soft_constraint_ptrs, &optimizer_ptrs))
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
  if (!ConfigureStaticDetectors(configfile_path, fs, optimizer_ptrs,
                                &detector_ptrs) ||
      !ConfigureManualDetectors(configfile_path, fs, optimizer_ptrs,
                                color_camera_ptrs, &detector_ptrs))
    return false;

  // Configure publishers
  std::vector<std::shared_ptr<Publisher>> publisher_ptrs;

  // Configure subscribers
  std::vector<std::shared_ptr<Subscriber>> subscriber_ptrs;

  // Configure refiners
  std::vector<std::shared_ptr<Refiner>> refiner_ptrs;
  if (!ConfigureRefiners(configfile_path, fs, optimizer_ptrs, &refiner_ptrs))
    return false;

  // Configure trackers
  std::vector<std::shared_ptr<Tracker>> tracker_ptrs;
  if (!ConfigureTrackers(configfile_path, fs, optimizer_ptrs, detector_ptrs,
                         publisher_ptrs, subscriber_ptrs, refiner_ptrs,
                         viewer_ptrs, &tracker_ptrs))
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

}  // namespace m3t

#endif  // M3T_INCLUDE_M3T_GENERATOR_H_
