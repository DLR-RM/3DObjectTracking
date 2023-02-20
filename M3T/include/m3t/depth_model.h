// SPDX-License-Identifier: MIT
// Copyright (c) 2023 Manuel Stoiber, German Aerospace Center (DLR)

#ifndef M3T_INCLUDE_M3T_DEPTH_MODEL_H_
#define M3T_INCLUDE_M3T_DEPTH_MODEL_H_

#include <filesystem/filesystem.h>
#include <m3t/body.h>
#include <m3t/common.h>
#include <m3t/model.h>
#include <m3t/normal_renderer.h>
#include <m3t/renderer_geometry.h>
#include <omp.h>

#include <filesystem/filesystem.h>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <array>
#include <atomic>
#include <fstream>
#include <iostream>
#include <memory>
#include <opencv2/opencv.hpp>
#include <random>
#include <string>
#include <vector>

namespace m3t {

/**
 * \brief \ref Model that holds a *Sparse Viewpoint Model* that is generated
 * from a \ref Body and that is used by the \ref DepthModality.
 *
 * \details For each viewpoint, the object stores a \ref View object that
 * includes data for all sampled surface points. Given the `body2camera_pose`
 * the closest view can be accessed using `GetClosestView()`.  The model class
 * allows to add associated bodies that are considered in the sampling of
 * surface points.
 *
 * @param occlusion_body_ptrs referenced \ref Body objects that lead to known
 * occlusions.
 */
class DepthModel : public Model {
 private:
  // Model definition
  static constexpr char kModelType = 'd';
  static constexpr int kVersionID = 9;

  // Fixed parameters
  static constexpr int kMaxPointSamplingTries = 100;

  // Renderer IDs
  static constexpr uchar kBackgroundID = 0;
  static constexpr uchar kMainBodyID = 255;

 public:
  /**
   * \brief Struct that contains all data related to a surface point and that is
   * used by the \ref DepthModel.
   * @param center_f_body 3D surface point.
   * @param normal_f_body 3D surface normal vector at surface point location.
   * @param depth_offsets differences between the depth value of the
   * `center_f_body` coordinate and the minimum depth value within a quadratic
   * kernel for which radius values are increasing by the parameter
   * `stride_depth_offset`.
   */
  struct DataPoint {
    Eigen::Vector3f center_f_body;
    Eigen::Vector3f normal_f_body;
    std::array<float, kMaxNDepthOffsets> depth_offsets{};
  };

  /**
   * \brief Struct that contains all data that is generated from the rendered
   * geometry of a body for a specific viewpoint and that is used by the \ref
   * DepthModel.
   * @param data_points vector of all surface point information.
   * @param orientation vector that points from the camera center to the body
   * center.
   * @param surface_area in square meter
   */
  struct View {
    std::vector<DataPoint> data_points;
    Eigen::Vector3f orientation;
    float surface_area;
  };

  // Constructors and setup methods
  DepthModel(const std::string &name, const std::shared_ptr<Body> &body_ptr,
             const std::filesystem::path &model_path,
             float sphere_radius = 0.8f, int n_divides = 4, int n_points = 200,
             float max_radius_depth_offset = 0.05f,
             float stride_depth_offset = 0.002f, bool use_random_seed = false,
             int image_size = 2000);
  DepthModel(const std::string &name,
             const std::filesystem::path &metafile_path,
             const std::shared_ptr<Body> &body_ptr);
  bool SetUp() override;

  // Configure occlusion bodies
  bool AddOcclusionBody(const std::shared_ptr<Body> &occlusion_body_ptr);
  bool DeleteOcclusionBody(const std::string &name);
  void ClearOcclusionBodies();

  // Main methods
  bool GetClosestView(const Transform3fA &body2camera_pose,
                      const View **closest_view) const;

  // Getter
  float max_surface_area() const;
  const std::vector<std::shared_ptr<Body>> &occlusion_body_ptrs() const;

 private:
  // Helper methods for model set up
  bool LoadMetaData();
  bool GenerateModel();
  bool LoadModel();
  bool SaveModel() const;

  // Helper methods for storing models with occlusion bodies
  void SaveOcclusionBodyData(std::ofstream *ofs) const;
  bool LoadOcclusionBodyData(std::ifstream *ifs);

  // Helper methods for view generation
  bool GeneratePointData(const FullNormalRenderer &main_renderer,
                         const FullSilhouetteRenderer &occlusion_renderer,
                         const Transform3fA &camera2body_pose,
                         std::vector<DataPoint> *data_points,
                         float *surface_area) const;
  static cv::Point2i SampleSurfacePointCoordinate(
      const cv::Mat &silhouette_image, std::mt19937 &generator);

  // Occlusion body ptrs
  std::vector<std::shared_ptr<Body>> occlusion_body_ptrs_{};

  // Model data
  std::vector<View> views_;
  float max_surface_area_ = 0;
};

}  // namespace m3t

#endif  // M3T_INCLUDE_M3T_DEPTH_MODEL_H_
