// SPDX-License-Identifier: MIT
// Copyright (c) 2022 Manuel Stoiber, German Aerospace Center (DLR)

#ifndef ICG_INCLUDE_ICG_DEPTH_MODEL_H_
#define ICG_INCLUDE_ICG_DEPTH_MODEL_H_

#include <filesystem/filesystem.h>
#include <icg/body.h>
#include <icg/common.h>
#include <icg/model.h>
#include <icg/normal_renderer.h>
#include <icg/renderer_geometry.h>
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

namespace icg {

/**
 * \brief \ref Model that holds a *Sparse Viewpoint Model* that is generated
 * from a \ref Body and that is used by the \ref DepthModality.
 *
 * \details For each viewpoint, the object stores a \ref View object that
 * includes data for all sampled surface points. Given the `body2camera_pose`
 * the closest view can be accessed using `GetClosestView()`.
 */
class DepthModel : public Model {
 private:
  // Some constants
  static constexpr char kModelType = 'd';
  static constexpr int kVersionID = 6;
  static constexpr int kMinSilhouetteSize = 1000;

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
   */
  struct View {
    std::vector<DataPoint> data_points;
    Eigen::Vector3f orientation;
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

  // Main methods
  bool GetClosestView(const Transform3fA &body2camera_pose,
                      const View **closest_view) const;

 private:
  // Helper methods for model set up
  bool LoadMetaData();
  bool GenerateModel();
  bool LoadModel();
  bool SaveModel() const;

  // Helper methods for view generation
  bool GeneratePointData(const FullNormalRenderer &renderer,
                         const Transform3fA &camera2body_pose,
                         std::vector<DataPoint> *data_points) const;
  static bool IsSilhouetteValid(const cv::Mat &silhouette_image);
  static cv::Point2i SampleSurfacePointCoordinate(
      const cv::Mat &silhouette_image, std::mt19937 &generator);

  // Model data
  std::vector<View> views_;
};

}  // namespace icg

#endif  // ICG_INCLUDE_ICG_DEPTH_MODEL_H_
