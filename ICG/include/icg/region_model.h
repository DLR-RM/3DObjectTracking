// SPDX-License-Identifier: MIT
// Copyright (c) 2022 Manuel Stoiber, German Aerospace Center (DLR)

#ifndef ICG_INCLUDE_ICG_REGION_MODEL_H_
#define ICG_INCLUDE_ICG_REGION_MODEL_H_

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
 * from a \ref Body and that is used by the \ref RegionModality.
 *
 * \details For each viewpoint, the object stores a \ref View object that
 * includes data for all sampled contour points. Given the `body2camera_pose`
 * the closest view can be accessed using `GetClosestView()`.
 */
class RegionModel : public Model {
 private:
  // Model definition
  static constexpr char kModelType = 'r';
  static constexpr int kVersionID = 6;

  // Fixed parameters
  static constexpr int kContourNormalApproxRadius = 3;
  static constexpr int kMinContourLength = 15;

 public:
  /**
   * \brief Struct that contains all data related to a contour point and that is
   * used by the \ref RegionModel.
   * @param center_f_body 3D contour point.
   * @param normal_f_body 3D surface normal vector at contour point location.
   * @param foreground_distance distance along the line defined by the normal
   * `normal_f_body` for which the foreground is not interrupted by the
   * background.
   * @param background_distance distance along the line defined by the normal
   * `normal_f_body` for which the background is not interrupted by the
   * foreground.
   * @param depth_offsets differences between the depth value of the
   * `center_f_body` coordinate and the minimum depth value within a quadratic
   * kernel for which radius values are increasing by the parameter
   * `stride_depth_offset`.
   */
  struct DataPoint {
    Eigen::Vector3f center_f_body;
    Eigen::Vector3f normal_f_body;
    float foreground_distance = 0.0f;
    float background_distance = 0.0f;
    std::array<float, kMaxNDepthOffsets> depth_offsets{};
  };

  /**
   * \brief Struct that contains all data that is generated from the rendered
   * geometry of a body for a specific viewpoint and that is used by the \ref
   * RegionModel.
   * @param data_points vector of all contour point information.
   * @param orientation vector that points from the camera center to the body
   * center.
   */
  struct View {
    std::vector<DataPoint> data_points;
    Eigen::Vector3f orientation;
  };

  // Constructors and setup methods
  RegionModel(const std::string &name, const std::shared_ptr<Body> &body_ptr,
              const std::filesystem::path &model_path,
              float sphere_radius = 0.8f, int n_divides = 4, int n_points = 200,
              float max_radius_depth_offset = 0.05f,
              float stride_depth_offset = 0.002f, bool use_random_seed = false,
              int image_size = 2000);
  RegionModel(const std::string &name,
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
  bool GenerateValidContours(const cv::Mat &silhouette_image,
                             std::vector<std::vector<cv::Point2i>> *contours,
                             int *total_contour_length_in_pixel) const;
  static cv::Point2i SampleContourPointCoordinate(
      const std::vector<std::vector<cv::Point2i>> &contours,
      int total_contour_length_in_pixel, std::mt19937 &generator);
  static bool CalculateContourSegment(
      const std::vector<std::vector<cv::Point2i>> &contours,
      cv::Point2i &center, std::vector<cv::Point2i> *contour_segment);
  static Eigen::Vector2f ApproximateNormalVector(
      const std::vector<cv::Point2i> &contour_segment);
  void CalculateLineDistances(
      const cv::Mat &silhouette_image,
      const std::vector<std::vector<cv::Point2i>> &contours,
      const cv::Point2i &center, const Eigen::Vector2f &normal,
      float pixel_to_meter, float *foreground_distance,
      float *background_distance) const;
  static void FindClosestContourPoint(
      const std::vector<std::vector<cv::Point2i>> &contours, float u, float v,
      int *u_contour, int *v_contour);

  // Model data
  std::vector<View> views_;
};

}  // namespace icg

#endif  // ICG_INCLUDE_ICG_REGION_MODEL_H_
