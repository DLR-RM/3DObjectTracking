// SPDX-License-Identifier: MIT
// Copyright (c) 2023 Manuel Stoiber and Anne Elisabeth Reichert,
// German Aerospace Center (DLR)

#ifndef M3T_INCLUDE_M3T_REGION_MODEL_H_
#define M3T_INCLUDE_M3T_REGION_MODEL_H_

#include <filesystem/filesystem.h>
#include <m3t/body.h>
#include <m3t/common.h>
#include <m3t/model.h>
#include <m3t/normal_renderer.h>
#include <m3t/renderer_geometry.h>
#include <m3t/silhouette_renderer.h>
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
 * from a \ref Body and that is used by the \ref RegionModality.
 *
 * \details For each viewpoint, the object stores a \ref View object that
 * includes data for all sampled contour points. Given the `body2camera_pose`
 * the closest view can be accessed using `GetClosestView()`. The model class
 * allows to add associated bodies that are considered in the sampling of
 * contour points and in the calculation of foregournd and background distances.
 *
 * @param fixed_body_ptrs referenced \ref Body objects that are fixed with
 * respect to the main `body_ptr` and that are considered to be part of a
 * different region.
 * @param movable_body_ptrs referenced \ref Body objects that are movable with
 * respect to the main `body_ptr` and that are considered to be part of a
 * different region.
 * @param fixed_same_region_body_ptrs referenced \ref Body objects that are
 * fixed with respect to the main `body_ptr` and that are considered to be part
 * of the same region as the main `body_ptr`.
 * @param movable_same_region_body_ptrs referenced \ref Body objects that are
 * movable with respect to the main `body_ptr` and that are considered to be
 * part of the same region as the main `body_ptr`.
 */
class RegionModel : public Model {
 private:
  // Model definition
  static constexpr char kModelType = 'r';
  static constexpr int kVersionID = 10;

  // Fixed parameters
  static constexpr int kContourNormalApproxRadius = 3;
  static constexpr int kMinContourLength = 15;
  static constexpr int kMaxPointSamplingTries = 100;
  static constexpr float kMaxSurfaceGradient = 10.0f;

  // Renderer IDs
  static constexpr uchar kBackgroundID = 0;
  static constexpr uchar kMainBodyID = 255;
  static constexpr uchar kDifferentBodyID = 120;

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
   * @param contour_length in meter
   */
  struct View {
    std::vector<DataPoint> data_points;
    Eigen::Vector3f orientation;
    float contour_length;
  };

 private:
  /**
   * \brief Struct that contains silhouette renderers to perform validation
   * checks or calculate foreground/background distances on contour points.
   * Contour points are obtained from the main_renderer.
   * @param occlusion is used to check if a contour point is occluded.
   * @param same_region is used to check if a neighboring point belongs to a
   * same-region body.
   * @param foreground is used to calculate foreground distances.
   * @param background is used to calculate background distances.
   */
  struct AssociatedRendererPtrs {
    std::shared_ptr<FullSilhouetteRenderer> occlusion{nullptr};
    std::shared_ptr<FullSilhouetteRenderer> same_region{nullptr};
    std::shared_ptr<FullSilhouetteRenderer> foreground{nullptr};
    std::shared_ptr<FullSilhouetteRenderer> background{nullptr};
  };

  /**
   * \brief Struct that contains corresponding renderer geometries for
   * silhouette renderers in \ref AssociatedRendererPtr
   */
  struct AssociatedRendererGeometryPtrs {
    std::vector<std::shared_ptr<RendererGeometry>> occlusion{};
    std::vector<std::shared_ptr<RendererGeometry>> same_region{};
    std::vector<std::shared_ptr<RendererGeometry>> foreground{};
    std::vector<std::shared_ptr<RendererGeometry>> background{};
  };

 public:
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

  // Configure associated bodies
  bool AddAssociatedBody(const std::shared_ptr<Body> &associated_body_ptr,
                         bool movable, bool same_region);
  bool DeleteAssociatedBody(const std::string &name);
  void ClearAssociatedBodies();

  // Main methods
  bool GetClosestView(const Transform3fA &body2camera_pose,
                      const View **closest_view) const;

  // Getter
  float max_contour_length() const;
  const std::vector<std::shared_ptr<Body>> &associated_body_ptrs() const;
  const std::vector<std::shared_ptr<Body>> &fixed_body_ptrs() const;
  const std::vector<std::shared_ptr<Body>> &movable_body_ptrs() const;
  const std::vector<std::shared_ptr<Body>> &fixed_same_region_body_ptrs() const;
  const std::vector<std::shared_ptr<Body>> &movable_same_region_body_ptrs()
      const;

 private:
  // Helper methods for model set up
  bool LoadMetaData();
  bool GenerateModel();
  bool LoadModel();
  bool SaveModel() const;

  // Helper methods for storing models with associated bodies
  void SaveAssociatedBodyData(std::ofstream *ofs) const;
  bool LoadAssociatedBodyData(std::ifstream *ifs);

  // Helper methods for rendering
  AssociatedRendererGeometryPtrs GenerateSetUpAssociatedRendererGeometries(
      int n_renderer_geometries) const;
  bool SetUpAssociatedRenderers(
      const AssociatedRendererGeometryPtrs &associated_renderer_geometry_ptrs,
      int thread_num, AssociatedRendererPtrs *associated_renderers);
  bool AddBodiesToAssociatedRenderers(
      const AssociatedRendererPtrs &associated_renderer_ptrs);
  static void RenderAndFetchAssociatedImages(
      const AssociatedRendererPtrs &associated_renderer_ptrs,
      const Transform3fA &camera2world_pose);

  // Helper methods for view generation
  bool GeneratePointData(const FullSilhouetteRenderer &main_renderer,
                         const AssociatedRendererPtrs &associated_renderer_ptrs,
                         const Transform3fA &camera2body_pose,
                         std::vector<DataPoint> *data_points,
                         float *contour_length) const;

  // Helper methods for contour generation
  bool GenerateValidContours(const cv::Mat &silhouette_image,
                             std::vector<std::vector<cv::Point2i>> *contours,
                             int *pixel_contour_length) const;
  static bool IsContourPointValid(
      float max_depth_difference, const cv::Point2i &image_coordinates,
      const FullSilhouetteRenderer &main_renderer,
      const AssociatedRendererPtrs &associated_renderer_ptrs);

  // Helper methods to calculate attributes of DataPoint
  static cv::Point2i SampleContourPointCoordinate(
      const std::vector<cv::Point2i> &valid_contour_points,
      std::mt19937 &generator);
  static bool CalculateContourSegment(
      const std::vector<std::vector<cv::Point2i>> &contours,
      cv::Point2i &center, std::vector<cv::Point2i> *contour_segment);
  static Eigen::Vector2f ApproximateNormalVector(
      const std::vector<cv::Point2i> &contour_segment);
  void CalculateLineDistances(
      const FullSilhouetteRenderer &main_renderer,
      const AssociatedRendererPtrs &associated_renderer_ptrs,
      const std::vector<std::vector<cv::Point2i>> &contours,
      const cv::Point2i &center, const Eigen::Vector2f &normal,
      float pixel_to_meter, float *foreground_distance,
      float *background_distance) const;
  static void FindClosestContourPoint(
      const std::vector<std::vector<cv::Point2i>> &contours, float u, float v,
      int *u_contour, int *v_contour);

  // Associated body ptrs
  std::vector<std::shared_ptr<Body>> associated_body_ptrs_{};
  std::vector<std::shared_ptr<Body>> fixed_body_ptrs_{};
  std::vector<std::shared_ptr<Body>> movable_body_ptrs_{};
  std::vector<std::shared_ptr<Body>> fixed_same_region_body_ptrs_{};
  std::vector<std::shared_ptr<Body>> movable_same_region_body_ptrs_{};

  // Model data
  std::vector<View> views_;
  float max_contour_length_ = 0;
};

}  // namespace m3t

#endif  // M3T_INCLUDE_M3T_REGION_MODEL_H_
