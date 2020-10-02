// SPDX-License-Identifier: MIT
// Copyright (c) 2020 Manuel Stoiber, German Aerospace Center (DLR)

#ifndef OBJECT_TRACKING_INCLUDE_RBGT_MODEL_H_
#define OBJECT_TRACKING_INCLUDE_RBGT_MODEL_H_

#include <rbgt/body.h>
#include <rbgt/common.h>
#include <rbgt/normal_image_renderer.h>
#include <rbgt/renderer_geometry.h>
#include <omp.h>

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <memory>
#include <opencv2/opencv.hpp>
#include <random>
#include <string>
#include <vector>

namespace rbgt {

// Class that stores a model of a body that consists of template views with
// multiple contour points. It includes all functionality to generate, save, and
// load the model
class Model {
 private:
  // Some constants
  static constexpr int kVersionID = 1;
  static constexpr int kContourNormalApproxRadius = 3;
  static constexpr int kMinContourLength = 15;
  static constexpr int kImageSizeSafetyBoundary = 20;

  // Struct with operator that compares two Vector3f and checks if v1 < v2
  struct CompareSmallerVector3f {
    bool operator()(const Eigen::Vector3f &v1,
                    const Eigen::Vector3f &v2) const {
      return v1[0] < v2[0] || (v1[0] == v2[0] && v1[1] < v2[1]) ||
             (v1[0] == v2[0] && v1[1] == v2[1] && v1[2] < v2[2]);
    }
  };

 public:
  using PointData = struct PointData {
    Eigen::Vector3f center_f_body;
    Eigen::Vector3f normal_f_body;
    float foreground_distance = 0.0f;
    float background_distance = 0.0f;
  };

  using TemplateView = struct TemplateView {
    std::vector<PointData> data_points;
    Eigen::Vector3f orientation;  // points from camera to body center
  };

  // Constructor and setters for default values
  Model(std::string name);
  void set_name(const std::string &name);
  void set_image_size(int image_size);
  void set_use_random_seed(bool use_random_seed);
  void set_verbose(bool verbose);

  // Main methods
  bool GenerateModel(const Body &body, float sphere_radius, int n_divides,
                     int n_points);
  bool LoadModel(const std::filesystem::path &directory,
                 const std::string &name);
  bool SaveModel(const std::filesystem::path &directory,
                 const std::string &name) const;
  bool GetClosestTemplateView(const Transform3fA &body2camera_pose,
                              const TemplateView **closest_template_view) const;

  // Getters
  const std::string &name() const;
  int image_size() const;
  bool use_random_seed() const;
  bool verbose() const;
  bool initialized() const;

 private:
  // Helper methods for point data
  bool GeneratePointData(const NormalImageRenderer &renderer,
                         const Transform3fA &camera2body_pose,
                         std::vector<PointData> *data_points) const;
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

  // Halper methods for view data
  bool SetUpRenderer(NormalImageRenderer *renderer) const;
  void GenerateGeodesicPoses(
      std::vector<Transform3fA> *camera2body_poses) const;
  void GenerateGeodesicPoints(
      std::set<Eigen::Vector3f, CompareSmallerVector3f> *geodesic_points) const;
  static void SubdivideTriangle(
      const Eigen::Vector3f &v1, const Eigen::Vector3f &v2,
      const Eigen::Vector3f &v3, int n_divides,
      std::set<Eigen::Vector3f, CompareSmallerVector3f> *geodesic_points);

  // Model data
  std::vector<TemplateView> template_views_;
  std::shared_ptr<Body> body_ptr_ = nullptr;

  // Parameters
  std::string name_{};
  float sphere_radius_{};
  int n_divides_{};
  int n_points_{};
  int image_size_ = 2000;
  bool use_random_seed_ = true;
  bool verbose_ = true;

  // Internal variables
  bool initialized_ = false;
};

}  // namespace rbgt

#endif  // OBJECT_TRACKING_INCLUDE_RBGT_MODEL_H_
