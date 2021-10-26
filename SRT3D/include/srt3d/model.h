// SPDX-License-Identifier: MIT
// Copyright (c) 2021 Manuel Stoiber, German Aerospace Center (DLR)

#ifndef OBJECT_TRACKING_INCLUDE_SRT3D_MODEL_H_
#define OBJECT_TRACKING_INCLUDE_SRT3D_MODEL_H_

#include <omp.h>
#include <srt3d/body.h>
#include <srt3d/common.h>
#include <srt3d/normal_renderer.h>
#include <srt3d/renderer_geometry.h>

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <array>
#include <atomic>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <memory>
#include <opencv2/opencv.hpp>
#include <random>
#include <string>
#include <vector>

namespace srt3d {

// Class that stores a sparse viewpoint model of a body that consists of views
// with multiple contour points. It includes all functionality to generate,
// save, and load the model
class Model {
 private:
  // Some constants
  static constexpr int kVersionID = 2;
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

  // Constructors and setup methods
  Model(const std::string &name, std::shared_ptr<Body> body_ptr,
        const std::filesystem::path &directory, const std::string &filename,
        float sphere_radius = 0.8, int n_divides = 4, int n_points = 200,
        bool use_random_seed = true, int image_size = 2000);
  bool SetUp();

  // Setters
  void set_name(const std::string &name);
  void set_body_ptr(std::shared_ptr<Body> body_ptr);
  void set_directory(const std::filesystem::path &directory);
  void set_filename(const std::string &filename);
  void set_sphere_radius(float sphere_radius);
  void set_n_divides(int n_divides);
  void set_n_points(int n_points);
  void set_use_random_seed(bool use_random_seed);
  void set_image_size(int image_size);

  // Main methods
  bool GetClosestTemplateView(const Transform3fA &body2camera_pose,
                              const TemplateView **closest_template_view) const;

  // Getters
  const std::string &name() const;
  std::shared_ptr<Body> body_ptr() const;
  const std::filesystem::path &directory() const;
  const std::string &filename() const;
  float sphere_radius() const;
  int n_divides() const;
  int n_points() const;
  bool use_random_seed() const;
  int image_size() const;
  bool set_up() const;

 private:
  // Helper methods for model set up
  bool GenerateModel();
  bool LoadModel();
  bool SaveModel() const;

  // Helper methods for point data
  bool GeneratePointData(const NormalRenderer &renderer,
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
  bool SetUpRenderer(
      const std::shared_ptr<RendererGeometry> &renderer_geometry_ptr,
      std::shared_ptr<NormalRenderer> *renderer) const;
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

  // Data
  std::string name_{};
  std::shared_ptr<Body> body_ptr_ = nullptr;
  std::filesystem::path directory_{};
  std::string filename_{};
  float sphere_radius_{};
  int n_divides_{};
  int n_points_{};
  bool use_random_seed_{};
  int image_size_{};
  bool set_up_ = false;
};

}  // namespace srt3d

#endif  // OBJECT_TRACKING_INCLUDE_SRT3D_MODEL_H_
