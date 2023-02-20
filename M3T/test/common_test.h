// SPDX-License-Identifier: MIT
// Copyright (c) 2023 Manuel Stoiber, German Aerospace Center (DLR)

#ifndef M3T_TEST_COMMON_TEST_H_
#define M3T_TEST_COMMON_TEST_H_

#include <filesystem/filesystem.h>
#include <m3t/basic_depth_renderer.h>
#include <m3t/body.h>
#include <m3t/color_histograms.h>
#include <m3t/common.h>
#include <m3t/depth_model.h>
#include <m3t/loader_camera.h>
#include <m3t/region_model.h>
#include <m3t/silhouette_renderer.h>
#include <m3t/static_detector.h>

#include <Eigen/Dense>
#include <Eigen/Geometry>

// Flags
constexpr bool kGenerateGroundTruth = false;

// Directories
const std::filesystem::path data_directory{"../../data/"};
const std::filesystem::path temp_directory{"../../temp/"};
const std::filesystem::path body_directory{data_directory / "_body"};
const std::filesystem::path sequence_directory{data_directory / "_sequence"};

// Set up functions for bodies
std::shared_ptr<m3t::Body> TriangleBodyPtrNoSetUp();
std::shared_ptr<m3t::Body> TriangleBodyPtr();
std::shared_ptr<m3t::Body> SchaumaBodyPtrNoSetUp();
std::shared_ptr<m3t::Body> SchaumaBodyPtr();

// Set up function for renderer
std::shared_ptr<m3t::FocusedBasicDepthRenderer>
FocusedBasicDepthRendererPtrNoSetUp(
    std::shared_ptr<m3t::Body> triangle_body_ptr,
    std::shared_ptr<m3t::Body> schauma_body_ptr,
    std::shared_ptr<m3t::Camera> camera_ptr);
std::shared_ptr<m3t::FocusedBasicDepthRenderer> FocusedBasicDepthRendererPtr(
    std::shared_ptr<m3t::Body> triangle_body_ptr,
    std::shared_ptr<m3t::Body> schauma_body_ptr,
    std::shared_ptr<m3t::Camera> camera_ptr);
std::shared_ptr<m3t::FocusedSilhouetteRenderer>
FocusedSilhouetteRendererPtrNoSetUp(
    std::shared_ptr<m3t::Body> triangle_body_ptr,
    std::shared_ptr<m3t::Body> schauma_body_ptr,
    std::shared_ptr<m3t::Camera> camera_ptr);
std::shared_ptr<m3t::FocusedSilhouetteRenderer> FocusedSilhouetteRendererPtr(
    std::shared_ptr<m3t::Body> triangle_body_ptr,
    std::shared_ptr<m3t::Body> schauma_body_ptr,
    std::shared_ptr<m3t::Camera> camera_ptr);

// Set up functions for camera
std::shared_ptr<m3t::LoaderColorCamera> ColorCameraPtrNoSetUp();
std::shared_ptr<m3t::LoaderColorCamera> ColorCameraPtr();
std::shared_ptr<m3t::LoaderDepthCamera> DepthCameraPtrNoSetUp();
std::shared_ptr<m3t::LoaderDepthCamera> DepthCameraPtr();

// Set up functions for model
std::shared_ptr<m3t::RegionModel> TriangleRegionModelPtrNoSetUp();
std::shared_ptr<m3t::RegionModel> TriangleRegionModelPtr();
std::shared_ptr<m3t::DepthModel> TriangleDepthModelPtrNoSetUp();
std::shared_ptr<m3t::DepthModel> TriangleDepthModelPtr();

// Set up functions for color histograms
std::shared_ptr<m3t::ColorHistograms> ColorHistogramsPtrNoSetUp();
std::shared_ptr<m3t::ColorHistograms> ColorHistogramsPtr();

// Helper methods for evaluation
bool CompareImages(const cv::Mat &image1, const cv::Mat &image2,
                   int max_intensity_difference, int max_n_wrong_pixels);
bool CompareLoadedImages(const std::filesystem::path &directory1,
                         const std::string &filename1,
                         const std::filesystem::path &directory2,
                         const std::string &filename2,
                         int max_intensity_difference, int max_n_wrong_pixels);
bool CompareToLoadedImage(const std::filesystem::path &directory,
                          const std::string &filename, const cv::Mat &image,
                          int max_intensity_difference, int max_n_wrong_pixels);
bool CompareToLoadedMatrix(const std::filesystem::path &directory,
                           const std::string &filename,
                           const Eigen::MatrixXf &matrix, float max_rel_error);

// Helper methods for ground truth generation
void SaveOrSkipImage(bool save_image, const std::filesystem::path &directory,
                     const std::string &filename, const cv::Mat &image);
bool CopyOrSkipImage(bool copy_image, const std::filesystem::path &directory,
                     const std::string &filename,
                     const std::filesystem::path &source_directory,
                     const std::string &source_filename);
bool SaveOrSkipMatrix(bool save_matrix, const std::filesystem::path &directory,
                      const std::string &filename,
                      const Eigen::MatrixXf &matrix);

#endif  // M3T_TEST_COMMON_TEST_H_
