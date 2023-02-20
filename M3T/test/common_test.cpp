// SPDX-License-Identifier: MIT
// Copyright (c) 2023 Manuel Stoiber, German Aerospace Center (DLR)

#include "common_test.h"

std::shared_ptr<m3t::Body> TriangleBodyPtrNoSetUp() {
  auto body_ptr{std::make_shared<m3t::Body>("triangle",
                                            body_directory / "triangle.yaml")};
  m3t::Transform3fA pose;
  pose.matrix() << 0.607676f, 0.408914f, -0.680823f, 0.472944f, 0.786584f,
      -0.428213f, 0.444880f, -0.213009f, -0.109620f, -0.805867f, -0.581860f,
      0.346384f, 0.0f, 0.0f, 0.0f, 1.0f;
  body_ptr->set_world2body_pose(pose);
  return body_ptr;
}

std::shared_ptr<m3t::Body> TriangleBodyPtr() {
  auto body_ptr{TriangleBodyPtrNoSetUp()};
  body_ptr->SetUp();
  return body_ptr;
}

std::shared_ptr<m3t::Body> SchaumaBodyPtrNoSetUp() {
  auto body_ptr{
      std::make_shared<m3t::Body>("schauma", body_directory / "schauma.yaml")};
  m3t::Transform3fA pose;
  pose.matrix() << 0.607676f, 0.408914f, -0.680823f, 0.297794f, 0.786584f,
      -0.428213f, 0.444880f, -0.189009f, -0.109620f, -0.805867f, -0.581860f,
      0.255284f, 0.0f, 0.0f, 0.0f, 1.0f;
  body_ptr->set_world2body_pose(pose);
  return body_ptr;
}

std::shared_ptr<m3t::Body> SchaumaBodyPtr() {
  auto body_ptr{SchaumaBodyPtrNoSetUp()};
  body_ptr->SetUp();
  return body_ptr;
}

std::shared_ptr<m3t::FocusedBasicDepthRenderer>
FocusedBasicDepthRendererPtrNoSetUp(
    std::shared_ptr<m3t::Body> triangle_body_ptr,
    std::shared_ptr<m3t::Body> schauma_body_ptr,
    std::shared_ptr<m3t::Camera> camera_ptr) {
  auto renderer_geometry_ptr{
      std::make_shared<m3t::RendererGeometry>("renderer_geometry")};
  renderer_geometry_ptr->SetUp();
  renderer_geometry_ptr->AddBody(triangle_body_ptr);
  renderer_geometry_ptr->AddBody(schauma_body_ptr);
  auto renderer_ptr{std::make_shared<m3t::FocusedBasicDepthRenderer>(
      "focused_basic_depth_renderer", renderer_geometry_ptr, camera_ptr)};
  renderer_ptr->AddReferencedBody(triangle_body_ptr);
  return renderer_ptr;
}

std::shared_ptr<m3t::FocusedBasicDepthRenderer> FocusedBasicDepthRendererPtr(
    std::shared_ptr<m3t::Body> triangle_body_ptr,
    std::shared_ptr<m3t::Body> schauma_body_ptr,
    std::shared_ptr<m3t::Camera> camera_ptr) {
  auto renderer_ptr{FocusedBasicDepthRendererPtrNoSetUp(
      triangle_body_ptr, schauma_body_ptr, camera_ptr)};
  renderer_ptr->SetUp();
  return renderer_ptr;
}

std::shared_ptr<m3t::FocusedSilhouetteRenderer>
FocusedSilhouetteRendererPtrNoSetUp(
    std::shared_ptr<m3t::Body> triangle_body_ptr,
    std::shared_ptr<m3t::Body> schauma_body_ptr,
    std::shared_ptr<m3t::Camera> camera_ptr) {
  auto renderer_geometry_ptr{
      std::make_shared<m3t::RendererGeometry>("renderer_geometry")};
  renderer_geometry_ptr->SetUp();
  renderer_geometry_ptr->AddBody(triangle_body_ptr);
  renderer_geometry_ptr->AddBody(schauma_body_ptr);
  auto renderer_ptr{std::make_shared<m3t::FocusedSilhouetteRenderer>(
      "focused_silhouette_renderer", renderer_geometry_ptr, camera_ptr,
      m3t::IDType::BODY)};
  renderer_ptr->AddReferencedBody(triangle_body_ptr);
  return renderer_ptr;
}

std::shared_ptr<m3t::FocusedSilhouetteRenderer> FocusedSilhouetteRendererPtr(
    std::shared_ptr<m3t::Body> triangle_body_ptr,
    std::shared_ptr<m3t::Body> schauma_body_ptr,
    std::shared_ptr<m3t::Camera> camera_ptr) {
  auto renderer_ptr{FocusedSilhouetteRendererPtrNoSetUp(
      triangle_body_ptr, schauma_body_ptr, camera_ptr)};
  renderer_ptr->SetUp();
  return renderer_ptr;
}

std::shared_ptr<m3t::LoaderColorCamera> ColorCameraPtrNoSetUp() {
  return std::make_shared<m3t::LoaderColorCamera>(
      "color_camera", sequence_directory / "color_camera.yaml");
}

std::shared_ptr<m3t::LoaderColorCamera> ColorCameraPtr() {
  auto camera_ptr{ColorCameraPtrNoSetUp()};
  camera_ptr->SetUp();
  return camera_ptr;
}

std::shared_ptr<m3t::LoaderDepthCamera> DepthCameraPtrNoSetUp() {
  return std::make_shared<m3t::LoaderDepthCamera>(
      "depth_camera", sequence_directory / "depth_camera.yaml");
}

std::shared_ptr<m3t::LoaderDepthCamera> DepthCameraPtr() {
  auto camera_ptr{DepthCameraPtrNoSetUp()};
  camera_ptr->SetUp();
  return camera_ptr;
}

std::shared_ptr<m3t::RegionModel> TriangleRegionModelPtrNoSetUp() {
  auto model_ptr{std::make_shared<m3t::RegionModel>(
      "triangle_region_model", body_directory / "triangle_region_model.yaml",
      TriangleBodyPtr())};
  return model_ptr;
}

std::shared_ptr<m3t::RegionModel> TriangleRegionModelPtr() {
  auto model_ptr{TriangleRegionModelPtrNoSetUp()};
  model_ptr->SetUp();
  return model_ptr;
}

std::shared_ptr<m3t::DepthModel> TriangleDepthModelPtrNoSetUp() {
  auto model_ptr{std::make_shared<m3t::DepthModel>(
      "triangle_depth_model", body_directory / "triangle_depth_model.yaml",
      TriangleBodyPtr())};
  model_ptr->set_use_random_seed(false);
  return model_ptr;
}

std::shared_ptr<m3t::DepthModel> TriangleDepthModelPtr() {
  auto model_ptr{TriangleDepthModelPtrNoSetUp()};
  model_ptr->SetUp();
  return model_ptr;
}

std::shared_ptr<m3t::ColorHistograms> ColorHistogramsPtrNoSetUp() {
  return std::make_shared<m3t::ColorHistograms>(
      "color_histogram",
      data_directory / "color_histograms_test" / "color_histograms.yaml");
}

std::shared_ptr<m3t::ColorHistograms> ColorHistogramsPtr() {
  auto color_histograms_ptr{ColorHistogramsPtrNoSetUp()};
  color_histograms_ptr->SetUp();
  return color_histograms_ptr;
}

bool CompareImages(const cv::Mat &image1, const cv::Mat &image2,
                   int max_intensity_difference, int max_n_wrong_pixels) {
  cv::Mat diff;
  cv::absdiff(image1, image2, diff);
  std::vector<cv::Mat> channels(image1.channels());
  cv::split(diff, channels);

  cv::Mat in_range_all{channels[0].size(), CV_8UC1, 1};
  for (int i = 0; i < image1.channels(); i++) {
    cv::Mat in_range_channel;
    cv::inRange(channels[i], 0, 1 + max_intensity_difference, in_range_channel);
    cv::bitwise_and(in_range_channel, in_range_all, in_range_all);
  }
  int n_wrong_pixels = image1.total() - cv::countNonZero(in_range_all);
  return n_wrong_pixels <= max_n_wrong_pixels;
}

bool CompareLoadedImages(const std::filesystem::path &directory1,
                         const std::string &filename1,
                         const std::filesystem::path &directory2,
                         const std::string &filename2,
                         int max_intensity_difference, int max_n_wrong_pixels) {
  std::filesystem::path path1{directory1 / filename1};
  std::filesystem::path path2{directory2 / filename2};
  cv::Mat image1{cv::imread(path1.string(), cv::IMREAD_UNCHANGED)};
  cv::Mat image2{cv::imread(path2.string(), cv::IMREAD_UNCHANGED)};
  if (image1.empty()) {
    std::cerr << "Could not load image " << path1 << std::endl;
    return false;
  }
  if (image2.empty()) {
    std::cerr << "Could not load image " << path2 << std::endl;
    return false;
  }
  return CompareImages(image1, image2, max_intensity_difference,
                       max_n_wrong_pixels);
}

bool CompareToLoadedImage(const std::filesystem::path &directory,
                          const std::string &filename, const cv::Mat &image,
                          int max_intensity_difference,
                          int max_n_wrong_pixels) {
  std::filesystem::path path{directory / filename};
  cv::Mat loaded_image{cv::imread(path.string(), cv::IMREAD_UNCHANGED)};
  if (loaded_image.empty()) {
    std::cerr << "Could not load image " << path << std::endl;
    return false;
  }
  return CompareImages(loaded_image, image, max_intensity_difference,
                       max_n_wrong_pixels);
}

bool CompareToLoadedMatrix(const std::filesystem::path &directory,
                           const std::string &filename,
                           const Eigen::MatrixXf &matrix, float max_rel_error) {
  std::filesystem::path path{directory / filename};
  std::ifstream ifs{path, std::ios::in | std::ios::binary};
  if (!ifs.is_open() || ifs.fail()) {
    ifs.close();
    std::cout << "Could not load matrix " << path << std::endl;
    return false;
  }
  Eigen::MatrixXf loaded_matrix;
  m3t::ReadValueFromTxt(ifs, &loaded_matrix);
  ifs.close();
  float rel_error =
      ((loaded_matrix - matrix).array().abs() / matrix.array())
          .unaryExpr([](float v) { return std::isfinite(v) ? v : 0.0f; })
          .maxCoeff();
  if (rel_error > max_rel_error) {
    std::cout << rel_error << " > " << max_rel_error << std::endl;
    return false;
  }
  return true;
}

void SaveOrSkipImage(bool save_image, const std::filesystem::path &directory,
                     const std::string &filename, const cv::Mat &image) {
  if (save_image) {
    cv::imshow(filename, image);
    std::filesystem::path path{directory / filename};
    if (cv::waitKey(0) == 'q') {
      std::cout << "Skip saving image " << path << std::endl;
      return;
    };
    cv::destroyWindow(filename);
    cv::imwrite(path.string(), image);
    std::cout << "Save image " << path << std::endl;
  }
}

bool CopyOrSkipImage(bool copy_image, const std::filesystem::path &directory,
                     const std::string &filename,
                     const std::filesystem::path &source_directory,
                     const std::string &source_filename) {
  if (copy_image) {
    std::filesystem::path source_path{source_directory / source_filename};
    cv::Mat source_image{
        cv::imread(source_path.string(), cv::IMREAD_UNCHANGED)};
    if (source_image.empty()) {
      std::cerr << "Could not load image " << source_path << std::endl;
      return false;
    }
    SaveOrSkipImage(copy_image, directory, filename, source_image);
  }
  return true;
}

bool SaveOrSkipMatrix(bool save_matrix, const std::filesystem::path &directory,
                      const std::string &filename,
                      const Eigen::MatrixXf &matrix) {
  if (save_matrix) {
    std::cout << filename << " = " << std::endl;
    std::cout << matrix << std::endl;
    std::cout << "Should matrix be saved? (y/n): ";
    char key;
    std::cin >> key;
    if (key == 'y') {
      std::filesystem::path path{directory / filename};
      std::ofstream ofs{path, std::ios::out | std::ios::binary};
      m3t::WriteValueToTxt(ofs, filename, matrix);
      ofs.flush();
      ofs.close();
      std::cout << "Save matrix " << path << std::endl;
    }
  }
  return true;
}
