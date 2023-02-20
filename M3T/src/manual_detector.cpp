// SPDX-License-Identifier: MIT
// Copyright (c) 2023 Manuel Stoiber and Anne Elisabeth Reichert,
// German Aerospace Center (DLR)

#include <m3t/manual_detector.h>

#include <opencv2/core/eigen.hpp>

namespace m3t {

PointDetector::PointDetector(const cv::Mat& image,
                             const std::string& window_name,
                             const std::filesystem::path& detector_image_path)
    : image_{image},
      window_name_{window_name},
      detector_image_path_{detector_image_path} {}

void PointDetector::set_image(const cv::Mat& image) { image_ = image; }

void PointDetector::set_window_name(const std::string& window_name) {
  window_name_ = window_name;
}

void PointDetector::set_detector_image_path(
    const std::filesystem::path& detector_image_path) {
  detector_image_path_ = detector_image_path;
}

void PointDetector::MouseClickCallback(int event, int x, int y, int flags,
                                       void* param) {
  if (event == cv::EVENT_LBUTTONDOWN) {
    auto point_detector{static_cast<PointDetector*>(param)};
    if (point_detector->detected_points_.size() >= 4) return;

    point_detector->detected_points_.push_back(cv::Point2f{float(x), float(y)});
    cv::circle(point_detector->viewer_image_, cv::Point2i{x, y}, 3,
               cv::Vec3b{255, 0, 255}, cv::FILLED, cv::LINE_8);
    cv::imshow(point_detector->window_name_, point_detector->viewer_image_);
  }
}

bool PointDetector::DetectPoints() {
  // Load detector image if it exists
  cv::Mat unmodified_viewer_image{image_};
  auto detector_image{cv::imread(detector_image_path_.string())};
  if (!detector_image.empty()) {
    int scale_factor = 5;
    cv::resize(
        detector_image, detector_image,
        cv::Size{
            image_.rows * image_.cols / (scale_factor * detector_image.cols),
            image_.rows * image_.cols / (scale_factor * detector_image.rows)});
    cv::Rect roi{cv::Point{0, 0}, detector_image.size()};
    detector_image.copyTo(unmodified_viewer_image(roi));
  }

  // Create window and callback
  unmodified_viewer_image.copyTo(viewer_image_);
  cv::namedWindow(window_name_, 1);
  cv::setMouseCallback(window_name_, PointDetector::MouseClickCallback, this);

  // Iterate until 4 points are detected or quit
  while (true) {
    imshow(window_name_, viewer_image_);
    char key = cv::waitKey(1000);
    if (key == 'c' || key == 8) {
      detected_points_.clear();
      unmodified_viewer_image.copyTo(viewer_image_);
    } else if (key == 'q' || key == 13) {
      cv::destroyWindow(window_name_);
      return detected_points_.size() >= 4;
    }
  }
}

const cv::Mat& PointDetector::image() const { return image_; }

const std::string& PointDetector::window_name() const { return window_name_; }

const std::filesystem::path& PointDetector::detector_image_path() const {
  return detector_image_path_;
}

const std::vector<cv::Point2f>& PointDetector::detected_points() const {
  return detected_points_;
}

ManualDetector::ManualDetector(
    const std::string& name,
    const std::shared_ptr<m3t::Optimizer>& optimizer_ptr,
    const std::shared_ptr<m3t::ColorCamera>& color_camera_ptr,
    const std::vector<cv::Point3f>& reference_points,
    const std::filesystem::path& detector_image_path, bool reset_joint_poses)
    : Detector{name, reset_joint_poses_},
      optimizer_ptr_{optimizer_ptr},
      color_camera_ptr_{color_camera_ptr},
      reference_points_{reference_points},
      detector_image_path_{detector_image_path} {}

ManualDetector::ManualDetector(
    const std::string& name, const std::filesystem::path& metafile_path,
    const std::shared_ptr<m3t::Optimizer>& optimizer_ptr,
    const std::shared_ptr<m3t::ColorCamera>& color_camera_ptr)
    : Detector{name, metafile_path},
      optimizer_ptr_{optimizer_ptr},
      color_camera_ptr_{color_camera_ptr} {}

bool ManualDetector::SetUp() {
  set_up_ = false;
  if (!metafile_path_.empty())
    if (!LoadMetaData()) return false;

  // Check if all required objects are set up
  if (!optimizer_ptr_->set_up()) {
    std::cerr << "Optimizer " << optimizer_ptr_->name() << " was not set up"
              << std::endl;
    return false;
  }
  if (!color_camera_ptr_->set_up()) {
    std::cerr << "Color camera " << color_camera_ptr_->name()
              << " was not set up" << std::endl;
    return false;
  }

  set_up_ = true;
  return true;
}

void ManualDetector::set_optimizer_ptr(
    const std::shared_ptr<Optimizer>& optimizer_ptr) {
  optimizer_ptr_ = optimizer_ptr;
  set_up_ = false;
}

void ManualDetector::set_color_camera_ptr(
    const std::shared_ptr<m3t::ColorCamera>& color_camera_ptr) {
  color_camera_ptr_ = color_camera_ptr;
  set_up_ = false;
}

void ManualDetector::set_reference_points(
    const std::vector<cv::Point3f>& reference_points) {
  reference_points_ = reference_points;
}

void ManualDetector::set_detector_image_path(
    const std::filesystem::path& detector_image_path) {
  detector_image_path_ = detector_image_path;
}

bool ManualDetector::DetectPoses(const std::set<std::string>& names,
                                 std::set<std::string>* detected_names) {
  if (!set_up_) {
    std::cerr << "Set up manual detector " << name_ << " first" << std::endl;
    return false;
  }

  // Detect points and calculate rotation and translation
  if (names.find(optimizer_ptr_->name()) != names.end()) {
    PointDetector point_detector{color_camera_ptr_->image(),
                                 "Detect " + optimizer_ptr_->name(),
                                 detector_image_path_};
    if (!point_detector.DetectPoints()) return false;
    cv::Mat rotation, translation;
    cv::solvePnP(reference_points_, point_detector.detected_points(),
                 GetCameraMatrixFromIntrinsics(), {}, rotation, translation,
                 false, cv::SOLVEPNP_EPNP);

    // Transform translation to Eigen
    m3t::Transform3fA link2world_pose;
    Eigen::Vector3f eigen_translation;
    cv::cv2eigen(translation, eigen_translation);
    link2world_pose.translation() = eigen_translation;

    // Transform rotation to Eigen
    cv::Mat rotation_matrix;
    Eigen::Matrix3f eigen_rotation;
    cv::Rodrigues(rotation, rotation_matrix);
    cv::cv2eigen(rotation_matrix, eigen_rotation);
    link2world_pose.linear() = eigen_rotation;

    // Validate pose
    if (link2world_pose.translation().norm() > kMaxDistance) {
      std::cout << "Detected pose from manual detector " << name_
                << " is not valid:" << std::endl
                << "detected link2world_pose = " << std::endl
                << link2world_pose.matrix() << std::endl;
      return true;
    }

    // Update pose
    UpdatePoses(link2world_pose, optimizer_ptr_);
    if (detected_names) detected_names->insert(optimizer_ptr_->name());
  }
  return true;
}

const std::shared_ptr<Optimizer>& ManualDetector::optimizer_ptr() const {
  return optimizer_ptr_;
}

const std::shared_ptr<m3t::ColorCamera>& ManualDetector::color_camera_ptr()
    const {
  return color_camera_ptr_;
}

const std::vector<cv::Point3f>& ManualDetector::reference_points() const {
  return reference_points_;
}

const std::filesystem::path& ManualDetector::detector_image_path() const {
  return detector_image_path_;
}

std::vector<std::shared_ptr<Optimizer>> ManualDetector::optimizer_ptrs() const {
  return {optimizer_ptr_};
}

std::shared_ptr<m3t::Camera> ManualDetector::camera_ptr() const {
  return color_camera_ptr_;
}

cv::Mat ManualDetector::GetCameraMatrixFromIntrinsics() const {
  return cv::Mat_<float>(3, 3) << color_camera_ptr_->intrinsics().fu, 0,
         color_camera_ptr_->intrinsics().ppu, 0,
         color_camera_ptr_->intrinsics().fv,
         color_camera_ptr_->intrinsics().ppv, 0, 0, 1.;
}

bool ManualDetector::LoadMetaData() {
  // Open file storage from yaml
  cv::FileStorage fs;
  if (!OpenYamlFileStorage(metafile_path_, &fs)) return false;

  // Read parameters from yaml
  if (!ReadRequiredValueFromYaml(fs, "reference_points", &reference_points_)) {
    std::cerr << "Could not read all required manual detector parameters from "
              << metafile_path_ << std::endl;
    return false;
  }
  ReadOptionalValueFromYaml(fs, "reset_joint_poses", &reset_joint_poses_);
  ReadOptionalValueFromYaml(fs, "detector_image_path", &detector_image_path_);
  fs.release();

  // Process parameters
  if (detector_image_path_.is_relative())
    detector_image_path_ = metafile_path_.parent_path() / detector_image_path_;
  return true;
}

}  // namespace m3t
