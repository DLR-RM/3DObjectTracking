// SPDX-License-Identifier: MIT
// Copyright (c) 2022 Manuel Stoiber, German Aerospace Center (DLR)

#include <icg/region_model.h>

namespace icg {

RegionModel::RegionModel(const std::string &name,
                         const std::shared_ptr<Body> &body_ptr,
                         const std::filesystem::path &model_path,
                         float sphere_radius, int n_divides, int n_points,
                         float max_radius_depth_offset,
                         float stride_depth_offset, bool use_random_seed,
                         int image_size)
    : Model{name,
            body_ptr,
            model_path,
            sphere_radius,
            n_divides,
            n_points,
            max_radius_depth_offset,
            stride_depth_offset,
            use_random_seed,
            image_size} {}

RegionModel::RegionModel(const std::string &name,
                         const std::filesystem::path &metafile_path,
                         const std::shared_ptr<Body> &body_ptr)
    : Model{name, metafile_path, body_ptr} {}

bool RegionModel::SetUp() {
  set_up_ = false;
  if (!metafile_path_.empty())
    if (!LoadMetaData()) return false;

  // Check if all required objects are set up
  if (!body_ptr_->set_up()) {
    std::cerr << "Body " << body_ptr_->name() << " was not set up" << std::endl;
    return false;
  }

  if (!DepthOffsetVariablesValid()) return false;
  if (use_random_seed_ || !LoadModel()) {
    if (!GenerateModel()) return false;
    if (!SaveModel()) return false;
  }
  set_up_ = true;
  return true;
}

bool RegionModel::GetClosestView(const Transform3fA &body2camera_pose,
                                 const View **closest_view) const {
  if (!set_up_) {
    std::cerr << "Set up region model " << name_ << " first" << std::endl;
    return false;
  }

  if (body2camera_pose.translation().norm() == 0.0f) {
    *closest_view = &views_[0];
    return true;
  }

  Eigen::Vector3f orientation{
      body2camera_pose.rotation().inverse() *
      body2camera_pose.translation().matrix().normalized()};

  float closest_dot = -1.0f;
  for (auto &view : views_) {
    float dot = orientation.dot(view.orientation);
    if (dot > closest_dot) {
      *closest_view = &view;
      closest_dot = dot;
    }
  }
  return true;
}

bool RegionModel::LoadMetaData() {
  // Open file storage from yaml
  cv::FileStorage fs;
  if (!OpenYamlFileStorage(metafile_path_, &fs)) return false;

  // Read parameters from yaml
  if (!ReadRequiredValueFromYaml(fs, "model_path", &model_path_)) {
    std::cerr << "Could not read all required body parameters from "
              << metafile_path_ << std::endl;
    return false;
  }
  ReadOptionalValueFromYaml(fs, "sphere_radius", &sphere_radius_);
  ReadOptionalValueFromYaml(fs, "n_divides", &n_divides_);
  ReadOptionalValueFromYaml(fs, "n_points", &n_points_);
  ReadOptionalValueFromYaml(fs, "max_radius_depth_offset",
                            &max_radius_depth_offset_);
  ReadOptionalValueFromYaml(fs, "stride_depth_offset", &stride_depth_offset_);
  ReadOptionalValueFromYaml(fs, "use_random_seed", &use_random_seed_);
  ReadOptionalValueFromYaml(fs, "image_size", &image_size_);
  fs.release();

  // Process parameters
  if (model_path_ == "INFER_FROM_NAME")
    model_path_ = metafile_path_.parent_path() / (name_ + ".bin");
  else if (model_path_.is_relative())
    model_path_ = metafile_path_.parent_path() / model_path_;
  return true;
}

bool RegionModel::GenerateModel() {
  // Generate camera poses
  std::vector<Transform3fA> camera2body_poses;
  GenerateGeodesicPoses(&camera2body_poses);

  // Create RendererGeometries in main thread to comply with GLFW thread safety
  // requirements
  std::vector<std::shared_ptr<RendererGeometry>> renderer_geometry_ptrs(
      omp_get_max_threads());
  for (auto &renderer_geometry_ptr : renderer_geometry_ptrs) {
    renderer_geometry_ptr = std::make_shared<RendererGeometry>("rg");
    renderer_geometry_ptr->SetUp();
  }

  // Generate template views
  std::cout << "Start generating model " << name_ << std::endl;
  views_.resize(camera2body_poses.size());
  bool cancel = false;
  std::atomic<int> count = 1;
#pragma omp parallel
  {
    std::shared_ptr<FullNormalRenderer> renderer_ptr;
    if (!SetUpRenderer(renderer_geometry_ptrs[omp_get_thread_num()],
                       &renderer_ptr))
      cancel = true;

#pragma omp for
    for (int i = 0; i < int(views_.size()); ++i) {
      if (cancel) continue;
      std::stringstream msg;
      msg << "Generate region model " << name_ << ": view " << count++ << " of "
          << views_.size() << std::endl;
      std::cout << msg.str();

      // Render images
      renderer_ptr->set_camera2world_pose(camera2body_poses[i]);
      renderer_ptr->StartRendering();
      renderer_ptr->FetchNormalImage();
      renderer_ptr->FetchDepthImage();

      // Generate data
      views_[i].orientation =
          camera2body_poses[i].matrix().col(2).segment(0, 3);
      views_[i].data_points.resize(n_points_);
      if (!GeneratePointData(*renderer_ptr, camera2body_poses[i],
                             &views_[i].data_points))
        cancel = true;
    }
  }
  if (cancel) return false;
  std::cout << "Finish generating model " << name_ << std::endl;
  return true;
}

bool RegionModel::LoadModel() {
  std::ifstream ifs{model_path_, std::ios::in | std::ios::binary};
  if (!ifs.is_open() || ifs.fail()) {
    ifs.close();
    std::cout << "Could not open model file " << model_path_ << std::endl;
    return false;
  }

  if (!LoadModelParameters(kVersionID, kModelType, &ifs)) {
    std::cout << "Model file " << model_path_
              << " was generated using different model parameters" << std::endl;
    return false;
  }

  if (!LoadBodyData(&ifs)) {
    std::cout << "Model file " << model_path_
              << " was generated using different body parameters" << std::endl;
    return false;
  }

  // Load view data
  size_t n_views;
  ifs.read((char *)(&n_views), sizeof(n_views));
  views_.clear();
  views_.reserve(n_views);
  for (size_t i = 0; i < n_views; i++) {
    View tv;
    tv.data_points.resize(n_points_);
    ifs.read((char *)(tv.data_points.data()), n_points_ * sizeof(DataPoint));
    ifs.read((char *)(tv.orientation.data()), sizeof(tv.orientation));
    views_.push_back(std::move(tv));
  }
  ifs.close();
  return true;
}

bool RegionModel::SaveModel() const {
  std::ofstream ofs{model_path_, std::ios::out | std::ios::binary};
  SaveModelParameters(kVersionID, kModelType, &ofs);
  SaveBodyData(&ofs);

  // Save main data
  size_t n_views = views_.size();
  ofs.write((const char *)(&n_views), sizeof(n_views));
  for (const auto &v : views_) {
    ofs.write((const char *)(v.data_points.data()),
              n_points_ * sizeof(DataPoint));
    ofs.write((const char *)(v.orientation.data()), sizeof(v.orientation));
  }
  ofs.flush();
  ofs.close();
  return true;
}

bool RegionModel::GeneratePointData(const FullNormalRenderer &renderer,
                                    const Transform3fA &camera2body_pose,
                                    std::vector<DataPoint> *data_points) const {
  // Compute silhouette
  std::vector<cv::Mat> normal_image_channels(4);
  cv::split(renderer.normal_image(), normal_image_channels);
  cv::Mat &silhouette_image{normal_image_channels[3]};

  // Generate contour
  int total_contour_length_in_pixel;
  std::vector<std::vector<cv::Point2i>> contours;
  if (!GenerateValidContours(silhouette_image, &contours,
                             &total_contour_length_in_pixel))
    return false;

  // Set up generator
  std::mt19937 generator{7};
  if (use_random_seed_)
    generator.seed(
        unsigned(std::chrono::system_clock::now().time_since_epoch().count()));

  // Calculate data for contour points
  for (auto data_point{begin(*data_points)}; data_point != end(*data_points);) {
    // Randomly sample point on contour and calculate 3D center
    cv::Point2i center{SampleContourPointCoordinate(
        contours, total_contour_length_in_pixel, generator)};
    Eigen::Vector3f center_f_camera{renderer.PointVector(center)};
    data_point->center_f_body = camera2body_pose * center_f_camera;

    // Calculate contour segment and approximate normal vector
    std::vector<cv::Point2i> contour_segment;
    if (!CalculateContourSegment(contours, center, &contour_segment)) continue;
    Eigen::Vector2f normal{ApproximateNormalVector(contour_segment)};
    Eigen::Vector3f normal_f_camera{normal.x(), normal.y(), 0.0f};
    data_point->normal_f_body = camera2body_pose.rotation() * normal_f_camera;

    // Calculate foreground and background distance
    float pixel_to_meter = center_f_camera(2) / renderer.intrinsics().fu;
    CalculateLineDistances(silhouette_image, contours, center, normal,
                           pixel_to_meter, &data_point->foreground_distance,
                           &data_point->background_distance);

    // Calculate depth offsets
    CalculateDepthOffsets(renderer, center, pixel_to_meter,
                          &data_point->depth_offsets);

    data_point++;
  }
  return true;
}

bool RegionModel::GenerateValidContours(
    const cv::Mat &silhouette_image,
    std::vector<std::vector<cv::Point2i>> *contours,
    int *total_contour_length_in_pixel) const {
  // test if outer border is empty
  for (int i = 0; i < image_size_; ++i) {
    if (silhouette_image.at<uchar>(0, i) ||
        silhouette_image.at<uchar>(image_size_ - 1, i) ||
        silhouette_image.at<uchar>(i, 0) ||
        silhouette_image.at<uchar>(i, image_size_ - 1)) {
      std::cerr << "BodyData does not fit into image" << std::endl
                << "Check body2camera_pose and maximum_body_diameter"
                << std::endl;
      cv::imshow("Silhouette Image", silhouette_image);
      cv::waitKey(0);
      return false;
    }
  }

  // Compute contours
  cv::findContours(silhouette_image, *contours, cv::RetrievalModes::RETR_LIST,
                   cv::ContourApproximationModes::CHAIN_APPROX_NONE);

  // Filter contours that are too short
  contours->erase(std::remove_if(begin(*contours), end(*contours),
                                 [](const std::vector<cv::Point2i> &contour) {
                                   return contour.size() < kMinContourLength;
                                 }),
                  end(*contours));

  // Test if contours are closed
  for (auto &contour : *contours) {
    if (abs(contour.front().x - contour.back().x) > 1 ||
        abs(contour.front().y - contour.back().y) > 1) {
      std::cerr << "Contours are not closed." << std::endl;
      return false;
    }
  }

  // Calculate total pixel length of contour
  *total_contour_length_in_pixel = 0;
  for (auto &contour : *contours) {
    *total_contour_length_in_pixel += int(contour.size());
  }

  // Check if pixel length is greater zero
  if (*total_contour_length_in_pixel == 0) {
    std::cerr << "No valid contour in image." << std::endl;
    return false;
  }
  return true;
}

cv::Point2i RegionModel::SampleContourPointCoordinate(
    const std::vector<std::vector<cv::Point2i>> &contours,
    int total_contour_length_in_pixel, std::mt19937 &generator) {
  int idx = int(generator() % total_contour_length_in_pixel);
  for (auto &contour : contours) {
    if (idx < contour.size())
      return contour[idx];
    else
      idx -= int(contour.size());
  }
  return cv::Point2i();  // Never reached
}

bool RegionModel::CalculateContourSegment(
    const std::vector<std::vector<cv::Point2i>> &contours, cv::Point2i &center,
    std::vector<cv::Point2i> *contour_segment) {
  for (auto &contour : contours) {
    for (int idx = 0; idx < contour.size(); ++idx) {
      if (contour.at(idx) == center) {
        int start_idx = idx - kContourNormalApproxRadius;
        int end_idx = idx + kContourNormalApproxRadius;
        if (start_idx < 0) {
          contour_segment->insert(end(*contour_segment),
                                  end(contour) + start_idx, end(contour));
          start_idx = 0;
        }
        if (end_idx >= int(contour.size())) {
          contour_segment->insert(end(*contour_segment),
                                  begin(contour) + start_idx, end(contour));
          start_idx = 0;
          end_idx = end_idx - int(contour.size());
        }
        contour_segment->insert(end(*contour_segment),
                                begin(contour) + start_idx,
                                begin(contour) + end_idx + 1);

        // Check quality of contour segment
        float segment_distance = std::hypotf(
            float(contour_segment->back().x - contour_segment->front().x),
            float(contour_segment->back().y - contour_segment->front().y));
        return segment_distance > float(kContourNormalApproxRadius);
      }
    }
  }
  std::cerr << "Could not find point on contour" << std::endl;
  return false;
}

Eigen::Vector2f RegionModel::ApproximateNormalVector(
    const std::vector<cv::Point2i> &contour_segment) {
  return Eigen::Vector2f{
      -float(contour_segment.back().y - contour_segment.front().y),
      float(contour_segment.back().x - contour_segment.front().x)}
      .normalized();
}

void RegionModel::CalculateLineDistances(
    const cv::Mat &silhouette_image,
    const std::vector<std::vector<cv::Point2i>> &contours,
    const cv::Point2i &center, const Eigen::Vector2f &normal,
    float pixel_to_meter, float *foreground_distance,
    float *background_distance) const {
  // Calculate starting positions and steps for both sides of the line
  float u_out = float(center.x) + 0.5f;
  float v_out = float(center.y) + 0.5f;
  float u_in = float(center.x) + 0.5f;
  float v_in = float(center.y) + 0.5f;
  float u_step, v_step;
  if (std::fabs(normal.y()) < std::fabs(normal.x())) {
    u_step = float(sgn(normal.x()));
    v_step = normal.y() / abs(normal.x());
  } else {
    u_step = normal.x() / abs(normal.y());
    v_step = float(sgn(normal.y()));
  }

  // Search for first inwards intersection with contour
  int u_in_endpoint, v_in_endpoint;
  while (true) {
    u_in -= u_step;
    v_in -= v_step;
    if (!silhouette_image.at<uchar>(int(v_in), int(u_in))) {
      FindClosestContourPoint(contours, u_in + u_step - 0.5f,
                              v_in + v_step - 0.5f, &u_in_endpoint,
                              &v_in_endpoint);
      *foreground_distance =
          pixel_to_meter * hypotf(float(u_in_endpoint - center.x),
                                  float(v_in_endpoint - center.y));
      break;
    }
  }

  // Search for first outwards intersection with contour
  int u_out_endpoint, v_out_endpoint;
  while (true) {
    u_out += u_step;
    v_out += v_step;
    if (int(u_out) < 0 || int(u_out) >= image_size_ || int(v_out) < 0 ||
        int(v_out) >= image_size_) {
      *background_distance = std::numeric_limits<float>::max();
      break;
    }
    if (silhouette_image.at<uchar>(int(v_out), int(u_out))) {
      FindClosestContourPoint(contours, u_out - 0.5f, v_out - 0.5f,
                              &u_out_endpoint, &v_out_endpoint);
      *background_distance =
          pixel_to_meter * hypotf(float(u_out_endpoint - center.x),
                                  float(v_out_endpoint - center.y));
      break;
    }
  }
}

void RegionModel::FindClosestContourPoint(
    const std::vector<std::vector<cv::Point2i>> &contours, float u, float v,
    int *u_contour, int *v_contour) {
  float min_distance = std::numeric_limits<float>::max();
  for (auto &contour : contours) {
    for (auto &point : contour) {
      float distance = hypotf(float(point.x) - u, float(point.y) - v);
      if (distance < min_distance) {
        *u_contour = point.x;
        *v_contour = point.y;
        min_distance = distance;
      }
    }
  }
}

}  // namespace icg
