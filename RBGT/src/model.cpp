// SPDX-License-Identifier: MIT
// Copyright (c) 2020 Manuel Stoiber, German Aerospace Center (DLR)

#include <rbgt/model.h>

namespace rbgt {

Model::Model(std::string name) : name_{std::move(name)} {}

void Model::set_name(const std::string &name) { name_ = name; }

void Model::set_image_size(int image_size) { image_size_ = image_size; }

void Model::set_use_random_seed(bool use_random_seed) {
  use_random_seed_ = use_random_seed;
}

void Model::set_verbose(bool verbose) { verbose_ = verbose; }

bool Model::GenerateModel(const Body &body, float sphere_radius, int n_divides,
                          int n_points) {
  initialized_ = false;
  body_ptr_ = std::make_shared<Body>(body);
  sphere_radius_ = sphere_radius;
  n_divides_ = n_divides;
  n_points_ = n_points;

  // Reset body2world_pose
  body_ptr_->set_body2world_pose(Transform3fA::Identity());

  // Generate camera poses
  std::vector<Transform3fA> camera2body_poses;
  GenerateGeodesicPoses(&camera2body_poses);

  // Create rendere for all threads
  NormalImageRenderer *renderers =
      new NormalImageRenderer[omp_get_max_threads()];
  for (int i = 0; i < omp_get_max_threads(); i++) {
    if (!SetUpRenderer(renderers + i)) {
      delete[] renderers;
      return false;
    }
  }

  // Generate template views
  template_views_.resize(camera2body_poses.size());
  bool cancel = false;
#pragma omp parallel
  {
    int tid = omp_get_thread_num();
#pragma omp for
    for (int i = 0; i < template_views_.size(); ++i) {
      if (cancel) continue;
      if (verbose_) {
        std::stringstream msg;
        msg << "Generate " << body_ptr_->name() << " template view " << i + 1
            << " of " << template_views_.size() << std::endl;
        std::cout << msg.str();
      }

      // Render images
      renderers[tid].set_camera2world_pose(camera2body_poses[i]);
      renderers[tid].StartRendering();
      renderers[tid].FetchNormalImage();
      renderers[tid].FetchDepthImage();

      // Generate data
      template_views_[i].orientation =
          camera2body_poses[i].matrix().col(2).segment(0, 3);
      template_views_[i].data_points.resize(n_points_);
      if (!GeneratePointData(renderers[tid], camera2body_poses[i],
                             &template_views_[i].data_points))
        cancel = true;
    }
  }
  delete[] renderers;
  if (cancel) return false;
  initialized_ = true;
  return true;
}

bool Model::LoadModel(const std::filesystem::path &directory,
                      const std::string &name) {
  initialized_ = false;

  // Ifstream for general info data
  std::filesystem::path info_path{directory / (name + ".txt")};
  std::ifstream info_ifs;
  info_ifs.open(info_path, std::ios::in | std::ios::binary);
  if (!info_ifs.is_open() || info_ifs.fail()) {
    info_ifs.close();
    std::cerr << "Could not open file stream " << info_path << std::endl;
    return false;
  }

  // Check version id
  int version_id;
  ReadValueFromFile(info_ifs, &version_id);
  if (version_id != kVersionID) {
    std::cerr << "Model files " << name << " with wrong version id"
              << std::endl;
    return false;
  }

  // Load template parameters
  int n_template_views;
  ReadValueFromFile(info_ifs, &sphere_radius_);
  ReadValueFromFile(info_ifs, &n_divides_);
  ReadValueFromFile(info_ifs, &n_points_);
  ReadValueFromFile(info_ifs, &image_size_);
  ReadValueFromFile(info_ifs, &n_template_views);

  // Load geometry data
  std::filesystem::path geometry_path;
  float geometry_unit_in_meter;
  bool geometry_counterclockwise;
  bool geometry_enable_culling;
  Transform3fA geometry2body_pose;
  float maximum_body_diameter;
  ReadValueFromFile(info_ifs, &geometry_path);
  ReadValueFromFile(info_ifs, &geometry_unit_in_meter);
  ReadValueFromFile(info_ifs, &geometry_counterclockwise);
  ReadValueFromFile(info_ifs, &geometry_enable_culling);
  ReadValueFromFile(info_ifs, &geometry2body_pose);
  ReadValueFromFile(info_ifs, &maximum_body_diameter);
  body_ptr_ = std::make_shared<Body>(
      "body", geometry_path, geometry_unit_in_meter, geometry_counterclockwise,
      geometry_enable_culling, maximum_body_diameter, geometry2body_pose);
  info_ifs.close();

  // Ifstream for main data
  std::filesystem::path data_path{directory / (name + ".bin")};
  std::ifstream data_ifs;
  data_ifs.open(data_path, std::ios::in | std::ios::binary);
  if (!data_ifs.is_open() || data_ifs.fail()) {
    data_ifs.close();
    std::cerr << "Could not open file stream " << data_path << std::endl;
    return false;
  }

  template_views_.clear();
  template_views_.reserve(n_template_views);
  for (auto i = 0; i < n_template_views; i++) {
    TemplateView tv;
    tv.data_points.resize(n_points_);
    data_ifs.read(reinterpret_cast<char *>(tv.data_points.data()),
                  n_points_ * sizeof(PointData));
    data_ifs.read(reinterpret_cast<char *>(tv.orientation.data()),
                  tv.orientation.size() * sizeof(Eigen::Vector3f::Scalar));
    template_views_.push_back(std::move(tv));
  }
  data_ifs.close();

  initialized_ = true;
  return true;
}

bool Model::SaveModel(const std::filesystem::path &directory,
                      const std::string &name) const {
  if (!initialized_) {
    std::cerr << "Model was not initialized" << std::endl;
    return false;
  }

  // Ofstream for general info data
  std::filesystem::path info_path{directory / (name + ".txt")};
  std::ofstream info_ofs{info_path};

  // Save template parameters
  WriteValueToFile(info_ofs, "version_id", kVersionID);
  WriteValueToFile(info_ofs, "sphere_radius", sphere_radius_);
  WriteValueToFile(info_ofs, "n_divides", n_divides_);
  WriteValueToFile(info_ofs, "n_points", n_points_);
  WriteValueToFile(info_ofs, "image_size", image_size_);
  WriteValueToFile(info_ofs, "n_template_views", int(template_views_.size()));

  // Save geometry data
  WriteValueToFile(info_ofs, "geometry_path", body_ptr_->geometry_path());
  WriteValueToFile(info_ofs, "geometry_unit_in_meter",
                   body_ptr_->geometry_unit_in_meter());
  WriteValueToFile(info_ofs, "geometry_defined_counterclockwise",
                   body_ptr_->geometry_counterclockwise());
  WriteValueToFile(info_ofs, "geometry_enable_culling",
                   body_ptr_->geometry_enable_culling());
  WriteValueToFile(info_ofs, "geometry2body_pose",
                   body_ptr_->geometry2body_pose());
  WriteValueToFile(info_ofs, "maximum_body_diameter",
                   body_ptr_->maximum_body_diameter());
  info_ofs.flush();
  info_ofs.close();

  // Ofstream for main data
  std::filesystem::path data_path{directory / (name + ".bin")};
  std::ofstream data_ofs{data_path, std::ios::out | std::ios::binary};

  for (const auto &tv : template_views_) {
    data_ofs.write(reinterpret_cast<const char *>(tv.data_points.data()),
                   n_points_ * sizeof(PointData));
    data_ofs.write(reinterpret_cast<const char *>(tv.orientation.data()),
                   tv.orientation.size() * sizeof(Eigen::Vector3f::Scalar));
  }
  data_ofs.flush();
  data_ofs.close();
  return true;
}

bool Model::GetClosestTemplateView(
    const Transform3fA &body2camera_pose,
    const TemplateView **closest_template_view) const {
  if (!initialized_) {
    std::cerr << "Template views were not initialized" << std::endl;
    return false;
  }

  Eigen::Vector3f orientation{
      body2camera_pose.rotation().inverse() *
      body2camera_pose.translation().matrix().normalized()};

  float closest_dot = -1.0f;
  for (auto &template_view : template_views_) {
    float dot = orientation.dot(template_view.orientation);
    if (dot > closest_dot) {
      *closest_template_view = &template_view;
      closest_dot = dot;
    }
  }
  return true;
}

const std::string &Model::name() const { return name_; }

int Model::image_size() const { return image_size_; }

bool Model::use_random_seed() const { return use_random_seed_; }

bool Model::verbose() const { return verbose_; }

bool Model::initialized() const { return initialized_; }

bool Model::GeneratePointData(const NormalImageRenderer &renderer,
                              const Transform3fA &camera2body_pose,
                              std::vector<PointData> *data_points) const {
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
  std::mt19937 generator;
  if (use_random_seed_)
    generator.seed(std::chrono::system_clock::now().time_since_epoch().count());
  else
    generator.seed(7);

  // Calculate data for contour points
  for (auto data_point{data_points->begin()};
       data_point != data_points->end();) {
    // Randomly sample point on contour and calculate 3D center
    cv::Point2i center{SampleContourPointCoordinate(
        contours, total_contour_length_in_pixel, generator)};
    Eigen::Vector3f center_f_camera{renderer.GetPointVector(center)};
    data_point->center_f_body = camera2body_pose * center_f_camera;

    // Calculate contour segment and approximate normal vector
    std::vector<cv::Point2i> contour_segment;
    if (!CalculateContourSegment(contours, center, &contour_segment)) continue;
    Eigen::Vector2f normal{ApproximateNormalVector(contour_segment)};
    Eigen::Vector3f normal_f_camera{normal.x(), normal.y(), 0.0f};
    data_point->normal_f_body = camera2body_pose.rotation() * normal_f_camera;

    // Calculate foreground and background distance
    float pixel_to_meter = center_f_camera[2] / renderer.intrinsics().fu;
    CalculateLineDistances(silhouette_image, contours, center, normal,
                           pixel_to_meter, &data_point->foreground_distance,
                           &data_point->background_distance);
    data_point++;
  }
  return true;
}

bool Model::GenerateValidContours(
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
  contours->erase(std::remove_if(contours->begin(), contours->end(),
                                 [](const std::vector<cv::Point2i> &contour) {
                                   return contour.size() < kMinContourLength;
                                 }),
                  contours->end());

  // Test if contours are closed
  for (auto &contour : *contours) {
    if (abs(contour.front().x - contour.back().x) > 1 ||
        abs(contour.front().y - contour.back().y) > 1) {
      std::cerr << "Contours are not closed. " << std::endl;
      return false;
    }
  }

  // Calculate total pixel length of contour
  *total_contour_length_in_pixel = 0;
  for (auto &contour : *contours) {
    *total_contour_length_in_pixel += contour.size();
  }

  // Check if pixel length is greater zero
  if (*total_contour_length_in_pixel == 0) {
    std::cerr << "No valid contour in image " << std::endl;
    return false;
  }
  return true;
}

cv::Point2i Model::SampleContourPointCoordinate(
    const std::vector<std::vector<cv::Point2i>> &contours,
    int total_contour_length_in_pixel, std::mt19937 &generator) {
  int idx = int(generator() % total_contour_length_in_pixel);
  for (auto &contour : contours) {
    if (idx < contour.size())
      return contour[idx];
    else
      idx -= contour.size();
  }
  return cv::Point2i();  // Never reached
}

bool Model::CalculateContourSegment(
    const std::vector<std::vector<cv::Point2i>> &contours, cv::Point2i &center,
    std::vector<cv::Point2i> *contour_segment) {
  for (auto &contour : contours) {
    for (int idx = 0; idx < contour.size(); ++idx) {
      if (contour.at(idx) == center) {
        int start_idx = idx - kContourNormalApproxRadius;
        int end_idx = idx + kContourNormalApproxRadius;
        if (start_idx < 0) {
          contour_segment->insert(contour_segment->end(),
                                  contour.end() + start_idx, contour.end());
          start_idx = 0;
        }
        if (end_idx >= int(contour.size())) {
          contour_segment->insert(contour_segment->end(),
                                  contour.begin() + start_idx, contour.end());
          start_idx = 0;
          end_idx = end_idx - int(contour.size());
        }
        contour_segment->insert(contour_segment->end(),
                                contour.begin() + start_idx,
                                contour.begin() + end_idx + 1);

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

Eigen::Vector2f Model::ApproximateNormalVector(
    const std::vector<cv::Point2i> &contour_segment) {
  return Eigen::Vector2f{
      -float(contour_segment.back().y - contour_segment.front().y),
      float(contour_segment.back().x - contour_segment.front().x)};
}

void Model::CalculateLineDistances(
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

void Model::FindClosestContourPoint(
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

bool Model::SetUpRenderer(NormalImageRenderer *renderer) const {
  auto renderer_geometry_ptr{std::make_shared<RendererGeometry>()};
  if (!renderer_geometry_ptr->AddBody(body_ptr_)) return false;
  Transform3fA pose;  // set later
  float focal_length = float(image_size_ - kImageSizeSafetyBoundary) *
                       sphere_radius_ / body_ptr_->maximum_body_diameter();
  float principal_point = float(image_size_) / 2.0f;
  Intrinsics intrinsics{focal_length,    focal_length, principal_point,
                        principal_point, image_size_,  image_size_};
  float z_min = sphere_radius_ - body_ptr_->maximum_body_diameter() * 0.5f;
  float z_max = sphere_radius_ + body_ptr_->maximum_body_diameter() * 0.5f;
  return renderer->Init("renderer", renderer_geometry_ptr, pose, intrinsics,
                        z_min, z_max);
}

void Model::GenerateGeodesicPoses(
    std::vector<Transform3fA> *camera2body_poses) const {
  // Generate geodesic points
  std::set<Eigen::Vector3f, CompareSmallerVector3f> geodesic_points;
  GenerateGeodesicPoints(&geodesic_points);

  // Generate geodesic poses from points
  Eigen::Vector3f downwards{0.0f, 1.0f, 0.0f};  // direction in body frame
  camera2body_poses->clear();
  for (const auto &geodesic_point : geodesic_points) {
    Transform3fA pose;
    pose = Eigen::Translation<float, 3>{geodesic_point * sphere_radius_};

    Eigen::Matrix3f Rotation;
    Rotation.col(2) = -geodesic_point;
    Rotation.col(0) = downwards.cross(-geodesic_point).normalized();
    if (Rotation.col(0).sum() == 0) {
      Rotation.col(0) = Eigen::Vector3f{1.0f, 0.0f, 0.0f};
    }
    Rotation.col(1) = Rotation.col(2).cross(Rotation.col(0));
    pose.rotate(Rotation);
    camera2body_poses->push_back(pose);
  }
}

void Model::GenerateGeodesicPoints(
    std::set<Eigen::Vector3f, CompareSmallerVector3f> *geodesic_points) const {
  // Define icosahedron
  constexpr float x = 0.525731112119133606f;
  constexpr float z = 0.850650808352039932f;
  std::vector<Eigen::Vector3f> icosahedron_points{
      {-x, 0.0, z}, {x, 0.0, z},  {-x, 0.0, -z}, {x, 0.0, -z},
      {0.0, z, x},  {0.0, z, -x}, {0.0, -z, x},  {0.0, -z, -x},
      {z, x, 0.0},  {-z, x, 0.0}, {z, -x, 0.0},  {-z, -x, 0.0}};
  std::vector<Eigen::Vector3i> icosahedron_ids{
      {0, 4, 1},  {0, 9, 4},  {9, 5, 4},  {4, 5, 8},  {4, 8, 1},
      {8, 10, 1}, {8, 3, 10}, {5, 3, 8},  {5, 2, 3},  {2, 7, 3},
      {7, 10, 3}, {7, 6, 10}, {7, 11, 6}, {11, 0, 6}, {0, 1, 6},
      {6, 1, 10}, {9, 0, 11}, {9, 11, 2}, {9, 2, 5},  {7, 2, 11}};

  // Create points
  geodesic_points->clear();
  for (const auto &icosahedron_id : icosahedron_ids) {
    SubdivideTriangle(icosahedron_points[icosahedron_id[0]],
                      icosahedron_points[icosahedron_id[1]],
                      icosahedron_points[icosahedron_id[2]], n_divides_,
                      geodesic_points);
  }
}

void Model::SubdivideTriangle(
    const Eigen::Vector3f &v1, const Eigen::Vector3f &v2,
    const Eigen::Vector3f &v3, int n_divides,
    std::set<Eigen::Vector3f, CompareSmallerVector3f> *geodesic_points) {
  if (n_divides == 0) {
    geodesic_points->insert(v1);
    geodesic_points->insert(v2);
    geodesic_points->insert(v3);
  } else {
    Eigen::Vector3f v12 = (v1 + v2).normalized();
    Eigen::Vector3f v13 = (v1 + v3).normalized();
    Eigen::Vector3f v23 = (v2 + v3).normalized();
    SubdivideTriangle(v1, v12, v13, n_divides - 1, geodesic_points);
    SubdivideTriangle(v2, v12, v23, n_divides - 1, geodesic_points);
    SubdivideTriangle(v3, v13, v23, n_divides - 1, geodesic_points);
    SubdivideTriangle(v12, v13, v23, n_divides - 1, geodesic_points);
  }
}

}  // namespace rbgt
