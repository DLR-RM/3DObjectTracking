// SPDX-License-Identifier: MIT
// Copyright (c) 2021 Manuel Stoiber, German Aerospace Center (DLR)

#include <srt3d/model.h>

namespace srt3d {

Model::Model(const std::string &name, std::shared_ptr<Body> body_ptr,
             const std::filesystem::path &directory,
             const std::string &filename, float sphere_radius, int n_divides,
             int n_points, bool use_random_seed, int image_size)
    : name_{name},
      body_ptr_{std::move(body_ptr)},
      directory_{directory},
      filename_{filename},
      sphere_radius_{sphere_radius},
      n_divides_{n_divides},
      n_points_{n_points},
      use_random_seed_{use_random_seed},
      image_size_{image_size} {}

bool Model::SetUp() {
  set_up_ = false;
  if (!LoadModel()) {
    if (!GenerateModel()) return false;
    if (!SaveModel()) return false;
  }
  set_up_ = true;
  return true;
}

void Model::set_name(const std::string &name) { name_ = name; }

void Model::set_body_ptr(std::shared_ptr<Body> body_ptr) {
  body_ptr_ = std::move(body_ptr);
  set_up_ = false;
}

void Model::set_directory(const std::filesystem::path &directory) {
  directory_ = directory;
  set_up_ = false;
}

void Model::set_filename(const std::string &filename) {
  filename_ = filename;
  set_up_ = false;
}

void Model::set_sphere_radius(float sphere_radius) {
  sphere_radius_ = sphere_radius;
  set_up_ = false;
}

void Model::set_n_divides(int n_divides) {
  n_divides_ = n_divides;
  set_up_ = false;
}

void Model::set_n_points(int n_points) {
  n_points_ = n_points;
  set_up_ = false;
}

void Model::set_use_random_seed(bool use_random_seed) {
  use_random_seed_ = use_random_seed;
  set_up_ = false;
}

void Model::set_image_size(int image_size) {
  image_size_ = image_size;
  set_up_ = false;
}

bool Model::GetClosestTemplateView(
    const Transform3fA &body2camera_pose,
    const TemplateView **closest_template_view) const {
  if (!set_up_) {
    std::cerr << "Set up model " << name_ << " first" << std::endl;
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

std::shared_ptr<Body> Model::body_ptr() const { return body_ptr_; }

const std::filesystem::path &Model::directory() const { return directory_; }

const std::string &Model::filename() const { return filename_; }

float Model::sphere_radius() const { return sphere_radius_; }

int Model::n_divides() const { return n_divides_; }

int Model::n_points() const { return n_points_; }

bool Model::use_random_seed() const { return use_random_seed_; }

int Model::image_size() const { return image_size_; }

bool Model::set_up() const { return set_up_; }

bool Model::GenerateModel() {
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
  template_views_.resize(camera2body_poses.size());
  bool cancel = false;
  std::atomic<int> count = 1;
#pragma omp parallel
  {
    std::shared_ptr<NormalRenderer> renderer_ptr;
    if (!SetUpRenderer(renderer_geometry_ptrs[omp_get_thread_num()],
                       &renderer_ptr))
      cancel = true;
#pragma omp for
    for (int i = 0; i < int(template_views_.size()); ++i) {
      if (cancel) continue;
      std::stringstream msg;
      msg << "Generate " << body_ptr_->name() << " template view " << count++
          << " of " << template_views_.size() << std::endl;
      std::cout << msg.str();

      // Render images
      renderer_ptr->set_camera2world_pose(camera2body_poses[i]);
      renderer_ptr->StartRendering();
      renderer_ptr->FetchNormalImage();
      renderer_ptr->FetchDepthImage();

      // Generate data
      template_views_[i].orientation =
          camera2body_poses[i].matrix().col(2).segment(0, 3);
      template_views_[i].data_points.resize(n_points_);
      if (!GeneratePointData(*renderer_ptr, camera2body_poses[i],
                             &template_views_[i].data_points))
        cancel = true;
    }
  }
  if (cancel) return false;
  std::cout << "Finish generating model " << name_ << std::endl;
  return true;
}

bool Model::LoadModel() {
  std::filesystem::path data_path{directory_ / filename_};
  std::ifstream data_ifs{data_path, std::ios::in | std::ios::binary};
  if (!data_ifs.is_open() || data_ifs.fail()) {
    data_ifs.close();
    std::cout << "Could not open model file " << data_path << std::endl;
    return false;
  }

  // Check if model file has correct parameters
  int version_id_file;
  float sphere_radius_file;
  int n_divides_file;
  int n_points_file;
  bool use_random_seed_file;
  int image_size_file;
  data_ifs.read((char *)(&version_id_file), sizeof(version_id_file));
  data_ifs.read((char *)(&sphere_radius_file), sizeof(sphere_radius_file));
  data_ifs.read((char *)(&n_divides_file), sizeof(n_divides_file));
  data_ifs.read((char *)(&n_points_file), sizeof(n_points_file));
  data_ifs.read((char *)(&use_random_seed_file), sizeof(use_random_seed_file));
  data_ifs.read((char *)(&image_size_file), sizeof(image_size_file));
  if (version_id_file != kVersionID || sphere_radius_file != sphere_radius_ ||
      n_divides_file != n_divides_ || n_points_file < n_points_ ||
      use_random_seed_file != use_random_seed_ ||
      image_size_file != image_size_) {
    std::cout << "Model file " << data_path
              << " was generated using different model parameters" << std::endl;
    return false;
  }

  // Check if model file has correct geometry data
  std::string geometry_path_string;
  std::string::size_type geometry_path_length;
  float geometry_unit_in_meter;
  bool geometry_counterclockwise;
  bool geometry_enable_culling;
  float maximum_body_diameter;
  Transform3fA geometry2body_pose;
  data_ifs.read((char *)(&geometry_path_length), sizeof(geometry_path_length));
  geometry_path_string.resize(geometry_path_length);
  data_ifs.read((char *)(geometry_path_string.data()), geometry_path_length);
  data_ifs.read((char *)(&geometry_unit_in_meter),
                sizeof(geometry_unit_in_meter));
  data_ifs.read((char *)(&geometry_counterclockwise),
                sizeof(geometry_counterclockwise));
  data_ifs.read((char *)(&geometry_enable_culling),
                sizeof(geometry_enable_culling));
  data_ifs.read((char *)(&maximum_body_diameter),
                sizeof(maximum_body_diameter));
  data_ifs.read((char *)(geometry2body_pose.data()),
                sizeof(geometry2body_pose));
  if (geometry_path_string != body_ptr_->geometry_path() ||
      geometry_unit_in_meter != body_ptr_->geometry_unit_in_meter() ||
      geometry_counterclockwise != body_ptr_->geometry_counterclockwise() ||
      geometry_enable_culling != body_ptr_->geometry_enable_culling() ||
      maximum_body_diameter != body_ptr_->maximum_body_diameter() ||
      geometry2body_pose.matrix() != body_ptr_->geometry2body_pose().matrix()) {
    std::cout << "Model file " << data_path
              << " was generated using different body parameters" << std::endl;
    return false;
  }

  // Load template view data
  size_t n_template_views;
  data_ifs.read((char *)(&n_template_views), sizeof(n_template_views));
  template_views_.clear();
  template_views_.reserve(n_template_views);
  for (size_t i = 0; i < n_template_views; i++) {
    TemplateView tv;
    tv.data_points.resize(n_points_);
    data_ifs.read((char *)(tv.data_points.data()),
                  n_points_ * sizeof(PointData));
    data_ifs.read((char *)(tv.orientation.data()), sizeof(tv.orientation));
    template_views_.push_back(std::move(tv));
  }
  data_ifs.close();
  return true;
}

bool Model::SaveModel() const {
  std::filesystem::path data_path{directory_ / filename_};
  std::ofstream data_ofs{data_path, std::ios::out | std::ios::binary};

  // Save template parameters
  size_t n_template_views = template_views_.size();
  data_ofs.write((const char *)(&kVersionID), sizeof(kVersionID));
  data_ofs.write((const char *)(&sphere_radius_), sizeof(sphere_radius_));
  data_ofs.write((const char *)(&n_divides_), sizeof(n_divides_));
  data_ofs.write((const char *)(&n_points_), sizeof(n_points_));
  data_ofs.write((const char *)(&use_random_seed_), sizeof(use_random_seed_));
  data_ofs.write((const char *)(&image_size_), sizeof(image_size_));

  // Save geometry data
  std::string geometry_path_string = body_ptr_->geometry_path().string();
  std::string::size_type geometry_path_length = geometry_path_string.length();
  float geometry_unit_in_meter = body_ptr_->geometry_unit_in_meter();
  bool geometry_counterclockwise = body_ptr_->geometry_counterclockwise();
  bool geometry_enable_culling = body_ptr_->geometry_enable_culling();
  float maximum_body_diameter = body_ptr_->maximum_body_diameter();
  Transform3fA geometry2body_pose = body_ptr_->geometry2body_pose();
  data_ofs.write((const char *)(&geometry_path_length),
                 sizeof(geometry_path_length));
  data_ofs.write((const char *)(geometry_path_string.data()),
                 geometry_path_length);
  data_ofs.write((const char *)(&geometry_unit_in_meter),
                 sizeof(geometry_unit_in_meter));
  data_ofs.write((const char *)(&geometry_counterclockwise),
                 sizeof(geometry_counterclockwise));
  data_ofs.write((const char *)(&geometry_enable_culling),
                 sizeof(geometry_enable_culling));
  data_ofs.write((const char *)(&maximum_body_diameter),
                 sizeof(maximum_body_diameter));
  data_ofs.write((const char *)(geometry2body_pose.data()),
                 sizeof(geometry2body_pose));

  // Save main data
  data_ofs.write((const char *)(&n_template_views), sizeof(n_template_views));
  for (const auto &tv : template_views_) {
    data_ofs.write((const char *)(tv.data_points.data()),
                   n_points_ * sizeof(PointData));
    data_ofs.write((const char *)(tv.orientation.data()),
                   sizeof(tv.orientation));
  }
  data_ofs.flush();
  data_ofs.close();
  return true;
}

bool Model::GeneratePointData(const NormalRenderer &renderer,
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
  std::mt19937 generator{7};
  if (use_random_seed_)
    generator.seed(
        unsigned(std::chrono::system_clock::now().time_since_epoch().count()));

  // Calculate data for contour points
  for (auto data_point{begin(*data_points)}; data_point != end(*data_points);) {
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
    float pixel_to_meter = center_f_camera(2) / renderer.intrinsics().fu;
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
  contours->erase(std::remove_if(begin(*contours), end(*contours),
                                 [](const std::vector<cv::Point2i> &contour) {
                                   return contour.size() < kMinContourLength;
                                 }),
                  end(*contours));

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
    *total_contour_length_in_pixel += int(contour.size());
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
      idx -= int(contour.size());
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

Eigen::Vector2f Model::ApproximateNormalVector(
    const std::vector<cv::Point2i> &contour_segment) {
  return Eigen::Vector2f{
      -float(contour_segment.back().y - contour_segment.front().y),
      float(contour_segment.back().x - contour_segment.front().x)}
      .normalized();
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

bool Model::SetUpRenderer(
    const std::shared_ptr<RendererGeometry> &renderer_geometry_ptr,
    std::shared_ptr<NormalRenderer> *renderer_ptr) const {
  // Set up renderer geometry
  auto copied_body_ptr{std::make_shared<Body>(*body_ptr_)};
  copied_body_ptr->set_body2world_pose(Transform3fA::Identity());
  if (!renderer_geometry_ptr->AddBody(copied_body_ptr)) return false;

  // Calculate parameters
  float focal_length = float(image_size_ - kImageSizeSafetyBoundary) *
                       sphere_radius_ / body_ptr_->maximum_body_diameter();
  float principal_point = float(image_size_) / 2.0f;
  Intrinsics intrinsics{focal_length,    focal_length, principal_point,
                        principal_point, image_size_,  image_size_};
  float z_min = sphere_radius_ - body_ptr_->maximum_body_diameter() * 0.5f;
  float z_max = sphere_radius_ + body_ptr_->maximum_body_diameter() * 0.5f;

  // Set up renderer
  *renderer_ptr = std::make_shared<NormalRenderer>(
      "renderer", renderer_geometry_ptr, Transform3fA::Identity(), intrinsics,
      z_min, z_max);
  return (*renderer_ptr)->SetUp();
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
      {-x, 0.0f, z}, {x, 0.0f, z},  {-x, 0.0f, -z}, {x, 0.0f, -z},
      {0.0f, z, x},  {0.0f, z, -x}, {0.0f, -z, x},  {0.0f, -z, -x},
      {z, x, 0.0f},  {-z, x, 0.0f}, {z, -x, 0.0f},  {-z, -x, 0.0f}};
  std::vector<std::array<int, 3>> icosahedron_ids{
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

}  // namespace srt3d
