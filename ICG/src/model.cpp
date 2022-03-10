// SPDX-License-Identifier: MIT
// Copyright (c) 2022 Manuel Stoiber, German Aerospace Center (DLR)

#include <icg/model.h>

namespace icg {

void Model::set_name(const std::string &name) { name_ = name; }

void Model::set_metafile_path(const std::filesystem::path &metafile_path) {
  metafile_path_ = metafile_path;
  set_up_ = false;
}

void Model::set_body_ptr(const std::shared_ptr<Body> &body_ptr) {
  body_ptr_ = body_ptr;
  set_up_ = false;
}

void Model::set_model_path(const std::filesystem::path &model_path) {
  model_path_ = model_path;
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

void Model::set_max_radius_depth_offset(float max_radius_depth_offset) {
  max_radius_depth_offset_ = max_radius_depth_offset;
  set_up_ = false;
}

void Model::set_stride_depth_offset(float stride_depth_offset) {
  stride_depth_offset_ = stride_depth_offset;
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

const std::string &Model::name() const { return name_; }

const std::filesystem::path &Model::metafile_path() const {
  return metafile_path_;
}

const std::shared_ptr<Body> &Model::body_ptr() const { return body_ptr_; }

const std::filesystem::path &Model::model_path() const { return model_path_; }

float Model::sphere_radius() const { return sphere_radius_; }

int Model::n_divides() const { return n_divides_; }

int Model::n_points() const { return n_points_; }

float Model::max_radius_depth_offset() const {
  return max_radius_depth_offset_;
}

float Model::stride_depth_offset() const { return stride_depth_offset_; }

bool Model::use_random_seed() const { return use_random_seed_; }

int Model::image_size() const { return image_size_; }

bool Model::set_up() const { return set_up_; }

Model::Model(const std::string &name, const std::shared_ptr<Body> &body_ptr,
             const std::filesystem::path &model_path, float sphere_radius,
             int n_divides, int n_points, float max_radius_depth_offset,
             float stride_depth_offset, bool use_random_seed, int image_size)
    : name_{name},
      body_ptr_{body_ptr},
      model_path_{model_path},
      sphere_radius_{sphere_radius},
      n_divides_{n_divides},
      n_points_{n_points},
      max_radius_depth_offset_{max_radius_depth_offset},
      stride_depth_offset_{stride_depth_offset},
      use_random_seed_{use_random_seed},
      image_size_{image_size} {}

Model::Model(const std::string &name,
             const std::filesystem::path &metafile_path,
             const std::shared_ptr<Body> &body_ptr)
    : name_{name}, metafile_path_{metafile_path}, body_ptr_{body_ptr} {}

bool Model::SetUpRenderer(
    const std::shared_ptr<RendererGeometry> &renderer_geometry_ptr,
    std::shared_ptr<FullNormalRenderer> *renderer_ptr) const {
  // Set up renderer geometry
  auto copied_body_ptr{std::make_shared<Body>(*body_ptr_)};
  copied_body_ptr->set_body2world_pose(Transform3fA::Identity());
  if (!renderer_geometry_ptr->AddBody(copied_body_ptr)) return false;

  // Calculate parameters
  float focal_length =
      float(image_size_ - kImageSizeSafetyBoundary) /
      tanf(asinf(body_ptr_->maximum_body_diameter() / sphere_radius_));
  float principal_point = float(image_size_) / 2.0f;
  Intrinsics intrinsics{focal_length,    focal_length, principal_point,
                        principal_point, image_size_,  image_size_};
  float z_min = sphere_radius_ - body_ptr_->maximum_body_diameter() * 0.5f;
  float z_max = sphere_radius_ + body_ptr_->maximum_body_diameter() * 0.5f;

  // Set up renderer
  *renderer_ptr = std::make_shared<FullNormalRenderer>(
      "renderer", renderer_geometry_ptr, Transform3fA::Identity(), intrinsics,
      z_min, z_max);
  return (*renderer_ptr)->SetUp();
}

bool Model::LoadModelParameters(int version_id, char model_type,
                                std::ifstream *ifs) {
  char model_type_file;
  ifs->read((char *)(&model_type_file), sizeof(model_type_file));
  if (model_type_file != model_type) {
    std::cerr << "Wrong model type" << std::endl;
    return false;
  }

  int version_id_file;
  ifs->read((char *)(&version_id_file), sizeof(version_id_file));
  if (version_id_file != version_id) {
    std::cerr << "Wrong version id" << std::endl;
    return false;
  }

  float sphere_radius_file;
  int n_divides_file;
  int n_points_file;
  float max_radius_depth_offset;
  float stride_depth_offset;
  bool use_random_seed_file;
  int image_size_file;
  ifs->read((char *)(&sphere_radius_file), sizeof(sphere_radius_file));
  ifs->read((char *)(&n_divides_file), sizeof(n_divides_file));
  ifs->read((char *)(&n_points_file), sizeof(n_points_file));
  ifs->read((char *)(&max_radius_depth_offset),
            sizeof(max_radius_depth_offset));
  ifs->read((char *)(&stride_depth_offset), sizeof(stride_depth_offset));
  ifs->read((char *)(&use_random_seed_file), sizeof(use_random_seed_file));
  ifs->read((char *)(&image_size_file), sizeof(image_size_file));
  return sphere_radius_file == sphere_radius_ && n_divides_file == n_divides_ &&
         n_points_file >= n_points_ &&
         max_radius_depth_offset == max_radius_depth_offset_ &&
         stride_depth_offset == stride_depth_offset_ &&
         use_random_seed_file == use_random_seed_ &&
         image_size_file == image_size_;
}

bool Model::LoadBodyData(std::ifstream *ifs) {
  // Check if model file has correct geometry data
  std::string geometry_path_string;
  std::string::size_type geometry_path_length;
  float geometry_unit_in_meter;
  bool geometry_counterclockwise;
  bool geometry_enable_culling;
  float maximum_body_diameter;
  Transform3fA geometry2body_pose;
  ifs->read((char *)(&geometry_path_length), sizeof(geometry_path_length));
  geometry_path_string.resize(geometry_path_length);
  ifs->read((char *)(geometry_path_string.data()), geometry_path_length);
  ifs->read((char *)(&geometry_unit_in_meter), sizeof(geometry_unit_in_meter));
  ifs->read((char *)(&geometry_counterclockwise),
            sizeof(geometry_counterclockwise));
  ifs->read((char *)(&geometry_enable_culling),
            sizeof(geometry_enable_culling));
  ifs->read((char *)(&maximum_body_diameter), sizeof(maximum_body_diameter));
  ifs->read((char *)(geometry2body_pose.data()), sizeof(geometry2body_pose));
  return std::filesystem::equivalent(
             std::filesystem::path{geometry_path_string},
             body_ptr_->geometry_path()) &&
         geometry_unit_in_meter == body_ptr_->geometry_unit_in_meter() &&
         geometry_counterclockwise == body_ptr_->geometry_counterclockwise() &&
         geometry_enable_culling == body_ptr_->geometry_enable_culling() &&
         maximum_body_diameter == body_ptr_->maximum_body_diameter() &&
         geometry2body_pose.matrix() ==
             body_ptr_->geometry2body_pose().matrix();
}

void Model::SaveModelParameters(int version_id, char model_type,
                                std::ofstream *ofs) const {
  ofs->write((const char *)(&model_type), sizeof(model_type));
  ofs->write((const char *)(&version_id), sizeof(version_id));
  ofs->write((const char *)(&sphere_radius_), sizeof(sphere_radius_));
  ofs->write((const char *)(&n_divides_), sizeof(n_divides_));
  ofs->write((const char *)(&n_points_), sizeof(n_points_));
  ofs->write((const char *)(&max_radius_depth_offset_),
             sizeof(max_radius_depth_offset_));
  ofs->write((const char *)(&stride_depth_offset_),
             sizeof(stride_depth_offset_));
  ofs->write((const char *)(&use_random_seed_), sizeof(use_random_seed_));
  ofs->write((const char *)(&image_size_), sizeof(image_size_));
}

void Model::SaveBodyData(std::ofstream *ofs) const {
  std::string geometry_path_string = body_ptr_->geometry_path().string();
  std::string::size_type geometry_path_length = geometry_path_string.length();
  float geometry_unit_in_meter = body_ptr_->geometry_unit_in_meter();
  bool geometry_counterclockwise = body_ptr_->geometry_counterclockwise();
  bool geometry_enable_culling = body_ptr_->geometry_enable_culling();
  float maximum_body_diameter = body_ptr_->maximum_body_diameter();
  Transform3fA geometry2body_pose = body_ptr_->geometry2body_pose();
  ofs->write((const char *)(&geometry_path_length),
             sizeof(geometry_path_length));
  ofs->write((const char *)(geometry_path_string.data()), geometry_path_length);
  ofs->write((const char *)(&geometry_unit_in_meter),
             sizeof(geometry_unit_in_meter));
  ofs->write((const char *)(&geometry_counterclockwise),
             sizeof(geometry_counterclockwise));
  ofs->write((const char *)(&geometry_enable_culling),
             sizeof(geometry_enable_culling));
  ofs->write((const char *)(&maximum_body_diameter),
             sizeof(maximum_body_diameter));
  ofs->write((const char *)(geometry2body_pose.data()),
             sizeof(geometry2body_pose));
}

bool Model::DepthOffsetVariablesValid() const {
  int n_values = int(max_radius_depth_offset_ / stride_depth_offset_ + 1.0f);
  if (n_values > kMaxNDepthOffsets) {
    std::cerr
        << "Array for depth_offsets has size " << kMaxNDepthOffsets
        << ". It is not possible to store " << n_values << " values."
        << "Reduce ratio: max_radius_depth_offset / stride_depth_offset to "
        << kMaxNDepthOffsets << std::endl;
    return false;
  }
  return true;
}

void Model::CalculateDepthOffsets(
    const FullNormalRenderer &renderer, const cv::Point2i &center,
    float pixel_to_meter,
    std::array<float, kMaxNDepthOffsets> *depth_offsets) const {
  // Precalculate variables in pixel coordinates
  int n_values = int(max_radius_depth_offset_ / stride_depth_offset_ + 1.0f);
  float stride = stride_depth_offset_ / pixel_to_meter;
  float max_diameter = 2.0f * n_values * stride;

  // Precalculate rounded variables to iterate over image
  int image_stride = int(stride + 1.0f);
  int n_image_strides = int(max_diameter / image_stride + 1.0f);
  int image_diameter = n_image_strides * image_stride;
  int image_radius_minus = image_diameter / 2;
  int image_radius_plus = image_diameter - image_radius_minus;

  // Calculate limits for iteration
  int v_min = std::max(center.y - image_radius_minus, 0);
  int v_max = std::min(center.y + image_radius_plus, image_size_ - 1);
  int u_min = std::max(center.x - image_radius_minus, 0);
  int u_max = std::min(center.x + image_radius_plus, image_size_ - 1);

  // Iterate image to find minimum values corresponding to a certain radius
  int v, u, i;
  float distance;
  const cv::Mat &image{renderer.depth_image()};
  const ushort *ptr_image;
  std::vector<ushort> min_values(kMaxNDepthOffsets,
                                 std::numeric_limits<ushort>::max());
  min_values[0] = image.at<ushort>(center);
  for (v = v_min; v <= v_max; v += image_stride) {
    ptr_image = image.ptr<ushort>(v);
    for (u = u_min; u <= u_max; u += image_stride) {
      distance = std::sqrt(square(u - center.x) + square(v - center.y));
      i = int(distance / stride);
      if (i < n_values) min_values[i] = std::min(min_values[i], ptr_image[u]);
    }
  }

  // Accumulate minimum values for circular regions and calculate depth offset
  float depth_center = renderer.Depth(center);
  (*depth_offsets)[0] = depth_center - renderer.Depth(min_values[0]);
  for (size_t i = 1; i < kMaxNDepthOffsets; ++i) {
    min_values[i] = std::min(min_values[i], min_values[i - 1]);
    (*depth_offsets)[i] = depth_center - renderer.Depth(min_values[i]);
  }
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
    if (geodesic_point[0] == 0.0f && geodesic_point[2] == 0.0f) {
      Rotation.col(0) = Eigen::Vector3f(1, 0, 0);
    } else {
      Rotation.col(0) = downwards.cross(-geodesic_point).normalized();
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

}  // namespace icg
