// SPDX-License-Identifier: MIT
// Copyright (c) 2023 Manuel Stoiber, German Aerospace Center (DLR)

#include <m3t/body.h>

#define TINYOBJLOADER_IMPLEMENTATION
#include <tiny_obj_loader/tiny_obj_loader.h>

namespace m3t {

std::atomic<uchar> Body::next_id_{1};

Body::Body(const std::string &name, const std::filesystem::path &geometry_path,
           float geometry_unit_in_meter, bool geometry_counterclockwise,
           bool geometry_enable_culling, const Transform3fA &geometry2body_pose)
    : name_{name},
      geometry_path_{geometry_path},
      geometry_unit_in_meter_{geometry_unit_in_meter},
      geometry_counterclockwise_{geometry_counterclockwise},
      geometry_enable_culling_{geometry_enable_culling},
      geometry2body_pose_{geometry2body_pose} {
  body_id_ = next_id_++;
  region_id_ = body_id_;
  geometry2world_pose_ = geometry2body_pose;
  world2geometry_pose_ = geometry2world_pose_.inverse();
}

Body::Body(const std::string &name, const std::filesystem::path &metafile_path)
    : name_{name}, metafile_path_{metafile_path} {
  body_id_ = next_id_++;
  region_id_ = body_id_;
}

bool Body::SetUp() {
  set_up_ = false;
  if (!metafile_path_.empty())
    if (!LoadMetaData()) return false;
  if (!LoadMeshData()) return false;
  if (!CalculateMaximumBodyDiameter()) return false;
  set_up_ = true;
  return true;
}

void Body::set_name(const std::string &name) { name_ = name; }

void Body::set_metafile_path(const std::filesystem::path &metafile_path) {
  metafile_path_ = metafile_path;
  set_up_ = false;
}

void Body::set_geometry_path(const std::filesystem::path &geometry_path) {
  geometry_path_ = geometry_path;
  set_up_ = false;
}

void Body::set_geometry_unit_in_meter(float geometry_unit_in_meter) {
  geometry_unit_in_meter_ = geometry_unit_in_meter;
  set_up_ = false;
}

void Body::set_geometry_counterclockwise(bool geometry_counterclockwise) {
  geometry_counterclockwise_ = geometry_counterclockwise;
}

void Body::set_geometry_enable_culling(bool geometry_enable_culling) {
  geometry_enable_culling_ = geometry_enable_culling;
}

void Body::set_geometry2body_pose(const Transform3fA &geometry2body_pose) {
  geometry2body_pose_ = geometry2body_pose;
  geometry2world_pose_ = body2world_pose_ * geometry2body_pose_;
  world2geometry_pose_ = geometry2world_pose_.inverse();
  set_up_ = false;
}

void Body::set_id(IDType id_type, uchar id) {
  if (id_type == IDType::BODY) body_id_ = id;
  if (id_type == IDType::REGION) region_id_ = id;
}

void Body::set_body_id(uchar body_id) { body_id_ = body_id; }

void Body::set_region_id(uchar region_id) { region_id_ = region_id; }

void Body::set_body2world_pose(const Transform3fA &body2world_pose) {
  body2world_pose_ = body2world_pose;
  world2body_pose_ = body2world_pose_.inverse();
  geometry2world_pose_ = body2world_pose_ * geometry2body_pose_;
  world2geometry_pose_ = geometry2world_pose_.inverse();
}

void Body::set_world2body_pose(const Transform3fA &world2body_pose) {
  world2body_pose_ = world2body_pose;
  body2world_pose_ = world2body_pose_.inverse();
  geometry2world_pose_ = body2world_pose_ * geometry2body_pose_;
  world2geometry_pose_ = geometry2world_pose_.inverse();
}

const std::string &Body::name() const { return name_; }

const std::filesystem::path &Body::metafile_path() const {
  return metafile_path_;
}

const std::filesystem::path &Body::geometry_path() const {
  return geometry_path_;
}

float Body::geometry_unit_in_meter() const { return geometry_unit_in_meter_; }

bool Body::geometry_counterclockwise() const {
  return geometry_counterclockwise_;
}

bool Body::geometry_enable_culling() const { return geometry_enable_culling_; }

const Transform3fA &Body::geometry2body_pose() const {
  return geometry2body_pose_;
}

uchar Body::get_id(IDType id_type) const {
  if (id_type == IDType::BODY) return body_id_;
  if (id_type == IDType::REGION) return region_id_;
}

uchar Body::body_id() const { return body_id_; }

uchar Body::region_id() const { return region_id_; }

const Transform3fA &Body::body2world_pose() const { return body2world_pose_; }

const Transform3fA &Body::world2body_pose() const { return world2body_pose_; }

const Transform3fA &Body::geometry2world_pose() const {
  return geometry2world_pose_;
}

const Transform3fA &Body::world2geometry_pose() const {
  return world2geometry_pose_;
}

const std::vector<std::array<int, 3>> &Body::mesh_indices() const {
  return mesh_indices_;
}

const std::vector<Eigen::Vector3f> &Body::vertices() const { return vertices_; }

float Body::maximum_body_diameter() const { return maximum_body_diameter_; }

bool Body::set_up() const { return set_up_; }

bool Body::LoadMetaData() {
  // Open file storage from yaml
  cv::FileStorage fs;
  if (!OpenYamlFileStorage(metafile_path_, &fs)) return false;

  // Read parameters from yaml
  if (!(ReadRequiredValueFromYaml(fs, "geometry_path", &geometry_path_) &&
        ReadRequiredValueFromYaml(fs, "geometry_unit_in_meter",
                                  &geometry_unit_in_meter_) &&
        ReadRequiredValueFromYaml(fs, "geometry_counterclockwise",
                                  &geometry_counterclockwise_) &&
        ReadRequiredValueFromYaml(fs, "geometry_enable_culling",
                                  &geometry_enable_culling_) &&
        ReadRequiredValueFromYaml(fs, "geometry2body_pose",
                                  &geometry2body_pose_))) {
    std::cerr << "Could not read all required body parameters from "
              << metafile_path_ << std::endl;
    return false;
  }
  ReadOptionalValueFromYaml(fs, "body_id", &body_id_);
  ReadOptionalValueFromYaml(fs, "region_id", &region_id_);
  fs.release();

  // Process parameters
  if (geometry_path_ == "INFER_FROM_NAME")
    geometry_path_ = metafile_path_.parent_path() / (name_ + ".obj");
  else if (geometry_path_.is_relative())
    geometry_path_ = metafile_path_.parent_path() / geometry_path_;
  geometry2world_pose_ = body2world_pose_ * geometry2body_pose_;
  world2geometry_pose_ = geometry2world_pose_.inverse();
  return true;
}

bool Body::LoadMeshData() {
  tinyobj::attrib_t attributes;
  std::vector<tinyobj::shape_t> shapes;
  std::vector<tinyobj::material_t> materials;
  std::string warning;
  std::string error;
  if (!tinyobj::LoadObj(&attributes, &shapes, &materials, &warning, &error,
                        geometry_path_.string().c_str(), nullptr, true,
                        false)) {
    std::cerr << "TinyObjLoader failed to load data from " << geometry_path_
              << std::endl;
    return false;
  }
  if (!error.empty()) std::cerr << error << std::endl;

  // Load vertices and scale them if needed
  vertices_.resize(attributes.vertices.size() / 3);
  memcpy(vertices_.data(), attributes.vertices.data(),
         sizeof(float) * attributes.vertices.size());
  if (geometry_unit_in_meter_ != 1.0f) {
    for (auto &vertex : vertices_) {
      vertex *= geometry_unit_in_meter_;
    }
  }

  // Reserve space
  mesh_indices_.clear();
  size_t n_vertices = 0;
  for (const auto &shape : shapes)
    n_vertices += shape.mesh.num_face_vertices.size();
  mesh_indices_.reserve(n_vertices);

  // Load mesh vertices
  for (const auto &shape : shapes) {
    size_t index_offset = 0;
    for (size_t f = 0; f < shape.mesh.num_face_vertices.size(); ++f) {
      if (shape.mesh.num_face_vertices[f] != 3) {
        std::cerr << "Mesh contains non triangle shapes" << std::endl;
        index_offset += shape.mesh.num_face_vertices[f];
        continue;
      }

      if (geometry_counterclockwise_) {
        mesh_indices_.push_back(std::array<int, 3>{
            shape.mesh.indices[index_offset].vertex_index,
            shape.mesh.indices[index_offset + 1].vertex_index,
            shape.mesh.indices[index_offset + 2].vertex_index});
      } else {
        mesh_indices_.push_back(std::array<int, 3>{
            shape.mesh.indices[index_offset + 2].vertex_index,
            shape.mesh.indices[index_offset + 1].vertex_index,
            shape.mesh.indices[index_offset].vertex_index});
      }
      index_offset += 3;
    }
  }
  return true;
}

bool Body::CalculateMaximumBodyDiameter() {
  float max_radius = 0.0f;
  for (const auto &vertex : vertices_) {
    max_radius = std::max(max_radius, (geometry2body_pose_ * vertex).norm());
  }
  maximum_body_diameter_ = 2.0f * max_radius;
  return true;
}

}  // namespace m3t
