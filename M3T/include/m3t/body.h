// SPDX-License-Identifier: MIT
// Copyright (c) 2023 Manuel Stoiber, German Aerospace Center (DLR)

#ifndef M3T_INCLUDE_M3T_BODY_H_
#define M3T_INCLUDE_M3T_BODY_H_

#include <m3t/common.h>

#include <filesystem/filesystem.h>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <array>
#include <atomic>
#include <iostream>
#include <string>
#include <vector>

namespace m3t {

/**
 * \brief Contains the geometry of a rigid body and stores the pose of the body
 * relative to a world coordinate frame.
 *
 * \details Calling the `SetUp()` method loads `mesh_indices` and `vertices`
 * from a wavefront obj file and automatically computes the
 * `maximum_body_diameter`.
 *
 * @param geometry_path path to wavefront obj file. Using `INFER_FROM_NAME` in
 * the metafile sets the path to `<name>.obj`.
 * @param geometry_unit_in_meter scale factor to scale the unit used in the
 * wavefront obj file to meter.
 * @param geometry_counterclockwise true if winding order of triangles in
 * wavefront obj is defined counter-clockwise.
 * @param geometry_enable_culling true if faces that are not facing towards the
 * camera should be culled.
 * @param geometry2body_pose transformation that allows setting a different
 * frame of reference for the object than defined by the wavefront obj file.
 * Should be used to center objects.
 * @param body_id value that is assigned to pixels by the \ref
 * FullSilhouetteRenderer or \ref FocusedSilhouetteRenderer is the \ref IDType
 * BODY is selected. During construction, a unique id is assigned.
 * @param region_id value that is assigned to pixels by the \ref
 * FullSilhouetteRenderer or \ref FocusedSilhouetteRenderer is the \ref IDType
 * REGION is selected. During construction, a unique id is assigned.
 */
class Body {
 public:
  // Constructors and initialization methods
  Body(const std::string &name, const std::filesystem::path &geometry_path,
       float geometry_unit_in_meter, bool geometry_counterclockwise,
       bool geometry_enable_culling, const Transform3fA &geometry2body_pose);
  Body(const std::string &name, const std::filesystem::path &metafile_path);
  bool SetUp();

  // Geometry setters
  void set_name(const std::string &name);
  void set_metafile_path(const std::filesystem::path &metafile_path);
  void set_geometry_path(const std::filesystem::path &geometry_path);
  void set_geometry_unit_in_meter(float geometry_unit_in_meter);
  void set_geometry_counterclockwise(bool geometry_counterclockwise);
  void set_geometry_enable_culling(bool geometry_enable_culling);
  void set_geometry2body_pose(const Transform3fA &geometry2body_pose);

  // ID setters
  void set_id(IDType id_type, uchar id);
  void set_body_id(uchar body_id);
  void set_region_id(uchar region_id);

  // Pose setters
  void set_body2world_pose(const Transform3fA &body2world_pose);
  void set_world2body_pose(const Transform3fA &world2body_pose);

  // Geometry getters
  const std::string &name() const;
  const std::filesystem::path &metafile_path() const;
  const std::filesystem::path &geometry_path() const;
  float geometry_unit_in_meter() const;
  bool geometry_counterclockwise() const;
  bool geometry_enable_culling() const;
  const Transform3fA &geometry2body_pose() const;

  // ID getters
  uchar get_id(IDType id_type) const;
  uchar body_id() const;
  uchar region_id() const;

  // Pose getters
  const Transform3fA &body2world_pose() const;
  const Transform3fA &world2body_pose() const;
  const Transform3fA &geometry2world_pose() const;
  const Transform3fA &world2geometry_pose() const;

  // Calculated value getters
  const std::vector<std::array<int, 3>> &mesh_indices() const;
  const std::vector<Eigen::Vector3f> &vertices() const;
  float maximum_body_diameter() const;

  // Internal state
  bool set_up() const;

 private:
  // Helper methods
  bool LoadMetaData();
  bool LoadMeshData();
  bool CalculateMaximumBodyDiameter();

  // Geometry data
  std::string name_{};
  std::filesystem::path metafile_path_{};
  std::filesystem::path geometry_path_{};
  float geometry_unit_in_meter_ = 1.0f;
  bool geometry_counterclockwise_ = true;
  bool geometry_enable_culling_ = true;
  Transform3fA geometry2body_pose_{Transform3fA::Identity()};

  // ID data
  uchar body_id_ = 0;
  uchar region_id_ = 0;

  // Pose data
  Transform3fA body2world_pose_{Eigen::Translation3f{0.0f, 0.0f, -10.0f}};
  Transform3fA world2body_pose_{Eigen::Translation3f{0.0f, 0.0f, 10.0f}};
  Transform3fA geometry2world_pose_{Eigen::Translation3f{0.0f, 0.0f, -10.0f}};
  Transform3fA world2geometry_pose_{Eigen::Translation3f{0.0f, 0.0f, 10.0f}};

  // Calculated values
  std::vector<std::array<int, 3>> mesh_indices_{};
  std::vector<Eigen::Vector3f> vertices_{};
  float maximum_body_diameter_ = 0.0f;

  // Internal state
  static std::atomic<uchar> next_id_;
  bool set_up_ = false;
};

}  // namespace m3t

#endif  // M3T_INCLUDE_M3T_BODY_H_
