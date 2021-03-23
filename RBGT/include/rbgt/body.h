// SPDX-License-Identifier: MIT
// Copyright (c) 2020 Manuel Stoiber, German Aerospace Center (DLR)

#ifndef OBJECT_TRACKING_INCLUDE_RBGT_BODY_H_
#define OBJECT_TRACKING_INCLUDE_RBGT_BODY_H_

#include <rbgt/common.h>

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <filesystem>
#include <iostream>
#include <string>
#include <vector>

namespace rbgt {

// Class that holds all body information such as name, geometry data, pose,
// and occlusion ids
class Body {
 public:
  // Constructors
  Body(std::string name, std::filesystem::path geometry_path,
       float geometry_unit_in_meter, bool geometry_counterclockwise,
       bool geometry_enable_culling, float maximum_body_diameter);
  Body(std::string name, std::filesystem::path geometry_path,
       float geometry_unit_in_meter, bool geometry_counterclockwise,
       bool geometry_enable_culling, float maximum_body_diameter,
       const Transform3fA &geometry2body_pose);

  // Geometry setters
  void set_name(const std::string &name);
  void set_geometry_path(const std::filesystem::path &geometry_path);
  void set_geometry_unit_in_meter(float geometry_unit_in_meter);
  void set_geometry_counterclockwise(bool geometry_counterclockwise);
  void set_geometry_enable_culling(bool geometry_enable_culling);
  void set_maximum_body_diameter(float maximum_body_diameter);
  void set_geometry2body_pose(const Transform3fA &geometry2body_pose);
  bool set_occlusion_mask_id(int occlusion_mask_id);

  // Pose setters
  void set_body2world_pose(const Transform3fA &body2world_pose);
  void set_world2body_pose(const Transform3fA &world2body_pose);

  // Geometry getters
  const std::string &name() const;
  const std::filesystem::path &geometry_path() const;
  float geometry_unit_in_meter() const;
  bool geometry_counterclockwise() const;
  bool geometry_enable_culling() const;
  float maximum_body_diameter() const;
  const Transform3fA &geometry2body_pose() const;
  int occlusion_mask_id() const;

  // Pose getters
  const Transform3fA &body2world_pose() const;
  const Transform3fA &world2body_pose() const;
  const Transform3fA &geometry2world_pose() const;
  const Transform3fA &world2geometry_pose() const;

 private:
  // Geometry data
  std::string name_{};
  std::filesystem::path geometry_path_{};
  float geometry_unit_in_meter_ = 1.0f;
  bool geometry_counterclockwise_ = true;
  bool geometry_enable_culling_ = true;
  float maximum_body_diameter_ = 0.0f;
  Transform3fA geometry2body_pose_{Transform3fA::Identity()};
  int occlusion_mask_id_ = 0;

  // Pose data
  Transform3fA body2world_pose_{Transform3fA::Identity()};
  Transform3fA world2body_pose_{Transform3fA::Identity()};
  Transform3fA geometry2world_pose_{Transform3fA::Identity()};
  Transform3fA world2geometry_pose_{Transform3fA::Identity()};
};

}  // namespace rbgt

#endif  // OBJECT_TRACKING_INCLUDE_RBGT_BODY_H_
