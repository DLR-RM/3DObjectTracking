// SPDX-License-Identifier: MIT
// Copyright (c) 2023 Manuel Stoiber, German Aerospace Center (DLR)

#ifndef M3T_INCLUDE_M3T_SOFT_CONSTRAINT_H_
#define M3T_INCLUDE_M3T_SOFT_CONSTRAINT_H_

#include <m3t/body.h>
#include <m3t/common.h>
#include <m3t/link.h>

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <iostream>
#include <string>
#include <vector>

namespace m3t {

/**
 * \brief Class that takes two \ref Link objects, defines the location and
 * constraint directions of a joint, and specifies standard deviations and
 * maximum allowed distances to calculate the gradient vectors and Hessian
 * matrices that act on each referenced link.
 *
 * The method `AddGradientsAndHessiansToLinks()` computes the link specific
 * gradient vectors and Hessian matrices of the constraint and adds them to the
 * gradient vectors and hessian matrices of the referenced links.
 *
 * @param link1_ptr defines the first of the two referenced \ref Link objects
 * that are considered.
 * @param link2_ptr defines the second of the two referenced \ref Link objects
 * that are considered.
 * @param body12joint1_pose pose of the first joint frame relative to the \ref
 * Body of the first link.
 * @param body22joint2_pose pose of the second joint frame relative to the \ref
 * Body of the second link.
 * @param constraint_directions defines which of the 6 joint directions are
 * constrained. The first three directions consider the rotation around x, y,
 * and z, while the second three parameters consider the x, y, and z
 * translation.
 * @param max_distance_rotation maximum valid distance in radians for which the
 * constraint is not active.
 * @param max_distance_translation maximum valid distance in meter for which the
 * constraint is not active.
 * @param standard_deviation_rotation standard deviation for the rotational
 * constraint in rad.
 * @param standard_deviation_translation standard deviation for the
 * translational constraint in meter.
 */
class SoftConstraint {
 public:
  // Constructors and setup methods
  SoftConstraint(
      const std::string &name, const std::shared_ptr<Link> &link1_ptr,
      const std::shared_ptr<Link> &link2_ptr,
      const Transform3fA &body12joint1_pose = Transform3fA::Identity(),
      const Transform3fA &body22joint2_pose = Transform3fA::Identity(),
      const std::array<bool, 6> &constraint_directions = {false, false, false,
                                                          false, false, false},
      float max_distance_rotation = 0.0f, float max_distance_translation = 0.0f,
      float standard_deviation_rotation = 0.01f,
      float standard_deviation_translation = 0.001f);
  SoftConstraint(const std::string &name,
                 const std::filesystem::path &metafile_path,
                 const std::shared_ptr<Link> &link1_ptr,
                 const std::shared_ptr<Link> &link2_ptr);
  bool SetUp();

  // Setters
  void set_name(const std::string &name);
  void set_metafile_path(const std::filesystem::path &metafile_path);
  void set_link1_ptr(const std::shared_ptr<Link> &link1_ptr);
  void set_link2_ptr(const std::shared_ptr<Link> &link2_ptr);
  void set_body12joint1_pose(const Transform3fA &body12joint1_pose);
  void set_body22joint2_pose(const Transform3fA &body22joint2_pose);
  void set_constraint_directions(
      const std::array<bool, 6> &constraint_directions);
  void set_max_distance_rotation(float max_distance_rotation);
  void set_max_distance_translation(float max_distance_translation);
  void set_standard_deviation_rotation(float standard_deviation_rotation);
  void set_standard_deviation_translation(float standard_deviation_translation);

  // Main methods
  bool AddGradientsAndHessiansToLinks();

  // Getters
  const std::string &name() const;
  const std::filesystem::path &metafile_path() const;
  const std::shared_ptr<Link> &link1_ptr() const;
  const std::shared_ptr<Link> &link2_ptr() const;
  const Transform3fA &body12joint1_pose() const;
  const Transform3fA &body22joint2_pose() const;
  const std::array<bool, 6> &constraint_directions() const;
  float max_distance_rotation() const;
  float max_distance_translation() const;
  float standard_deviation_rotation() const;
  float standard_deviation_translation() const;
  bool set_up() const;

 private:
  // Helper methods
  bool LoadMetaData();
  void PrecomputeInternalVariables();
  void AddGradientsAndHessiansToLink(
      const Transform3fA &joint22joint1_pose,
      const Transform3fA &body2joint1_pose, float sign,
      const std::shared_ptr<Link> &link_ptr) const;
  Eigen::VectorXf ConsideredRotationVector(
      const Transform3fA &joint22joint1_pose) const;
  Eigen::VectorXf ConsideredTranslationVector(
      const Transform3fA &joint22joint1_pose) const;
  Eigen::MatrixXf UnprojectedConstraintJacobianRotation(
      const Transform3fA &joint22joint1_pose,
      const Transform3fA &body2joint1_pose) const;
  Eigen::MatrixXf UnprojectedConstraintJacobianTranslation(
      const Transform3fA &joint22joint1_pose,
      const Transform3fA &body2joint1_pose) const;

  // Internal data
  int n_constraints_rotation_ = 0;
  int n_constraints_translation_ = 0;

  // Parameters and state variable
  std::string name_{};
  std::filesystem::path metafile_path_{};
  std::shared_ptr<Link> link1_ptr_{};
  std::shared_ptr<Link> link2_ptr_{};
  Transform3fA body12joint1_pose_{Transform3fA::Identity()};
  Transform3fA body22joint2_pose_{Transform3fA::Identity()};
  std::array<bool, 6> constraint_directions_{false, false, false,
                                             false, false, false};
  float max_distance_rotation_ = 0.0f;
  float max_distance_translation_ = 0.0f;
  float standard_deviation_rotation_ = 0.01f;
  float standard_deviation_translation_ = 0.001f;
  bool set_up_ = false;
};

}  // namespace m3t

#endif  // M3T_INCLUDE_M3T_SOFT_CONSTRAINT_H_
