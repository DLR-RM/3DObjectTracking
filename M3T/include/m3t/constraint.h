// SPDX-License-Identifier: MIT
// Copyright (c) 2023 Manuel Stoiber, German Aerospace Center (DLR)

#ifndef M3T_INCLUDE_M3T_CONSTRAINT_H_
#define M3T_INCLUDE_M3T_CONSTRAINT_H_

#include <m3t/body.h>
#include <m3t/common.h>
#include <m3t/link.h>
#include <m3t/modality.h>

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <iostream>
#include <string>
#include <vector>

namespace m3t {

/**
 * \brief Class that takes two \ref Link objects, defines the location as well
 * as constraint directions of a joint, and calculates the residual and
 * constraint Jacobian for the defined data.
 *
 * The method `CalculateResidualAndConstraintJacobian()` computes the residual
 * for constraint directions. In addition, it computes the constraint Jacobian
 * with respect to the degrees of freedom of the kinematic structure, using the
 * Jacobian matrices from both referenced \ref Link objects.
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
 */
class Constraint {
 private:
  static constexpr bool kOrthogonalityConstraintExperiment = false;
  static constexpr bool kDoubleConstraintRotation = false;

 public:
  // Constructors and setup methods
  Constraint(const std::string &name, const std::shared_ptr<Link> &link1_ptr,
             const std::shared_ptr<Link> &link2_ptr,
             const Transform3fA &body12joint1_pose = Transform3fA::Identity(),
             const Transform3fA &body22joint2_pose = Transform3fA::Identity(),
             const std::array<bool, 6> &constraint_directions = {
                 false, false, false, false, false, false});
  Constraint(const std::string &name,
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

  // Main methods
  bool CalculateResidualAndConstraintJacobian();

  // Additional methods
  int NumberOfConstraints();

  // Getters residual and constraint Jacobian
  const Eigen::VectorXf &residual();
  const Eigen::MatrixXf &constraint_jacobian();

  // Getters
  const std::string &name() const;
  const std::filesystem::path &metafile_path() const;
  const std::shared_ptr<Link> &link1_ptr() const;
  const std::shared_ptr<Link> &link2_ptr() const;
  const Transform3fA &body12joint1_pose() const;
  const Transform3fA &body22joint2_pose() const;
  const std::array<bool, 6> &constraint_directions() const;
  bool set_up() const;

 private:
  // Helper methods
  bool LoadMetaData();
  Eigen::VectorXf Residual(const Transform3fA &joint22joint1_pose);
  Eigen::MatrixXf UnprojectedConstraintJacobian(
      const Transform3fA &joint22joint1_pose,
      const Transform3fA &body2joint1_pose);

  // Internal data
  Eigen::VectorXf residual_{};
  Eigen::MatrixXf constraint_jacobian_{};

  // Parameters and state variable
  std::string name_{};
  std::filesystem::path metafile_path_{};
  std::shared_ptr<Link> link1_ptr_{};
  std::shared_ptr<Link> link2_ptr_{};
  Transform3fA body12joint1_pose_{Transform3fA::Identity()};
  Transform3fA body22joint2_pose_{Transform3fA::Identity()};
  std::array<bool, 6> constraint_directions_{false, false, false,
                                             false, false, false};
  bool set_up_ = false;
};

}  // namespace m3t

#endif  // M3T_INCLUDE_M3T_CONSTRAINT_H_
