// SPDX-License-Identifier: MIT
// Copyright (c) 2023 Manuel Stoiber, German Aerospace Center (DLR)

#ifndef M3T_INCLUDE_M3T_LINK_H_
#define M3T_INCLUDE_M3T_LINK_H_

#include <m3t/body.h>
#include <m3t/common.h>
#include <m3t/modality.h>

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <iostream>
#include <string>
#include <unsupported/Eigen/MatrixFunctions>
#include <vector>

namespace m3t {

/**
 * \brief Class that combines all \ref Modality objects used to track a single
 * \ref Body, that defines the location of a joint to form a tree-like
 * kinematic structure together with other \ref Link objects, and that is used
 * by an \ref Optimizer.
 *
 * \details Before the main methods of the class can be used and after set up,
 * both the `jacobian_size` and `first_jacobian_index` have to be defined using
 * `DefineJacobian`. The value `jacobian_size` thereby provides the size of the
 * Jacobian while the parameter `first_jacobian_index` defines the location of
 * the \ref Link object's free directions within the Jacobian. The method
 * `CalculateJacobian()` calculates the Jacobian of the \ref Link based on the
 * Jacobian of the parent \ref Link. The method `CalculateGradientAndHessian()`
 * combines the gradient vectors and Hessian matrices from all referenced \ref
 * Modality objects. `UpdatePoses()` takes the variation vector for the
 * kinematic structure to update the pose of the joint and the referenced \ref
 * Body. If the \ref Link has no parent, only the pose of the \ref Body is
 * updated. The variation is thereby considered in the joint coordinate frame.
 * If the \ref Link does not reference a \ref Body, the internal
 * `link2world_pose` is updated.
 *
 * @param body_ptr referenced \ref Body object that contains the pose.
 * @param modality_ptrs defines the referenced \ref Modality objects that are
 * considered. At least one modality has to be assigned.
 * @param child_link_ptrs defines the referenced \ref Link objects that are
 * considered children of this \ref Link and that are part of the kinematic
 * structure.
 * @param body2joint_pose pose of the joint relative to the \ref Body. The pose
 * is updated if `fixed_body2joint_pose = false` and a parent \ref Link
 * exists.
 * @param joint2parent_pose pose of the parent \ref Link relative to the joint.
 * The pose is updated if `fixed_body2joint_pose = true` and a parent \ref
 * Link exists.
 * @param link2world_pose pose of the link relative to the world frame. If a
 * `body_ptr` is assigned to the link, `link2world_pose` is equal to the bodies
 * `body2world_pose`.
 * @param free_directions defines which of the 6 joint directions can be
 * variated. The first three directions consider the rotation around x, y, and
 * z, while the second three parameters consider the x, y, and z translation. If
 * the \ref Link has a parent \ref Link, free directions in the joint are
 * variated and the \ref Body pose is updated based on the pose of the parent
 * \ref Body. Otherwise, free directions in the joint are variated and the \ref
 * Body pose is updated based on its previous pose.
 * @param fixed_body2joint_pose if `true` and a parent \ref Link is provided,
 * the `joint2parent_pose` is updated while the `body2joint_pose` remains fixed.
 * If `false` and a parent \ref Link exists, the `body2joint_pose` is updated.
 */
class Link {
 public:
  // Constructors and setup methods
  Link(const std::string &name, const std::shared_ptr<Body> &body_ptr = nullptr,
       const Transform3fA &body2joint_pose = Transform3fA::Identity(),
       const Transform3fA &joint2parent_pose = Transform3fA::Identity(),
       const Transform3fA &link2world_pose = Transform3fA::Identity(),
       const std::array<bool, 6> &free_directions = {true, true, true, true,
                                                     true, true},
       bool fixed_body2joint_pose = true);
  Link(const std::string &name, const std::filesystem::path &metafile_path,
       const std::shared_ptr<Body> &body_ptr = nullptr);
  bool SetUp();

  // Configure modalities and child links
  bool AddModality(const std::shared_ptr<Modality> &modality_ptr);
  bool DeleteModality(const std::string &name);
  void ClearModalities();
  bool AddChildLink(const std::shared_ptr<Link> &child_link_ptr);
  bool DeleteChildLink(const std::string &name);
  void ClearChildLinks();

  // Setters
  void set_name(const std::string &name);
  void set_metafile_path(const std::filesystem::path &metafile_path);
  void set_body_ptr(const std::shared_ptr<Body> &body_ptr);
  void set_body2joint_pose(const Transform3fA &body2joint_pose);
  void set_joint2parent_pose(const Transform3fA &joint2parent_pose);
  void set_link2world_pose(const Transform3fA &link2world_pose);
  void set_free_directions(const std::array<bool, 6> &free_directions);
  void set_fixed_body2joint_pose(bool fixed_body2joint_pose);

  // Main methods
  bool DefineJacobian(int jacobian_size, int first_jacobian_index);
  bool CalculateJacobian(const std::shared_ptr<Link> &parent_link_ptr);
  bool CalculateGradientAndHessian();
  bool AddToGradientAndHessian(
      const Eigen::Matrix<float, 6, 1> &gradient_summand,
      const Eigen::Matrix<float, 6, 6> &hessian_summand);
  bool UpdatePoses(const std::shared_ptr<Link> &parent_link_ptr,
                   Eigen::VectorXf theta);
  void ResetJointPoses();

  // Additional Methods
  int DegreesOfFreedom() const;

  // Getters gradient, Hessian, and Jacobian
  const Eigen::Matrix<float, 6, 1> &gradient();
  const Eigen::Matrix<float, 6, 6> &hessian();
  const Eigen::Matrix<float, 6, Eigen::Dynamic> &jacobian();

  // Getters
  const std::string &name() const;
  const std::filesystem::path &metafile_path() const;
  const std::shared_ptr<Body> &body_ptr() const;
  const std::vector<std::shared_ptr<Modality>> &modality_ptrs() const;
  const std::vector<std::shared_ptr<Link>> &child_link_ptrs() const;
  const Transform3fA &body2joint_pose() const;
  const Transform3fA &joint2parent_pose() const;
  const Transform3fA &link2world_pose() const;
  const std::array<bool, 6> &free_directions() const;
  bool fixed_body2joint_pose() const;
  int first_jacobian_index() const;
  bool set_up() const;

 private:
  // Helper methods
  bool LoadMetaData();
  bool IsSetup(bool check_jacobian_defined);
  static Eigen::Matrix<float, 6, 6> Adjoint(const Transform3fA &pose);

  // Internal data
  Eigen::Matrix<float, 6, 1> gradient_{};
  Eigen::Matrix<float, 6, 6> hessian_{};
  Eigen::Matrix<float, 6, Eigen::Dynamic> jacobian_{};
  int first_jacobian_index_ = 0;

  // Parameters
  std::string name_{};
  std::filesystem::path metafile_path_{};
  std::shared_ptr<Body> body_ptr_ = nullptr;
  std::vector<std::shared_ptr<Modality>> modality_ptrs_{};
  std::vector<std::shared_ptr<Link>> child_link_ptrs_{};
  Transform3fA body2joint_pose_{Transform3fA::Identity()};
  Transform3fA joint2parent_pose_{Transform3fA::Identity()};
  Transform3fA default_body2joint_pose_{Transform3fA::Identity()};
  Transform3fA default_joint2parent_pose_{Transform3fA::Identity()};
  Transform3fA link2world_pose_{Transform3fA::Identity()};
  std::array<bool, 6> free_directions_{true, true, true, true, true, true};
  bool fixed_body2joint_pose_ = true;

  // State variables
  bool jacobian_defined_ = false;
  bool set_up_ = false;
};

}  // namespace m3t

#endif  // M3T_INCLUDE_M3T_LINK_H_
