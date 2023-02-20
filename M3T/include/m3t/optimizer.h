// SPDX-License-Identifier: MIT
// Copyright (c) 2023 Manuel Stoiber, German Aerospace Center (DLR)

#ifndef M3T_INCLUDE_M3T_OPTIMIZER_H_
#define M3T_INCLUDE_M3T_OPTIMIZER_H_

#include <m3t/body.h>
#include <m3t/common.h>
#include <m3t/constraint.h>
#include <m3t/link.h>
#include <m3t/soft_constraint.h>

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <iostream>
#include <string>
#include <vector>

namespace m3t {

/**
 * \brief Class that takes a single root \ref Link object of a kinematic
 * structure as well as additional \ref Constraint and \ref SoftConstraint
 * objects and uses the corresponding gradient vectors, Hessian matrices,
 * constraint Jacobians, and residuals of those objects to update all poses of
 * the kinematic structure.
 *
 * \details The method `CalculateConsistentPoses()` updates the poses of all
 * \ref Body and \ref Link objects in the kinematic structure. Starting from
 * the root link, it ensures that body and link poses are consistent with
 * defined joint poses. The method `CalculateOptimization()` uses information
 * from all \ref Link, \ref Constraint, and \ref SoftConstraint objects that are
 * part of the kinematic structure to update poses. During the optimization,
 * both poses from \Body objects as well as joint and link poses from \ref Link
 * objects are updated. The method `ReferencedLinks()` provides a list of all
 * links that are considered by the optimizer.
 *
 * @param root_link_ptr referenced \ref Link object that is at the root of
 * the corresponding tree-like kinematic structure that should be optimized.
 * @param constraint_ptrs defines the referenced \ref Constraint objects that
 * introduce additional constraints to the kinematic structure
 * @param soft_constraint_ptrs defines the referenced \ref SoftConstraint
 * objects that introduce additional soft constraints to the kinematic structure
 * @param tikhonov_parameter_rotation regularization parameter for rotation.
 * @param tikhonov_parameter_translation regularization parameter for
 * translation.
 */
class Optimizer {
 public:
  // Constructors and setup methods
  Optimizer(const std::string &name, const std::shared_ptr<Link> &root_link_ptr,
            float tikhonov_parameter_rotation = 1000.0f,
            float tikhonov_parameter_translation = 30000.0f);
  Optimizer(const std::string &name, const std::filesystem::path &metafile_path,
            const std::shared_ptr<Link> &root_link_ptr);
  bool SetUp();

  // Configure constraints
  bool AddConstraint(const std::shared_ptr<Constraint> &constraint_ptr);
  bool DeleteConstraint(const std::string &name);
  void ClearConstraints();
  bool AddSoftConstraint(
      const std::shared_ptr<SoftConstraint> &soft_constraint_ptr);
  bool DeleteSoftConstraint(const std::string &name);
  void ClearSoftConstraints();

  // Setters
  void set_name(const std::string &name);
  void set_metafile_path(const std::filesystem::path &metafile_path);
  void set_tikhonov_parameter_rotation(float tikhonov_parameter_rotation);
  void set_tikhonov_parameter_translation(float tikhonov_parameter_translation);

  // Main methods
  bool CalculateConsistentPoses();
  bool CalculateOptimization(int iteration, int corr_iteration,
                             int opt_iteration);

  // Additional methods
  std::vector<std::shared_ptr<Link>> ReferencedLinks();

  // Getters
  const std::string &name() const;
  const std::filesystem::path &metafile_path() const;
  const std::shared_ptr<Link> &root_link_ptr() const;
  const std::vector<std::shared_ptr<Constraint>> &constraint_ptrs() const;
  const std::vector<std::shared_ptr<SoftConstraint>> &soft_constraint_ptrs()
      const;
  float tikhonov_parameter_rotation() const;
  float tikhonov_parameter_translation() const;
  bool set_up() const;

 private:
  // Helper methods
  bool LoadMetaData();
  int DegreesOfFreedom() const;
  int DegreesOfFreedom(const std::shared_ptr<Link> &link_ptr) const;
  int NumberOfConstraints() const;
  bool DefineJacobians();
  bool DefineJacobians(const std::shared_ptr<Link> &link_ptr,
                       int *first_jacobian_index);
  void DefineTikhonovVector();
  void DefineTikhonovVector(const std::shared_ptr<Link> &link_ptr,
                            Eigen::VectorXf *tikhonov_vector) const;
  void AddReferencedLinks(
      const std::shared_ptr<Link> &link_ptr,
      std::vector<std::shared_ptr<Link>> *referenced_links) const;

  // Helper methods for optimization
  bool CalculateDataLinks();
  bool CalculateDataLinks(const std::shared_ptr<Link> &link_ptr,
                          const std::shared_ptr<Link> &parent_link_ptr);
  bool CalculateDataConstraints() const;
  void AddProjectedGradientsAndHessians(Eigen::VectorXf *b,
                                        Eigen::MatrixXf *a) const;
  void AddProjectedGradientsAndHessians(const std::shared_ptr<Link> &link_ptr,
                                        Eigen::VectorXf *b,
                                        Eigen::MatrixXf *a) const;
  void AddResidualsAndConstraintJacobians(Eigen::VectorXf *b,
                                          Eigen::MatrixXf *a) const;
  bool UpdatePoses(const Eigen::VectorXf &theta);
  static bool UpdatePoses(const std::shared_ptr<Link> &link_ptr,
                          const std::shared_ptr<Link> &parent_link_ptr,
                          const Eigen::VectorXf &theta);

  // Internal data
  int degrees_of_freedom_{};
  Eigen::VectorXf tikhonov_vector_{};

  // Data
  std::string name_{};
  std::filesystem::path metafile_path_{};
  std::shared_ptr<Link> root_link_ptr_ = nullptr;
  std::vector<std::shared_ptr<Constraint>> constraint_ptrs_{};
  std::vector<std::shared_ptr<SoftConstraint>> soft_constraint_ptrs_{};
  float tikhonov_parameter_rotation_ = 1000.0f;
  float tikhonov_parameter_translation_ = 30000.0f;
  bool set_up_ = false;
};

}  // namespace m3t

#endif  // M3T_INCLUDE_M3T_OPTIMIZER_H_
