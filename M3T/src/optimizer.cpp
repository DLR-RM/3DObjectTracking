// SPDX-License-Identifier: MIT
// Copyright (c) 2023 Manuel Stoiber, German Aerospace Center (DLR)

#include <m3t/optimizer.h>

namespace m3t {

Optimizer::Optimizer(const std::string &name,
                     const std::shared_ptr<Link> &root_link_ptr,
                     float tikhonov_parameter_rotation,
                     float tikhonov_parameter_translation)
    : name_{name},
      root_link_ptr_{root_link_ptr},
      tikhonov_parameter_rotation_{tikhonov_parameter_rotation},
      tikhonov_parameter_translation_{tikhonov_parameter_translation} {}

Optimizer::Optimizer(const std::string &name,
                     const std::filesystem::path &metafile_path,
                     const std::shared_ptr<Link> &root_link_ptr)
    : name_{name},
      metafile_path_{metafile_path},
      root_link_ptr_{root_link_ptr} {}

bool Optimizer::SetUp() {
  set_up_ = false;
  if (!metafile_path_.empty())
    if (!LoadMetaData()) return false;

  // Check if all required objects are set up
  if (!root_link_ptr_) {
    std::cerr << "No root link assigned to optimizer " << name_ << std::endl;
    return false;
  }
  for (auto &link_ptr : ReferencedLinks()) {
    if (!link_ptr->set_up()) {
      std::cerr << "Link " << link_ptr->name() << " was not set up"
                << std::endl;
      return false;
    }
  }
  for (auto &constraint_ptr : constraint_ptrs_) {
    if (!constraint_ptr->set_up()) {
      std::cerr << "Constraint " << constraint_ptr->name() << " was not set up"
                << std::endl;
      return false;
    }
  }
  for (auto &soft_constraint_ptr : soft_constraint_ptrs_) {
    if (!soft_constraint_ptr->set_up()) {
      std::cerr << "SoftConstraint " << soft_constraint_ptr->name()
                << " was not set up" << std::endl;
      return false;
    }
  }

  // Initialize internal values and referenced links
  degrees_of_freedom_ = DegreesOfFreedom();
  if (!DefineJacobians()) return false;
  if (!UpdatePoses(Eigen::VectorXf::Zero(degrees_of_freedom_))) return false;
  DefineTikhonovVector();

  set_up_ = true;
  return true;
}

bool Optimizer::AddConstraint(
    const std::shared_ptr<Constraint> &constraint_ptr) {
  set_up_ = false;
  if (!AddPtrIfNameNotExists(constraint_ptr, &constraint_ptrs_)) {
    std::cerr << "Constraint " << constraint_ptr->name() << " already exists"
              << std::endl;
    return false;
  }
  return true;
}

bool Optimizer::DeleteConstraint(const std::string &name) {
  set_up_ = false;
  if (!DeletePtrIfNameExists(name, &constraint_ptrs_)) {
    std::cerr << "Constraint " << name << " not found" << std::endl;
    return false;
  }
  return true;
}

void Optimizer::ClearConstraints() {
  set_up_ = false;
  constraint_ptrs_.clear();
}

bool Optimizer::AddSoftConstraint(
    const std::shared_ptr<SoftConstraint> &soft_constraint_ptr) {
  set_up_ = false;
  if (!AddPtrIfNameNotExists(soft_constraint_ptr, &soft_constraint_ptrs_)) {
    std::cerr << "Soft constraint " << soft_constraint_ptr->name()
              << " already exists" << std::endl;
    return false;
  }
  return true;
}

bool Optimizer::DeleteSoftConstraint(const std::string &name) {
  set_up_ = false;
  if (!DeletePtrIfNameExists(name, &soft_constraint_ptrs_)) {
    std::cerr << "Soft constraint " << name << " not found" << std::endl;
    return false;
  }
  return true;
}

void Optimizer::ClearSoftConstraints() {
  set_up_ = false;
  soft_constraint_ptrs_.clear();
}

void Optimizer::set_name(const std::string &name) { name_ = name; }

void Optimizer::set_metafile_path(const std::filesystem::path &metafile_path) {
  metafile_path_ = metafile_path;
  set_up_ = false;
}

void Optimizer::set_tikhonov_parameter_rotation(
    float tikhonov_parameter_rotation) {
  tikhonov_parameter_rotation_ = tikhonov_parameter_rotation;
  set_up_ = false;
}

void Optimizer::set_tikhonov_parameter_translation(
    float tikhonov_parameter_translation) {
  tikhonov_parameter_translation_ = tikhonov_parameter_translation;
  set_up_ = false;
}

bool Optimizer::CalculateConsistentPoses() {
  if (!set_up_) {
    std::cerr << "Set up optimizer " << name_ << " first" << std::endl;
    return false;
  }

  return UpdatePoses(Eigen::VectorXf::Zero(degrees_of_freedom_));
}

bool Optimizer::CalculateOptimization(int iteration, int corr_iteration,
                                      int opt_iteration) {
  if (!set_up_) {
    std::cerr << "Set up optimizer " << name_ << " first" << std::endl;
    return false;
  }

  // Assamble coefficient matrix and column vector
  int size{degrees_of_freedom_ + NumberOfConstraints()};
  Eigen::VectorXf b{Eigen::VectorXf::Zero(size)};
  Eigen::MatrixXf a{Eigen::MatrixXf::Zero(size, size)};
  if (!CalculateDataLinks()) return false;
  if (!CalculateDataConstraints()) return false;
  AddProjectedGradientsAndHessians(&b, &a);
  AddResidualsAndConstraintJacobians(&b, &a);
  a.diagonal().topRows(degrees_of_freedom_) += tikhonov_vector_;

  //// Optimize and update pose
  Eigen::LDLT<Eigen::MatrixXf, Eigen::Lower> ldlt{a};
  Eigen::VectorXf theta{ldlt.solve(b)};

  if (theta.array().isNaN().isZero()) return UpdatePoses(theta);
  return true;
}

std::vector<std::shared_ptr<Link>> Optimizer::ReferencedLinks() {
  std::vector<std::shared_ptr<Link>> referenced_links;
  AddReferencedLinks(root_link_ptr_, &referenced_links);
  return referenced_links;
}

const std::string &Optimizer::name() const { return name_; }

const std::filesystem::path &Optimizer::metafile_path() const {
  return metafile_path_;
}

const std::shared_ptr<Link> &Optimizer::root_link_ptr() const {
  return root_link_ptr_;
}

const std::vector<std::shared_ptr<Constraint>> &Optimizer::constraint_ptrs()
    const {
  return constraint_ptrs_;
}

const std::vector<std::shared_ptr<SoftConstraint>>
    &Optimizer::soft_constraint_ptrs() const {
  return soft_constraint_ptrs_;
}

float Optimizer::tikhonov_parameter_rotation() const {
  return tikhonov_parameter_rotation_;
}

float Optimizer::tikhonov_parameter_translation() const {
  return tikhonov_parameter_translation_;
}

bool Optimizer::set_up() const { return set_up_; }

bool Optimizer::LoadMetaData() {
  // Open file storage from yaml
  cv::FileStorage fs;
  if (!OpenYamlFileStorage(metafile_path_, &fs)) return false;

  // Read parameters from yaml
  ReadOptionalValueFromYaml(fs, "tikhonov_parameter_rotation",
                            &tikhonov_parameter_rotation_);
  ReadOptionalValueFromYaml(fs, "tikhonov_parameter_translation",
                            &tikhonov_parameter_translation_);
  fs.release();
  return true;
}

int Optimizer::DegreesOfFreedom() const {
  return DegreesOfFreedom(root_link_ptr_);
}

int Optimizer::DegreesOfFreedom(const std::shared_ptr<Link> &link_ptr) const {
  int dof = link_ptr->DegreesOfFreedom();
  for (auto &child_link_ptr : link_ptr->child_link_ptrs())
    dof += DegreesOfFreedom(child_link_ptr);
  return dof;
}

int Optimizer::NumberOfConstraints() const {
  int n = 0;
  for (auto &constraint_ptr : constraint_ptrs_)
    n += constraint_ptr->NumberOfConstraints();
  return n;
}

bool Optimizer::DefineJacobians() {
  int first_jacobian_index = 0;
  return DefineJacobians(root_link_ptr_, &first_jacobian_index);
}

bool Optimizer::DefineJacobians(const std::shared_ptr<Link> &link_ptr,
                                int *first_jacobian_index) {
  if (!link_ptr->DefineJacobian(degrees_of_freedom_, *first_jacobian_index))
    return false;
  *first_jacobian_index += link_ptr->DegreesOfFreedom();
  for (auto &child_link_ptr : link_ptr->child_link_ptrs())
    if (!DefineJacobians(child_link_ptr, first_jacobian_index)) return false;
  return true;
}

void Optimizer::DefineTikhonovVector() {
  tikhonov_vector_.resize(degrees_of_freedom_);
  DefineTikhonovVector(root_link_ptr_, &tikhonov_vector_);
}

void Optimizer::DefineTikhonovVector(const std::shared_ptr<Link> &link_ptr,
                                     Eigen::VectorXf *tikhonov_vector) const {
  int idx = link_ptr->first_jacobian_index();
  for (int direction = 0; direction < 6; ++direction) {
    if (link_ptr->free_directions()[direction]) {
      if (direction < 3)
        (*tikhonov_vector)[idx] = tikhonov_parameter_rotation_;
      else
        (*tikhonov_vector)[idx] = tikhonov_parameter_translation_;
      idx++;
    }
  }
  for (auto &child_link_ptr : link_ptr->child_link_ptrs())
    DefineTikhonovVector(child_link_ptr, tikhonov_vector);
}

void Optimizer::AddReferencedLinks(
    const std::shared_ptr<Link> &link_ptr,
    std::vector<std::shared_ptr<Link>> *referenced_links) const {
  referenced_links->push_back(link_ptr);
  for (auto &child_link_ptr : link_ptr->child_link_ptrs())
    AddReferencedLinks(child_link_ptr, referenced_links);
}

bool Optimizer::CalculateDataLinks() {
  if (!CalculateDataLinks(root_link_ptr_, nullptr)) return false;
  for (auto &soft_constraint_ptr : soft_constraint_ptrs_)
    soft_constraint_ptr->AddGradientsAndHessiansToLinks();
  return true;
}

bool Optimizer::CalculateDataLinks(
    const std::shared_ptr<Link> &link_ptr,
    const std::shared_ptr<Link> &parent_link_ptr) {
  if (!link_ptr->CalculateJacobian(parent_link_ptr)) return false;
  if (!link_ptr->CalculateGradientAndHessian()) return false;
  for (auto &child_link_ptr : link_ptr->child_link_ptrs())
    if (!CalculateDataLinks(child_link_ptr, link_ptr)) return false;
  return true;
}

bool Optimizer::CalculateDataConstraints() const {
  for (auto &constraint_ptr : constraint_ptrs_)
    if (!constraint_ptr->CalculateResidualAndConstraintJacobian()) return false;
  return true;
}

void Optimizer::AddProjectedGradientsAndHessians(Eigen::VectorXf *b,
                                                 Eigen::MatrixXf *a) const {
  AddProjectedGradientsAndHessians(root_link_ptr_, b, a);
}

void Optimizer::AddProjectedGradientsAndHessians(
    const std::shared_ptr<Link> &link_ptr, Eigen::VectorXf *b,
    Eigen::MatrixXf *a) const {
  const auto &gradient{link_ptr->gradient()};
  const auto &hessian{link_ptr->hessian()};
  const auto &jacobian{link_ptr->jacobian()};
  b->topRows(degrees_of_freedom_) += jacobian.transpose() * gradient;
  a->topLeftCorner(degrees_of_freedom_, degrees_of_freedom_)
      .triangularView<Eigen::Lower>() -=
      jacobian.transpose() * hessian * jacobian;
  for (auto &child_link_ptr : link_ptr->child_link_ptrs())
    AddProjectedGradientsAndHessians(child_link_ptr, b, a);
}

void Optimizer::AddResidualsAndConstraintJacobians(Eigen::VectorXf *b,
                                                   Eigen::MatrixXf *a) const {
  int idx = degrees_of_freedom_;
  for (auto &constraint_ptr : constraint_ptrs_) {
    int size = constraint_ptr->NumberOfConstraints();
    b->segment(idx, size) = constraint_ptr->residual();
    a->block(idx, 0, size, degrees_of_freedom_) =
        -constraint_ptr->constraint_jacobian();
    idx += size;
  }
}

bool Optimizer::UpdatePoses(const Eigen::VectorXf &theta) {
  return UpdatePoses(root_link_ptr_, nullptr, theta);
}

bool Optimizer::UpdatePoses(const std::shared_ptr<Link> &link_ptr,
                            const std::shared_ptr<Link> &parent_link_ptr,
                            const Eigen::VectorXf &theta) {
  if (!link_ptr->UpdatePoses(parent_link_ptr, theta)) return false;
  for (auto &child_link_ptr : link_ptr->child_link_ptrs())
    if (!UpdatePoses(child_link_ptr, link_ptr, theta)) return false;
  return true;
}

}  // namespace m3t
