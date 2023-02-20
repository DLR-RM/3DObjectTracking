// SPDX-License-Identifier: MIT
// Copyright (c) 2023 Manuel Stoiber, German Aerospace Center (DLR)

#include <m3t/link.h>

namespace m3t {

Link::Link(const std::string &name, const std::shared_ptr<Body> &body_ptr,
           const Transform3fA &body2joint_pose,
           const Transform3fA &joint2parent_pose,
           const Transform3fA &link2world_pose,
           const std::array<bool, 6> &free_directions,
           bool fixed_body2joint_pose)
    : name_{name},
      body_ptr_{body_ptr},
      body2joint_pose_{body2joint_pose},
      default_body2joint_pose_{body2joint_pose},
      joint2parent_pose_{joint2parent_pose},
      default_joint2parent_pose_{joint2parent_pose},
      link2world_pose_{link2world_pose},
      free_directions_{free_directions},
      fixed_body2joint_pose_{fixed_body2joint_pose} {}

Link::Link(const std::string &name, const std::filesystem::path &metafile_path,
           const std::shared_ptr<Body> &body_ptr)
    : name_{name}, metafile_path_{metafile_path}, body_ptr_{body_ptr} {}

bool Link::SetUp() {
  set_up_ = false;
  jacobian_defined_ = false;
  if (!metafile_path_.empty())
    if (!LoadMetaData()) return false;

  if (body_ptr_) {
    // Check if all required objects are set up
    if (!body_ptr_->set_up()) {
      std::cerr << "Body " << body_ptr_->name() << " was not set up"
                << std::endl;
      return false;
    }
    for (auto &modality_ptr : modality_ptrs_) {
      if (!modality_ptr->set_up()) {
        std::cerr << "Modality " << modality_ptr->name() << " was not set up"
                  << std::endl;
        return false;
      }
    }

    // Check if all modalities reference the main body
    for (auto &modality_ptr : modality_ptrs_) {
      if (modality_ptr->body_ptr()->name() != body_ptr_->name()) {
        std::cerr << "Modalities that reference different bodies than the main "
                  << "body were assigned to link " << name_ << std::endl;
        return false;
      }
    }
  } else {
    if (!modality_ptrs_.empty()) {
      std::cerr << "No main body assigned to link " << name_ << std::endl;
      return false;
    }
  }

  set_up_ = true;
  return true;
}

bool Link::AddModality(const std::shared_ptr<Modality> &modality_ptr) {
  set_up_ = false;
  if (!AddPtrIfNameNotExists(modality_ptr, &modality_ptrs_)) {
    std::cerr << "Modality " << modality_ptr->name() << " already exists"
              << std::endl;
    return false;
  }
  return true;
}

bool Link::DeleteModality(const std::string &name) {
  set_up_ = false;
  if (!DeletePtrIfNameExists(name, &modality_ptrs_)) {
    std::cerr << "Modality " << name << " not found" << std::endl;
    return false;
  }
  return true;
}

void Link::ClearModalities() {
  set_up_ = false;
  modality_ptrs_.clear();
}

bool Link::AddChildLink(const std::shared_ptr<Link> &child_link_ptr) {
  set_up_ = false;
  if (!AddPtrIfNameNotExists(child_link_ptr, &child_link_ptrs_)) {
    std::cerr << "Child link " << child_link_ptr->name() << " already exists"
              << std::endl;
    return false;
  }
  return true;
}

bool Link::DeleteChildLink(const std::string &name) {
  set_up_ = false;
  if (!DeletePtrIfNameExists(name, &child_link_ptrs_)) {
    std::cerr << "Child link " << name << " not found" << std::endl;
    return false;
  }
  return true;
}

void Link::ClearChildLinks() {
  set_up_ = false;
  child_link_ptrs_.clear();
}

void Link::set_name(const std::string &name) { name_ = name; }

void Link::set_metafile_path(const std::filesystem::path &metafile_path) {
  metafile_path_ = metafile_path;
  set_up_ = false;
}

void Link::set_body_ptr(const std::shared_ptr<Body> &body_ptr) {
  body_ptr_ = body_ptr;
  set_up_ = false;
}

void Link::set_body2joint_pose(const Transform3fA &body2joint_pose) {
  body2joint_pose_ = body2joint_pose;
  default_body2joint_pose_ = body2joint_pose;
}

void Link::set_joint2parent_pose(const Transform3fA &joint2parent_pose) {
  joint2parent_pose_ = joint2parent_pose;
  default_joint2parent_pose_ = joint2parent_pose;
}

void Link::set_link2world_pose(const Transform3fA &link2world_pose) {
  link2world_pose_ = link2world_pose;
}

void Link::set_free_directions(const std::array<bool, 6> &free_directions) {
  free_directions_ = free_directions;
  set_up_ = false;
}

void Link::set_fixed_body2joint_pose(bool fixed_body2joint_pose) {
  fixed_body2joint_pose_ = fixed_body2joint_pose;
}

bool Link::DefineJacobian(int jacobian_size, int first_jacobian_index) {
  if (!IsSetup(false)) return false;
  jacobian_.resize(Eigen::NoChange, jacobian_size);
  first_jacobian_index_ = first_jacobian_index;
  jacobian_defined_ = true;
  return true;
}

bool Link::CalculateJacobian(const std::shared_ptr<Link> &parent_link_ptr) {
  if (!IsSetup(true)) return false;

  // project parent Jacobian
  if (parent_link_ptr) {
    auto parent2body_pose{(joint2parent_pose_ * body2joint_pose_).inverse()};
    auto dtheta_body_dtheta_parent{Adjoint(parent2body_pose)};
    jacobian_ = dtheta_body_dtheta_parent * parent_link_ptr->jacobian_;
  } else {
    jacobian_.setZero();
  }

  // Add joint Jacobian
  auto joint2body_pose{body2joint_pose_.inverse()};
  auto dtheta_body_dtheta_joint{Adjoint(joint2body_pose)};
  int jacobian_idx = first_jacobian_index_;
  for (int direction = 0; direction < 6; ++direction) {
    if (free_directions_[direction]) {
      jacobian_.col(jacobian_idx) = dtheta_body_dtheta_joint.col(direction);
      jacobian_idx++;
    }
  }
  return true;
}

bool Link::CalculateGradientAndHessian() {
  if (!IsSetup(true)) return false;
  gradient_.setZero();
  hessian_.setZero();
  for (auto &modality_ptr : modality_ptrs_) {
    gradient_ += modality_ptr->gradient();
    hessian_ += modality_ptr->hessian();
  }
  return true;
}

bool Link::AddToGradientAndHessian(
    const Eigen::Matrix<float, 6, 1> &gradient_summand,
    const Eigen::Matrix<float, 6, 6> &hessian_summand) {
  if (!IsSetup(true)) return false;

  gradient_ += gradient_summand;
  hessian_ += hessian_summand;
  return true;
}

bool Link::UpdatePoses(const std::shared_ptr<Link> &parent_link_ptr,
                       Eigen::VectorXf theta) {
  if (!IsSetup(true)) return false;

  // Extract variation vector for link
  Eigen::Matrix<float, 6, 1> theta_link;
  int jacobian_idx = first_jacobian_index_;
  for (int direction = 0; direction < 6; ++direction) {
    if (free_directions_[direction]) {
      theta_link(direction) = theta(jacobian_idx);
      jacobian_idx++;
    } else {
      theta_link(direction) = 0.0f;
    }
  }

  // Calculate pose variation
  Transform3fA pose_variation{Transform3fA::Identity()};
  pose_variation.translate(theta_link.tail<3>());
  pose_variation.rotate(Vector2Skewsymmetric(theta_link.head<3>()).exp());

  // Update poses
  if (parent_link_ptr) {
    if (fixed_body2joint_pose_)
      joint2parent_pose_ = joint2parent_pose_ * pose_variation;
    else
      body2joint_pose_ = pose_variation * body2joint_pose_;
    link2world_pose_ = parent_link_ptr->link2world_pose() * joint2parent_pose_ *
                       body2joint_pose_;
    if (body_ptr_) body_ptr_->set_body2world_pose(link2world_pose_);
  } else {
    link2world_pose_ = link2world_pose() * body2joint_pose_.inverse() *
                       pose_variation * body2joint_pose_;
    if (body_ptr_) body_ptr_->set_body2world_pose(link2world_pose_);
  }
  return true;
}

void Link::ResetJointPoses() {
  body2joint_pose_ = default_body2joint_pose_;
  joint2parent_pose_ = default_joint2parent_pose_;
}

int Link::DegreesOfFreedom() const {
  return std::count(begin(free_directions_), end(free_directions_), true);
}

const Eigen::Matrix<float, 6, 1> &Link::gradient() { return gradient_; }

const Eigen::Matrix<float, 6, 6> &Link::hessian() { return hessian_; }

const Eigen::Matrix<float, 6, Eigen::Dynamic> &Link::jacobian() {
  return jacobian_;
}

const std::string &Link::name() const { return name_; }

const std::filesystem::path &Link::metafile_path() const {
  return metafile_path_;
}

const std::shared_ptr<Body> &Link::body_ptr() const { return body_ptr_; }

const std::vector<std::shared_ptr<Modality>> &Link::modality_ptrs() const {
  return modality_ptrs_;
}

const std::vector<std::shared_ptr<Link>> &Link::child_link_ptrs() const {
  return child_link_ptrs_;
}

const Transform3fA &Link::body2joint_pose() const { return body2joint_pose_; }

const Transform3fA &Link::joint2parent_pose() const {
  return joint2parent_pose_;
}

const std::array<bool, 6> &Link::free_directions() const {
  return free_directions_;
}

bool Link::fixed_body2joint_pose() const { return fixed_body2joint_pose_; }

const Transform3fA &Link::link2world_pose() const {
  if (body_ptr_)
    return body_ptr_->body2world_pose();
  else
    return link2world_pose_;
}

int Link::first_jacobian_index() const { return first_jacobian_index_; }

bool Link::set_up() const { return set_up_; }

bool Link::LoadMetaData() {
  // Open file storage from yaml
  cv::FileStorage fs;
  if (!OpenYamlFileStorage(metafile_path_, &fs)) return false;

  // Read parameters from yaml
  std::vector<char> free_directions_vector;
  ReadOptionalValueFromYaml(fs, "body2joint_pose", &body2joint_pose_);
  ReadOptionalValueFromYaml(fs, "joint2parent_pose", &joint2parent_pose_);
  ReadOptionalValueFromYaml(fs, "link2world_pose", &link2world_pose_);
  ReadOptionalValueFromYaml(fs, "free_directions", &free_directions_vector);
  ReadOptionalValueFromYaml(fs, "fixed_body2joint_pose",
                            &fixed_body2joint_pose_);
  default_body2joint_pose_ = body2joint_pose_;
  default_joint2parent_pose_ = joint2parent_pose_;

  // Process parameters
  if (free_directions_vector.size() == 6) {
    for (int i = 0; i < 6; ++i)
      free_directions_[i] = bool(free_directions_vector[i]);
  } else {
    std::cerr << "Parameter \"free_directions\" in " << metafile_path_
              << " does not contain 6 boolean values." << std::endl;
    return false;
  }

  fs.release();
  return true;
}

bool Link::IsSetup(bool check_jacobian_defined) {
  if (!set_up_) {
    std::cerr << "Set up link " << name_ << " first" << std::endl;
    return false;
  }
  if (check_jacobian_defined && !jacobian_defined_) {
    std::cerr << "Define jacobian of link " << name_ << " first" << std::endl;
    return false;
  }
  return true;
}

Eigen::Matrix<float, 6, 6> Link::Adjoint(const Transform3fA &pose) {
  Eigen::Matrix<float, 6, 6> m{Eigen::Matrix<float, 6, 6>::Zero()};
  m.topLeftCorner<3, 3>() = pose.rotation();
  m.bottomLeftCorner<3, 3>() =
      Vector2Skewsymmetric(pose.translation()) * pose.rotation();
  m.bottomRightCorner<3, 3>() = pose.rotation();
  return m;
}

}  // namespace m3t
