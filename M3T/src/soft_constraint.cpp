// SPDX-License-Identifier: MIT
// Copyright (c) 2023 Manuel Stoiber, German Aerospace Center (DLR)

#include <m3t/soft_constraint.h>

namespace m3t {

SoftConstraint::SoftConstraint(const std::string &name,
                               const std::shared_ptr<Link> &link1_ptr,
                               const std::shared_ptr<Link> &link2_ptr,
                               const Transform3fA &body12joint1_pose,
                               const Transform3fA &body22joint2_pose,
                               const std::array<bool, 6> &constraint_directions,
                               float max_distance_rotation,
                               float max_distance_translation,
                               float standard_deviation_rotation,
                               float standard_deviation_translation)
    : name_{name},
      link1_ptr_{link1_ptr},
      link2_ptr_{link2_ptr},
      body12joint1_pose_{body12joint1_pose},
      body22joint2_pose_{body22joint2_pose},
      constraint_directions_{constraint_directions},
      max_distance_rotation_{max_distance_rotation},
      max_distance_translation_{max_distance_translation},
      standard_deviation_rotation_{standard_deviation_rotation},
      standard_deviation_translation_{standard_deviation_translation} {}

SoftConstraint::SoftConstraint(const std::string &name,
                               const std::filesystem::path &metafile_path,
                               const std::shared_ptr<Link> &link1_ptr,
                               const std::shared_ptr<Link> &link2_ptr)
    : name_{name},
      metafile_path_{metafile_path},
      link1_ptr_{link1_ptr},
      link2_ptr_{link2_ptr} {}

bool SoftConstraint::SetUp() {
  set_up_ = false;
  if (!metafile_path_.empty())
    if (!LoadMetaData()) return false;

  // Check if all required objects are set up
  if (!link1_ptr_->set_up()) {
    std::cerr << "Link " << link1_ptr_->name() << " was not set up"
              << std::endl;
    return false;
  }
  if (!link2_ptr_->set_up()) {
    std::cerr << "Link " << link2_ptr_->name() << " was not set up"
              << std::endl;
    return false;
  }

  PrecomputeInternalVariables();
  set_up_ = true;
  return true;
}

void SoftConstraint::set_name(const std::string &name) { name_ = name; }

void SoftConstraint::set_metafile_path(
    const std::filesystem::path &metafile_path) {
  metafile_path_ = metafile_path;
  set_up_ = false;
}

void SoftConstraint::set_link1_ptr(const std::shared_ptr<Link> &link1_ptr) {
  link1_ptr_ = link1_ptr;
  set_up_ = false;
}

void SoftConstraint::set_link2_ptr(const std::shared_ptr<Link> &link2_ptr) {
  link2_ptr_ = link2_ptr;
  set_up_ = false;
}

void SoftConstraint::set_body12joint1_pose(
    const Transform3fA &body12joint1_pose) {
  body12joint1_pose_ = body12joint1_pose;
}

void SoftConstraint::set_body22joint2_pose(
    const Transform3fA &body22joint2_pose) {
  body22joint2_pose_ = body22joint2_pose;
}

void SoftConstraint::set_constraint_directions(
    const std::array<bool, 6> &constraint_directions) {
  constraint_directions_ = constraint_directions;
  set_up_ = false;
}

void SoftConstraint::set_max_distance_rotation(float max_distance_rotation) {
  max_distance_rotation_ = max_distance_rotation;
}

void SoftConstraint::set_max_distance_translation(
    float max_distance_translation) {
  max_distance_translation_ = max_distance_translation;
}

void SoftConstraint::set_standard_deviation_rotation(
    float standard_deviation_rotation) {
  standard_deviation_rotation_ = standard_deviation_rotation;
}

void SoftConstraint::set_standard_deviation_translation(
    float standard_deviation_translation) {
  standard_deviation_translation_ = standard_deviation_translation;
}

bool SoftConstraint::AddGradientsAndHessiansToLinks() {
  if (!set_up_) {
    std::cerr << "Set up constraint " << name_ << " first" << std::endl;
    return false;
  }

  // Calculate required poses
  auto body22joint1_pose{body12joint1_pose_ *
                         link1_ptr_->link2world_pose().inverse() *
                         link2_ptr_->link2world_pose()};
  auto joint22joint1_pose{body22joint1_pose * body22joint2_pose_.inverse()};

  // Add gradients and hessians to both links
  AddGradientsAndHessiansToLink(joint22joint1_pose, body12joint1_pose_, -1.0f,
                                link1_ptr_);
  AddGradientsAndHessiansToLink(joint22joint1_pose, body22joint1_pose, 1.0f,
                                link2_ptr_);
  return true;
}

const std::string &SoftConstraint::name() const { return name_; }

const std::filesystem::path &SoftConstraint::metafile_path() const {
  return metafile_path_;
}

const std::shared_ptr<Link> &SoftConstraint::link1_ptr() const {
  return link1_ptr_;
}

const std::shared_ptr<Link> &SoftConstraint::link2_ptr() const {
  return link2_ptr_;
}

const Transform3fA &SoftConstraint::body12joint1_pose() const {
  return body12joint1_pose_;
}

const Transform3fA &SoftConstraint::body22joint2_pose() const {
  return body22joint2_pose_;
}

const std::array<bool, 6> &SoftConstraint::constraint_directions() const {
  return constraint_directions_;
}

float SoftConstraint::max_distance_rotation() const {
  return max_distance_rotation_;
}

float SoftConstraint::max_distance_translation() const {
  return max_distance_translation_;
}

float SoftConstraint::standard_deviation_rotation() const {
  return standard_deviation_rotation_;
}

float SoftConstraint::standard_deviation_translation() const {
  return standard_deviation_translation_;
}

bool SoftConstraint::set_up() const { return set_up_; }

bool SoftConstraint::LoadMetaData() {
  // Open file storage from yaml
  cv::FileStorage fs;
  if (!OpenYamlFileStorage(metafile_path_, &fs)) return false;

  // Read parameters from yaml
  std::vector<char> constraint_directions_vector;
  ReadOptionalValueFromYaml(fs, "body12joint1_pose", &body12joint1_pose_);
  ReadOptionalValueFromYaml(fs, "body22joint2_pose", &body22joint2_pose_);
  ReadOptionalValueFromYaml(fs, "constraint_directions",
                            &constraint_directions_vector);
  ReadOptionalValueFromYaml(fs, "max_distance_rotation",
                            &max_distance_rotation_);
  ReadOptionalValueFromYaml(fs, "max_distance_translation",
                            &max_distance_translation_);
  ReadOptionalValueFromYaml(fs, "standard_deviation_rotation",
                            &standard_deviation_rotation_);
  ReadOptionalValueFromYaml(fs, "standard_deviation_translation",
                            &standard_deviation_translation_);

  // Process parameters
  if (constraint_directions_vector.size() == 6) {
    for (int i = 0; i < 6; ++i)
      constraint_directions_[i] = bool(constraint_directions_vector[i]);
  } else {
    std::cerr << "Parameter \"constraint_directions\" in " << metafile_path_
              << " does not contain 6 boolean values." << std::endl;
    return false;
  }

  fs.release();
  return true;
}

void SoftConstraint::PrecomputeInternalVariables() {
  n_constraints_rotation_ = int(constraint_directions_[0]) +
                            int(constraint_directions_[1]) +
                            int(constraint_directions_[2]);
  n_constraints_translation_ = int(constraint_directions_[3]) +
                               int(constraint_directions_[4]) +
                               int(constraint_directions_[5]);
}

void SoftConstraint::AddGradientsAndHessiansToLink(
    const Transform3fA &joint22joint1_pose,
    const Transform3fA &body2joint1_pose, float sign,
    const std::shared_ptr<Link> &link_ptr) const {
  Eigen::Matrix<float, 6, 1> gradient{Eigen::Matrix<float, 6, 1>::Zero()};
  Eigen::Matrix<float, 6, 6> hessian{Eigen::Matrix<float, 6, 6>::Zero()};
  if (n_constraints_rotation_) {
    const auto &identity_matrix{Eigen::MatrixXf::Identity(
        n_constraints_rotation_, n_constraints_rotation_)};
    const auto &rotation_vector{ConsideredRotationVector(joint22joint1_pose)};
    float distance_rotation = rotation_vector.norm();
    if (distance_rotation > max_distance_rotation_) {
      const auto &jacobian{UnprojectedConstraintJacobianRotation(
          joint22joint1_pose, body2joint1_pose)};
      gradient -= (sign / square(standard_deviation_rotation_)) *
                  jacobian.transpose() *
                  (rotation_vector -
                   rotation_vector.normalized() * max_distance_rotation_);
      hessian -= (1.0f / square(standard_deviation_rotation_)) *
                 jacobian.transpose() *
                 (identity_matrix -
                  (max_distance_rotation_ / distance_rotation) *
                      (identity_matrix -
                       rotation_vector.normalized() *
                           rotation_vector.normalized().transpose())) *
                 jacobian;
    }
  }
  if (n_constraints_translation_) {
    const auto &identity_matrix{Eigen::MatrixXf::Identity(
        n_constraints_translation_, n_constraints_translation_)};
    const auto &translation_vector{
        ConsideredTranslationVector(joint22joint1_pose)};
    float distance_translation = translation_vector.norm();
    if (distance_translation > max_distance_translation_) {
      const auto &jacobian{UnprojectedConstraintJacobianTranslation(
          joint22joint1_pose, body2joint1_pose)};
      gradient -= (sign / square(standard_deviation_translation_)) *
                  jacobian.transpose() *
                  (translation_vector -
                   translation_vector.normalized() * max_distance_translation_);
      hessian -= (1.0f / square(standard_deviation_translation_)) *
                 jacobian.transpose() *
                 (identity_matrix -
                  (max_distance_translation_ / distance_translation) *
                      (identity_matrix -
                       translation_vector.normalized() *
                           translation_vector.normalized().transpose())) *
                 jacobian;
    }
  }
  link_ptr->AddToGradientAndHessian(gradient, hessian);
}

Eigen::VectorXf SoftConstraint::ConsideredRotationVector(
    const Transform3fA &joint22joint1_pose) const {
  Eigen::AngleAxisf angle_axis{joint22joint1_pose.rotation()};
  Eigen::Vector3f vector{angle_axis.angle() * angle_axis.axis()};

  Eigen::VectorXf considered_vector{
      Eigen::VectorXf::Zero(n_constraints_rotation_)};
  for (int direction = 0, residual_idx = 0; direction < 3; ++direction) {
    if (constraint_directions_[direction]) {
      considered_vector(residual_idx) = vector(direction);
      residual_idx++;
    }
  }
  return considered_vector;
}

Eigen::VectorXf SoftConstraint::ConsideredTranslationVector(
    const Transform3fA &joint22joint1_pose) const {
  Eigen::Vector3f vector{joint22joint1_pose.translation()};

  Eigen::VectorXf considered_vector{
      Eigen::VectorXf::Zero(n_constraints_translation_)};
  for (int direction = 0, residual_idx = 0; direction < 3; ++direction) {
    if (constraint_directions_[direction + 3]) {
      considered_vector(residual_idx) = vector(direction);
      residual_idx++;
    }
  }
  return considered_vector;
}

Eigen::MatrixXf SoftConstraint::UnprojectedConstraintJacobianRotation(
    const Transform3fA &joint22joint1_pose,
    const Transform3fA &body2joint1_pose) const {
  Eigen::Matrix3f body2joint1_rotation{body2joint1_pose.rotation().matrix()};
  Eigen::AngleAxisf angle_axis{joint22joint1_pose.rotation()};
  Eigen::Vector3f axis{angle_axis.axis()};
  float angle_half = 0.5f * angle_axis.angle();
  Eigen::Matrix3f variation_matrix{
      xcotx(angle_half) * Eigen::Matrix3f::Identity() -
      angle_half * Vector2Skewsymmetric(axis) +
      (1.0f - xcotx(angle_half)) * axis * axis.transpose()};

  Eigen::MatrixXf jacobian{Eigen::MatrixXf::Zero(n_constraints_rotation_, 6)};
  for (int direction = 0, jacobian_idx = 0; direction < 3; ++direction) {
    if (constraint_directions_[direction]) {
      jacobian.row(jacobian_idx).head<3>() =
          variation_matrix.row(direction) * body2joint1_rotation;
      jacobian_idx++;
    }
  }
  return jacobian;
}

Eigen::MatrixXf SoftConstraint::UnprojectedConstraintJacobianTranslation(
    const Transform3fA &joint22joint1_pose,
    const Transform3fA &body2joint1_pose) const {
  Transform3fA body2joint2_pose{joint22joint1_pose.inverse() *
                                body2joint1_pose};
  Eigen::Vector3f joint22body_translation{
      body2joint2_pose.inverse().translation().matrix()};
  Eigen::Matrix3f body2joint1_rotation{body2joint1_pose.rotation().matrix()};

  Eigen::MatrixXf jacobian{
      Eigen::MatrixXf::Zero(n_constraints_translation_, 6)};
  for (int direction = 0, jacobian_idx = 0; direction < 3; ++direction) {
    if (constraint_directions_[direction + 3]) {
      jacobian.row(jacobian_idx).head<3>() =
          joint22body_translation.cross(body2joint1_rotation.row(direction));
      jacobian.row(jacobian_idx).tail<3>() =
          body2joint1_rotation.row(direction);
      jacobian_idx++;
    }
  }
  return jacobian;
}

}  // namespace m3t
