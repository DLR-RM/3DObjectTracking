// SPDX-License-Identifier: MIT
// Copyright (c) 2023 Manuel Stoiber, German Aerospace Center (DLR)

#include <m3t/constraint.h>

namespace m3t {

Constraint::Constraint(const std::string &name,
                       const std::shared_ptr<Link> &link1_ptr,
                       const std::shared_ptr<Link> &link2_ptr,
                       const Transform3fA &body12joint1_pose,
                       const Transform3fA &body22joint2_pose,
                       const std::array<bool, 6> &constraint_directions)
    : name_{name},
      link1_ptr_{link1_ptr},
      link2_ptr_{link2_ptr},
      body12joint1_pose_{body12joint1_pose},
      body22joint2_pose_{body22joint2_pose},
      constraint_directions_{constraint_directions} {}

Constraint::Constraint(const std::string &name,
                       const std::filesystem::path &metafile_path,
                       const std::shared_ptr<Link> &link1_ptr,
                       const std::shared_ptr<Link> &link2_ptr)
    : name_{name},
      metafile_path_{metafile_path},
      link1_ptr_{link1_ptr},
      link2_ptr_{link2_ptr} {}

bool Constraint::SetUp() {
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

  set_up_ = true;
  return true;
}

void Constraint::set_name(const std::string &name) { name_ = name; }

void Constraint::set_metafile_path(const std::filesystem::path &metafile_path) {
  metafile_path_ = metafile_path;
  set_up_ = false;
}

void Constraint::set_link1_ptr(const std::shared_ptr<Link> &link1_ptr) {
  link1_ptr_ = link1_ptr;
  set_up_ = false;
}

void Constraint::set_link2_ptr(const std::shared_ptr<Link> &link2_ptr) {
  link2_ptr_ = link2_ptr;
  set_up_ = false;
}

void Constraint::set_body12joint1_pose(const Transform3fA &body12joint1_pose) {
  body12joint1_pose_ = body12joint1_pose;
}

void Constraint::set_body22joint2_pose(const Transform3fA &body22joint2_pose) {
  body22joint2_pose_ = body22joint2_pose;
}

void Constraint::set_constraint_directions(
    const std::array<bool, 6> &constraint_directions) {
  constraint_directions_ = constraint_directions;
}

bool Constraint::CalculateResidualAndConstraintJacobian() {
  if (!set_up_) {
    std::cerr << "Set up constraint " << name_ << " first" << std::endl;
    return false;
  }

  // Calculate required poses
  auto body22joint1_pose{body12joint1_pose_ *
                         link1_ptr_->link2world_pose().inverse() *
                         link2_ptr_->link2world_pose()};
  auto joint22joint1_pose{body22joint1_pose * body22joint2_pose_.inverse()};

  // Calculate residual
  residual_ = Residual(joint22joint1_pose);

  // Calculate jacobian
  constraint_jacobian_ =
      UnprojectedConstraintJacobian(joint22joint1_pose, body22joint1_pose) *
          link2_ptr_->jacobian() -
      UnprojectedConstraintJacobian(joint22joint1_pose, body12joint1_pose_) *
          link1_ptr_->jacobian();
  return true;
}

int Constraint::NumberOfConstraints() {
  if (kOrthogonalityConstraintExperiment) {
    if (kDoubleConstraintRotation)
      return 9;
    else
      return 6;
  }
  return std::count(begin(constraint_directions_), end(constraint_directions_),
                    true);
}

const Eigen::VectorXf &Constraint::residual() { return residual_; }

const Eigen::MatrixXf &Constraint::constraint_jacobian() {
  return constraint_jacobian_;
}

const std::string &Constraint::name() const { return name_; }

const std::filesystem::path &Constraint::metafile_path() const {
  return metafile_path_;
}

const std::shared_ptr<Link> &Constraint::link1_ptr() const {
  return link1_ptr_;
}

const std::shared_ptr<Link> &Constraint::link2_ptr() const {
  return link2_ptr_;
}

const Transform3fA &Constraint::body12joint1_pose() const {
  return body12joint1_pose_;
}

const Transform3fA &Constraint::body22joint2_pose() const {
  return body22joint2_pose_;
}

const std::array<bool, 6> &Constraint::constraint_directions() const {
  return constraint_directions_;
}

bool Constraint::set_up() const { return set_up_; }

bool Constraint::LoadMetaData() {
  // Open file storage from yaml
  cv::FileStorage fs;
  if (!OpenYamlFileStorage(metafile_path_, &fs)) return false;

  // Read parameters from yaml
  std::vector<char> constraint_directions_vector;
  ReadOptionalValueFromYaml(fs, "body12joint1_pose", &body12joint1_pose_);
  ReadOptionalValueFromYaml(fs, "body22joint2_pose", &body22joint2_pose_);
  ReadOptionalValueFromYaml(fs, "constraint_directions",
                            &constraint_directions_vector);

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

Eigen::VectorXf Constraint::Residual(const Transform3fA &joint22joint1_pose) {
  Eigen::AngleAxisf angle_axis{joint22joint1_pose.rotation()};
  Eigen::Vector3f rotation_vector{angle_axis.angle() * angle_axis.axis()};
  Eigen::Vector3f translation_vector{joint22joint1_pose.translation()};

  // Calculations for normal vector constraint experiment
  if (kOrthogonalityConstraintExperiment) {
    Eigen::VectorXf residual{Eigen::VectorXf::Zero(NumberOfConstraints())};
    for (int i = 0; i < NumberOfConstraints(); ++i) {
      if (i < 3)
        residual(i) = translation_vector(i);
      else if (i < 6)
        residual(i) =
            joint22joint1_pose.rotation().matrix()(i % 3, (i + 1) % 3);
      else
        residual(i) =
            joint22joint1_pose.rotation().matrix()(i % 3, (i + 2) % 3);
    }
    return residual;
  }

  Eigen::VectorXf residual{Eigen::VectorXf::Zero(NumberOfConstraints())};
  int residual_idx = 0;
  for (int direction = 0; direction < 6; ++direction) {
    if (constraint_directions_[direction]) {
      if (direction < 3)
        residual(residual_idx) = rotation_vector(direction);
      else
        residual(residual_idx) = translation_vector(direction - 3);
      residual_idx++;
    }
  }
  return residual;
}

Eigen::MatrixXf Constraint::UnprojectedConstraintJacobian(
    const Transform3fA &joint22joint1_pose,
    const Transform3fA &body2joint1_pose) {
  Transform3fA body2joint2_pose{joint22joint1_pose.inverse() *
                                body2joint1_pose};
  Eigen::Vector3f joint22body_translation{
      body2joint2_pose.inverse().translation().matrix()};
  Eigen::Matrix3f body2joint1_rotation{body2joint1_pose.rotation().matrix()};
  Eigen::AngleAxisf angle_axis{joint22joint1_pose.rotation()};
  Eigen::Vector3f axis{angle_axis.axis()};
  float angle_half = 0.5f * angle_axis.angle();
  Eigen::Matrix3f variation_matrix{
      xcotx(angle_half) * Eigen::Matrix3f::Identity() -
      angle_half * Vector2Skewsymmetric(axis) +
      (1.0f - xcotx(angle_half)) * axis * axis.transpose()};

  // Calculations for normal vector constraint experiment
  if (kOrthogonalityConstraintExperiment) {
    Eigen::MatrixXf jacobian{Eigen::MatrixXf::Zero(NumberOfConstraints(), 6)};
    Eigen::Matrix3f body2joint2_rotation{body2joint2_pose.rotation().matrix()};
    for (int i = 0; i < NumberOfConstraints(); ++i) {
      if (i < 3) {
        jacobian.row(i).head<3>() =
            joint22body_translation.cross(body2joint1_rotation.row(i));
        jacobian.row(i).tail<3>() = body2joint1_rotation.row(i);
      } else if (i < 6) {
        Eigen::Vector3f joint22joint1_rotation_row =
            joint22joint1_pose.rotation().matrix().row(i % 3);
        int i1 = (i + 2) % 3;
        int i2 = (i + 3) % 3;
        jacobian.row(i).head<3>() =
            joint22joint1_rotation_row(i1) * body2joint2_rotation.row(i2) -
            joint22joint1_rotation_row(i2) * body2joint2_rotation.row(i1);
      } else {
        Eigen::Vector3f joint22joint1_rotation_row =
            joint22joint1_pose.rotation().matrix().row(i % 3);
        int i1 = (i + 0) % 3;
        int i2 = (i + 1) % 3;
        jacobian.row(i).head<3>() =
            joint22joint1_rotation_row(i1) * body2joint2_rotation.row(i2) -
            joint22joint1_rotation_row(i2) * body2joint2_rotation.row(i1);
      }
    }
    return jacobian;
  }

  Eigen::MatrixXf jacobian{Eigen::MatrixXf::Zero(NumberOfConstraints(), 6)};
  int jacobian_idx = 0;
  for (int direction = 0; direction < 6; ++direction) {
    if (constraint_directions_[direction]) {
      if (direction < 3) {
        jacobian.row(jacobian_idx).head<3>() =
            variation_matrix.row(direction) * body2joint1_rotation;
      } else {
        jacobian.row(jacobian_idx).head<3>() = joint22body_translation.cross(
            body2joint1_rotation.row(direction - 3));
        jacobian.row(jacobian_idx).tail<3>() =
            body2joint1_rotation.row(direction - 3);
      }
      jacobian_idx++;
    }
  }
  return jacobian;
}

}  // namespace m3t
