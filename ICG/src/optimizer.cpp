// SPDX-License-Identifier: MIT
// Copyright (c) 2022 Manuel Stoiber, German Aerospace Center (DLR)

#include <icg/optimizer.h>

namespace icg {

Optimizer::Optimizer(const std::string &name, float tikhonov_parameter_rotation,
                     float tikhonov_parameter_translation)
    : name_{name},
      tikhonov_parameter_rotation_{tikhonov_parameter_rotation},
      tikhonov_parameter_translation_{tikhonov_parameter_translation} {}

Optimizer::Optimizer(const std::string &name,
                     const std::filesystem::path &metafile_path)
    : name_{name}, metafile_path_{metafile_path} {}

bool Optimizer::SetUp() {
  set_up_ = false;
  if (!metafile_path_.empty())
    if (!LoadMetaData()) return false;

  // Check if all required objects are set up
  if (modality_ptrs_.empty()) {
    std::cerr << "No modalitys were assigned to optimizer " << name_
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

  set_up_ = true;
  return true;
}

bool Optimizer::AddModality(const std::shared_ptr<Modality> &modality_ptr) {
  set_up_ = false;
  if (!AddPtrIfNameNotExists(modality_ptr, &modality_ptrs_)) {
    std::cerr << "Modality " << modality_ptr->name() << " already exists"
              << std::endl;
    return false;
  }
  return true;
}

bool Optimizer::DeleteModality(const std::string &name) {
  set_up_ = false;
  if (!DeletePtrIfNameExists(name, &modality_ptrs_)) {
    std::cerr << "Modality " << name << " not found" << std::endl;
    return false;
  }
  return true;
}

void Optimizer::ClearModalities() {
  set_up_ = false;
  modality_ptrs_.clear();
}

void Optimizer::set_name(const std::string &name) { name_ = name; }

void Optimizer::set_metafile_path(const std::filesystem::path &metafile_path) {
  metafile_path_ = metafile_path;
  set_up_ = false;
}

void Optimizer::set_tikhonov_parameter_rotation(
    float tikhonov_parameter_rotation) {
  tikhonov_parameter_rotation_ = tikhonov_parameter_rotation;
}

void Optimizer::set_tikhonov_parameter_translation(
    float tikhonov_parameter_translation) {
  tikhonov_parameter_translation_ = tikhonov_parameter_translation;
}

bool Optimizer::CalculateOptimization(int iteration, int corr_iteration,
                                      int opt_iteration) {
  if (!set_up_) {
    std::cerr << "Set up optimizer " << name_ << " first" << std::endl;
    return false;
  }
  if (modality_ptrs_.size() == 0) {
    std::cerr << "Add modalities to optimizer " << name_ << std::endl;
    return false;
  }

  // Assemble matrix and vector for linear equation
  Eigen::Matrix<float, 6, 6> a{Eigen::Matrix<float, 6, 6>::Zero()};
  Eigen::Matrix<float, 6, 1> b{Eigen::Matrix<float, 6, 1>::Zero()};
  for (auto &modality_ptr : modality_ptrs_) {
    a -= modality_ptr->hessian();
    b += modality_ptr->gradient();
  }
  a.diagonal().head<3>().array() += tikhonov_parameter_rotation_;
  a.diagonal().tail<3>().array() += tikhonov_parameter_translation_;

  // Optimize and update pose
  Eigen::FullPivLU<Eigen::Matrix<float, 6, 6>> lu{a};
  if (lu.isInvertible()) {
    Eigen::Matrix<float, 6, 1> theta{lu.solve(b)};
    Transform3fA pose_variation{Transform3fA::Identity()};
    pose_variation.rotate(Vector2Skewsymmetric(theta.head<3>()).exp());
    pose_variation.translate(theta.tail<3>());
    const auto &body_ptr{modality_ptrs_[0]->body_ptr()};
    body_ptr->set_body2world_pose(body_ptr->body2world_pose() * pose_variation);
  }
  return true;
}

const std::string &Optimizer::name() const { return name_; }

const std::filesystem::path &Optimizer::metafile_path() const {
  return metafile_path_;
}

const std::vector<std::shared_ptr<Modality>> &Optimizer::modality_ptrs() const {
  return modality_ptrs_;
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

}  // namespace icg
