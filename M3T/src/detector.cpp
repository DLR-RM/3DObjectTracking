// SPDX-License-Identifier: MIT
// Copyright (c) 2023 Manuel Stoiber, German Aerospace Center (DLR)

#include <m3t/detector.h>

namespace m3t {

void Detector::set_name(const std::string &name) { name_ = name; }

void Detector::set_metafile_path(const std::filesystem::path &metafile_path) {
  metafile_path_ = metafile_path;
  set_up_ = false;
}

void Detector::set_reset_joint_poses(bool reset_joint_poses) {
  reset_joint_poses_ = reset_joint_poses;
}

const std::string &Detector::name() const { return name_; }

const std::filesystem::path &Detector::metafile_path() const {
  return metafile_path_;
}

bool Detector::reset_joint_poses() const { return reset_joint_poses_; }

std::vector<std::shared_ptr<Optimizer>> Detector::optimizer_ptrs() const {
  return {nullptr};
}

std::shared_ptr<Camera> Detector::camera_ptr() const { return nullptr; }

bool Detector::set_up() const { return set_up_; }

Detector::Detector(const std::string &name, bool reset_joint_poses)
    : name_{name}, reset_joint_poses_{reset_joint_poses} {}

Detector::Detector(const std::string &name,
                   const std::filesystem::path &metafile_path)
    : name_{name}, metafile_path_{metafile_path} {}

void Detector::UpdatePoses(const Transform3fA &link2world_pose,
                           const std::shared_ptr<Optimizer> &optimizer_ptr) {
  const auto &root_link_ptr{optimizer_ptr->root_link_ptr()};
  root_link_ptr->set_link2world_pose(link2world_pose);
  if (root_link_ptr->body_ptr())
    root_link_ptr->body_ptr()->set_body2world_pose(link2world_pose);
  if (reset_joint_poses_) {
    for (const auto &link_ptr : optimizer_ptr->ReferencedLinks())
      link_ptr->ResetJointPoses();
  }
  optimizer_ptr->CalculateConsistentPoses();
}

}  // namespace m3t
