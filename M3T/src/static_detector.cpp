// SPDX-License-Identifier: MIT
// Copyright (c) 2023 Manuel Stoiber, German Aerospace Center (DLR)

#include <m3t/static_detector.h>

namespace m3t {

StaticDetector::StaticDetector(const std::string &name,
                               const std::shared_ptr<Optimizer> &optimizer_ptr,
                               const Transform3fA &link2world_pose,
                               bool reset_joint_poses)
    : Detector{name, reset_joint_poses},
      optimizer_ptr_{optimizer_ptr},
      link2world_pose_{link2world_pose} {}

StaticDetector::StaticDetector(
    const std::string &name, const std::filesystem::path &metafile_path,
    const std::shared_ptr<m3t::Optimizer> &optimizer_ptr)
    : Detector{name, metafile_path}, optimizer_ptr_{optimizer_ptr} {};

bool StaticDetector::SetUp() {
  set_up_ = false;
  if (!metafile_path_.empty())
    if (!LoadMetaData()) return false;

  // Check if all required objects are set up
  if (!optimizer_ptr_->set_up()) {
    std::cerr << "Optimizer " << optimizer_ptr_->name() << " was not set up"
              << std::endl;
    return false;
  }

  set_up_ = true;
  return true;
}

void StaticDetector::set_optimizer_ptr(
    const std::shared_ptr<Optimizer> &optimizer_ptr) {
  optimizer_ptr_ = optimizer_ptr;
  set_up_ = false;
}

void StaticDetector::set_link2world_pose(const Transform3fA &link2world_pose) {
  link2world_pose_ = link2world_pose;
}

bool StaticDetector::DetectPoses(const std::set<std::string> &names,
                                 std::set<std::string> *detected_names) {
  if (!set_up_) {
    std::cerr << "Set up static detector " << name_ << " first" << std::endl;
    return false;
  }

  if (names.find(optimizer_ptr_->name()) != names.end()) {
    UpdatePoses(link2world_pose_, optimizer_ptr_);
    if (detected_names) detected_names->insert(optimizer_ptr_->name());
  }
  return true;
}

const std::shared_ptr<Optimizer> &StaticDetector::optimizer_ptr() const {
  return optimizer_ptr_;
}

const Transform3fA &StaticDetector::link2world_pose() const {
  return link2world_pose_;
}

std::vector<std::shared_ptr<Optimizer>> StaticDetector::optimizer_ptrs() const {
  return {optimizer_ptr_};
}

bool StaticDetector::LoadMetaData() {
  // Open file storage from yaml
  cv::FileStorage fs;
  if (!OpenYamlFileStorage(metafile_path_, &fs)) return false;

  // Read parameters from yaml
  if (!ReadRequiredValueFromYaml(fs, "link2world_pose", &link2world_pose_)) {
    std::cerr << "Could not read all required static detector parameters from "
              << metafile_path_ << std::endl;
    return false;
  }
  ReadOptionalValueFromYaml(fs, "reset_joint_poses", &reset_joint_poses_);
  fs.release();
  return true;
}

}  // namespace m3t
