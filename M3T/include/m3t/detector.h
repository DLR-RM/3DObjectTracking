// SPDX-License-Identifier: MIT
// Copyright (c) 2023 Manuel Stoiber, German Aerospace Center (DLR)

#ifndef M3T_INCLUDE_M3T_DETECTOR_H_
#define M3T_INCLUDE_M3T_DETECTOR_H_

#include <m3t/camera.h>
#include <m3t/common.h>
#include <m3t/optimizer.h>

namespace m3t {

/**
 * \brief Abstract class that is able to set the poses of \ref Link and \ref
 * Body objects assigned to a referenced \ref Optimizer objects.
 *
 * \details Using the main method `DetectPoses()`, the poses of all \ref Link
 * and \ref Body objects of \ref Optimizer objects that are specified in `names`
 * are defined. Names of optimizers are returnd in `detected_names`.
 *
 * @param reset_joint_poses reset the joint poses of all referenced \ref Link
 * objects.
 */
class Detector {
 public:
  // Setup method
  virtual bool SetUp() = 0;

  // Setters
  void set_name(const std::string &name);
  void set_metafile_path(const std::filesystem::path &metafile_path);
  void set_reset_joint_poses(bool reset_joint_poses);

  // Main methods
  virtual bool DetectPoses(const std::set<std::string> &names,
                           std::set<std::string> *detected_names) = 0;

  // Getters
  const std::string &name() const;
  const std::filesystem::path &metafile_path() const;
  bool reset_joint_poses() const;
  virtual std::vector<std::shared_ptr<Optimizer>> optimizer_ptrs() const;
  virtual std::shared_ptr<Camera> camera_ptr() const;
  bool set_up() const;

 protected:
  // Constructor
  Detector(const std::string &name, bool reset_joint_poses);
  Detector(const std::string &name, const std::filesystem::path &metafile_path);

  // Helper methods
  void UpdatePoses(const Transform3fA &link2world_pose,
                   const std::shared_ptr<Optimizer> &optimizer_ptr);

  // Variables and data
  std::string name_{};
  bool reset_joint_poses_ = true;
  std::filesystem::path metafile_path_{};
  bool set_up_ = false;
};

}  // namespace m3t

#endif  // M3T_INCLUDE_M3T_DETECTOR_H_
