// SPDX-License-Identifier: MIT
// Copyright (c) 2023 Manuel Stoiber, German Aerospace Center (DLR)

#ifndef M3T_INCLUDE_M3T_STATIC_DETECTOR_H_
#define M3T_INCLUDE_M3T_STATIC_DETECTOR_H_

#include <m3t/common.h>
#include <m3t/detector.h>
#include <m3t/optimizer.h>

#include <filesystem/filesystem.h>
#include <Eigen/Geometry>
#include <iostream>
#include <opencv2/opencv.hpp>
#include <string>

namespace m3t {

/**
 * \brief \ref Detector that sets the pose of all \ref Link and \ref Body
 * objects referenced by a single \ref Optimizer object based on a defined pose
 * for the root_link.
 *
 * @param optimizer_ptr referenced \ref Optimizer object for which referenced
 * body and link poses are set.
 * @param link2world_pose pose that is assigned to the root_link.
 */
class StaticDetector : public Detector {
 public:
  // Constructor and setup method
  StaticDetector(const std::string &name,
                 const std::shared_ptr<Optimizer> &optimizer_ptr,
                 const Transform3fA &link2world_pose,
                 bool reset_joint_poses = true);
  StaticDetector(const std::string &name,
                 const std::filesystem::path &metafile_path,
                 const std::shared_ptr<m3t::Optimizer> &optimizer_ptr);
  bool SetUp() override;

  // Setters
  void set_optimizer_ptr(const std::shared_ptr<Optimizer> &optimizer_ptr);
  void set_link2world_pose(const Transform3fA &link2world_pose);

  // Main methods
  bool DetectPoses(const std::set<std::string> &names,
                   std::set<std::string> *detected_names = nullptr) override;

  // Getters
  const std::shared_ptr<Optimizer> &optimizer_ptr() const;
  const Transform3fA &link2world_pose() const;
  std::vector<std::shared_ptr<Optimizer>> optimizer_ptrs() const override;

 private:
  bool LoadMetaData();

  std::shared_ptr<Optimizer> optimizer_ptr_{};
  Transform3fA link2world_pose_{};
};

}  // namespace m3t

#endif  // M3T_INCLUDE_M3T_STATIC_DETECTOR_H_
