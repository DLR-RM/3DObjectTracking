// SPDX-License-Identifier: MIT
// Copyright (c) 2022 Manuel Stoiber, German Aerospace Center (DLR)

#ifndef ICG_INCLUDE_ICG_STATIC_DETECTOR_H_
#define ICG_INCLUDE_ICG_STATIC_DETECTOR_H_

#include <icg/body.h>
#include <icg/common.h>
#include <icg/detector.h>

#include <filesystem/filesystem.h>
#include <Eigen/Geometry>
#include <iostream>
#include <opencv2/opencv.hpp>
#include <string>

namespace icg {

/**
 * \brief \ref Detector that assigns a defined pose to a referenced \ref Body
 * object.
 *
 * @param body_ptr referenced \ref Body object for which the pose is set.
 * @param body2world_pose pose that is assigned to the \ref Body object.
 */
class StaticDetector : public Detector {
 public:
  // Constructor and setup method
  StaticDetector(const std::string &name, const std::shared_ptr<Body> &body_ptr,
                 const Transform3fA &body2world_pose);
  StaticDetector(const std::string &name,
                 const std::filesystem::path &metafile_path,
                 const std::shared_ptr<icg::Body> &body_ptr);
  bool SetUp() override;

  // Setters
  void set_body_ptr(const std::shared_ptr<Body> &body_ptr);
  void set_body2world_pose(const Transform3fA &body2world_pose);

  // Main methods
  bool DetectBody() override;

  // Getters
  const std::shared_ptr<Body> &body_ptr() const;
  const Transform3fA &body2world_pose() const;
  std::vector<std::shared_ptr<Body>> body_ptrs() const override;

 private:
  bool LoadMetaData();

  std::shared_ptr<Body> body_ptr_{};
  Transform3fA body2world_pose_{};
};

}  // namespace icg

#endif  // ICG_INCLUDE_ICG_STATIC_DETECTOR_H_
