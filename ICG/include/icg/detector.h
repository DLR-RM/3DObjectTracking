// SPDX-License-Identifier: MIT
// Copyright (c) 2022 Manuel Stoiber, German Aerospace Center (DLR)

#ifndef ICG_INCLUDE_ICG_DETECTOR_H_
#define ICG_INCLUDE_ICG_DETECTOR_H_

#include <icg/body.h>
#include <icg/camera.h>
#include <icg/common.h>

namespace icg {

/**
 * \brief Abstract class that is able to set the pose of a referenced \ref Body
 * object.
 *
 * \details Using the main method `DetectBody()`, the pose of the referenced
 * \ref Body object is set.
 *
 */
class Detector {
 public:
  // Setup method
  virtual bool SetUp() = 0;

  // Setters
  void set_name(const std::string &name);
  void set_metafile_path(const std::filesystem::path &metafile_path);

  // Main methods
  virtual bool DetectBody() = 0;

  // Getters
  const std::string &name() const;
  const std::filesystem::path &metafile_path() const;
  virtual std::vector<std::shared_ptr<Body>> body_ptrs() const;
  virtual std::shared_ptr<Camera> camera_ptr() const;
  bool set_up() const;

 protected:
  // Constructor
  Detector(const std::string &name);
  Detector(const std::string &name, const std::filesystem::path &metafile_path);

  // Variables and data
  std::string name_{};
  std::filesystem::path metafile_path_{};
  bool set_up_ = false;
};

}  // namespace icg

#endif  // ICG_INCLUDE_ICG_DETECTOR_H_
