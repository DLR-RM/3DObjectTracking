// SPDX-License-Identifier: MIT
// Copyright (c) 2022 Manuel Stoiber, German Aerospace Center (DLR)

#ifndef ICG_INCLUDE_ICG_OPTIMIZER_H_
#define ICG_INCLUDE_ICG_OPTIMIZER_H_

#include <icg/body.h>
#include <icg/common.h>
#include <icg/modality.h>

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <iostream>
#include <string>
#include <unsupported/Eigen/MatrixFunctions>
#include <vector>

namespace icg {

/**
 * \brief Class that uses the gradient vectors and Hessian matrices from
 * multiple \ref Modality objects to update the pose of a \ref Body.
 *
 * \details With the main method `CalculateOptimization()`, information from all
 * referenced modalities is used to update the pose of the body referenced by
 * the modalities. It is important that all modalities reference the same body.
 *
 * @param modality_ptrs defines the referenced \ref Modality objects that are
 * considered.
 * @param tikhonov_parameter_rotation regularization parameter for rotation.
 * @param tikhonov_parameter_translation regularization parameter for
 * translation.
 */
class Optimizer {
 public:
  // Constructors and setup methods
  Optimizer(const std::string &name,
            float tikhonov_parameter_rotation = 1000.0f,
            float tikhonov_parameter_translation = 30000.0f);
  Optimizer(const std::string &name,
            const std::filesystem::path &metafile_path);
  bool SetUp();

  // Configure modalities
  bool AddModality(const std::shared_ptr<Modality> &modality_ptr);
  bool DeleteModality(const std::string &name);
  void ClearModalities();

  // Setters
  void set_name(const std::string &name);
  void set_metafile_path(const std::filesystem::path &metafile_path);
  void set_tikhonov_parameter_rotation(float tikhonov_parameter_rotation);
  void set_tikhonov_parameter_translation(float tikhonov_parameter_translation);

  // Main methods
  bool CalculateOptimization(int iteration, int corr_iteration,
                             int opt_iteration);

  // Getters
  const std::string &name() const;
  const std::filesystem::path &metafile_path() const;
  const std::vector<std::shared_ptr<Modality>> &modality_ptrs() const;
  float tikhonov_parameter_rotation() const;
  float tikhonov_parameter_translation() const;
  bool set_up() const;

 private:
  // Helper methods
  bool LoadMetaData();

  // Data
  std::string name_{};
  std::filesystem::path metafile_path_{};
  std::vector<std::shared_ptr<Modality>> modality_ptrs_{};

  float tikhonov_parameter_rotation_ = 1000.0f;
  float tikhonov_parameter_translation_ = 30000.0f;
  bool set_up_ = false;
};

}  // namespace icg

#endif  // ICG_INCLUDE_ICG_OPTIMIZER_H_
