// SPDX-License-Identifier: MIT
// Copyright (c) 2023 Manuel Stoiber, German Aerospace Center (DLR)

#ifndef M3T_INCLUDE_M3T_REFINER_H_
#define M3T_INCLUDE_M3T_REFINER_H_

#include <m3t/body.h>
#include <m3t/constraint.h>
#include <m3t/link.h>
#include <m3t/modality.h>
#include <m3t/optimizer.h>

#include <string>
#include <vector>

namespace m3t {

/**
 * \brief Class that coordinates \ref Optimizer, \ref Modality, and \ref
 * Renderer objects to refine the pose of multiple \ref Body objects.
 *
 * \details The main exception with respect to the \ref Tracker is that
 * modalities are started every time before correspondences are calculated.
 * Using the `RefinePoses()` method, body poses are refined. During `SetUp()`
 * all required objeccts like \ref Link, \ref Constraint, \ref Modality, \ref
 * Model, \ref Renderer, \ref RendererGeometry, and \ref Body objects are
 * derived from \ref Optimizer objects.
 *
 * @param optimizer_ptrs referenced \ref Optimizer objects that are
 * considered in the refinement.
 * @param n_corr_iterations number of times new correspondences are established.
 * @param n_update_iterations number of times the pose is updated for each
 * correspondence iteration.
 * @param visualization_time time in milliseconds visualizations are shown. 0
 * corresponds to infinite time.
 */
class Refiner {
 public:
  // Constructor and setup method
  Refiner(const std::string &name, int n_corr_iterations = 7,
          int n_update_iterations = 2, int visualization_time = 0);
  Refiner(const std::string &name, const std::filesystem::path &metafile_path);
  bool SetUp(bool set_up_all_objects = true);

  // Configure optimizer
  bool AddOptimizer(const std::shared_ptr<Optimizer> &optimizer_ptr);
  bool DeleteOptimizer(const std::string &name);
  void ClearOptimizers();

  // Setters
  void set_name(const std::string &name);
  void set_metafile_path(const std::filesystem::path &metafile_path);
  void set_n_corr_iterations(int n_corr_iterations);
  void set_n_update_iterations(int n_update_iterations);
  void set_visualization_time(int visualization_time);

  // Main method
  bool RefinePoses(const std::set<std::string> &names);

  // Methods of for advanced use
  bool ExecuteRefinementStep();

  // Individual steps of refinement
  bool CalculateConsistentPoses();
  bool StartModalities(int corr_iteration);
  bool CalculateCorrespondences(int corr_iteration);
  bool VisualizeCorrespondences(int save_idx);
  bool CalculateGradientAndHessian(int corr_iteration, int opt_iteration);
  bool CalculateOptimization(int corr_iteration, int opt_iteration);
  bool VisualizeOptimization(int save_idx);

  // Getters
  const std::string &name() const;
  const std::filesystem::path &metafile_path() const;
  const std::vector<std::shared_ptr<Optimizer>> &optimizer_ptrs() const;
  const std::vector<std::shared_ptr<Constraint>> &constraint_ptrs() const;
  const std::vector<std::shared_ptr<Link>> &link_ptrs() const;
  const std::vector<std::shared_ptr<Modality>> &modality_ptrs() const;
  const std::vector<std::shared_ptr<Model>> &model_ptrs() const;
  const std::vector<std::shared_ptr<RendererGeometry>> &renderer_geometry_ptrs()
      const;
  const std::vector<std::shared_ptr<Body>> &body_ptrs() const;
  const std::vector<std::shared_ptr<Renderer>> &start_modality_renderer_ptrs()
      const;
  const std::vector<std::shared_ptr<Renderer>> &correspondence_renderer_ptrs()
      const;
  const std::vector<std::shared_ptr<ColorHistograms>> &color_histograms_ptrs()
      const;
  int n_corr_iterations() const;
  int n_update_iterations() const;
  int visualization_time() const;
  bool set_up() const;

 private:
  // Helper methods
  bool LoadMetaData();
  void AssambleInternallyUsedObjectPtrs(const std::set<std::string> &names);
  void AssembleDerivedObjectPtrs();
  bool SetUpAllObjects();
  bool AreAllObjectsSetUp();

  // Objects
  std::vector<std::shared_ptr<Optimizer>> optimizer_ptrs_{};
  std::vector<std::shared_ptr<Constraint>> constraint_ptrs_{};
  std::vector<std::shared_ptr<Link>> link_ptrs_{};
  std::vector<std::shared_ptr<Modality>> modality_ptrs_{};
  std::vector<std::shared_ptr<Model>> model_ptrs_{};
  std::vector<std::shared_ptr<RendererGeometry>> renderer_geometry_ptrs_{};
  std::vector<std::shared_ptr<Body>> body_ptrs_{};
  std::vector<std::shared_ptr<Renderer>> start_modality_renderer_ptrs_{};
  std::vector<std::shared_ptr<Renderer>> correspondence_renderer_ptrs_{};
  std::vector<std::shared_ptr<ColorHistograms>> color_histograms_ptrs_{};

  // Internally used objects
  std::vector<std::shared_ptr<Optimizer>> used_optimizer_ptrs_{};
  std::vector<std::shared_ptr<Modality>> used_modality_ptrs_{};
  std::vector<std::shared_ptr<Renderer>> used_start_modality_renderer_ptrs_{};
  std::vector<std::shared_ptr<Renderer>> used_correspondence_renderer_ptrs_{};
  std::vector<std::shared_ptr<ColorHistograms>> used_color_histograms_ptrs_{};

  // Parameters
  std::string name_{};
  std::filesystem::path metafile_path_{};
  int n_corr_iterations_ = 7;
  int n_update_iterations_ = 2;
  int visualization_time_ = 0;
  bool set_up_ = false;
};

}  // namespace m3t

#endif  // M3T_INCLUDE_M3T_REFINER_H_
