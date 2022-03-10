// SPDX-License-Identifier: MIT
// Copyright (c) 2022 Manuel Stoiber, German Aerospace Center (DLR)

#include <icg/refiner.h>

namespace icg {

Refiner::Refiner(const std::string &name, int n_corr_iterations,
                 int n_update_iterations, int visualization_time)
    : name_{name},
      n_corr_iterations_{n_corr_iterations},
      n_update_iterations_{n_update_iterations},
      visualization_time_{visualization_time} {}

Refiner::Refiner(const std::string &name,
                 const std::filesystem::path &metafile_path)
    : name_{name}, metafile_path_{metafile_path} {}

bool Refiner::SetUp(bool set_up_all_objects) {
  set_up_ = false;
  if (!metafile_path_.empty())
    if (!LoadMetaData()) return false;
  AssembleDerivedObjectPtrs();
  if (set_up_all_objects) {
    if (!SetUpAllObjects()) return false;
  } else {
    if (!AreAllObjectsSetUp()) return false;
  }
  set_up_ = true;
  return true;
}

bool Refiner::AddOptimizer(const std::shared_ptr<Optimizer> &optimizer_ptr) {
  set_up_ = false;
  if (!AddPtrIfNameNotExists(optimizer_ptr, &optimizer_ptrs_)) {
    std::cerr << "Optimizer " << optimizer_ptr->name() << " already exists"
              << std::endl;
    return false;
  }
  return true;
}

bool Refiner::DeleteOptimizer(const std::string &name) {
  set_up_ = false;
  if (!DeletePtrIfNameExists(name, &optimizer_ptrs_)) {
    std::cerr << "Optimizer " << name << " not found" << std::endl;
    return false;
  }
  return true;
}

void Refiner::ClearOptimizers() {
  set_up_ = false;
  optimizer_ptrs_.clear();
}

void Refiner::set_name(const std::string &name) { name_ = name; }

void Refiner::set_metafile_path(const std::filesystem::path &metafile_path) {
  metafile_path_ = metafile_path;
  set_up_ = false;
}

void Refiner::set_n_corr_iterations(int n_corr_iterations) {
  n_corr_iterations_ = n_corr_iterations;
}

void Refiner::set_n_update_iterations(int n_update_iterations) {
  n_update_iterations_ = n_update_iterations;
}

void Refiner::set_visualization_time(int visualization_time) {
  visualization_time_ = visualization_time;
}

bool Refiner::RefinePoses() {
  if (!set_up_) {
    std::cerr << "Set up refiner " << name_ << " first" << std::endl;
    return false;
  }

  for (int corr_iteration = 0; corr_iteration < n_corr_iterations_;
       ++corr_iteration) {
    int corr_save_idx = corr_iteration;
    if (!StartModalities(corr_iteration)) return false;
    if (!CalculateCorrespondences(corr_iteration)) return false;
    if (!VisualizeCorrespondences(corr_save_idx)) return false;
    for (int update_iteration = 0; update_iteration < n_update_iterations_;
         ++update_iteration) {
      int update_save_idx =
          corr_save_idx * n_update_iterations_ + update_iteration;
      if (!CalculateGradientAndHessian(corr_iteration, update_iteration))
        return false;
      if (!CalculateOptimization(corr_iteration, update_iteration))
        return false;
      if (!VisualizeOptimization(update_save_idx)) return false;
    }
  }
  return true;
}

bool Refiner::StartModalities(int corr_iteration) {
  for (auto &modality_ptr : modality_ptrs_) {
    if (!modality_ptr->StartModality(0, corr_iteration)) return false;
  }
  return true;
}

bool Refiner::CalculateCorrespondences(int corr_iteration) {
  for (auto &correspondence_renderer_ptr : correspondence_renderer_ptrs_) {
    if (!correspondence_renderer_ptr->StartRendering()) return false;
  }
  for (auto &modality_ptr : modality_ptrs_) {
    if (!modality_ptr->CalculateCorrespondences(0, corr_iteration))
      return false;
  }
  return true;
}

bool Refiner::VisualizeCorrespondences(int save_idx) {
  bool imshow_correspondences = false;
  for (auto &modality_ptr : modality_ptrs_) {
    if (!modality_ptr->VisualizeCorrespondences(save_idx)) return false;
    if (modality_ptr->imshow_correspondence()) imshow_correspondences = true;
  }
  if (imshow_correspondences) {
    if (cv::waitKey(visualization_time_) == 'q') return false;
  }
  return true;
}

bool Refiner::CalculateGradientAndHessian(int corr_iteration,
                                          int update_iteration) {
  for (auto &modality_ptr : modality_ptrs_) {
    if (!modality_ptr->CalculateGradientAndHessian(0, corr_iteration,
                                                   update_iteration))
      return false;
  }
  return true;
}

bool Refiner::CalculateOptimization(int corr_iteration, int update_iteration) {
  for (auto &optimizer_ptr : optimizer_ptrs_) {
    if (!optimizer_ptr->CalculateOptimization(0, corr_iteration,
                                              update_iteration))
      return false;
  }
  return true;
}

bool Refiner::VisualizeOptimization(int save_idx) {
  bool imshow_pose_update = false;
  for (auto &modality_ptr : modality_ptrs_) {
    if (!modality_ptr->VisualizeOptimization(save_idx)) return false;
    if (modality_ptr->imshow_optimization()) imshow_pose_update = true;
  }
  if (imshow_pose_update) {
    if (cv::waitKey(visualization_time_) == 'q') return false;
  }
  return true;
}

const std::string &Refiner::name() const { return name_; }

const std::filesystem::path &Refiner::metafile_path() const {
  return metafile_path_;
}

const std::vector<std::shared_ptr<Optimizer>> &Refiner::optimizer_ptrs() const {
  return optimizer_ptrs_;
}

const std::vector<std::shared_ptr<Modality>> &Refiner::modality_ptrs() const {
  return modality_ptrs_;
}

const std::vector<std::shared_ptr<Model>> &Refiner::model_ptrs() const {
  return model_ptrs_;
}

const std::vector<std::shared_ptr<RendererGeometry>>
    &Refiner::renderer_geometry_ptrs() const {
  return renderer_geometry_ptrs_;
}

const std::vector<std::shared_ptr<Body>> &Refiner::body_ptrs() const {
  return body_ptrs_;
}

const std::vector<std::shared_ptr<Renderer>>
    &Refiner::start_modality_renderer_ptrs() const {
  return start_modality_renderer_ptrs_;
}

const std::vector<std::shared_ptr<Renderer>>
    &Refiner::correspondence_renderer_ptrs() const {
  return correspondence_renderer_ptrs_;
}

int Refiner::n_corr_iterations() const { return n_corr_iterations_; }

int Refiner::n_update_iterations() const { return n_update_iterations_; }

int Refiner::visualization_time() const { return visualization_time_; }

bool Refiner::set_up() const { return set_up_; }

bool Refiner::LoadMetaData() {
  // Open file storage from yaml
  cv::FileStorage fs;
  if (!OpenYamlFileStorage(metafile_path_, &fs)) return false;

  // Read parameters from yaml
  ReadOptionalValueFromYaml(fs, "n_corr_iterations", &n_corr_iterations_);
  ReadOptionalValueFromYaml(fs, "n_update_iterations", &n_update_iterations_);
  ReadOptionalValueFromYaml(fs, "visualization_time", &visualization_time_);
  fs.release();
  return true;
}

void Refiner::AssembleDerivedObjectPtrs() {
  modality_ptrs_.clear();
  model_ptrs_.clear();
  start_modality_renderer_ptrs_.clear();
  correspondence_renderer_ptrs_.clear();
  renderer_geometry_ptrs_.clear();
  body_ptrs_.clear();

  // Assemble objects from optimizer
  for (auto &optimizer_ptr : optimizer_ptrs_) {
    AddPtrsIfNameNotExists(optimizer_ptr->modality_ptrs(), &modality_ptrs_);
  }

  // Assemble objects from modalities
  for (auto &modality_ptr : modality_ptrs_) {
    AddPtrIfNameNotExists(modality_ptr->model_ptr(), &model_ptrs_);

    AddPtrsIfNameNotExists(modality_ptr->start_modality_renderer_ptrs(),
                           &start_modality_renderer_ptrs_);
    AddPtrsIfNameNotExists(modality_ptr->correspondence_renderer_ptrs(),
                           &correspondence_renderer_ptrs_);

    AddPtrIfNameNotExists(modality_ptr->body_ptr(), &body_ptrs_);
  }

  // Assemble objects from models
  for (auto &model_ptr : model_ptrs_) {
    AddPtrIfNameNotExists(model_ptr->body_ptr(), &body_ptrs_);
  }

  // Assemble objects from renderers
  for (auto &start_modality_renderer_ptr : start_modality_renderer_ptrs_) {
    AddPtrIfNameNotExists(start_modality_renderer_ptr->renderer_geometry_ptr(),
                          &renderer_geometry_ptrs_);
    AddPtrsIfNameNotExists(start_modality_renderer_ptr->referenced_body_ptrs(),
                           &body_ptrs_);
  }
  for (auto &correspondence_renderer_ptr : correspondence_renderer_ptrs_) {
    AddPtrIfNameNotExists(correspondence_renderer_ptr->renderer_geometry_ptr(),
                          &renderer_geometry_ptrs_);
    AddPtrsIfNameNotExists(correspondence_renderer_ptr->referenced_body_ptrs(),
                           &body_ptrs_);
  }

  // Assemble objects from renderer geometry
  for (auto &renderer_geometry_ptr : renderer_geometry_ptrs_) {
    AddPtrsIfNameNotExists(renderer_geometry_ptr->body_ptrs(), &body_ptrs_);
  }
}

bool Refiner::SetUpAllObjects() {
  return SetUpObjectPtrs(&body_ptrs_) &&
         SetUpObjectPtrs(&renderer_geometry_ptrs_) &&
         SetUpObjectPtrs(&start_modality_renderer_ptrs_) &&
         SetUpObjectPtrs(&correspondence_renderer_ptrs_) &&
         SetUpObjectPtrs(&model_ptrs_) && SetUpObjectPtrs(&modality_ptrs_) &&
         SetUpObjectPtrs(&optimizer_ptrs_);
}

bool Refiner::AreAllObjectsSetUp() {
  return AreObjectPtrsSetUp(&body_ptrs_) &&
         AreObjectPtrsSetUp(&renderer_geometry_ptrs_) &&
         AreObjectPtrsSetUp(&start_modality_renderer_ptrs_) &&
         AreObjectPtrsSetUp(&correspondence_renderer_ptrs_) &&
         AreObjectPtrsSetUp(&model_ptrs_) &&
         AreObjectPtrsSetUp(&modality_ptrs_) &&
         AreObjectPtrsSetUp(&optimizer_ptrs_);
}

}  // namespace icg
