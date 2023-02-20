// SPDX-License-Identifier: MIT
// Copyright (c) 2023 Manuel Stoiber, German Aerospace Center (DLR)

#include <m3t/modality.h>

namespace m3t {

void Modality::set_name(const std::string &name) { name_ = name; }

void Modality::set_metafile_path(const std::filesystem::path &metafile_path) {
  metafile_path_ = metafile_path;
  set_up_ = false;
}

void Modality::set_body_ptr(const std::shared_ptr<Body> &body_ptr) {
  body_ptr_ = body_ptr;
  set_up_ = false;
}

void Modality::set_visualize_pose_result(bool visualize_pose_result) {
  visualize_pose_result_ = visualize_pose_result;
}

void Modality::set_visualize_gradient_optimization(
    bool visualize_gradient_optimization) {
  visualize_gradient_optimization_ = visualize_gradient_optimization;
}

void Modality::set_visualize_hessian_optimization(
    bool visualize_hessian_optimization) {
  visualize_hessian_optimization_ = visualize_hessian_optimization;
}

void Modality::set_display_visualization(bool display_visualization) {
  display_visualization_ = display_visualization;
}

void Modality::StartSavingVisualizations(
    const std::filesystem::path &save_directory,
    const std::string &save_image_type) {
  save_visualizations_ = true;
  save_directory_ = save_directory;
  save_image_type_ = save_image_type;
}

void Modality::StopSavingVisualizations() { save_visualizations_ = false; }

const Eigen::Matrix<float, 6, 1> &Modality::gradient() const {
  return gradient_;
}

const Eigen::Matrix<float, 6, 6> &Modality::hessian() const { return hessian_; }

const std::string &Modality::name() const { return name_; }

const std::filesystem::path &Modality::metafile_path() const {
  return metafile_path_;
}

const std::shared_ptr<Body> &Modality::body_ptr() const { return body_ptr_; }

std::shared_ptr<Model> Modality::model_ptr() const { return nullptr; }

std::vector<std::shared_ptr<Renderer>> Modality::start_modality_renderer_ptrs()
    const {
  return {};
}

std::vector<std::shared_ptr<Renderer>> Modality::correspondence_renderer_ptrs()
    const {
  return {};
}

std::vector<std::shared_ptr<Renderer>> Modality::results_renderer_ptrs() const {
  return {};
}

std::shared_ptr<ColorHistograms> Modality::color_histograms_ptr() const {
  return nullptr;
}

bool Modality::set_up() const { return set_up_; }

bool Modality::imshow_correspondence() const { return imshow_correspondence_; }

bool Modality::imshow_optimization() const { return imshow_optimization_; }

bool Modality::imshow_result() const { return imshow_result_; }

bool Modality::visualize_pose_result() const { return visualize_pose_result_; }

bool Modality::visualize_gradient_optimization() const {
  return visualize_gradient_optimization_;
}

bool Modality::visualize_hessian_optimization() const {
  return visualize_hessian_optimization_;
}

bool Modality::display_visualization() const { return display_visualization_; }

const std::filesystem::path &Modality::save_directory() const {
  return save_directory_;
}

const std::string &Modality::save_image_type() const {
  return save_image_type_;
}

Modality::Modality(const std::string &name,
                   const std::shared_ptr<Body> &body_ptr)
    : name_{name}, body_ptr_{body_ptr} {}

Modality::Modality(const std::string &name,
                   const std::filesystem::path &metafile_path,
                   const std::shared_ptr<Body> &body_ptr)
    : name_{name}, metafile_path_{metafile_path}, body_ptr_{body_ptr} {}

void Modality::VisualizePose() {
  std::cout << "----------------------------------------"
            << "----------------------------------------" << std::endl;
  std::cout << name_ << ": Body2World Pose = " << std::endl;
  for (int i = 0; i < 4; ++i) {
    for (int j = 0; j < 4; ++j) {
      std::cout << body_ptr_->body2world_pose().matrix()(i, j) << ", ";
    }
    std::cout << std::endl;
  }
}

void Modality::VisualizeGradient() {
  std::cout << "----------------------------------------"
            << "----------------------------------------" << std::endl;
  std::cout << name_ << ": Gradient = " << std::endl;
  std::cout << gradient_ << std::endl;
}

void Modality::VisualizeHessian() {
  std::cout << "----------------------------------------"
            << "----------------------------------------" << std::endl;
  std::cout << name_ << ": Hessian = " << std::endl;
  std::cout << hessian_ << std::endl;
}

}  // namespace m3t
