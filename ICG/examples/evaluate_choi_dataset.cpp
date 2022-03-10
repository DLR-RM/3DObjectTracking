// SPDX-License-Identifier: MIT
// Copyright (c) 2022 Manuel Stoiber, German Aerospace Center (DLR)

#include "choi_evaluator.h"

int main() {
  // Directories
  std::filesystem::path dataset_directory{"/your/path/"};
  std::filesystem::path external_directory{"/your/path/"};
  std::filesystem::path result_path{"/your/path/"};

  // Dataset configuration
  std::vector<std::string> body_names{"kinect_box", "milk", "orange_juice",
                                      "tide"};

  // Run experiments
  ChoiEvaluator evaluator{"evaluator", dataset_directory, external_directory,
                          body_names};
  evaluator.set_region_modality_setter([&](auto r) {
    r->set_n_lines(200);
    r->set_min_continuous_distance(3.0f);
    r->set_function_length(8);
    r->set_distribution_length(12);
    r->set_function_amplitude(0.43f);
    r->set_function_slope(0.5f);
    r->set_learning_rate(1.3f);
    r->set_scales({2, 1});
    r->set_standard_deviations({5.0f});
    r->set_n_histogram_bins(16);
    r->set_learning_rate_f(0.2f);
    r->set_learning_rate_b(0.2f);
    r->set_unconsidered_line_length(0.5f);
    r->set_max_considered_line_length(20.0f);
    r->set_measured_depth_offset_radius(0.01f);
    r->set_measured_occlusion_radius(0.01f);
    r->set_measured_occlusion_threshold(0.03f);
  });
  evaluator.set_depth_modality_setter([&](auto d) {
    d->set_n_points(200);
    d->set_stride_length(0.005f);
    d->set_considered_distances({0.01f});
    d->set_standard_deviations({0.01f, 0.001f});
    d->set_measured_depth_offset_radius(0.01f);
    d->set_measured_occlusion_radius(0.01f);
    d->set_measured_occlusion_threshold(0.03f);
  });
  evaluator.set_optimizer_setter([&](auto o) {
    o->set_tikhonov_parameter_rotation(1000.0f);
    o->set_tikhonov_parameter_translation(30000.0f);
  });
  evaluator.set_tracker_setter([&](auto t) {
    t->set_n_update_iterations(2);
    t->set_n_corr_iterations(4);
  });
  evaluator.set_use_region_modality(true);
  evaluator.set_use_depth_modality(true);
  evaluator.set_measure_occlusions_region(true);
  evaluator.set_measure_occlusions_depth(true);
  evaluator.set_visualize_frame_results(false);
  evaluator.set_visualize_tracking(false);
  evaluator.SetUp();
  evaluator.Evaluate();
  evaluator.SaveResults(result_path);
}
