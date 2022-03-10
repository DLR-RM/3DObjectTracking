// SPDX-License-Identifier: MIT
// Copyright (c) 2022 Manuel Stoiber, German Aerospace Center (DLR)

#include "opt_evaluator.h"

int main() {
  // Directories
  std::filesystem::path dataset_directory{"/your/path/"};
  std::filesystem::path external_directory{"/your/path/"};
  std::filesystem::path result_path{"/your/path/"};

  // Dataset configuration
  std::vector<std::string> body_names{"soda",  "chest", "ironman",
                                      "house", "bike",  "jet"};
  std::vector<std::string> body_orientations{"b", "f", "l", "r"};
  std::vector<std::string> motion_patterns{
      "tr_1", "tr_2", "tr_3", "tr_4", "tr_5", "zo_1", "zo_2", "zo_3",
      "zo_4", "zo_5", "ir_1", "ir_2", "ir_3", "ir_4", "ir_5", "or_1",
      "or_2", "or_3", "or_4", "or_5", "fl",   "ml",   "fm"};

  // Run experiments
  OPTEvaluator evaluator{"evaluator", dataset_directory, external_directory,
                         body_names,  body_orientations, motion_patterns};
  evaluator.set_region_modality_setter([&](auto r) {
    r->set_n_lines(200);
    r->set_min_continuous_distance(3.0f);
    r->set_function_length(8);
    r->set_distribution_length(12);
    r->set_function_amplitude(0.43f);
    r->set_function_slope(0.5f);
    r->set_learning_rate(1.3f);
    r->set_scales({6, 4, 1});
    r->set_standard_deviations({15.0f, 5.0f, 1.5f});
    r->set_n_histogram_bins(16);
    r->set_learning_rate_f(0.2f);
    r->set_learning_rate_b(0.2f);
    r->set_unconsidered_line_length(0.5f);
    r->set_max_considered_line_length(20.0f);
  });
  evaluator.set_depth_modality_setter([&](auto d) {
    d->set_n_points(200);
    d->set_stride_length(0.005f);
    d->set_considered_distances({0.05f, 0.02f, 0.01f});
    d->set_standard_deviations({0.035f, 0.035f, 0.025f});
  });
  evaluator.set_optimizer_setter([&](auto o) {
    o->set_tikhonov_parameter_rotation(1000.0f);
    if (o->modality_ptrs()[0]->body_ptr()->name() == "soda")
      o->set_tikhonov_parameter_rotation(70000.0f);
    o->set_tikhonov_parameter_translation(30000.0f);
  });
  evaluator.set_tracker_setter([&](auto t) {
    t->set_n_update_iterations(2);
    t->set_n_corr_iterations(4);
  });
  evaluator.set_use_region_modality(true);
  evaluator.set_use_depth_modality(true);
  evaluator.set_use_random_seed(false);
  evaluator.set_n_vertices_evaluation(1000);
  evaluator.set_visualize_frame_results(false);
  evaluator.set_visualize_tracking(false);
  evaluator.SetUp();
  evaluator.Evaluate();
  evaluator.SaveResults(result_path);
}
