// SPDX-License-Identifier: MIT
// Copyright (c) 2022 Manuel Stoiber, German Aerospace Center (DLR)

#include "rbot_evaluator.h"

int main() {
  // Directories
  std::filesystem::path dataset_directory{"/your/path/"};
  std::filesystem::path external_directory{"/your/path/"};
  std::filesystem::path result_directory{"/your/path/"};

  // Dataset configuration
  std::vector<std::string> body_names{
      "ape",  "bakingsoda", "benchviseblue", "broccolisoup", "cam",
      "can",  "cat",        "clown",         "cube",         "driller",
      "duck", "eggbox",     "glue",          "iron",         "koalacandy",
      "lamp", "phone",      "squirrel"};
  std::vector<std::string> sequence_names{
      "a_regular", "b_dynamiclight", "c_noisy", "d_occlusion", "d_occlusion"};
  std::vector<bool> sequence_occlusions{false, false, false, false, true};

  // Run experiments
  RBOTEvaluator evaluator{"evaluator", dataset_directory, external_directory,
                          body_names,  sequence_names,    sequence_occlusions};
  evaluator.set_region_modality_setter([&](auto r) {
    r->set_n_lines(200);
    r->set_min_continuous_distance(6.0f);
    r->set_function_length(8);
    r->set_distribution_length(12);
    r->set_function_amplitude(0.36f);
    r->set_function_slope(0.0f);
    r->set_learning_rate(1.3f);
    r->set_scales({5, 2, 2, 1});
    r->set_standard_deviations({15.0f, 5.0f, 3.5f, 1.5f});
    r->set_n_histogram_bins(32);
    r->set_learning_rate_f(0.2f);
    r->set_learning_rate_b(0.2f);
    r->set_unconsidered_line_length(1.0f);
    r->set_max_considered_line_length(18.0f);
    r->set_modeled_depth_offset_radius(0.0f);
    r->set_modeled_occlusion_radius(0.0f);
    r->set_modeled_occlusion_threshold(0.03f);
  });
  evaluator.set_optimizer_setter([&](auto o) {
    o->set_tikhonov_parameter_rotation(1000.0f);
    o->set_tikhonov_parameter_translation(100000.0f);
  });
  evaluator.set_tracker_setter([&](auto t) {
    t->set_n_update_iterations(2);
    t->set_n_corr_iterations(7);
  });
  evaluator.set_visualize_all_results(false);
  evaluator.SaveResults(result_directory);
  evaluator.SetUp();
  evaluator.Evaluate();
}
