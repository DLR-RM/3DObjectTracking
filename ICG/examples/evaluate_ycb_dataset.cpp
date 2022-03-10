// SPDX-License-Identifier: MIT
// Copyright (c) 2022 Manuel Stoiber, German Aerospace Center (DLR)

#include "ycb_evaluator.h"

int main() {
  // Directories
  std::filesystem::path dataset_directory{"/your/path/"};
  std::filesystem::path external_directory{"/your/path/"};
  std::filesystem::path result_path{"/your/path/"};

  // Dataset configuration
  std::vector<std::string> body_names{"002_master_chef_can",
                                      "003_cracker_box",
                                      "004_sugar_box",
                                      "005_tomato_soup_can",
                                      "006_mustard_bottle",
                                      "007_tuna_fish_can",
                                      "008_pudding_box",
                                      "009_gelatin_box",
                                      "010_potted_meat_can",
                                      "011_banana",
                                      "019_pitcher_base",
                                      "021_bleach_cleanser",
                                      "024_bowl",
                                      "025_mug",
                                      "035_power_drill",
                                      "036_wood_block",
                                      "037_scissors",
                                      "040_large_marker",
                                      "051_large_clamp",
                                      "052_extra_large_clamp",
                                      "061_foam_brick"};
  std::vector<int> sequence_ids{48, 49, 50, 51, 52, 53, 54, 55, 56, 57, 58, 59};

  // Run experiments
  YCBEvaluator evaluator{"evaluator", dataset_directory, external_directory,
                         sequence_ids, body_names};
  evaluator.set_region_modality_setter([&](auto r) {
    r->set_n_lines(200);
    r->set_min_continuous_distance(3.0f);
    r->set_function_length(8);
    r->set_distribution_length(12);
    r->set_function_amplitude(0.43f);
    r->set_function_slope(0.5f);
    r->set_learning_rate(1.3f);
    r->set_scales({7, 4, 2});
    r->set_standard_deviations({25.0f, 15.0f, 10.0f});
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
    d->set_considered_distances({0.07f, 0.05f, 0.04f});
    d->set_standard_deviations({0.05f, 0.03f, 0.02f});
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
  evaluator.set_evaluate_refinement(false);
  evaluator.set_use_matlab_gt_poses(true);
  evaluator.set_run_sequentially(true);
  evaluator.set_use_random_seed(false);
  evaluator.set_n_vertices_evaluation(1000);
  evaluator.set_visualize_frame_results(false);
  evaluator.set_visualize_tracking(false);
  evaluator.set_use_region_modality(true);
  evaluator.set_use_depth_modality(true);
  evaluator.set_measure_occlusions_region(true);
  evaluator.set_measure_occlusions_depth(true);
  evaluator.set_model_occlusions_region(false);
  evaluator.set_model_occlusions_depth(false);
  evaluator.SetUp();
  evaluator.Evaluate();
  evaluator.SaveResults(result_path);
}
