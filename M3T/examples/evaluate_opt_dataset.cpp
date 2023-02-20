// SPDX-License-Identifier: MIT
// Copyright (c) 2023 Manuel Stoiber, German Aerospace Center (DLR)

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
    r->set_n_lines_max(200);
    r->set_use_adaptive_coverage(false);
    r->set_min_continuous_distance(3.0f);
    r->set_function_length(8);
    r->set_distribution_length(12);
    r->set_function_amplitude(0.43f);
    r->set_function_slope(0.5f);
    r->set_learning_rate(1.3f);
    r->set_scales({6, 4, 1});
    r->set_standard_deviations({15.0f, 5.0f, 1.5f}); // {25.0f, 10.0f, 2.5f}
    r->set_n_histogram_bins(16); // 32
    r->set_learning_rate_f(0.2f);
    r->set_learning_rate_b(0.2f);
    r->set_unconsidered_line_length(0.5f);
    r->set_max_considered_line_length(20.0f);
  });
  evaluator.set_depth_modality_setter([&](auto d) {
    d->set_n_points_max(200);
    d->set_use_adaptive_coverage(false);
    d->set_use_depth_scaling(false);
    d->set_stride_length(0.005f);
    d->set_considered_distances({0.05f, 0.02f, 0.01f});
    d->set_standard_deviations({0.035f, 0.035f, 0.025f});
  });
  evaluator.set_texture_modality_setter([&](auto t) {
    t->set_descriptor_type(m3t::TextureModality::DescriptorType::ORB);
    t->set_focused_image_size(200);
    t->set_descriptor_distance_threshold(0.7f);
    t->set_tukey_norm_constant(20.0f);
    t->set_standard_deviations({5.0f, 1.0f, 0.5f});
    t->set_max_keyframe_rotation_difference(10.0f * m3t::kPi / 180.0f);
    t->set_max_keyframe_age(1000);
    t->set_n_keyframes(1);
    t->set_orb_n_features(300);
    t->set_orb_scale_factor(1.2f);
    t->set_orb_n_levels(3);
    t->set_brisk_threshold(35);
    t->set_brisk_octave(3);
    t->set_brisk_pattern_scale(0.6f);
    t->set_daisy_radius(8.0f);
    t->set_daisy_q_radius(3);
    t->set_daisy_q_theta(4);
    t->set_daisy_q_hist(8);
    t->set_freak_orientation_normalized(true);
    t->set_freak_scale_normalized(true);
    t->set_freak_pattern_scale(16.0f);
    t->set_freak_n_octaves(4);
    t->set_sift_n_features(0);
    t->set_sift_n_octave_layers(3);
    t->set_sift_contrast_threshold(0.04);
    t->set_sift_edge_threshold(10.0);
    t->set_sift_sigma(0.7f);
  });
  evaluator.set_optimizer_setter([&](auto o) {
    o->set_tikhonov_parameter_rotation(1000.0f);
    o->set_tikhonov_parameter_translation(30000.0f);
    // if(o->root_link_ptrs()[0]->body_ptr()->name() == "soda")
    //    o->set_tikhonov_parameter_rotation(30000.0f);
  });
  evaluator.set_tracker_setter([&](auto t) {
    t->set_n_update_iterations(2);
    t->set_n_corr_iterations(4); // 7
  });
  evaluator.set_run_sequentially(true);
  evaluator.set_use_random_seed(false);
  evaluator.set_n_vertices_evaluation(1000);
  evaluator.set_visualize_frame_results(false);
  evaluator.set_visualize_tracking(false);
  evaluator.set_use_region_modality(true);
  evaluator.set_use_depth_modality(true);
  evaluator.set_use_texture_modality(true);
  evaluator.SetUp();
  evaluator.Evaluate();
  evaluator.SaveResults(result_path);
  return 0;
}
