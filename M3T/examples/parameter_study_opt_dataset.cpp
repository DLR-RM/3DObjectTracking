// SPDX-License-Identifier: MIT
// Copyright (c) 2023 Manuel Stoiber, German Aerospace Center (DLR)

#include "opt_evaluator.h"

int main() {
  // Directories
  std::filesystem::path dataset_directory{"/your/path/"};
  std::filesystem::path external_directory{"/your/path/"};

  // Dataset configuration
  std::vector<std::string> body_names{"soda",  "chest", "ironman",
                                      "house", "bike",  "jet"};
  std::vector<std::string> body_orientations{"b", "f", "l", "r"};
  std::vector<std::string> motion_patterns{
      "tr_1", "tr_2", "tr_3", "tr_4", "tr_5", "zo_1", "zo_2", "zo_3",
      "zo_4", "zo_5", "ir_1", "ir_2", "ir_3", "ir_4", "ir_5", "or_1",
      "or_2", "or_3", "or_4", "or_5", "fl",   "ml",   "fm"};

  // Results
  int n_experiments = 1;
  std::vector<float> area_under_curve(n_experiments);
  std::vector<float> execution_time(n_experiments);

  // Configuration region modality
  std::vector<int> r_n_lines_max{200};
  std::vector<bool> r_use_adaptive_coverage{false};
  std::vector<float> r_min_continuous_distance{3.0f};
  std::vector<float> r_function_length{8};
  std::vector<float> r_distribution_length{12};
  std::vector<float> r_function_amplitude{0.43f};
  std::vector<float> r_function_slope{0.5f};
  std::vector<float> r_learning_rate{1.3f};
  std::vector<std::vector<int>> r_scales{{6, 4, 1}};
  std::vector<std::vector<float>> r_standard_deviations{{15.0f, 5.0f, 1.5f}}; // {25.0f, 10.0f, 2.5f}
  std::vector<int> r_n_histogram_bins{16};  // 32
  std::vector<float> r_learning_rate_f{0.2f};
  std::vector<float> r_learning_rate_b{0.2f};
  std::vector<float> r_unconsidered_line_length{0.5f};
  std::vector<float> r_max_considered_line_length{20.0};

  // Configuration depth modality
  std::vector<int> d_n_points_max{200};
  std::vector<bool> d_use_adaptive_coverage{false};
  std::vector<bool> d_use_depth_scaling{false};
  std::vector<float> d_stride_length{0.005f};
  std::vector<std::vector<float>> d_considered_distances{{0.05f, 0.02f, 0.01f}};
  std::vector<std::vector<float>> d_standard_deviations{{0.035f, 0.035f, 0.025f}};

  // Configuration texture modality
  std::vector<m3t::TextureModality::DescriptorType> t_descriptor_type{
      m3t::TextureModality::DescriptorType::ORB};
  std::vector<int> t_focused_image_size{200};
  std::vector<float> t_descriptor_distance_threshold{0.7f};
  std::vector<float> t_tukey_norm_constant{20.0f};
  std::vector<std::vector<float>> t_standard_deviations{{5.0f, 1.0f, 0.5f}};
  std::vector<float> t_max_keyframe_rotation_difference{10.0f * m3t::kPi /
                                                        180.0f};
  std::vector<int> t_max_keyframe_age{1000};
  std::vector<int> t_n_keyframes{1};
  std::vector<int> t_orb_n_features{300};
  std::vector<float> t_orb_scale_factor{1.2f};
  std::vector<int> t_orb_n_levels{3};
  std::vector<int> t_brisk_threshold{35};
  std::vector<int> t_brisk_octave{3};
  std::vector<float> t_brisk_pattern_scale{0.6f};
  std::vector<float> t_daisy_radius{8.0f};
  std::vector<int> t_daisy_q_radius{3};
  std::vector<int> t_daisy_q_theta{4};
  std::vector<int> t_daisy_q_hist{8};
  std::vector<bool> t_freak_orientation_normalized{true};
  std::vector<bool> t_freak_scale_normalized{true};
  std::vector<float> t_freak_pattern_scale{16.0f};
  std::vector<int> t_freak_n_octaves{4};
  std::vector<int> t_sift_n_features{0};
  std::vector<int> t_sift_n_octave_layers{3};
  std::vector<double> t_sift_contrast_threshold{0.04};
  std::vector<double> t_sift_edge_threshold{10.0};
  std::vector<double> t_sift_sigma{0.7f};

  // Configuration optimizer
  std::vector<float> tikhonov_parameter_rotation{1000.0f};
  std::vector<float> tikhonov_parameter_translation{30000.0f};

  // Configuration tracker
  std::vector<int> n_update_iterations{2};
  std::vector<int> n_corr_iterations{4};  // 7

  // Run experiments
  OPTEvaluator evaluator{"evaluator", dataset_directory, external_directory,
                         body_names,  body_orientations, motion_patterns};
  evaluator.set_run_sequentially(false);
  evaluator.set_use_random_seed(false);
  evaluator.set_n_vertices_evaluation(1000);
  evaluator.set_visualize_frame_results(false);
  evaluator.set_visualize_tracking(false);
  evaluator.set_use_region_modality(true);
  evaluator.set_use_depth_modality(true);
  evaluator.set_use_texture_modality(true);
  evaluator.SetUp();
  for (int i = 0; i < n_experiments; ++i) {
    evaluator.set_region_modality_setter([&](auto r) {
      r->set_n_lines_max(m3t::LastValidValue(r_n_lines_max, i));
      r->set_use_adaptive_coverage(
          m3t::LastValidValue(r_use_adaptive_coverage, i));
      r->set_min_continuous_distance(
          m3t::LastValidValue(r_min_continuous_distance, i));
      r->set_function_length(m3t::LastValidValue(r_function_length, i));
      r->set_distribution_length(m3t::LastValidValue(r_distribution_length, i));
      r->set_function_amplitude(m3t::LastValidValue(r_function_amplitude, i));
      r->set_function_slope(m3t::LastValidValue(r_function_slope, i));
      r->set_learning_rate(m3t::LastValidValue(r_learning_rate, i));
      r->set_scales(m3t::LastValidValue(r_scales, i));
      r->set_standard_deviations(m3t::LastValidValue(r_standard_deviations, i));
      r->set_n_histogram_bins(m3t::LastValidValue(r_n_histogram_bins, i));
      r->set_learning_rate_f(m3t::LastValidValue(r_learning_rate_f, i));
      r->set_learning_rate_b(m3t::LastValidValue(r_learning_rate_b, i));
      r->set_unconsidered_line_length(
          m3t::LastValidValue(r_unconsidered_line_length, i));
      r->set_max_considered_line_length(
          m3t::LastValidValue(r_max_considered_line_length, i));
    });
    evaluator.set_depth_modality_setter([&](auto d) {
      d->set_n_points_max(m3t::LastValidValue(d_n_points_max, i));
      d->set_use_adaptive_coverage(
          m3t::LastValidValue(d_use_adaptive_coverage, i));
      d->set_use_depth_scaling(m3t::LastValidValue(d_use_depth_scaling, i));
      d->set_stride_length(m3t::LastValidValue(d_stride_length, i));
      d->set_considered_distances(
          m3t::LastValidValue(d_considered_distances, i));
      d->set_standard_deviations(m3t::LastValidValue(d_standard_deviations, i));
    });
    evaluator.set_texture_modality_setter([&](auto t) {
      t->set_descriptor_type(m3t::LastValidValue(t_descriptor_type, i));
      t->set_focused_image_size(m3t::LastValidValue(t_focused_image_size, i));
      t->set_descriptor_distance_threshold(
          m3t::LastValidValue(t_descriptor_distance_threshold, i));
      t->set_tukey_norm_constant(m3t::LastValidValue(t_tukey_norm_constant, i));
      t->set_standard_deviations(m3t::LastValidValue(t_standard_deviations, i));
      t->set_max_keyframe_rotation_difference(
          m3t::LastValidValue(t_max_keyframe_rotation_difference, i));
      t->set_max_keyframe_age(m3t::LastValidValue(t_max_keyframe_age, i));
      t->set_n_keyframes(m3t::LastValidValue(t_n_keyframes, i));
      t->set_orb_n_features(m3t::LastValidValue(t_orb_n_features, i));
      t->set_orb_scale_factor(m3t::LastValidValue(t_orb_scale_factor, i));
      t->set_orb_n_levels(m3t::LastValidValue(t_orb_n_levels, i));
      t->set_brisk_threshold(m3t::LastValidValue(t_brisk_threshold, i));
      t->set_brisk_octave(m3t::LastValidValue(t_brisk_octave, i));
      t->set_brisk_pattern_scale(m3t::LastValidValue(t_brisk_pattern_scale, i));
      t->set_daisy_radius(m3t::LastValidValue(t_daisy_radius, i));
      t->set_daisy_q_radius(m3t::LastValidValue(t_daisy_q_radius, i));
      t->set_daisy_q_theta(m3t::LastValidValue(t_daisy_q_theta, i));
      t->set_daisy_q_hist(m3t::LastValidValue(t_daisy_q_hist, i));
      t->set_freak_orientation_normalized(
          m3t::LastValidValue(t_freak_orientation_normalized, i));
      t->set_freak_scale_normalized(
          m3t::LastValidValue(t_freak_scale_normalized, i));
      t->set_freak_pattern_scale(m3t::LastValidValue(t_freak_pattern_scale, i));
      t->set_freak_n_octaves(m3t::LastValidValue(t_freak_n_octaves, i));
      t->set_sift_n_features(m3t::LastValidValue(t_sift_n_features, i));
      t->set_sift_n_octave_layers(
          m3t::LastValidValue(t_sift_n_octave_layers, i));
      t->set_sift_contrast_threshold(
          m3t::LastValidValue(t_sift_contrast_threshold, i));
      t->set_sift_edge_threshold(m3t::LastValidValue(t_sift_edge_threshold, i));
      t->set_sift_sigma(m3t::LastValidValue(t_sift_sigma, i));
    });
    evaluator.set_optimizer_setter([&](auto o) {
      o->set_tikhonov_parameter_rotation(
          m3t::LastValidValue(tikhonov_parameter_rotation, i));
      o->set_tikhonov_parameter_translation(
          m3t::LastValidValue(tikhonov_parameter_translation, i));
      // if(o->root_link_ptrs()[0]->body_ptr()->name() == "soda")
      //   o->set_tikhonov_parameter_rotation(
      //       m3t::LastValidValue(tikhonov_parameter_translation, i));
    });
    evaluator.set_tracker_setter([&](auto t) {
      t->set_n_update_iterations(m3t::LastValidValue(n_update_iterations, i));
      t->set_n_corr_iterations(m3t::LastValidValue(n_corr_iterations, i));
    });
    evaluator.Evaluate();
    area_under_curve[i] = evaluator.area_under_curve();
    execution_time[i] = evaluator.execution_time();
  }

  // Calculate maximum
  size_t max_i;
  float max_area_under_curve = 0.0f;
  float max_execution_time = 0.0f;
  for (int i = 0; i < n_experiments; ++i) {
    if (area_under_curve[i] > max_area_under_curve) {
      max_i = i;
      max_area_under_curve = area_under_curve[i];
      max_execution_time = execution_time[i];
    }
  }

  // Print results
  std::cout << std::endl << std::string(80, '-') << std::endl;
  for (int i = 0; i < n_experiments; ++i) {
    std::cout << "Experiment " << i << ": ";
    m3t::PrintValue("r_n_lines_max", r_n_lines_max, i);
    m3t::PrintValue("r_use_adaptive_coverage", r_use_adaptive_coverage, i);
    m3t::PrintValue("r_min_continuous_distance", r_min_continuous_distance, i);
    m3t::PrintValue("r_function_length", r_function_length, i);
    m3t::PrintValue("r_distribution_length", r_distribution_length, i);
    m3t::PrintValue("r_function_amplitude", r_function_amplitude, i);
    m3t::PrintValue("r_function_slope", r_function_slope, i);
    m3t::PrintValue("r_learning_rate", r_learning_rate, i);
    m3t::PrintValues("r_scales", r_scales, i);
    m3t::PrintValues("r_standard_deviations", r_standard_deviations, i);
    m3t::PrintValue("r_n_histogram_bins", r_n_histogram_bins, i);
    m3t::PrintValue("r_learning_rate_f", r_learning_rate_f, i);
    m3t::PrintValue("r_learning_rate_b", r_learning_rate_b, i);
    m3t::PrintValue("r_unconsidered_line_length", r_unconsidered_line_length,
                    i);
    m3t::PrintValue("r_max_considered_line_length",
                    r_max_considered_line_length, i);
    m3t::PrintValue("d_n_points_max", d_n_points_max, i);
    m3t::PrintValue("d_use_adaptive_coverage", d_use_adaptive_coverage, i);
    m3t::PrintValue("d_use_depth_scaling", d_use_depth_scaling, i);
    m3t::PrintValue("d_stride_length", d_stride_length, i);
    m3t::PrintValues("d_considered_distances", d_considered_distances, i);
    m3t::PrintValues("d_standard_deviations", d_standard_deviations, i);
    m3t::PrintValue("t_focused_image_size", t_focused_image_size, i);
    m3t::PrintValue("t_descriptor_distance_threshold",
                    t_descriptor_distance_threshold, i);
    m3t::PrintValue("t_tukey_norm_constant", t_tukey_norm_constant, i);
    m3t::PrintValues("t_standard_deviations", t_standard_deviations, i);
    m3t::PrintValue("t_max_keyframe_rotation_difference",
                    t_max_keyframe_rotation_difference, i);
    m3t::PrintValue("t_max_keyframe_age", t_max_keyframe_age, i);
    m3t::PrintValue("t_n_keyframes", t_n_keyframes, i);
    m3t::PrintValue("t_orb_n_features", t_orb_n_features, i);
    m3t::PrintValue("t_orb_scale_factor", t_orb_scale_factor, i);
    m3t::PrintValue("t_orb_n_levels", t_orb_n_levels, i);
    m3t::PrintValue("t_brisk_threshold", t_brisk_threshold, i);
    m3t::PrintValue("t_brisk_octave", t_brisk_octave, i);
    m3t::PrintValue("t_brisk_pattern_scale", t_brisk_pattern_scale, i);
    m3t::PrintValue("t_daisy_radius", t_daisy_radius, i);
    m3t::PrintValue("t_daisy_q_radius", t_daisy_q_radius, i);
    m3t::PrintValue("t_daisy_q_theta", t_daisy_q_theta, i);
    m3t::PrintValue("t_daisy_q_hist", t_daisy_q_hist, i);
    m3t::PrintValue("t_freak_orientation_normalized",
                    t_freak_orientation_normalized, i);
    m3t::PrintValue("t_freak_scale_normalized", t_freak_scale_normalized, i);
    m3t::PrintValue("t_freak_pattern_scale", t_freak_pattern_scale, i);
    m3t::PrintValue("t_freak_n_octaves", t_freak_n_octaves, i);
    m3t::PrintValue("t_sift_n_features", t_sift_n_features, i);
    m3t::PrintValue("t_sift_n_octave_layers", t_sift_n_octave_layers, i);
    m3t::PrintValue("t_sift_contrast_threshold", t_sift_contrast_threshold, i);
    m3t::PrintValue("t_sift_edge_threshold", t_sift_edge_threshold, i);
    m3t::PrintValue("t_sift_sigma", t_sift_sigma, i);
    m3t::PrintValue("tikhonov_parameter_rotation", tikhonov_parameter_rotation,
                    i);
    m3t::PrintValue("tikhonov_parameter_translation",
                    tikhonov_parameter_translation, i);
    m3t::PrintValue("n_update_iterations", n_update_iterations, i);
    m3t::PrintValue("n_corr_iterations", n_corr_iterations, i);
    std::cout << std::endl
              << "execution_time = " << execution_time[i]
              << " us, area_under_curve = " << area_under_curve[i] << std::endl;
  }
  std::cout << std::string(80, '-') << std::endl;
  std::cout << "Maximum: Experiment " << max_i
            << ": execution_time = " << max_execution_time
            << " us, area_under_curve = " << max_area_under_curve << std::endl;
  std::cout << std::string(80, '-') << std::endl;
  return 0;
}
