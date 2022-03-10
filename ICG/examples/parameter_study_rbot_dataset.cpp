// SPDX-License-Identifier: MIT
// Copyright (c) 2022 Manuel Stoiber, German Aerospace Center (DLR)

#include "rbot_evaluator.h"

int main() {
  // Directories
  std::filesystem::path dataset_directory{"/your/path/"};
  std::filesystem::path external_directory{"/your/path/"};

  // Dataset configuration
  std::vector<std::string> body_names{
      "ape",  "bakingsoda", "benchviseblue", "broccolisoup", "cam",
      "can",  "cat",        "clown",         "cube",         "driller",
      "duck", "eggbox",     "glue",          "iron",         "koalacandy",
      "lamp", "phone",      "squirrel"};
  std::vector<std::string> sequence_names{
      "a_regular", "b_dynamiclight", "c_noisy", "d_occlusion", "d_occlusion"};
  std::vector<bool> sequence_occlusions{false, false, false, false, true};

  // Default tracker configuration
  int n_experiments = 1;
  std::vector<float> tracking_successes(n_experiments);
  std::vector<float> execution_time(n_experiments);

  // Configuration region modality
  std::vector<int> r_n_lines{200};
  std::vector<float> r_min_continuous_distance{6.0f};
  std::vector<float> r_function_length{8};
  std::vector<float> r_distribution_length{12};
  std::vector<float> r_function_amplitude{0.36f};
  std::vector<float> r_function_slope{0.0f};
  std::vector<float> r_learning_rate{1.3f};
  std::vector<std::vector<int>> r_scales{{5, 2, 2, 1}};
  std::vector<std::vector<float>> r_standard_deviations{
      {15.0f, 5.0f, 3.5f, 1.5f}};
  std::vector<int> r_n_histogram_bins{32};
  std::vector<float> r_learning_rate_f{0.2f};
  std::vector<float> r_learning_rate_b{0.2f};
  std::vector<float> r_unconsidered_line_length{1.0f};
  std::vector<float> r_max_considered_line_length{18.0f};
  std::vector<float> r_modeled_depth_offset_radius{0.0f};
  std::vector<float> r_modeled_occlusion_radius{0.0f};
  std::vector<float> r_modeled_occlusion_threshold{0.03f};

  // Configuration optimizer
  std::vector<float> tikhonov_parameter_rotation{1000.0f};
  std::vector<float> tikhonov_parameter_translation{100000.0f};

  // Configuration tracker
  std::vector<int> n_update_iterations{2};
  std::vector<int> n_corr_iterations{7};

  // Run experiments
  RBOTEvaluator evaluator{"evaluator", dataset_directory, external_directory,
                          body_names,  sequence_names,    sequence_occlusions};
  evaluator.set_visualize_all_results(false);
  evaluator.SetUp();
  for (int i = 0; i < n_experiments; ++i) {
    evaluator.set_region_modality_setter([&](auto r) {
      r->set_n_lines(icg::LastValidValue(r_n_lines, i));
      r->set_min_continuous_distance(
          icg::LastValidValue(r_min_continuous_distance, i));
      r->set_function_length(icg::LastValidValue(r_function_length, i));
      r->set_distribution_length(icg::LastValidValue(r_distribution_length, i));
      r->set_function_amplitude(icg::LastValidValue(r_function_amplitude, i));
      r->set_function_slope(icg::LastValidValue(r_function_slope, i));
      r->set_learning_rate(icg::LastValidValue(r_learning_rate, i));
      r->set_scales(icg::LastValidValue(r_scales, i));
      r->set_standard_deviations(icg::LastValidValue(r_standard_deviations, i));
      r->set_n_histogram_bins(icg::LastValidValue(r_n_histogram_bins, i));
      r->set_learning_rate_f(icg::LastValidValue(r_learning_rate_f, i));
      r->set_learning_rate_b(icg::LastValidValue(r_learning_rate_b, i));
      r->set_unconsidered_line_length(
          icg::LastValidValue(r_unconsidered_line_length, i));
      r->set_max_considered_line_length(
          icg::LastValidValue(r_max_considered_line_length, i));
      r->set_modeled_depth_offset_radius(
          icg::LastValidValue(r_modeled_depth_offset_radius, i));
      r->set_modeled_occlusion_radius(
          icg::LastValidValue(r_modeled_occlusion_radius, i));
      r->set_modeled_occlusion_threshold(
          icg::LastValidValue(r_modeled_occlusion_threshold, i));
    });
    evaluator.set_optimizer_setter([&](auto o) {
      o->set_tikhonov_parameter_rotation(
          icg::LastValidValue(tikhonov_parameter_rotation, i));
      o->set_tikhonov_parameter_translation(
          icg::LastValidValue(tikhonov_parameter_translation, i));
    });
    evaluator.set_tracker_setter([&](auto t) {
      t->set_n_update_iterations(icg::LastValidValue(n_update_iterations, i));
      t->set_n_corr_iterations(icg::LastValidValue(n_corr_iterations, i));
    });
    evaluator.Evaluate();
    tracking_successes[i] = evaluator.tracking_success();
    execution_time[i] = evaluator.execution_time();
  }

  // Calculate maximum
  size_t max_i;
  float max_tracking_success = 0.0f;
  float max_execution_time = 0.0f;
  for (int i = 0; i < n_experiments; ++i) {
    if (tracking_successes[i] > max_tracking_success) {
      max_i = i;
      max_tracking_success = tracking_successes[i];
      max_execution_time = execution_time[i];
    }
  }

  // Print results
  std::cout << std::endl << std::string(80, '-') << std::endl;
  for (int i = 0; i < n_experiments; ++i) {
    std::cout << "Experiment " << i << ": ";
    icg::PrintValue("r_n_lines", r_n_lines, i);
    icg::PrintValue("r_min_continuous_distance", r_min_continuous_distance, i);
    icg::PrintValue("r_function_length", r_function_length, i);
    icg::PrintValue("r_distribution_length", r_distribution_length, i);
    icg::PrintValue("r_function_amplitude", r_function_amplitude, i);
    icg::PrintValue("r_function_slope", r_function_slope, i);
    icg::PrintValue("r_learning_rate", r_learning_rate, i);
    icg::PrintValues("r_scales", r_scales, i);
    icg::PrintValues("r_standard_deviations", r_standard_deviations, i);
    icg::PrintValue("r_n_histogram_bins", r_n_histogram_bins, i);
    icg::PrintValue("r_learning_rate_f", r_learning_rate_f, i);
    icg::PrintValue("r_learning_rate_b", r_learning_rate_b, i);
    icg::PrintValue("r_unconsidered_line_length", r_unconsidered_line_length,
                    i);
    icg::PrintValue("r_max_considered_line_length",
                    r_max_considered_line_length, i);
    icg::PrintValue("r_modeled_depth_offset_radius",
                    r_modeled_depth_offset_radius, i);
    icg::PrintValue("r_modeled_occlusion_radius", r_modeled_occlusion_radius,
                    i);
    icg::PrintValue("r_modeled_occlusion_threshold",
                    r_modeled_occlusion_threshold, i);
    icg::PrintValue("tikhonov_parameter_rotation", tikhonov_parameter_rotation,
                    i);
    icg::PrintValue("tikhonov_parameter_translation",
                    tikhonov_parameter_translation, i);
    icg::PrintValue("n_update_iterations", n_update_iterations, i);
    icg::PrintValue("n_corr_iterations", n_corr_iterations, i);
    std::cout << std::endl
              << "execution_time = " << execution_time[i]
              << " us, tracking success = " << tracking_successes[i]
              << std::endl;
  }
  std::cout << std::string(80, '-') << std::endl;
  std::cout << "Maximum: Experiment " << max_i
            << ": execution_time = " << max_execution_time
            << " us, tracking success = " << max_tracking_success << std::endl;
  std::cout << std::string(80, '-') << std::endl;
}
