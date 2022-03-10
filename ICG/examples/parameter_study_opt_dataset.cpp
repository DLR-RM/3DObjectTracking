// SPDX-License-Identifier: MIT
// Copyright (c) 2022 Manuel Stoiber, German Aerospace Center (DLR)

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
  std::vector<int> r_n_lines{200};
  std::vector<float> r_min_continuous_distance{3.0f};
  std::vector<float> r_function_length{8};
  std::vector<float> r_distribution_length{12};
  std::vector<float> r_function_amplitude{0.43f};
  std::vector<float> r_function_slope{0.5f};
  std::vector<float> r_learning_rate{1.3f};
  std::vector<std::vector<int>> r_scales{{6, 4, 1}};
  std::vector<std::vector<float>> r_standard_deviations{{15.0f, 5.0f, 1.5f}};
  std::vector<int> r_n_histogram_bins{16};
  std::vector<float> r_learning_rate_f{0.2f};
  std::vector<float> r_learning_rate_b{0.2f};
  std::vector<float> r_unconsidered_line_length{0.5f};
  std::vector<float> r_max_considered_line_length{20.0};

  // Configuration depth modality
  std::vector<int> d_n_points{200};
  std::vector<float> d_stride_length{0.005f};
  std::vector<std::vector<float>> d_considered_distances{{0.05f, 0.02f, 0.01f}};
  std::vector<std::vector<float>> d_standard_deviations{{0.035, 0.035, 0.025f}};

  // Configuration optimizer
  std::vector<float> tikhonov_parameter_rotation{1000.0f};
  std::vector<float> tikhonov_parameter_rotation_soda{70000.0f};
  std::vector<float> tikhonov_parameter_translation{30000.0f};

  // Configuration tracker
  std::vector<int> n_update_iterations{2};
  std::vector<int> n_corr_iterations{4};

  // Run experiments
  OPTEvaluator evaluator{"evaluator", dataset_directory, external_directory,
                         body_names,  body_orientations, motion_patterns};
  evaluator.set_use_region_modality(true);
  evaluator.set_use_depth_modality(true);
  evaluator.set_use_random_seed(false);
  evaluator.set_n_vertices_evaluation(1000);
  evaluator.set_visualize_frame_results(false);
  evaluator.set_visualize_tracking(false);
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
    });
    evaluator.set_depth_modality_setter([&](auto d) {
      d->set_n_points(icg::LastValidValue(d_n_points, i));
      d->set_stride_length(icg::LastValidValue(d_stride_length, i));
      d->set_considered_distances(
          icg::LastValidValue(d_considered_distances, i));
      d->set_standard_deviations(icg::LastValidValue(d_standard_deviations, i));
    });
    evaluator.set_optimizer_setter([&](auto o) {
      o->set_tikhonov_parameter_rotation(
          icg::LastValidValue(tikhonov_parameter_rotation, i));
      if (o->modality_ptrs()[0]->body_ptr()->name() == "soda")
        o->set_tikhonov_parameter_rotation(
            icg::LastValidValue(tikhonov_parameter_rotation_soda, i));
      o->set_tikhonov_parameter_translation(
          icg::LastValidValue(tikhonov_parameter_translation, i));
    });
    evaluator.set_tracker_setter([&](auto t) {
      t->set_n_update_iterations(icg::LastValidValue(n_update_iterations, i));
      t->set_n_corr_iterations(icg::LastValidValue(n_corr_iterations, i));
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
    icg::PrintValue("d_n_points", d_n_points, i);
    icg::PrintValue("d_stride_length", d_stride_length, i);
    icg::PrintValues("d_considered_distances", d_considered_distances, i);
    icg::PrintValues("d_standard_deviations", d_standard_deviations, i);
    icg::PrintValue("tikhonov_parameter_rotation", tikhonov_parameter_rotation,
                    i);
    icg::PrintValue("tikhonov_parameter_rotation_soda",
                    tikhonov_parameter_rotation_soda, i);
    icg::PrintValue("tikhonov_parameter_translation",
                    tikhonov_parameter_translation, i);
    icg::PrintValue("n_update_iterations", n_update_iterations, i);
    icg::PrintValue("n_corr_iterations", n_corr_iterations, i);
    std::cout << std::endl
              << "execution_time = " << execution_time[i]
              << " us, area_under_curve = " << area_under_curve[i] << std::endl;
  }
  std::cout << std::string(80, '-') << std::endl;
  std::cout << "Maximum: Experiment " << max_i
            << ": execution_time = " << max_execution_time
            << " us, area_under_curve = " << max_area_under_curve << std::endl;
  std::cout << std::string(80, '-') << std::endl;
}
