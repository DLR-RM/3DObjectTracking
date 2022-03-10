// SPDX-License-Identifier: MIT
// Copyright (c) 2022 Manuel Stoiber, German Aerospace Center (DLR)

#include "choi_evaluator.h"

int main() {
  // Directories
  std::filesystem::path dataset_directory{"/your/path/"};
  std::filesystem::path external_directory{"/your/path/"};

  // Dataset configuration
  std::vector<std::string> body_names{"kinect_box", "milk", "orange_juice",
                                      "tide"};

  // Results
  int n_experiments = 1;
  std::vector<float> mean_translation_error(n_experiments);
  std::vector<float> mean_rotation_error(n_experiments);
  std::vector<float> execution_time(n_experiments);

  // Configuration region modality
  std::vector<int> r_n_lines{200};
  std::vector<float> r_min_continuous_distance{3.0f};
  std::vector<float> r_function_length{8};
  std::vector<float> r_distribution_length{12};
  std::vector<float> r_function_amplitude{0.43f};
  std::vector<float> r_function_slope{0.5f};
  std::vector<float> r_learning_rate{1.3f};
  std::vector<std::vector<int>> r_scales{{2, 1}};
  std::vector<std::vector<float>> r_standard_deviations{{5.0f}};
  std::vector<int> r_n_histogram_bins{16};
  std::vector<float> r_learning_rate_f{0.2f};
  std::vector<float> r_learning_rate_b{0.2f};
  std::vector<float> r_unconsidered_line_length{0.5f};
  std::vector<float> r_max_considered_line_length{20.0};
  std::vector<float> r_measured_depth_offset_radius{0.01f};
  std::vector<float> r_measured_occlusion_radius{0.01f};
  std::vector<float> r_measured_occlusion_threshold{0.03f};

  // Configuration depth modality
  std::vector<int> d_n_points{200};
  std::vector<float> d_stride_length{0.005f};
  std::vector<std::vector<float>> d_considered_distances{{0.01f}};
  std::vector<std::vector<float>> d_standard_deviations{{0.01f, 0.001f}};
  std::vector<float> d_measured_depth_offset_radius{0.01f};
  std::vector<float> d_measured_occlusion_radius{0.01f};
  std::vector<float> d_measured_occlusion_threshold{0.03f};

  // Configuration optimizer
  std::vector<float> tikhonov_parameter_rotation{1000.0f};
  std::vector<float> tikhonov_parameter_translation{30000.0f};

  // Configuration tracker
  std::vector<int> n_update_iterations{2};
  std::vector<int> n_corr_iterations{4};

  // Run experiments
  ChoiEvaluator evaluator{"evaluator", dataset_directory, external_directory,
                          body_names};
  evaluator.set_use_region_modality(true);
  evaluator.set_use_depth_modality(true);
  evaluator.set_measure_occlusions_region(true);
  evaluator.set_measure_occlusions_depth(true);
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
      r->set_measured_depth_offset_radius(
          icg::LastValidValue(r_measured_depth_offset_radius, i));
      r->set_measured_occlusion_radius(
          icg::LastValidValue(r_measured_occlusion_radius, i));
      r->set_measured_occlusion_threshold(
          icg::LastValidValue(r_measured_occlusion_threshold, i));
    });
    evaluator.set_depth_modality_setter([&](auto d) {
      d->set_n_points(icg::LastValidValue(d_n_points, i));
      d->set_stride_length(icg::LastValidValue(d_stride_length, i));
      d->set_considered_distances(
          icg::LastValidValue(d_considered_distances, i));
      d->set_standard_deviations(icg::LastValidValue(d_standard_deviations, i));
      d->set_measured_depth_offset_radius(
          icg::LastValidValue(d_measured_depth_offset_radius, i));
      d->set_measured_occlusion_radius(
          icg::LastValidValue(d_measured_occlusion_radius, i));
      d->set_measured_occlusion_threshold(
          icg::LastValidValue(d_measured_occlusion_threshold, i));
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
    mean_translation_error[i] = evaluator.mean_translation_error();
    mean_rotation_error[i] = evaluator.mean_rotation_error();
    execution_time[i] = evaluator.execution_time();
  }

  // Calculate minimum
  size_t min_i;
  float min_mean_translation_error = std::numeric_limits<float>::max();
  float min_mean_rotation_error = std::numeric_limits<float>::max();
  float min_execution_time = std::numeric_limits<float>::max();
  for (int i = 0; i < n_experiments; ++i) {
    if (mean_translation_error[i] < min_mean_translation_error) {
      min_i = i;
      min_mean_translation_error = mean_translation_error[i];
      min_mean_rotation_error = mean_rotation_error[i];
      min_execution_time = execution_time[i];
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
    icg::PrintValue("r_measured_depth_offset_radius",
                    r_measured_depth_offset_radius, i);
    icg::PrintValue("r_measured_occlusion_radius", r_measured_occlusion_radius,
                    i);
    icg::PrintValue("r_measured_occlusion_threshold",
                    r_measured_occlusion_threshold, i);
    icg::PrintValue("d_n_points", d_n_points, i);
    icg::PrintValue("d_stride_length", d_stride_length, i);
    icg::PrintValues("d_considered_distances", d_considered_distances, i);
    icg::PrintValues("d_standard_deviations", d_standard_deviations, i);
    icg::PrintValue("d_measured_depth_offset_radius",
                    d_measured_depth_offset_radius, i);
    icg::PrintValue("d_measured_occlusion_radius", d_measured_occlusion_radius,
                    i);
    icg::PrintValue("d_measured_occlusion_threshold",
                    d_measured_occlusion_threshold, i);
    icg::PrintValue("tikhonov_parameter_rotation", tikhonov_parameter_rotation,
                    i);
    icg::PrintValue("tikhonov_parameter_translation",
                    tikhonov_parameter_translation, i);
    icg::PrintValue("n_update_iterations", n_update_iterations, i);
    icg::PrintValue("n_corr_iterations", n_corr_iterations, i);
    std::cout << std::endl
              << "execution_time = " << execution_time[i] << " us, "
              << "mean_translation_error = " << mean_translation_error[i]
              << " mm, "
              << "mean_rotation_error = " << mean_rotation_error[i] << " deg"
              << std::endl;
  }
  std::cout << std::string(80, '-') << std::endl;
  std::cout << "Minimum: Experiment " << min_i
            << ": execution_time = " << min_execution_time << " us, "
            << "mean_translation_error = " << min_mean_translation_error
            << " mm, "
            << "mean_rotation_error = " << min_mean_rotation_error << " deg"
            << std::endl;
  std::cout << std::string(80, '-') << std::endl;
}
