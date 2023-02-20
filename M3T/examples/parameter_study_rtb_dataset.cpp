// SPDX-License-Identifier: MIT
// Copyright (c) 2023 Manuel Stoiber, German Aerospace Center (DLR)

#include "rtb_evaluator.h"

int main() {
  // Directories
  std::filesystem::path dataset_directory{"/your/path/"};
  std::filesystem::path external_directory{"/your/path/"};

  // Dataset configuration
  std::vector<std::string> object_names{"gripper",       "medical_pliers",
                                        "medical_robot", "picker_robot",
                                        "robot_fingers", "robot_wrist"};
  std::vector<std::string> difficulty_levels{"test_easy", "test_medium",
                                             "test_hard"};
  std::vector<std::string> depth_names{"depth_ground_truth",
                                       "depth_azure_kinect",
                                       "depth_active_stereo", "depth_stereo"};
  std::vector<int> sequence_numbers{0, 1, 2, 3, 4, 5, 6, 7, 8, 9};

  // Results
  int n_experiments = 1;
  std::vector<float> add_auc(n_experiments);
  std::vector<float> adds_auc(n_experiments);
  std::vector<float> execution_time(n_experiments);

  // Configuration region modality
  std::vector<int> r_n_lines_max{300};
  std::vector<bool> r_use_adaptive_coverage{true};
  std::vector<float> r_min_continuous_distance{3.0f};
  std::vector<float> r_function_length{8};
  std::vector<float> r_distribution_length{12};
  std::vector<float> r_function_amplitude{0.43f};
  std::vector<float> r_function_slope{0.5f};
  std::vector<float> r_learning_rate{1.3f};
  std::vector<std::vector<int>> r_scales{{9, 7, 5, 2}};
  std::vector<std::vector<float>> r_standard_deviations{
      {25.0f, 15.0f, 10.0f}};
  std::vector<float> r_unconsidered_line_length{0.5f};
  std::vector<float> r_max_considered_line_length{20.0};
  std::vector<int> r_n_histogram_bins{16};
  std::vector<float> r_learning_rate_f{0.2f};
  std::vector<float> r_learning_rate_b{0.2f};

  // Configuration depth modality
  std::vector<int> d_n_points_max{300};
  std::vector<bool> d_use_adaptive_coverage{true};
  std::vector<bool> d_use_depth_scaling{true};
  std::vector<float> d_stride_length{0.008f};
  std::vector<std::vector<float>> d_considered_distances{
      {0.1f, 0.08f, 0.05f}};
  std::vector<std::vector<float>> d_standard_deviations{
      {0.05f, 0.03f, 0.02f}};

  // Configuration optimizer
  std::vector<float> tikhonov_parameter_rotation{100.0f};
  std::vector<float> tikhonov_parameter_translation{1000.0f};

  // Configuration tracker
  std::vector<int> n_update_iterations{2};
  std::vector<int> n_corr_iterations{6};

  // Run experiments
  RTBEvaluator evaluator{"evaluator",     dataset_directory, external_directory,
                         object_names,    difficulty_levels, depth_names,
                         sequence_numbers};
  evaluator.set_evaluation_mode(RTBEvaluator::EvaluationMode::COMBINED);
  evaluator.set_run_sequentially(false);
  evaluator.set_use_random_seed(false);
  evaluator.set_n_vertices_evaluation(1000);
  evaluator.set_visualize_frame_results(false);
  evaluator.set_visualize_tracking(false);
  evaluator.set_use_shared_color_histograms(true);
  evaluator.set_use_region_checking(true);
  evaluator.set_use_silhouette_checking(true);
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
      r->set_unconsidered_line_length(
          m3t::LastValidValue(r_unconsidered_line_length, i));
      r->set_max_considered_line_length(
          m3t::LastValidValue(r_max_considered_line_length, i));
      if (!r->use_shared_color_histograms()) {
        r->set_n_histogram_bins(m3t::LastValidValue(r_n_histogram_bins, i));
        r->set_learning_rate_f(m3t::LastValidValue(r_learning_rate_f, i));
        r->set_learning_rate_b(m3t::LastValidValue(r_learning_rate_b, i));
      }
    });
    evaluator.set_color_histograms_setter([&](auto h) {
      h->set_n_bins(m3t::LastValidValue(r_n_histogram_bins, i));
      h->set_learning_rate_f(m3t::LastValidValue(r_learning_rate_f, i));
      h->set_learning_rate_b(m3t::LastValidValue(r_learning_rate_b, i));
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
    evaluator.set_optimizer_setter([&](auto o) {
      o->set_tikhonov_parameter_rotation(
          m3t::LastValidValue(tikhonov_parameter_rotation, i));
      o->set_tikhonov_parameter_translation(
          m3t::LastValidValue(tikhonov_parameter_translation, i));
    });
    evaluator.set_tracker_setter([&](auto t) {
      t->set_n_update_iterations(m3t::LastValidValue(n_update_iterations, i));
      t->set_n_corr_iterations(m3t::LastValidValue(n_corr_iterations, i));
    });
    evaluator.Evaluate();
    add_auc[i] = evaluator.add_auc();
    adds_auc[i] = evaluator.adds_auc();
    execution_time[i] = evaluator.execution_time();
  }

  // Calculate maximum
  size_t max_i;
  float max_add_auc = 0.0f;
  float max_adds_auc = 0.0f;
  float max_execution_time = 0.0f;
  for (int i = 0; i < n_experiments; ++i) {
    if (adds_auc[i] > max_adds_auc) {
      max_i = i;
      max_add_auc = add_auc[i];
      max_adds_auc = adds_auc[i];
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
    m3t::PrintValue("r_unconsidered_line_length", r_unconsidered_line_length,
                    i);
    m3t::PrintValue("r_max_considered_line_length",
                    r_max_considered_line_length, i);
    m3t::PrintValue("r_n_histogram_bins", r_n_histogram_bins, i);
    m3t::PrintValue("r_learning_rate_f", r_learning_rate_f, i);
    m3t::PrintValue("r_learning_rate_b", r_learning_rate_b, i);
    m3t::PrintValue("d_n_points_max", d_n_points_max, i);
    m3t::PrintValue("d_use_adaptive_coverage", d_use_adaptive_coverage, i);
    m3t::PrintValue("d_use_depth_scaling", d_use_depth_scaling, i);
    m3t::PrintValue("d_stride_length", d_stride_length, i);
    m3t::PrintValues("d_considered_distances", d_considered_distances, i);
    m3t::PrintValues("d_standard_deviations", d_standard_deviations, i);
    m3t::PrintValue("tikhonov_parameter_rotation", tikhonov_parameter_rotation,
                    i);
    m3t::PrintValue("tikhonov_parameter_translation",
                    tikhonov_parameter_translation, i);
    m3t::PrintValue("n_update_iterations", n_update_iterations, i);
    m3t::PrintValue("n_corr_iterations", n_corr_iterations, i);
    std::cout << std::endl
              << "execution_time = " << execution_time[i]
              << " us, add_auc = " << add_auc[i]
              << ", adds_auc = " << adds_auc[i] << std::endl;
  }
  std::cout << std::string(80, '-') << std::endl;
  std::cout << "Maximum: Experiment " << max_i
            << ": execution_time = " << max_execution_time
            << " us, add_auc = " << max_add_auc
            << ", adds_auc = " << max_adds_auc << std::endl;
  std::cout << std::string(80, '-') << std::endl;
}
