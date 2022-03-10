// SPDX-License-Identifier: MIT
// Copyright (c) 2022 Manuel Stoiber, German Aerospace Center (DLR)

#include "ycb_evaluator.h"

int main() {
  // Directories
  std::filesystem::path dataset_directory{"/your/path/"};
  std::filesystem::path external_directory{"/your/path/"};

  constexpr bool kEvaluateSundermeyer = false;
  constexpr bool kEvaluateCosyPose = false;
  constexpr bool kEvaluatePoseCNN = false;
  constexpr bool kEvaluatePoseCNNICP = false;
  constexpr bool kUseRefinement = false;
  bool evaluate_refinement{};
  std::string detector_folder{};
  if (kEvaluateSundermeyer) {
    evaluate_refinement = true;
    detector_folder = "sundermeyer";
  }
  if (kEvaluateCosyPose) {
    evaluate_refinement = true;
    detector_folder = "cosypose";
  }
  if (kEvaluatePoseCNN) {
    evaluate_refinement = true;
    detector_folder = "posecnn_rss2018";
  }
  if (kEvaluatePoseCNNICP) {
    evaluate_refinement = true;
    detector_folder = "posecnn_icp_rss2018";
  }

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

  // Results
  int n_experiments = 1;
  std::vector<float> add_auc(n_experiments);
  std::vector<float> adds_auc(n_experiments);
  std::vector<float> execution_time(n_experiments);

  // Configuration region modality
  std::vector<int> r_n_lines{200};
  std::vector<float> r_min_continuous_distance{3.0f};
  std::vector<float> r_function_length{8};
  std::vector<float> r_distribution_length{12};
  std::vector<float> r_function_amplitude{0.43f};
  std::vector<float> r_function_slope{0.5f};
  std::vector<float> r_learning_rate{1.3f};
  std::vector<std::vector<int>> r_scales{{7, 4, 2}};
  std::vector<std::vector<float>> r_standard_deviations{{25.0f, 15.0f, 10.0f}};
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
  std::vector<std::vector<float>> d_considered_distances{{0.07f, 0.05f, 0.04f}};
  std::vector<std::vector<float>> d_standard_deviations{{0.05f, 0.03f, 0.02f}};
  std::vector<float> d_measured_depth_offset_radius{0.01f};
  std::vector<float> d_measured_occlusion_radius{0.01f};
  std::vector<float> d_measured_occlusion_threshold{0.03f};

  // Configuration optimizer
  std::vector<float> tikhonov_parameter_rotation{1000.0f};
  std::vector<float> tikhonov_parameter_translation{30000.0f};

  // Configuration tracker
  std::vector<int> n_update_iterations{2};
  std::vector<int> n_corr_iterations{4};

  // Configurations for refinement
  if (kEvaluateSundermeyer || kEvaluateCosyPose || kEvaluatePoseCNN ||
      kEvaluatePoseCNNICP) {
    n_corr_iterations = {0};
    if (kUseRefinement) {
      d_stride_length = {0.01f};
      d_considered_distances = {{0.30f, 0.25f, 0.10f}};
      d_standard_deviations = {{0.10f, 0.05f, 0.02f}};
      tikhonov_parameter_translation = {100.0f};
      n_corr_iterations = {7};
    }
  }

  // Run experiments
  YCBEvaluator evaluator{"evaluator", dataset_directory, external_directory,
                         sequence_ids, body_names};
  evaluator.set_detector_folder(detector_folder);
  evaluator.set_evaluate_refinement(evaluate_refinement);
  evaluator.set_use_matlab_gt_poses(true);
  evaluator.set_run_sequentially(false);
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
    evaluator.set_refiner_setter([&](auto r) {
      r->set_n_update_iterations(icg::LastValidValue(n_update_iterations, i));
      r->set_n_corr_iterations(icg::LastValidValue(n_corr_iterations, i));
    });
    evaluator.set_tracker_setter([&](auto t) {
      t->set_n_update_iterations(icg::LastValidValue(n_update_iterations, i));
      t->set_n_corr_iterations(icg::LastValidValue(n_corr_iterations, i));
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
