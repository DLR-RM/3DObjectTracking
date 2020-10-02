// SPDX-License-Identifier: MIT
// Copyright (c) 2020 Manuel Stoiber, German Aerospace Center (DLR)

#include "evaluator.h"

int main() {
  // Change accordingly
  std::string dataset_path{"/your/path/to/rbot/dataset/"};
  std::string result_path{"/your/path/to/store/results/"};

  std::vector<std::string> body_names{
      "ape",  "bakingsoda", "benchviseblue", "broccolisoup", "cam",
      "can",  "cat",        "clown",         "cube",         "driller",
      "duck", "eggbox",     "glue",          "iron",         "koalacandy",
      "lamp", "phone",      "squirrel"};
  std::string sequence_name{"c_noisy"};
  bool use_occlusions = false;
  std::vector<std::string> result_folders{"without_regularization",
                                          "with_linear_function",
                                          "without_approximation"};

  // Evaluation without tikhonov regularization
  Evaluator evaluator_0;
  evaluator_0.set_visualize_all_results(false);
  evaluator_0.Init(dataset_path, body_names, {sequence_name}, {use_occlusions});
  evaluator_0.region_modality_ptr()->set_tikhonov_parameter_rotation(0);
  evaluator_0.region_modality_ptr()->set_tikhonov_parameter_translation(0);
  evaluator_0.Evaluate();
  evaluator_0.SaveResults(result_path + result_folders[0]);

  // Evaluate with linear function
  Evaluator evaluator_1;
  evaluator_1.set_visualize_all_results(false);
  evaluator_1.Init(dataset_path, body_names, {sequence_name}, {use_occlusions});
  evaluator_1.region_modality_ptr()->set_use_linear_function(true);
  evaluator_1.Evaluate();
  evaluator_1.SaveResults(result_path + result_folders[1]);

  // Evaluation without approximation of derivatives
  Evaluator evaluator_2;
  evaluator_2.set_visualize_all_results(false);
  evaluator_2.Init(dataset_path, body_names, {sequence_name}, {use_occlusions});
  evaluator_2.region_modality_ptr()->set_probability_threshold(0.0f);
  evaluator_2.region_modality_ptr()->set_use_const_variance(true);
  evaluator_2.Evaluate();
  evaluator_2.SaveResults(result_path + result_folders[2]);
}
