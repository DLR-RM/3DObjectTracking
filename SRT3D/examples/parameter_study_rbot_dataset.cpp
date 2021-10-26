// SPDX-License-Identifier: MIT
// Copyright (c) 2021 Manuel Stoiber, German Aerospace Center (DLR)

#include "rbot_evaluator.h"

int main() {
  // Directories
  std::string dataset_directory{"/your/path/to/the/rbot/dataset/"};

  // Dataset configuration
  std::vector<std::string> body_names{
      "ape",  "bakingsoda", "benchviseblue", "broccolisoup", "cam",
      "can",  "cat",        "clown",         "cube",         "driller",
      "duck", "eggbox",     "glue",          "iron",         "koalacandy",
      "lamp", "phone",      "squirrel"};
  std::vector<std::string> sequence_names{
      "a_regular", "b_dynamiclight", "c_noisy", "d_occlusion", "d_occlusion"};
  std::vector<bool> sequence_occlusions{false, false, false, false, true};

  // Experiment configuration
  bool study_function_amplitude = false;
  bool study_function_slope = false;
  bool study_learning_rate = false;
  bool study_tikhonov_parameter = false;
  std::vector<float> function_amplitude{0.2f,   0.225f, 0.25f,  0.275f, 0.3f,
                                        0.325f, 0.35f,  0.375f, 0.4f,   0.425f,
                                        0.45f,  0.475f, 0.5f};
  std::vector<float> function_slope{0.0,   0.125, 0.25,  0.375, 0.5,
                                    0.625, 0.75,  0.875, 1.0,   1.125,
                                    1.25,  1.375, 1.5};
  std::vector<float> learning_rate{0.0f,  0.25f, 0.5f,  0.75f, 1.0f,
                                   1.25f, 1.5f,  1.75f, 2.0f,  2.25f,
                                   2.5f,  2.75f, 3.0f};
  std::vector<float> tikhonov_parameter_rotation{
      100.0f,  177.0f,   316.0f,   562.0f,   1000.0f,  1770.0f,  3160.0f,
      5620.0f, 10000.0f, 17700.0f, 31600.0f, 56200.0f, 100000.0f};
  std::vector<float> tikhonov_parameter_translation{
      10000.0f,   17700.0f,   31600.0f,   56200.0f,   100000.0f,
      177000.0f,  316000.0f,  562000.0f,  1000000.0f, 1770000.0f,
      3160000.0f, 5620000.0f, 10000000.0f};
  std::vector<float> tracking_successes(13);

  // Run experiments
  for (size_t i = 0; i < tracking_successes.size(); ++i) {
    RBOTEvaluator evaluator{"evaluator", dataset_directory, body_names,
                            sequence_names, sequence_occlusions};
    evaluator.set_visualize_all_results(false);
    evaluator.set_region_modality_setter([&](auto r) {
      if (study_function_amplitude)
        r->set_function_amplitude(function_amplitude[i]);
      if (study_function_slope) r->set_function_slope(function_slope[i]);
      if (study_learning_rate) r->set_learning_rate(learning_rate[i]);
      if (study_tikhonov_parameter) {
        r->set_tikhonov_parameter_rotation(tikhonov_parameter_rotation[i]);
        r->set_tikhonov_parameter_translation(
            tikhonov_parameter_translation[i]);
      }
    });
    evaluator.SetUp();
    evaluator.Evaluate();
    tracking_successes[i] = evaluator.tracking_success();
  }

  // Print results
  std::cout << std::endl << std::string(80, '-') << std::endl;
  for (size_t i = 0; i < tracking_successes.size(); ++i) {
    if (study_function_amplitude)
      std::cout << "function_amplitude = " << function_amplitude[i] << ", ";
    if (study_function_slope)
      std::cout << ", function_slope = " << function_slope[i] << ", ";
    if (study_learning_rate)
      std::cout << ", learning_rate = " << learning_rate[i] << ", ";
    if (study_tikhonov_parameter)
      std::cout << ", tikhonov_parameter_rotatione = "
                << tikhonov_parameter_rotation[i] << ", "
                << ", tikhonov_parameter_translation = "
                << tikhonov_parameter_translation[i] << ", ";
    std::cout << "tracking success = " << tracking_successes[i] << std::endl;
  }
  std::cout << std::string(80, '-') << std::endl;
}
