// SPDX-License-Identifier: MIT
// Copyright (c) 2021 Manuel Stoiber, German Aerospace Center (DLR)

#include "opt_evaluator.h"

int main() {
  // Directories
  std::string dataset_directory{"/your/path/to/the/opt/dataset/"};

  // Dataset configuration
  std::vector<std::string> body_names{"soda",  "chest", "ironman",
                                      "house", "bike",  "jet"};
  std::vector<std::string> body_orientations{"b", "f", "l", "r"};
  std::vector<std::string> motion_patterns{
      "tr_1", "tr_2", "tr_3", "tr_4", "tr_5", "zo_1", "zo_2", "zo_3",
      "zo_4", "zo_5", "ir_1", "ir_2", "ir_3", "ir_4", "ir_5", "or_1",
      "or_2", "or_3", "or_4", "or_5", "fl",   "ml",   "fm"};

  // Default values
  float default_tikhonov_parameter_rotation_soda = 500000.0f;
  float default_function_slope = 0.5f;
  float default_function_amplitude = 0.42f;

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
  std::vector<float> tikhonov_parameter_rotation_soda{
      10000.0f,   17700.0f,   31600.0f,   56200.0f,   100000.0f,
      177000.0f,  316000.0f,  562000.0f,  1000000.0f, 1770000.0f,
      3160000.0f, 5620000.0f, 10000000.0f};
  std::vector<std::vector<float>> area_under_curve(13);

  // Run experiments
  for (size_t i = 0; i < area_under_curve.size(); ++i) {
    area_under_curve[i].resize(body_names.size());
    for (size_t j = 0; j < body_names.size(); ++j) {
      OPTEvaluator evaluator{"evaluator",
                             dataset_directory,
                             {body_names[j]},
                             body_orientations,
                             motion_patterns};
      evaluator.SetUp();
      evaluator.set_region_modality_setter([&](auto r) {
        // set detault parameters
        if (body_names[j] == "soda")
          r->set_tikhonov_parameter_rotation(
              default_tikhonov_parameter_rotation_soda);
        r->set_function_slope(default_function_slope);
        r->set_function_amplitude(default_function_amplitude);

        // Set parameters for experiment configuration
        if (study_function_amplitude)
          r->set_function_amplitude(function_amplitude[i]);
        if (study_function_slope) r->set_function_slope(function_slope[i]);
        if (study_learning_rate) r->set_learning_rate(learning_rate[i]);
        if (study_tikhonov_parameter) {
          r->set_tikhonov_parameter_rotation(tikhonov_parameter_rotation[i]);
          r->set_tikhonov_parameter_translation(
              tikhonov_parameter_translation[i]);
          if (body_names[j] == "soda")
            r->set_tikhonov_parameter_rotation(
                tikhonov_parameter_rotation_soda[i]);
        }
      });
      evaluator.Evaluate();
      area_under_curve[i][j] = evaluator.area_under_curve();
    }
  }

  // Print results
  std::cout << std::endl << std::string(80, '-') << std::endl;
  for (size_t i = 0; i < area_under_curve.size(); ++i) {
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
    std::cout << "... : ";
    float mean_auc = 0.0f;
    for (size_t j = 0; j < body_names.size(); ++j) {
      mean_auc += area_under_curve[i][j];
      std::cout << body_names[j] << ": " << area_under_curve[i][j] << ", ";
    }
    std::cout << std::endl;
    std::cout << "all: " << mean_auc / body_names.size() << std::endl;
  }
  std::cout << std::string(80, '-') << std::endl;
}
