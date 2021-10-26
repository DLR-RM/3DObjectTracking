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

  // Tracker configuration
  float tikhonov_parameter_rotation_soda = 500000.0f;
  float function_slope = 0.5f;
  float function_amplitude = 0.42f;
  std::vector<float> area_under_curve(6);

  // Run experiments
  for (size_t i = 0; i < body_names.size(); ++i) {
    OPTEvaluator evaluator{"evaluator",
                           dataset_directory,
                           {body_names[i]},
                           body_orientations,
                           motion_patterns};
    evaluator.SetUp();
    evaluator.set_region_modality_setter([&](auto r) {
      if (body_names[i] == "soda")
        r->set_tikhonov_parameter_rotation(tikhonov_parameter_rotation_soda);
      r->set_function_slope(function_slope);
      r->set_function_amplitude(function_amplitude);
    });
    evaluator.Evaluate();
    area_under_curve[i] = evaluator.area_under_curve();
  }

  // Print results
  std::cout << std::endl << std::string(80, '-') << std::endl;
  float mean_auc = 0.0f;
  for (size_t i = 0; i < body_names.size(); ++i) {
    mean_auc += area_under_curve[i];
    std::cout << body_names[i] << ": " << area_under_curve[i] << ", ";
  }
  std::cout << std::endl;
  std::cout << "all: " << mean_auc / body_names.size() << std::endl;
  std::cout << std::string(80, '-') << std::endl;
}
