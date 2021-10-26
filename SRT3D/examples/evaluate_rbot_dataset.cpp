// SPDX-License-Identifier: MIT
// Copyright (c) 2021 Manuel Stoiber, German Aerospace Center (DLR)

#include "rbot_evaluator.h"

int main() {
  // Directories
  std::string dataset_directory{"/your/path/to/the/rbot/dataset/"};
  std::string result_directory{"/your/path/to/store/results/"};

  // Dataset configuration
  std::vector<std::string> body_names{
      "ape",  "bakingsoda", "benchviseblue", "broccolisoup", "cam",
      "can",  "cat",        "clown",         "cube",         "driller",
      "duck", "eggbox",     "glue",          "iron",         "koalacandy",
      "lamp", "phone",      "squirrel"};
  std::vector<std::string> sequence_names{
      "a_regular", "b_dynamiclight", "c_noisy", "d_occlusion", "d_occlusion"};
  std::vector<bool> sequence_occlusions{false, false, false, false, true};

  // Run experiments
  RBOTEvaluator evaluator{"evaluator", dataset_directory, body_names,
                          sequence_names, sequence_occlusions};
  evaluator.set_visualize_all_results(false);
  evaluator.SaveResults(result_directory);
  evaluator.SetUp();
  evaluator.Evaluate();

  // Print results
  std::cout << std::endl << std::string(80, '-') << std::endl;
  std::cout << "tracking success = " << evaluator.tracking_success()
            << std::endl;
  std::cout << std::string(80, '-') << std::endl;
}
