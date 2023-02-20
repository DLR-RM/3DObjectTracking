// SPDX-License-Identifier: MIT
// Copyright (c) 2023 Manuel Stoiber, German Aerospace Center (DLR)

#include <m3t/constraint.h>
#include <m3t/link.h>
#include <m3t/optimizer.h>

#include <Eigen/Geometry>

// Script that evaluates the convergence of rotational constraints
int main(int argc, char* argv[]) {
  // Parameters
  bool test_constraints = false;
  int n_runs = 100000;
  int n_bodies_max = 50;

  // Iterate over n runs and n bodies
  std::vector<float> accumulated_optimization_times(n_bodies_max);
  for (int i = 0; i < n_runs; ++i) {
    for (int n_bodies = 1; n_bodies <= n_bodies_max; ++n_bodies) {
      std::vector<std::shared_ptr<m3t::Link>> link_ptrs(n_bodies + 1);
      std::vector<std::shared_ptr<m3t::Constraint>> constraint_ptrs(n_bodies);

      // Define root link and optimizer
      link_ptrs[0] = std::make_shared<m3t::Link>("root link");
      link_ptrs[0]->set_free_directions(
          {false, false, false, false, false, false});
      auto optimizer_ptr{
          std::make_shared<m3t::Optimizer>("optimizer", link_ptrs[0])};

      // Create links and constraints
      if (test_constraints) {
        for (int j = 1; j <= n_bodies; ++j) {
          link_ptrs[j] =
              std::make_shared<m3t::Link>("link" + std::to_string(j));
          optimizer_ptr->root_link_ptr()->AddChildLink(link_ptrs[j]);
          constraint_ptrs[j - 1] = std::make_shared<m3t::Constraint>(
              "constraint" + std::to_string(j), link_ptrs[j - 1], link_ptrs[j]);
          constraint_ptrs[j - 1]->set_body12joint1_pose(
              m3t::Transform3fA{Eigen::Translation3f{-0.01f, 0.0f, 0.0f}});
          constraint_ptrs[j - 1]->set_constraint_directions(
              {false, true, true, true, true, true});
          optimizer_ptr->AddConstraint(constraint_ptrs[j - 1]);
        }
        for (auto& link_ptr : link_ptrs) link_ptr->SetUp();
        for (auto& constraint_ptr : constraint_ptrs) constraint_ptr->SetUp();
      } else {
        for (int j = 1; j <= n_bodies; ++j) {
          link_ptrs[j] =
              std::make_shared<m3t::Link>("link" + std::to_string(j));
          link_ptrs[j]->set_joint2parent_pose(
              m3t::Transform3fA{Eigen::Translation3f{0.01f, 0.0f, 0.0f}});
          link_ptrs[j]->set_free_directions(
              {true, false, false, false, false, false});
          link_ptrs[j - 1]->AddChildLink(link_ptrs[j]);
        }
        for (auto& link_ptr : link_ptrs) link_ptr->SetUp();
      }
      optimizer_ptr->SetUp();

      // Execute optimization and measure time
      auto begin_time{std::chrono::high_resolution_clock::now()};
      optimizer_ptr->CalculateOptimization(0, 0, 0);
      auto end_time{std::chrono::high_resolution_clock::now()};
      accumulated_optimization_times[n_bodies - 1] +=
          float(std::chrono::duration_cast<std::chrono::microseconds>(
                    end_time - begin_time)
                    .count());
    }
    std::cout << "Iteration: " << i << std::endl;
  }

  // Print results
  for (int n_bodies = 1; n_bodies <= n_bodies_max; ++n_bodies) {
    float avg_optimizatino_time =
        accumulated_optimization_times[n_bodies - 1] / n_runs;
    std::cout << "(" << n_bodies << "," << avg_optimizatino_time / 1000.0f
              << ")";
  }

  return 0;
}
