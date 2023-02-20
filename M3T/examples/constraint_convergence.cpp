// SPDX-License-Identifier: MIT
// Copyright (c) 2023 Manuel Stoiber, German Aerospace Center (DLR)

#include <m3t/constraint.h>
#include <m3t/link.h>
#include <m3t/optimizer.h>

#include <Eigen/Geometry>

Eigen::Vector3f RandomVector(std::uniform_real_distribution<float>* dis,
                             std::mt19937* gen) {
  float theta_x = acosf((*dis)(*gen));
  float theta_z = (*dis)(*gen) * m3t::kPi;
  return Eigen::AngleAxisf(theta_z, Eigen::Vector3f::UnitZ()) *
         Eigen::AngleAxisf(theta_x, Eigen ::Vector3f::UnitX()) *
         Eigen::Vector3f::UnitZ();
}

m3t::Transform3fA RandomPose(std::uniform_real_distribution<float>* dis,
                             std::mt19937* gen) {
  float random_rotation_angle = (*dis)(*gen) * m3t::kPi;
  Eigen::Vector3f random_rotation_axis{RandomVector(dis, gen)};
  Eigen::AngleAxisf random_angle_axis;
  random_angle_axis.angle() = random_rotation_angle;
  random_angle_axis.axis() = random_rotation_axis;
  Eigen::Vector3f random_translation{RandomVector(dis, gen) * (*dis)(*gen)};

  m3t::Transform3fA random_pose{m3t::Transform3fA::Identity()};
  random_pose.rotate(random_angle_axis);
  random_pose.translate(random_translation);
  return random_pose;
}

float RotationalError(const m3t::Transform3fA& pose_error,
                      const std::array<bool, 6>& constraint_directions) {
  Eigen::AngleAxisf angle_axis{pose_error.rotation()};
  Eigen::Vector3f constraint_axis{angle_axis.axis()};
  for (int i = 0; i < 3; ++i)
    if (!constraint_directions[i]) constraint_axis(i) = 0.0f;
  return angle_axis.angle() * constraint_axis.norm();
}

float TranslationalError(const m3t::Transform3fA& pose_error,
                         const std::array<bool, 6>& constraint_directions) {
  Eigen::Vector3f constraint_translation{pose_error.translation().matrix()};
  for (int i = 0; i < 3; ++i)
    if (!constraint_directions[i + 3]) constraint_translation(i) = 0.0f;
  return constraint_translation.norm();
}

void PrintPlotData(const std::vector<std::vector<float>>& errors,
                   const std::string& color) {
  int n_iterations = errors.size();
  int n_runs = errors[0].size();
  for (int q = 0; q <= 100; q += 1) {
    int idx = (q / 100.0f) * (n_runs - 1);
    std::cout << "\\addplot[color = " << color
              << ", opacity = 0.3, name path = p" << q << "] coordinates{";
    for (int i = 0; i < n_iterations; ++i)
      std::cout << "(" << i << ", " << errors[i][idx] << ")";
    std::cout << "};" << std::endl;
  }
  for (int q = 0; q <= 100; q += 10) {
    int idx = (q / 100.0f) * (n_runs - 1);
    std::cout << "\\addplot[color = " << color << ", name path = dashed_p" << q
              << "] coordinates{";
    for (int i = 0; i < n_iterations; ++i)
      std::cout << "(" << i << ", " << errors[i][idx] << ")";
    std::cout << "};" << std::endl;
  }
  std::cout << "\\addplot[color = " << color
            << ", fill opacity = 0.1] fill between[of=p0 and p100];"
            << std::endl;
}

// Script that evaluates the convergence of rotational constraints
int main(int argc, char* argv[]) {
  // Parameters
  int n_runs = 100001;
  int n_iterations = 5;
  bool print_plot_data = true;
  std::array<bool, 6> constraint_directions{true, true, true,
                                            true,  true, true};

  // Iterate over n runs
  std::random_device rd;
  std::mt19937 gen{rd()};
  std::uniform_real_distribution<float> dis(-1.0, 1.0);
  std::vector<std::vector<float>> rotational_errors(n_iterations);
  std::vector<std::vector<float>> translational_errors(n_iterations);
  for (int i = 0; i < n_runs; ++i) {
    // Random poses
    auto body12joint1_pose{RandomPose(&dis, &gen)};
    auto body22joint2_pose{RandomPose(&dis, &gen)};

    // Set up links
    auto link1_ptr{std::make_shared<m3t::Link>("link1")};
    auto link2_ptr{std::make_shared<m3t::Link>("link2")};
    link2_ptr->set_body2joint_pose(body22joint2_pose);
    link2_ptr->set_joint2parent_pose(body12joint1_pose.inverse());
    link1_ptr->AddChildLink(link2_ptr);
    link1_ptr->SetUp();
    link2_ptr->SetUp();

    // Set up constraint
    auto constraint_ptr{std::make_shared<m3t::Constraint>(
        "constraint", link1_ptr, link2_ptr, body12joint1_pose,
        body22joint2_pose)};
    constraint_ptr->set_constraint_directions(constraint_directions);
    constraint_ptr->SetUp();

    // Set up optimizer
    auto optimizer_ptr{
        std::make_shared<m3t::Optimizer>("optimizer", link1_ptr)};
    optimizer_ptr->AddConstraint(constraint_ptr);
    optimizer_ptr->SetUp();

    // Set random pose
    link2_ptr->set_joint2parent_pose(body12joint1_pose.inverse() *
                                     RandomPose(&dis, &gen));
    optimizer_ptr->CalculateConsistentPoses();

    // Evaluate
    for (int j = 0; j < n_iterations; ++j) {
      m3t::Transform3fA pose_error{body12joint1_pose *
                                   link2_ptr->joint2parent_pose()};
      rotational_errors[j].push_back(
          RotationalError(pose_error, constraint_directions));
      translational_errors[j].push_back(
          TranslationalError(pose_error, constraint_directions));
      optimizer_ptr->CalculateOptimization(0, 0, 0);
    }
  }

  // Sort results
  for (int i = 0; i < n_iterations; ++i) {
    std::sort(begin(rotational_errors[i]), end(rotational_errors[i]));
    std::sort(begin(translational_errors[i]), end(translational_errors[i]));
  }

  // Print results
  for (int i = 0; i < n_iterations; ++i) {
    std::cout << "Iteration: " << i << std::endl;
    std::cout << "Rotational Error:";
    for (int q = 0; q <= 100; q += 25) {
      int idx = (q / 100.0f) * (n_runs - 1);
      std::cout << " q" << q << " = " << rotational_errors[i][idx];
    }
    std::cout << std::endl;
    std::cout << "Translational Error:";
    for (int q = 0; q <= 100; q += 25) {
      int idx = (q / 100.0f) * (n_runs - 1);
      std::cout << " q" << q << " = " << translational_errors[i][idx];
    }
    std::cout << std::endl;
  }

  // Print plot data
  if (print_plot_data) {
    std::cout << std::endl << "Rotational Plots:" << std::endl << std::endl;
    PrintPlotData(rotational_errors, "dlrred");
    std::cout << std::endl << "Translational Plots:" << std::endl << std::endl;
    PrintPlotData(translational_errors, "dlrblue");
  }
  return 0;
}
