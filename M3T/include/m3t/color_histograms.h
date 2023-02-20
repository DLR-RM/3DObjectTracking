// SPDX-License-Identifier: MIT
// Copyright (c) 2023 Manuel Stoiber, German Aerospace Center (DLR)

#ifndef M3T_INCLUDE_M3T_COLOR_HISTOGRAMS_H_
#define M3T_INCLUDE_M3T_COLOR_HISTOGRAMS_H_

#include <filesystem/filesystem.h>
#include <m3t/common.h>

#include <algorithm>
#include <opencv2/opencv.hpp>
#include <string>
#include <vector>

namespace m3t {

/**
 * \brief Class that computes and holds a color histogram for foreground and
 * background and that is used by the \ref RegionModality.
 *
 * \details It allows adding pixel colors to the internal memory using
 * `AddForegroundColor()` and `AddBackgroundColor()`. Based on those memory
 * values, the foreground and background color histograms can be calculated
 * using `InitializeHistograms()` and `UpdateHistograms()`. `UpdateHistograms()`
 * thereby uses online adaptation with the defined learning rates to take into
 * account previous histogram values. To get the probabilities for a specific
 * `pixel_color`, the method `GetProbabilities()` is used. The internal memory
 * can be cleared using `ClearMemory()`.
 *
 * @param n_bins number of bins that is used to discretize each
 * dimension of the RGB color space. Has to be 2, 4, 8, 16, 32, or 64.
 * @param learning_rate_f learning rate that is used in the update of the
 * foreground histogram.
 * @param learning_rate_b learning rate that is used in the update of the
 * background histogram.
 */
class ColorHistograms {
 public:
  // Constructor and setup method
  ColorHistograms(const std::string &name, int n_bins = 16,
                  float learning_rate_f = 0.2f, float learning_rate_b = 0.2f);
  ColorHistograms(const std::string &name,
                  const std::filesystem::path &metafile_path);
  bool SetUp();

  // Setters
  void set_name(const std::string &name);
  void set_metafile_path(const std::filesystem::path &metafile_path);
  void set_n_bins(int n_bins);
  void set_learning_rate_f(float learning_rate_f);
  void set_learning_rate_b(float learning_rate_b);

  // Main methods
  bool ClearMemory();
  void AddForegroundColor(const cv::Vec3b &pixel_color);
  void AddBackgroundColor(const cv::Vec3b &pixel_color);
  bool InitializeHistograms();
  bool UpdateHistograms();
  void GetProbabilities(const cv::Vec3b &pixel_color,
                        float *pixel_color_probability_f,
                        float *pixel_color_probability_b) const;

  // Getters
  const std::string &name() const;
  const std::filesystem::path &metafile_path() const;
  int n_bins() const;
  float learning_rate_f() const;
  float learning_rate_b() const;
  bool set_up() const;

 private:
  // Helper methods
  bool LoadMetaData();
  bool PrecalculateVariables();
  void SetUpHistograms();
  void CalculateHistogram(float learning_rate,
                          const std::vector<float> &histogram_memory,
                          std::vector<float> *histogram);

  // Internal data objects
  std::vector<float> histogram_memory_f_{};
  std::vector<float> histogram_memory_b_{};
  std::vector<float> histogram_f_{};
  std::vector<float> histogram_b_{};

  // Precalculated variables
  int bitshift_{};
  int n_bins_squared_{};
  int n_bins_cubed_{};

  // Parameters
  std::string name_{};
  std::filesystem::path metafile_path_{};
  int n_bins_ = 16;
  float learning_rate_f_ = 0.2f;
  float learning_rate_b_ = 0.2f;

  // State
  bool histogram_valid_ = false;
  bool set_up_ = false;
};

}  // namespace m3t

#endif  // M3T_INCLUDE_M3T_COLOR_HISTOGRAMS_H_
