// SPDX-License-Identifier: MIT
// Copyright (c) 2023 Manuel Stoiber, German Aerospace Center (DLR)

#include <m3t/color_histograms.h>

namespace m3t {

ColorHistograms::ColorHistograms(const std::string& name, int n_bins,
                                 float learning_rate_f, float learning_rate_b)
    : name_{name},
      n_bins_{n_bins},
      learning_rate_f_{learning_rate_f},
      learning_rate_b_{learning_rate_b} {}

ColorHistograms::ColorHistograms(const std::string& name,
                                 const std::filesystem::path& metafile_path)
    : name_{name}, metafile_path_{metafile_path} {}

bool ColorHistograms::SetUp() {
  set_up_ = false;
  if (!metafile_path_.empty())
    if (!LoadMetaData()) return false;
  if (!PrecalculateVariables()) return false;
  SetUpHistograms();
  set_up_ = true;
  return true;
}

void ColorHistograms::set_name(const std::string& name) { name_ = name; }

void ColorHistograms::set_metafile_path(
    const std::filesystem::path& metafile_path) {
  metafile_path_ = metafile_path;
  set_up_ = false;
}

void ColorHistograms::set_n_bins(int n_bins) {
  n_bins_ = n_bins;
  set_up_ = false;
}

void ColorHistograms::set_learning_rate_f(float learning_rate_f) {
  learning_rate_f_ = learning_rate_f;
}

void ColorHistograms::set_learning_rate_b(float learning_rate_b) {
  learning_rate_b_ = learning_rate_b;
}

bool ColorHistograms::ClearMemory() {
  if (!set_up_) {
    std::cerr << "Set up color histogram " << name_ << " first" << std::endl;
    return false;
  }
  std::fill(begin(histogram_memory_f_), end(histogram_memory_f_), 0.0f);
  std::fill(begin(histogram_memory_b_), end(histogram_memory_b_), 0.0f);
  return true;
}

void ColorHistograms::AddForegroundColor(const cv::Vec3b& pixel_color) {
  histogram_memory_f_[int((pixel_color[0] >> bitshift_) * n_bins_squared_) +
                      int((pixel_color[1] >> bitshift_) * n_bins_) +
                      int(pixel_color[2] >> bitshift_)] += 1.0f;
}

void ColorHistograms::AddBackgroundColor(const cv::Vec3b& pixel_color) {
  histogram_memory_b_[int((pixel_color[0] >> bitshift_) * n_bins_squared_) +
                      int((pixel_color[1] >> bitshift_) * n_bins_) +
                      int(pixel_color[2] >> bitshift_)] += 1.0f;
}

bool ColorHistograms::InitializeHistograms() {
  if (!set_up_) {
    std::cerr << "Set up color histogram " << name_ << " first" << std::endl;
    return false;
  }
  CalculateHistogram(1.0f, histogram_memory_f_, &histogram_f_);
  CalculateHistogram(1.0f, histogram_memory_b_, &histogram_b_);
  ClearMemory();
  return true;
}

bool ColorHistograms::UpdateHistograms() {
  if (!set_up_) {
    std::cerr << "Set up color histogram " << name_ << " first" << std::endl;
    return false;
  }
  CalculateHistogram(learning_rate_f_, histogram_memory_f_, &histogram_f_);
  CalculateHistogram(learning_rate_b_, histogram_memory_b_, &histogram_b_);
  ClearMemory();
  return true;
}

void ColorHistograms::GetProbabilities(const cv::Vec3b& pixel_color,
                                       float* pixel_color_probability_f,
                                       float* pixel_color_probability_b) const {
  int idx = (pixel_color[0] >> bitshift_) * n_bins_squared_ +
            (pixel_color[1] >> bitshift_) * n_bins_ +
            (pixel_color[2] >> bitshift_);
  *pixel_color_probability_f = histogram_f_[idx];
  *pixel_color_probability_b = histogram_b_[idx];
}

const std::string& ColorHistograms::name() const { return name_; }

const std::filesystem::path& ColorHistograms::metafile_path() const {
  return metafile_path_;
}

int ColorHistograms::n_bins() const { return n_bins_; }

float ColorHistograms::learning_rate_f() const { return learning_rate_f_; }

float ColorHistograms::learning_rate_b() const { return learning_rate_b_; }

bool ColorHistograms::set_up() const { return set_up_; }

bool ColorHistograms::LoadMetaData() {
  // Open file storage from yaml
  cv::FileStorage fs;
  if (!OpenYamlFileStorage(metafile_path_, &fs)) return false;

  // Read parameters from yaml
  ReadOptionalValueFromYaml(fs, "n_bins", &n_bins_);
  ReadOptionalValueFromYaml(fs, "learning_rate_f", &learning_rate_f_);
  ReadOptionalValueFromYaml(fs, "learning_rate_b", &learning_rate_b_);
  fs.release();
  return true;
}

bool ColorHistograms::PrecalculateVariables() {
  switch (n_bins_) {
    case 2:
      bitshift_ = 7;
      break;
    case 4:
      bitshift_ = 6;
      break;
    case 8:
      bitshift_ = 5;
      break;
    case 16:
      bitshift_ = 4;
      break;
    case 32:
      bitshift_ = 3;
      break;
    case 64:
      bitshift_ = 2;
      break;
    default:
      std::cerr << "n_bins = " << n_bins_ << " in histogram " << name_
                << " not valid."
                << "Has to be of value 2, 4, 8, 16, 32, or 64" << std::endl;
      return false;
  }
  n_bins_squared_ = pow_int(n_bins_, 2);
  n_bins_cubed_ = pow_int(n_bins_, 3);
  return true;
}

void ColorHistograms::SetUpHistograms() {
  histogram_memory_f_.resize(n_bins_cubed_);
  histogram_memory_b_.resize(n_bins_cubed_);
  histogram_f_.resize(n_bins_cubed_);
  histogram_b_.resize(n_bins_cubed_);
  std::fill(begin(histogram_memory_f_), end(histogram_memory_f_), 0.0f);
  std::fill(begin(histogram_memory_b_), end(histogram_memory_b_), 0.0f);
  float uniform_value = 1.0f / float(n_bins_cubed_);
  std::fill(begin(histogram_f_), end(histogram_f_), uniform_value);
  std::fill(begin(histogram_b_), end(histogram_b_), uniform_value);
}

void ColorHistograms::CalculateHistogram(
    float learning_rate, const std::vector<float>& histogram_memory,
    std::vector<float>* histogram) {
  // Calculate sum for normalization
  float sum = 0.0f;
#ifndef _DEBUG
#pragma omp simd
#endif
  for (int i = 0; i < n_bins_cubed_; i++) {
    sum += histogram_memory[i];
  }

  // Apply uniform value if sum is zero and learning rate 1
  if (!sum) {
    if (learning_rate == 1.0f) {
      float uniform_value = 1.0f / n_bins_cubed_;
      std::fill(begin(*histogram), end(*histogram), uniform_value);
    }
    return;
  }

  // Calculate histogram
  float complement_learning_rate = 1.0f - learning_rate;
  float learning_rate_divide_sum = learning_rate / sum;
  if (complement_learning_rate == 0.0f) {
#ifndef _DEBUG
#pragma omp simd
#endif
    for (int i = 0; i < n_bins_cubed_; i++) {
      (*histogram)[i] = histogram_memory[i] * learning_rate_divide_sum;
    }
  } else {
#ifndef _DEBUG
#pragma omp simd
#endif
    for (int i = 0; i < n_bins_cubed_; i++) {
      (*histogram)[i] *= complement_learning_rate;
      (*histogram)[i] += histogram_memory[i] * learning_rate_divide_sum;
    }
  }
}

}  // namespace m3t
