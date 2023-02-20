// SPDX-License-Identifier: MIT
// Copyright (c) 2023 Manuel Stoiber, German Aerospace Center (DLR)

#include <gtest/gtest.h>
#include <m3t/color_histograms.h>

#include "common_test.h"

const std::filesystem::path color_histograms_test_directory{
    data_directory / "color_histograms_test"};

class ColorHistogramsTest : public testing::Test {
 protected:
  void SetUp() override {
    color_histograms_ptr_ = std::make_shared<m3t::ColorHistograms>(
        name_, n_bins_, learning_rate_f_, learning_rate_b_);
  }

  bool TestColorHistogramsParameters(
      const m3t::ColorHistograms &color_histograms) {
    return color_histograms.name() == name_ &&
           color_histograms.n_bins() == n_bins_ &&
           color_histograms.learning_rate_f() == learning_rate_f_ &&
           color_histograms.learning_rate_b() == learning_rate_b_;
  }

  std::shared_ptr<m3t::ColorHistograms> color_histograms_ptr_;
  std::string name_{"color_histogram"};
  std::filesystem::path metafile_path_{color_histograms_test_directory /
                                       "color_histograms.yaml"};
  int n_bins_ = 32;
  float learning_rate_f_ = 0.5f;
  float learning_rate_b_ = 0.5f;
  cv::Vec3b color1{122, 154, 63};
  cv::Vec3b color2{78, 64, 187};
};

TEST_F(ColorHistogramsTest, SetUpFromData) {
  ASSERT_TRUE(color_histograms_ptr_->SetUp());
  ASSERT_TRUE(color_histograms_ptr_->set_up());
  ASSERT_TRUE(TestColorHistogramsParameters(*color_histograms_ptr_));
}

TEST_F(ColorHistogramsTest, SetUpFromMetaFile) {
  m3t::ColorHistograms color_histograms{name_, metafile_path_};
  ASSERT_TRUE(color_histograms.SetUp());
  ASSERT_TRUE(color_histograms.set_up());
  ASSERT_TRUE(TestColorHistogramsParameters(color_histograms));
}

TEST_F(ColorHistogramsTest, ParameterChanges) {
  ASSERT_TRUE(color_histograms_ptr_->SetUp());
  ASSERT_TRUE(color_histograms_ptr_->set_up());
  color_histograms_ptr_->set_metafile_path(metafile_path_);
  ASSERT_FALSE(color_histograms_ptr_->set_up());
  ASSERT_TRUE(color_histograms_ptr_->SetUp());
  ASSERT_TRUE(color_histograms_ptr_->set_up());
  color_histograms_ptr_->set_n_bins(n_bins_);
  ASSERT_FALSE(color_histograms_ptr_->set_up());
  ASSERT_TRUE(color_histograms_ptr_->SetUp());
  ASSERT_TRUE(color_histograms_ptr_->set_up());
  ASSERT_TRUE(TestColorHistogramsParameters(*color_histograms_ptr_));
}

TEST_F(ColorHistogramsTest, TestWithoutSetUp) {
  ASSERT_FALSE(color_histograms_ptr_->ClearMemory());
  ASSERT_FALSE(color_histograms_ptr_->InitializeHistograms());
  ASSERT_FALSE(color_histograms_ptr_->UpdateHistograms());
}

TEST_F(ColorHistogramsTest, TestHistogramCalculation) {
  float probability_f, probability_b;
  ASSERT_TRUE(color_histograms_ptr_->SetUp());
  color_histograms_ptr_->AddForegroundColor(color1);
  color_histograms_ptr_->AddBackgroundColor(color2);
  color_histograms_ptr_->ClearMemory();
  color_histograms_ptr_->UpdateHistograms();
  color_histograms_ptr_->GetProbabilities(color1, &probability_f,
                                          &probability_b);
  ASSERT_EQ(probability_f, 1.0f / float(n_bins_ * n_bins_ * n_bins_));
  ASSERT_EQ(probability_b, 1.0f / float(n_bins_ * n_bins_ * n_bins_));
  color_histograms_ptr_->AddForegroundColor(color1);
  color_histograms_ptr_->AddForegroundColor(color2);
  color_histograms_ptr_->AddBackgroundColor(color2);
  color_histograms_ptr_->InitializeHistograms();
  color_histograms_ptr_->GetProbabilities(color1, &probability_f, &probability_b);
  ASSERT_EQ(probability_f, 0.5f);
  ASSERT_EQ(probability_b, 0.0f);
  color_histograms_ptr_->GetProbabilities(color2, &probability_f,
                                          &probability_b);
  ASSERT_EQ(probability_f, 0.5f);
  ASSERT_EQ(probability_b, 1.0f);
  color_histograms_ptr_->AddForegroundColor(color2);
  color_histograms_ptr_->AddBackgroundColor(color1);
  color_histograms_ptr_->UpdateHistograms();
  color_histograms_ptr_->GetProbabilities(color1, &probability_f,
                                          &probability_b);
  ASSERT_EQ(probability_f, 0.25f);
  ASSERT_EQ(probability_b, 0.5f);
  color_histograms_ptr_->GetProbabilities(color2, &probability_f,
                                          &probability_b);
  ASSERT_EQ(probability_f, 0.75f);
  ASSERT_EQ(probability_b, 0.5f);
}
