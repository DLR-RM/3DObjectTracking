// SPDX-License-Identifier: MIT
// Copyright (c) 2020 Manuel Stoiber, German Aerospace Center (DLR)

#ifndef OBJECT_TRACKING_INCLUDE_RBGT_COMMON_H_
#define OBJECT_TRACKING_INCLUDE_RBGT_COMMON_H_

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <filesystem>
#include <fstream>
#include <opencv2/opencv.hpp>
#include <string>
#include <vector>

namespace rbgt {

// Commonly used types
using Transform3fA = Eigen::Transform<float, 3, Eigen::Affine>;

// Commonly used constants
constexpr float kPi = 3.1415926535897f;

// Commonly used structs
using Intrinsics = struct Intrinsics {
  float fu, fv;
  float ppu, ppv;
  int width, height;
};

// Commonly used mathematic functions
template <typename T>
inline int sgn(T value) {
  if (value < T(0))
    return -1;
  else if (value > T(0))
    return 1;
  else
    return 0;
}

template <typename T>
inline float sgnf(T value) {
  if (value < T(0))
    return -1.0f;
  else if (value > T(0))
    return 1.0f;
  else
    return 0.0f;
}

inline Eigen::Matrix3f Vector2Skewsymmetric(const Eigen::Vector3f &vector) {
  Eigen::Matrix3f skew_symmetric;
  skew_symmetric << 0.0f, -vector(2), vector(1), vector(2), 0.0f, -vector(0),
      -vector(1), vector(0), 0.0f;
  return skew_symmetric;
}

// Commonly used functions to read and write value to file
void ReadValueFromFile(std::ifstream &ifs, bool *value);
void ReadValueFromFile(std::ifstream &ifs, int *value);
void ReadValueFromFile(std::ifstream &ifs, float *value);
void ReadValueFromFile(std::ifstream &ifs, std::string *value);
void ReadValueFromFile(std::ifstream &ifs, Transform3fA *value);
void ReadValueFromFile(std::ifstream &ifs, Intrinsics *value);
void ReadValueFromFile(std::ifstream &ifs, std::filesystem::path *value);

void WriteValueToFile(std::ofstream &ofs, const std::string &name, bool value);
void WriteValueToFile(std::ofstream &ofs, const std::string &name, int value);
void WriteValueToFile(std::ofstream &ofs, const std::string &name, float value);
void WriteValueToFile(std::ofstream &ofs, const std::string &name,
                      const std::string &value);
void WriteValueToFile(std::ofstream &ofs, const std::string &name,
                      const Transform3fA &value);
void WriteValueToFile(std::ofstream &ofs, const std::string &name,
                      const Intrinsics &value);
void WriteValueToFile(std::ofstream &ofs, const std::string &name,
                      const std::filesystem::path &value);

// Commonly used functions to plot points to image
void DrawPointInImage(const Eigen::Vector3f &point_f_camera,
                      const cv::Vec3b &color, const Intrinsics &intrinsics,
                      cv::Mat *image);

}  // namespace rbgt

#endif  // OBJECT_TRACKING_INCLUDE_RBGT_COMMON_H_
