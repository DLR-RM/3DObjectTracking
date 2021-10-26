// SPDX-License-Identifier: MIT
// Copyright (c) 2021 Manuel Stoiber, German Aerospace Center (DLR)

#include <srt3d/common.h>

namespace srt3d {

void ReadValueFromFile(std::ifstream &ifs, bool *value) {
  std::string parsed;
  std::getline(ifs, parsed);
  std::getline(ifs, parsed, '\t');
  *value = stoi(parsed);
  std::getline(ifs, parsed);
}

void ReadValueFromFile(std::ifstream &ifs, int *value) {
  std::string parsed;
  std::getline(ifs, parsed);
  std::getline(ifs, parsed, '\t');
  *value = stoi(parsed);
  std::getline(ifs, parsed);
}

void ReadValueFromFile(std::ifstream &ifs, float *value) {
  std::string parsed;
  std::getline(ifs, parsed);
  std::getline(ifs, parsed, '\t');
  *value = stof(parsed);
  std::getline(ifs, parsed);
}

void ReadValueFromFile(std::ifstream &ifs, std::string *value) {
  std::string parsed;
  std::getline(ifs, parsed);
  std::getline(ifs, *value, '\t');
  std::getline(ifs, parsed);
}

void ReadValueFromFile(std::ifstream &ifs, Transform3fA *value) {
  std::string parsed;
  std::getline(ifs, parsed);
  Eigen::Matrix4f mat;
  for (int i = 0; i < 4; ++i) {
    for (int j = 0; j < 4; ++j) {
      std::getline(ifs, parsed, '\t');
      mat(i, j) = stof(parsed);
    }
    std::getline(ifs, parsed);
  }
  *value = Transform3fA(mat);
}

void ReadValueFromFile(std::ifstream &ifs, Intrinsics *value) {
  std::string parsed;
  std::getline(ifs, parsed);
  std::getline(ifs, parsed);
  std::getline(ifs, parsed, '\t');
  value->fu = stof(parsed);
  std::getline(ifs, parsed, '\t');
  value->fv = stof(parsed);
  std::getline(ifs, parsed, '\t');
  value->ppu = stof(parsed);
  std::getline(ifs, parsed, '\t');
  value->ppv = stof(parsed);
  std::getline(ifs, parsed, '\t');
  value->width = stoi(parsed);
  std::getline(ifs, parsed, '\t');
  value->height = stoi(parsed);
  std::getline(ifs, parsed);
}

void ReadValueFromFile(std::ifstream &ifs, std::filesystem::path *value) {
  std::string parsed;
  std::getline(ifs, parsed);
  std::getline(ifs, parsed, '\t');
  value->assign(parsed);
  std::getline(ifs, parsed);
}

void WriteValueToFile(std::ofstream &ofs, const std::string &name, bool value) {
  ofs << name << std::endl;
  ofs << value << "\t" << std::endl;
}

void WriteValueToFile(std::ofstream &ofs, const std::string &name, int value) {
  ofs << name << std::endl;
  ofs << value << "\t" << std::endl;
}

void WriteValueToFile(std::ofstream &ofs, const std::string &name,
                      float value) {
  ofs << name << std::endl;
  ofs << value << "\t" << std::endl;
}

void WriteValueToFile(std::ofstream &ofs, const std::string &name,
                      const std::string &value) {
  ofs << name << std::endl;
  ofs << value << "\t" << std::endl;
}

void WriteValueToFile(std::ofstream &ofs, const std::string &name,
                      const Transform3fA &value) {
  ofs << name << std::endl;
  for (int i = 0; i < 4; ++i) {
    for (int j = 0; j < 4; ++j) {
      ofs << value.matrix()(i, j) << "\t";
    }
    ofs << std::endl;
  }
}

void WriteValueToFile(std::ofstream &ofs, const std::string &name,
                      const Intrinsics &value) {
  ofs << name << std::endl;
  ofs << "f_x, \t f_y, \t pp_x, \t pp_y, \t width, \t height" << std::endl;
  ofs << value.fu << "\t";
  ofs << value.fv << "\t";
  ofs << value.ppu << "\t";
  ofs << value.ppv << "\t";
  ofs << value.width << "\t";
  ofs << value.height << "\t";
  ofs << std::endl;
}

void WriteValueToFile(std::ofstream &ofs, const std::string &name,
                      const std::filesystem::path &value) {
  ofs << name << std::endl;
  ofs << value.string() << "\t" << std::endl;
}

void DrawPointInImage(const Eigen::Vector3f &point_f_camera,
                      const cv::Vec3b &color, const Intrinsics &intrinsics,
                      cv::Mat *image) {
  int u = int(point_f_camera(0) * intrinsics.fu / point_f_camera(2) +
              intrinsics.ppu + 0.5);
  int v = int(point_f_camera(1) * intrinsics.fv / point_f_camera(2) +
              intrinsics.ppv + 0.5);
  if (u >= 1 && v >= 1 && u <= image->cols - 2 && v <= image->rows - 2) {
    image->at<cv::Vec3b>(v, u) = color;
    image->at<cv::Vec3b>(v - 1, u) = color;
    image->at<cv::Vec3b>(v + 1, u) = color;
    image->at<cv::Vec3b>(v, u - 1) = color;
    image->at<cv::Vec3b>(v, u + 1) = color;
  }
}

}  // namespace srt3d
