// SPDX-License-Identifier: MIT
// Copyright (c) 2023 Manuel Stoiber, German Aerospace Center (DLR)

#include <m3t/common.h>

#include <opencv2/core/eigen.hpp>

namespace m3t {

bool Equivalent(const std::filesystem::path &path1,
                const std::filesystem::path &path2) {
  try {
    return std::filesystem::equivalent(path1, path2);
  } catch (std::filesystem::filesystem_error const &e) {
    return false;
  }
}

void ReadValueFromTxt(std::ifstream &ifs, bool *value) {
  std::string parsed;
  std::getline(ifs, parsed);
  std::getline(ifs, parsed, ',');
  *value = stoi(parsed);
  std::getline(ifs, parsed);
}

void ReadValueFromTxt(std::ifstream &ifs, int *value) {
  std::string parsed;
  std::getline(ifs, parsed);
  std::getline(ifs, parsed, ',');
  *value = stoi(parsed);
  std::getline(ifs, parsed);
}

void ReadValueFromTxt(std::ifstream &ifs, float *value) {
  std::string parsed;
  std::getline(ifs, parsed);
  std::getline(ifs, parsed, ',');
  *value = stof(parsed);
  std::getline(ifs, parsed);
}

void ReadValueFromTxt(std::ifstream &ifs, std::string *value) {
  std::string parsed;
  std::getline(ifs, parsed);
  std::getline(ifs, *value, ',');
  std::getline(ifs, parsed);
}

void ReadValueFromTxt(std::ifstream &ifs, Transform3fA *value) {
  std::string parsed;
  std::getline(ifs, parsed);
  Eigen::Matrix4f mat;
  for (int i = 0; i < 4; ++i) {
    for (int j = 0; j < 4; ++j) {
      std::getline(ifs, parsed, ',');
      mat(i, j) = stof(parsed);
    }
    std::getline(ifs, parsed);
  }
  *value = Transform3fA(mat);
}

void ReadValueFromTxt(std::ifstream &ifs, Eigen::MatrixXf *value) {
  std::string parsed;
  std::getline(ifs, parsed);
  std::getline(ifs, parsed, ',');
  int rows = stoi(parsed);
  std::getline(ifs, parsed, ',');
  int cols = stoi(parsed);
  std::getline(ifs, parsed);
  value->resize(rows, cols);
  for (int i = 0; i < rows; ++i) {
    for (int j = 0; j < cols; ++j) {
      std::getline(ifs, parsed, ',');
      (*value)(i, j) = stof(parsed);
    }
    std::getline(ifs, parsed);
  }
}

void ReadValueFromTxt(std::ifstream &ifs, Intrinsics *value) {
  std::string parsed;
  std::getline(ifs, parsed);
  std::getline(ifs, parsed);
  std::getline(ifs, parsed, ',');
  value->fu = stof(parsed);
  std::getline(ifs, parsed, ',');
  value->fv = stof(parsed);
  std::getline(ifs, parsed, ',');
  value->ppu = stof(parsed);
  std::getline(ifs, parsed, ',');
  value->ppv = stof(parsed);
  std::getline(ifs, parsed, ',');
  value->width = stoi(parsed);
  std::getline(ifs, parsed, ',');
  value->height = stoi(parsed);
  std::getline(ifs, parsed);
}

void ReadValueFromTxt(std::ifstream &ifs, std::filesystem::path *value) {
  std::string parsed;
  std::getline(ifs, parsed);
  std::getline(ifs, parsed, ',');
  value->assign(parsed);
  std::getline(ifs, parsed);
}

void WriteValueToTxt(std::ofstream &ofs, const std::string &name, bool value) {
  ofs << name << std::endl;
  ofs << value << "," << std::endl;
}

void WriteValueToTxt(std::ofstream &ofs, const std::string &name, int value) {
  ofs << name << std::endl;
  ofs << value << "," << std::endl;
}

void WriteValueToTxt(std::ofstream &ofs, const std::string &name, float value) {
  ofs << name << std::endl;
  ofs << value << "," << std::endl;
}

void WriteValueToTxt(std::ofstream &ofs, const std::string &name,
                     const std::string &value) {
  ofs << name << std::endl;
  ofs << value << "," << std::endl;
}

void WriteValueToTxt(std::ofstream &ofs, const std::string &name,
                     const Transform3fA &value) {
  ofs << name << std::endl;
  for (int i = 0; i < 4; ++i) {
    for (int j = 0; j < 4; ++j) {
      ofs << value.matrix()(i, j) << ",\t";
    }
    ofs << std::endl;
  }
}

void WriteValueToTxt(std::ofstream &ofs, const std::string &name,
                     const Eigen::MatrixXf &value) {
  ofs << name << std::endl;
  ofs << value.rows() << ",\t" << value.cols() << "," << std::endl;
  for (int i = 0; i < value.rows(); ++i) {
    for (int j = 0; j < value.cols(); ++j) {
      ofs << value(i, j) << ",\t";
    }
    ofs << std::endl;
  }
}

void WriteValueToTxt(std::ofstream &ofs, const std::string &name,
                     const Intrinsics &value) {
  ofs << name << std::endl;
  ofs << "f_x,\tf_y,\tpp_x,\tpp_y,\twidth,\theight" << std::endl;
  ofs << value.fu << ",\t";
  ofs << value.fv << ",\t";
  ofs << value.ppu << ",\t";
  ofs << value.ppv << ",\t";
  ofs << value.width << ",\t";
  ofs << value.height << ",";
  ofs << std::endl;
}

void WriteValueToTxt(std::ofstream &ofs, const std::string &name,
                     const std::filesystem::path &value) {
  ofs << name << std::endl;
  ofs << value.string() << "," << std::endl;
}

bool OpenYamlFileStorage(const std::filesystem::path &metafile_path,
                         cv::FileStorage *file_storage) {
  if (!std::filesystem::exists(metafile_path) ||
      metafile_path.filename().extension() != ".yaml") {
    std::cerr << "Could not find yaml file " << metafile_path << std::endl;
    return false;
  }

  try {
    file_storage->open(cv::samples::findFile(metafile_path.string()),
                       cv::FileStorage::READ);
  } catch (cv::Exception e) {
    std::cerr << "Could not open yaml file " << metafile_path << std::endl;
    return false;
  }

  if (!file_storage->isOpened()) {
    std::cerr << "Could not open yaml file " << metafile_path << std::endl;
    return false;
  }
  return true;
}

void ReadValueFromYaml(const cv::FileStorage &file_storage,
                       const std::string &name, std::filesystem::path *value) {
  std::string path;
  file_storage[name] >> path;
  *value = std::filesystem::path{path};
}

void ReadValueFromYaml(const cv::FileStorage &file_storage,
                       const std::string &name, Intrinsics *value) {
  cv::FileNode intrinsics = file_storage[name];
  assert(intrinsics.type() != cv::FileNode::SEQ);

  intrinsics["f_u"] >> value->fu;
  intrinsics["f_v"] >> value->fv;
  intrinsics["pp_x"] >> value->ppu;
  intrinsics["pp_y"] >> value->ppv;
  intrinsics["width"] >> value->width;
  intrinsics["height"] >> value->height;
}

void ReadValueFromYaml(const cv::FileStorage &file_storage,
                       const std::string &name, Transform3fA *value) {
  cv::Mat matrix;
  file_storage[name] >> matrix;
  cv::cv2eigen(matrix, value->matrix());
}

void ReadValueFromYaml(const cv::FileStorage &file_storage,
                       const std::string &name, Eigen::MatrixXf *value) {
  cv::Mat matrix;
  file_storage[name] >> matrix;
  cv::cv2eigen(matrix, *value);
}

void ReadValueFromYaml(const cv::FileStorage &file_storage,
                       const std::string &name,
                       std::vector<cv::Point3f> *value) {
  value->clear();
  cv::FileNode file_node = file_storage[name];
  for (const auto &it : file_node) {
    value->push_back(cv::Point3d{double(it[0]), double(it[1]), double(it[2])});
  }
}

void ReadValueFromYaml(const cv::FileStorage &file_storage,
                       const std::string &name,
                       std::map<std::string, int> *value) {
  value->clear();
  cv::FileNode file_node = file_storage[name];
  for (const auto &it : file_node) {
    value->insert(std::make_pair(it["name"].string(), int(it["id"])));
  }
}

void WriteValueToYaml(cv::FileStorage file_storage, const std::string &name,
                      Intrinsics intrinsics) {
  file_storage << name;
  file_storage << "{"
               << "f_u" << intrinsics.fu;
  file_storage << "f_v" << intrinsics.fv;
  file_storage << "pp_x" << intrinsics.ppu;
  file_storage << "pp_y" << intrinsics.ppv;
  file_storage << "width" << intrinsics.width;
  file_storage << "height" << intrinsics.height << "}";
}

void WriteValueToYaml(cv::FileStorage file_storage, const std::string &name,
                      Transform3fA transformation) {
  cv::Mat camera2_world_pose;
  cv::eigen2cv(transformation.matrix(), camera2_world_pose);
  file_storage << name << camera2_world_pose;
}

void DrawPointInImage(const Eigen::Vector3f &point_f_camera,
                      const cv::Vec3b &color, const Intrinsics &intrinsics,
                      cv::Mat *image) {
  int u = int(point_f_camera(0) * intrinsics.fu / point_f_camera(2) +
              intrinsics.ppu + 0.5);
  int v = int(point_f_camera(1) * intrinsics.fv / point_f_camera(2) +
              intrinsics.ppv + 0.5);
  cv::circle(*image, cv::Point2i{u, v}, 1, color, cv::FILLED);
}

void DrawLineInImage(const Eigen::Vector3f &point1_f_camera,
                     const Eigen::Vector3f &point2_f_camera,
                     const cv::Vec3b &color, const Intrinsics &intrinsics,
                     cv::Mat *image) {
  int u1 = int(point1_f_camera(0) * intrinsics.fu / point1_f_camera(2) +
               intrinsics.ppu + 0.5);
  int v1 = int(point1_f_camera(1) * intrinsics.fv / point1_f_camera(2) +
               intrinsics.ppv + 0.5);
  int u2 = int(point2_f_camera(0) * intrinsics.fu / point2_f_camera(2) +
               intrinsics.ppu + 0.5);
  int v2 = int(point2_f_camera(1) * intrinsics.fv / point2_f_camera(2) +
               intrinsics.ppv + 0.5);
  cv::line(*image, cv::Point2i{u1, v1}, cv::Point2i{u2, v2}, color);
}

void DrawFocusedPointInImage(const Eigen::Vector3f &point_f_camera,
                             const cv::Vec3b &color,
                             const Intrinsics &intrinsics, float corner_u,
                             float corner_v, float scale, cv::Mat *image) {
  float u =
      point_f_camera(0) * intrinsics.fu / point_f_camera(2) + intrinsics.ppu;
  float v =
      point_f_camera(1) * intrinsics.fv / point_f_camera(2) + intrinsics.ppv;
  int u_focused = int((u - corner_u) * scale + 0.5f);
  int v_focused = int((v - corner_v) * scale + 0.5f);
  cv::circle(*image, cv::Point2i{u_focused, v_focused}, 1, color, cv::FILLED);
}

}  // namespace m3t
