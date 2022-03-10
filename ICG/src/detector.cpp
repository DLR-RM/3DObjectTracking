// SPDX-License-Identifier: MIT
// Copyright (c) 2022 Manuel Stoiber, German Aerospace Center (DLR)

#include <icg/detector.h>

namespace icg {

void Detector::set_name(const std::string &name) { name_ = name; }

void Detector::set_metafile_path(const std::filesystem::path &metafile_path) {
  metafile_path_ = metafile_path;
  set_up_ = false;
}

const std::string &Detector::name() const { return name_; }

const std::filesystem::path &Detector::metafile_path() const {
  return metafile_path_;
}

std::vector<std::shared_ptr<Body>> Detector::body_ptrs() const {
  return {nullptr};
}

std::shared_ptr<Camera> Detector::camera_ptr() const { return nullptr; }

bool Detector::set_up() const { return set_up_; }

Detector::Detector(const std::string &name) : name_{name} {}

Detector::Detector(const std::string &name,
                   const std::filesystem::path &metafile_path)
    : name_{name}, metafile_path_{metafile_path} {}

}  // namespace icg
