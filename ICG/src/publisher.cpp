// SPDX-License-Identifier: MIT
// Copyright (c) 2022 Manuel Stoiber, German Aerospace Center (DLR)

#include <icg/publisher.h>

namespace icg {

void Publisher::set_name(const std::string &name) { name_ = name; }

void Publisher::set_metafile_path(const std::filesystem::path &metafile_path) {
  metafile_path_ = metafile_path;
  set_up_ = false;
}

const std::string &Publisher::name() const { return name_; }

const std::filesystem::path &Publisher::metafile_path() const {
  return metafile_path_;
}

bool Publisher::set_up() const { return set_up_; }

Publisher::Publisher(const std::string &name) : name_{name} {}

Publisher::Publisher(const std::string &name,
                     const std::filesystem::path &metafile_path)
    : name_{name}, metafile_path_{metafile_path} {}

}  // namespace icg
