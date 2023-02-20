// SPDX-License-Identifier: MIT
// Copyright (c) 2023 Manuel Stoiber, German Aerospace Center (DLR)

#include <m3t/publisher.h>

namespace m3t {

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

}  // namespace m3t
