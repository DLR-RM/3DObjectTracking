// SPDX-License-Identifier: MIT
// Copyright (c) 2023 Manuel Stoiber, German Aerospace Center (DLR)

#include <m3t/subscriber.h>

namespace m3t {

void Subscriber::set_name(const std::string &name) { name_ = name; }

void Subscriber::set_metafile_path(const std::filesystem::path &metafile_path) {
  metafile_path_ = metafile_path;
  set_up_ = false;
}

const std::string &Subscriber::name() const { return name_; }

const std::filesystem::path &Subscriber::metafile_path() const {
  return metafile_path_;
}

bool Subscriber::set_up() const { return set_up_; }

Subscriber::Subscriber(const std::string &name) : name_{name} {}

Subscriber::Subscriber(const std::string &name,
                     const std::filesystem::path &metafile_path)
    : name_{name}, metafile_path_{metafile_path} {}

}  // namespace m3t
