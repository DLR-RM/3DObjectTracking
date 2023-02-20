// SPDX-License-Identifier: MIT
// Copyright (c) 2023 Manuel Stoiber, German Aerospace Center (DLR)

#ifndef M3T_INCLUDE_M3T_SUBSCRIBER_H_
#define M3T_INCLUDE_M3T_SUBSCRIBER_H_

#include <m3t/common.h>

namespace m3t {

/**
 * \brief Abstract class that defines a subscriber that can be used to
 * subscribe to any data source using the `UpdateSubscriber` method.
 */
class Subscriber {
 public:
  // Setup method
  virtual bool SetUp() = 0;

  // Setters
  void set_name(const std::string &name);
  void set_metafile_path(const std::filesystem::path &metafile_path);

  // Main methods
  virtual bool UpdateSubscriber(int iteration) = 0;

  // Getters
  const std::string &name() const;
  const std::filesystem::path &metafile_path() const;
  bool set_up() const;

 protected:
  // Constructor
  Subscriber(const std::string &name);
  Subscriber(const std::string &name,
            const std::filesystem::path &metafile_path);

  // Variables
  std::string name_{};
  std::filesystem::path metafile_path_{};
  bool set_up_ = false;
};

}  // namespace m3t

#endif  // M3T_INCLUDE_M3T_SUBSCRIBER_H_
