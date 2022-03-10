// SPDX-License-Identifier: MIT
// Copyright (c) 2022 Manuel Stoiber, German Aerospace Center (DLR)

#ifndef ICG_INCLUDE_ICG_PUBLISHER_H_
#define ICG_INCLUDE_ICG_PUBLISHER_H_

#include <icg/common.h>

namespace icg {

/**
 * \brief Abstract class that defines a publisher that can be used to
 * publish any data.
 */
class Publisher {
 public:
  // Setup method
  virtual bool SetUp() = 0;

  // Setters
  void set_name(const std::string &name);
  void set_metafile_path(const std::filesystem::path &metafile_path);

  // Main methods
  virtual bool UpdatePublisher(int iteration) = 0;

  // Getters
  const std::string &name() const;
  const std::filesystem::path &metafile_path() const;
  bool set_up() const;

 protected:
  // Constructor
  Publisher(const std::string &name);
  Publisher(const std::string &name,
            const std::filesystem::path &metafile_path);

  // Variables
  std::string name_{};
  std::filesystem::path metafile_path_{};
  bool set_up_ = false;
};

}  // namespace icg

#endif  // ICG_INCLUDE_ICG_PUBLISHER_H_
