// SPDX-License-Identifier: MIT
// Copyright (c) 2022 Manuel Stoiber, German Aerospace Center (DLR)

#include <filesystem/filesystem.h>
#include <icg/generator.h>
#include <icg/tracker.h>

int main(int argc, char *argv[]) {
  if (argc != 2) {
    std::cerr << "Not enough arguments: Provide configfile_path";
    return 0;
  }
  const std::filesystem::path configfile_path{argv[1]};

  std::shared_ptr<icg::Tracker> tracker_ptr;
  if (!GenerateConfiguredTracker(configfile_path, &tracker_ptr)) return 0;

  if (!tracker_ptr->SetUp()) return 0;
  if (!tracker_ptr->RunTrackerProcess(false, false)) return 0;
  return 0;
}
