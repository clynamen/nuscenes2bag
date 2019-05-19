#pragma once

#include <rosbag/bag.h>

#include "nuscenes2bag/DatasetTypes.hpp"

#include <filesystem>
#include <optional>
#include <vector>

namespace nuscenes2bag {

struct NuScenes2Bag {

public:
  NuScenes2Bag();

  void convertDirectory(const std::filesystem::path &inDatasetPath,
                        const std::filesystem::path &outputRosbagPath, 
                        int32_t threadNumber);

private:
  std::string inDatasetPathString;
};

}