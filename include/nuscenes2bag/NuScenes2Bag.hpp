#pragma once

#include <rosbag/bag.h>

#include "nuscenes2bag/DatasetTypes.hpp"

#if CMAKE_CXX_STANDARD >= 17
#include <optional>
#include <filesystem>
namespace fs = std::filesystem;
#else
#include <boost/filesystem.hpp>
namespace fs = boost::filesystem;
#endif

namespace nuscenes2bag {

struct NuScenes2Bag {

public:
  NuScenes2Bag();

  void convertDirectory(const fs::path &inDatasetPath,
                        const fs::path &outputRosbagPath,
                        int32_t threadNumber,
#if CMAKE_CXX_STANDARD >= 17
                        std::optional<int32_t> sceneNumberOpt
#else
                        int32_t sceneNumberOpt
#endif
                        );

private:
  std::string inDatasetPathString;
};

}