#pragma once

#include "sensor_msgs/PointCloud2.h"
#include <pcl_ros/point_cloud.h>

#if CMAKE_CXX_STANDARD >= 17
#include <filesystem>
namespace fs = std::filesystem;
#else
#include <boost/filesystem.hpp>
namespace fs = boost::filesystem;
#endif

namespace nuscenes2bag {

#if CMAKE_CXX_STANDARD >= 17
std::optional<sensor_msgs::PointCloud2> readLidarFileXYZIR(const fs::path& filePath);
#else
sensor_msgs::PointCloud2Ptr readLidarFileXYZIR(const fs::path& filePath);
#endif

}
