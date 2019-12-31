#pragma once

#include "nuscenes2bag/Filesystem.hpp"
#include "sensor_msgs/PointCloud2.h"
#include <pcl_ros/point_cloud.h>

namespace nuscenes2bag {

boost::optional<sensor_msgs::PointCloud2> readLidarFile(const fs::path& filePath);

}
