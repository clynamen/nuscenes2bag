#pragma once

#include "sensor_msgs/PointCloud2.h"
#include <pcl_ros/point_cloud.h>
#include "nuscenes2bag/Filesystem.hpp"

namespace nuscenes2bag {

boost::optional<sensor_msgs::PointCloud2> readLidarFileXYZIR(const fs::path& filePath);

}
