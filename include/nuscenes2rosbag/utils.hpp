#include "nuscenes2rosbag/SampleSetDescriptor.hpp"

#include <string>
#include "ros/ros.h"

std::string toLower(const std::string_view& str);

bool string_icontains(const std::string_view &string,
                      const std::string_view &sub);

ros::Time stampUs2RosTime(uint64_t stampUs);

std::string topicNameForSampleSetType(const std::string dirName, 
    const SampleSetType sampleSetType);

std::string topicNameDefault(const std::string& dirName);

std::string topicNameForCamera(const std::string& dirName);
