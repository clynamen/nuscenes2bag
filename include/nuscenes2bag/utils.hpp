#pragma once

#include "nuscenes2bag/SampleSetDescriptor.hpp"

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

template <class T> T uniq(T t) {
    sort(t.begin(), t.end());
    t.erase(unique(t.begin(), t.end()), t.end());
    return t;
}
