#include <string>
#include "ros/ros.h"

bool string_icontains(const std::string_view &string,
                      const std::string_view &sub);

ros::Time stampUs2RosTime(uint64_t stampUs);


std::string topicNameForCamera(const std::string dirName);