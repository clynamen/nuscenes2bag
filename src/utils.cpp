#include "nuscenes2bag/utils.hpp"

#include <iostream>

namespace nuscenes2bag {

std::string
toLower(const std::string& str)
{
  std::string lowerStr(str);
  std::transform(lowerStr.begin(),
                 lowerStr.end(),
                 lowerStr.begin(),
                 [](unsigned char c) { return std::tolower(c); });
  return lowerStr;
}

bool
string_icontains(const std::string& string, const std::string& sub)
{
  std::string lowerString = toLower(string);
  std::string lowerSub = toLower(sub);
  return lowerString.find(lowerSub) != std::string::npos;
}

ros::Time
stampUs2RosTime(uint64_t stampUs)
{
  ros::Time t;
  t = t.fromNSec(stampUs * 1000);
  return t;
}

}