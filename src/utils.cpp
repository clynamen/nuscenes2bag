#include "nuscenes2bag/utils.hpp"

#include <iostream>

std::string
toLower(const std::string_view& str)
{
  std::string lowerStr;
  std::transform(
    str.begin(), str.end(), std::back_inserter(lowerStr), ::tolower);
  return lowerStr;
}

bool
string_icontains(const std::string_view& string, const std::string_view& sub)
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

std::string
topicNameForSampleType(const std::string dirName, const SampleType SampleType)
{
  switch (SampleType) {
    case SampleType::CAMERA:
      return topicNameForCamera(dirName);
      // default:
      //     //pass
  }
  return topicNameDefault(dirName);
}

std::string
topicNameDefault(const std::string& dirName)
{
  return std::string("/") + toLower(dirName);
}

std::string
topicNameForCamera(const std::string& dirName)
{
  std::string lowerDirName;
  std::transform(dirName.begin(),
                 dirName.end(),
                 std::back_inserter(lowerDirName),
                 ::tolower);
  return std::string("/") + lowerDirName + "/image";
}