#include "nuscenes2bag/utils.hpp"

std::string toLower(const std::string_view& str) {
  std::string lowerStr;
  std::transform(str.begin(), str.end(), std::back_inserter(lowerStr),
                 ::tolower);
  return lowerStr;
}

bool string_icontains(const std::string_view &string,
                      const std::string_view &sub) {
  std::string lowerString;
  std::string lowerSub;
  std::transform(string.begin(), string.end(), std::back_inserter(lowerString),
                 ::tolower);
  std::transform(sub.begin(), sub.end(), std::back_inserter(lowerSub),
                 ::tolower);
  return lowerString.find(lowerSub) != std::string::npos;
}

ros::Time stampUs2RosTime(uint64_t stampUs) {
  ros::Time t;
  t = t.fromNSec(stampUs * 1000);
  return t;
}

std::string topicNameForSampleSetType(const std::string dirName, 
    const SampleSetType sampleSetType) {
  switch(sampleSetType) {
    case SampleSetType::CAMERA:          
        return topicNameForCamera(dirName);
    // default:
    //     //pass
  }
  return topicNameDefault(dirName);
}

std::string topicNameDefault(const std::string& dirName) {
    return  std::string("/") + toLower(dirName);
}

std::string topicNameForCamera(const std::string& dirName) {
  std::string lowerDirName;
  std::transform(dirName.begin(), dirName.end(), std::back_inserter(lowerDirName),
                 ::tolower);
  return std::string("/") + lowerDirName + "/image";
}