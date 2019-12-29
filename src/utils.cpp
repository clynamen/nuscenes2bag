#include "nuscenes2bag/utils.hpp"

#include <iostream>

namespace nuscenes2bag {

#if CMAKE_CXX_STANDARD >= 17

std::string
toLower(const std::string_view& str)
{
  std::string lowerStr;
  std::transform(
    str.begin(), str.end(), std::back_inserter(lowerStr), ::tolower);
  return lowerStr;
}

#else

std::string
toLower(const std::string& str)
{
  std::string lowerStr(str);
  std::transform(lowerStr.begin(), lowerStr.end(), lowerStr.begin(),
    [](unsigned char c)
  {
    return std::tolower(c);
  });
  return lowerStr;
}

#endif

#if CMAKE_CXX_STANDARD >= 17
bool
string_icontains(const std::string_view& string, const std::string_view& sub)
#else
bool string_icontains(const std::string& string, const std::string& sub)
#endif
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