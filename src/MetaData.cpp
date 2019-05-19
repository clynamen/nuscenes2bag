#include "nuscenes2bag/MetaDataTypes.hpp"
#include <sstream>

template<>
std::string
to_debug_string(const SceneInfo& o)
{
  std::ostringstream os;
  os << SHOW_FIRST_MEMBER(token) << SHOW_MEMBER(sampleNumber)
     << SHOW_MEMBER(sceneId) << SHOW_MEMBER(name)
     << SHOW_LAST_MEMBER(description);
  return os.str();
}

template<>
std::string
to_debug_string(const SampleInfo& o)
{
  std::ostringstream os;
  os << SHOW_FIRST_MEMBER(token) << SHOW_LAST_MEMBER(timeStamp);
  return os.str();
}
template<>
std::string
to_debug_string(const SampleDataInfo& o)
{
  std::ostringstream os;
  os << "{" << SHOW_FIRST_MEMBER(token) << SHOW_LAST_MEMBER(fileName);
  return os.str();
}