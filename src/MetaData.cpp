#include "nuscenes2bag/MetaData.hpp"
#include <sstream>


template <> std::string to_debug_string(const SceneInfo& s) {
    std::ostringstream os;
    os << "{" << "token: " << s.token
      << ", sampleNumber: " << s.sampleNumber
      << ", sceneId: " << s.sceneId
      << ", name: " << s.name
      << ", description: " << s.description
    << "}";
    return os.str(); 
}

template <> std::string to_debug_string(const SampleInfo& o) {
    std::ostringstream os;
    os << SHOW_FIRST_MEMBER(token)  
       << SHOW_LAST_MEMBER(timeStamp);
    return os.str(); 
}
template <> std::string to_debug_string(const SampleDataInfo& o) {
    std::ostringstream os;
    os << "{" 
       << SHOW_FIRST_MEMBER(token)
       << SHOW_LAST_MEMBER(fileName);
    return os.str(); 
}