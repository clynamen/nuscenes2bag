#include "nuscenes2bag/SampleSetDescriptor.hpp"

// SampleSetDescriptor::SampleSetDescriptor(
//     const std::string_view& directoryName, SampleSetType setType) 
//     : directoryName(directoryName),
//     setType(setType)
//     {

// }


// FileSystemSampleSet::FileSystemSampleSet(const SampleSetDescriptor& descriptor, 
//     const std::filesystem::path& directoryPath) 
//     : descriptor(descriptor), 
//     directoryPath(directoryPath) {

// }

template <> std::string to_debug_string(const SampleSetDescriptor& v) {
    std::ostringstream os;
    os << "{" << "sceneId: " << v.sceneId
      << ", directoryName: " << v.directoryName 
    //   << ", setType: " << setType 
    << "}";
    return os.str(); 
}


template <> std::string to_debug_string(const FileSystemSampleSet& v) {
    std::ostringstream os;
    os << "{" << "descriptor: " << to_debug_string(v.descriptor)
      << ", directoryPath: " << v.directoryPath << "}";
    return os.str(); 
}
