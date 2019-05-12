#pragma once

#include <filesystem>
#include <sstream>
#include <string>
#include <iostream>

#include "nuscenes2bag/ToDebugString.hpp"

enum class SampleSetType {
    CAMERA,
    RADAR,
    LIDAR
};

typedef uint32_t SceneId;

struct SampleSetDescriptor {
    SceneId sceneId;
    std::string directoryName;
    SampleSetType setType;
};

template <> std::string to_debug_string(const SampleSetDescriptor& v);

struct FileSystemSampleSet {
    SampleSetDescriptor descriptor;
    std::filesystem::path directoryPath;
};

template <> std::string to_debug_string(const FileSystemSampleSet& v);