#pragma once

#include <filesystem>

enum class SampleSetType {
    CAMERA,
    RADAR,
    LIDAR
};

struct SampleSetDescriptor {
    // SampleSetDescriptor() = default;
    // SampleSetDescriptor(const SampleSetDescriptor& o) = default;
    // SampleSetDescriptor(const std::string_view& directoryName, SampleSetType setType);

    std::string directoryName;
    SampleSetType setType;
};

struct FileSystemSampleSet {
    // FileSystemSampleSet() = default;
    // FileSystemSampleSet(const FileSystemSampleSet& o) = default;
    // FileSystemSampleSet(const SampleSetDescriptor& descriptor, 
    //     const std::filesystem::path& directoryPath);

    SampleSetDescriptor descriptor;
    std::filesystem::path directoryPath;
};