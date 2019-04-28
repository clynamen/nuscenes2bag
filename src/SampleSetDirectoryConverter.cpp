#include "nuscenes2rosbag/SampleSetDirectoryConverter.hpp"

SampleSetDirectoryConverter::SampleSetDirectoryConverter(
    const std::filesystem::path &path)
    : directoryPath(path), running(false) {}

SampleSetDirectoryConverter::~SampleSetDirectoryConverter() {}

void SampleSetDirectoryConverter::process() {
  running = true;
  processInternal();
}

bool SampleSetDirectoryConverter::isRunning() { return running; }

void SampleSetDirectoryConverter::stop() { running = false; }