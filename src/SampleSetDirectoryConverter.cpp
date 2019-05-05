#include "nuscenes2bag/SampleSetDirectoryConverter.hpp"

#include <iostream>

namespace fs = std::filesystem;

SampleSetDirectoryConverter::SampleSetDirectoryConverter(
    const std::filesystem::path &path)
    : directoryPath(path), running(false) {}

SampleSetDirectoryConverter::~SampleSetDirectoryConverter() {}

void SampleSetDirectoryConverter::submit() {
  running = true;
  submitInternal();
}

void SampleSetDirectoryConverter::process() {
  running = true;
  processInternal();
}

bool SampleSetDirectoryConverter::isRunning() { return running; }

void SampleSetDirectoryConverter::stop() { running = false; }

const std::regex SINGLE_SAMPLE_FILENAME_REGEX("^n(\\d+)-.*__(\\d+)\\..*");

std::optional<ExtractedFileNameInfo> getInfoFromFilename(const std::string &fname) {
  int sceneId;
  uint64_t stampUs;

  std::smatch m;
  auto matched = std::regex_search(fname, m, SINGLE_SAMPLE_FILENAME_REGEX);
  if (matched) {
    try {
      sceneId = std::stoi(m[1]);
      stampUs = std::stoull(m[2]);
      return std::optional<ExtractedFileNameInfo>{{sceneId, stampUs}};
    } catch (const std::exception &e) {
      std::cout << "Unable to parse " << fname << std::endl;
    }
  }

  return std::nullopt;
}

std::vector<std::pair<fs::path, ExtractedFileNameInfo>>
getFilesInSampleSetDirectory(const std::filesystem::path &directoryPath) {
  std::vector<std::pair<fs::path, ExtractedFileNameInfo>> files;

  for (const auto &entry : fs::directory_iterator(directoryPath)) {
    if (entry.is_regular_file()) {
      const auto &fname = entry.path().filename();
      auto fileInfoOpt = getInfoFromFilename(fname);
      if (fileInfoOpt.has_value()) {
        auto fileInfo = fileInfoOpt.value();
        // std::cout << "Processing image " << fileInfo.sceneId << "  " <<
        // fileInfo.stampUs << std::endl;
        files.emplace_back(entry.path(), fileInfo);
      } else {
        std::cout << "Skipping " << fname << std::endl;
      }
    }
  }

  return files;
}

std::vector<std::pair<fs::path, ExtractedFileNameInfo>>
getFilesInSampleSetDirectoryOrderedByTime(const std::filesystem::path &directoryPath) {
  auto files = getFilesInSampleSetDirectory(directoryPath);

  std::sort(files.begin(), files.end(), [](const auto &l, const auto &r) {
    return l.second.stampUs < r.second.stampUs;
  });
  return files;
}