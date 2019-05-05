#pragma once

#include <filesystem>
#include <optional>
#include <thread>
#include <regex>

struct ExtractedFileNameInfo {
  int sceneId;
  uint64_t stampUs;
};

extern const std::regex SINGLE_SAMPLE_FILENAME_REGEX;

class SampleSetDirectoryConverter {
public:
  SampleSetDirectoryConverter(const std::filesystem::path &path);
  virtual ~SampleSetDirectoryConverter();

  void submit();
  void process();

  bool isRunning();
  void stop();

protected:
  virtual void submitInternal() = 0;
  virtual void processInternal() = 0;

  std::optional<ExtractedFileNameInfo>
  getInfoFromFilename(const std::string &fname);

  std::filesystem::path directoryPath;

private:
  bool running;
};

std::optional<ExtractedFileNameInfo>
getInfoFromFilename(const std::string &fname);

std::vector<std::pair<std::filesystem::path, ExtractedFileNameInfo>>
getFilesInSampleSetDirectory(const std::filesystem::path &directoryPath);

std::vector<std::pair<std::filesystem::path, ExtractedFileNameInfo>>
getFilesInSampleSetDirectoryOrderedByTime(const std::filesystem::path &directoryPath);