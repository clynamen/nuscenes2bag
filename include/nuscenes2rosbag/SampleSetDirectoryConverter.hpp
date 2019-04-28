#pragma once

#include <filesystem>

class SampleSetDirectoryConverter {
public:
  SampleSetDirectoryConverter(const std::filesystem::path &path);
  virtual ~SampleSetDirectoryConverter();

  void process();

  bool isRunning();
  void stop();

protected:
  virtual void processInternal() = 0;

  std::filesystem::path directoryPath;

private:
  bool running;
};