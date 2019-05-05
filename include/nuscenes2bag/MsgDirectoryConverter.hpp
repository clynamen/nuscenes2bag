#pragma once

#include "nuscenes2bag/SampleQueue.hpp"
#include "nuscenes2bag/SampleSetDirectoryConverter.hpp"
#include "nuscenes2bag/utils.hpp"
#include "nuscenes2bag/FileProgress.hpp"

#include <optional>

template <class T>
std::optional<T> processSingleFileFun(
    const std::pair<std::filesystem::path, ExtractedFileNameInfo> &fileInfo);


template <class T>
class MsgDirectoryConverter : public SampleSetDirectoryConverter {
public:
  MsgDirectoryConverter(SampleQueueProducer<T> &&queue,
                        FileProgress& fileProgress,
                        const std::filesystem::path &path) :  
                        SampleSetDirectoryConverter(path), 
                        fileProgress(fileProgress),
                        queue(queue) {}

  void submitInternal() override {
    files = getFilesInSampleSetDirectoryOrderedByTime(directoryPath);
    // std::cout << "Adding " << files.size() << " to process " << std::endl;
    fileProgress.addToProcess(files.size());
  }

  void processInternal() override {
    for (const auto &file : files) {
      if (!isRunning()) {
        break;
      }

      std::optional<T> msg = processSingleFile(file);

      while (true) {
        if (queue.canPush()) {
          if(msg.has_value()) {
            queue.push(std::move(msg.value()));
          }
          fileProgress.addToProcessed(1);
          break;
        } else if (!isRunning()) {
          break;
        } else {
          std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
      }
    }

    queue.close();
  }

  std::optional<T> processSingleFile(
      const std::pair<std::filesystem::path, ExtractedFileNameInfo> &fileInfo) {
    return processSingleFileFun<T>(fileInfo);
  }

  FileProgress& fileProgress;
  SampleQueueProducer<T> queue;
  std::vector<std::pair<std::filesystem::path, ExtractedFileNameInfo>> files;
};