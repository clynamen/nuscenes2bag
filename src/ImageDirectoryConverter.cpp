#include "nuscenes2rosbag/ImageDirectoryConverter.hpp"
#include "nuscenes2rosbag/utils.hpp"
#include <thread>

namespace fs = std::filesystem;

ImageDirectoryConverter::ImageDirectoryConverter(
    SampleQueueProducer<sensor_msgs::Image> &&queue,
    const std::filesystem::path &path)
    : SampleSetDirectoryConverter(path), queue(std::move(queue)) {}


std::optional<ExtractedFileNameInfoImage>
getInfoFromFilename(const std::string &fname) {
  int sceneId;
  uint64_t stampUs;

  std::smatch m;
  auto matched = std::regex_search(fname, m, IMAGE_REGEX);
  if (matched) {
    try {
      sceneId = std::stoi(m[1]);
      stampUs = std::stoull(m[2]);
      return std::optional<ExtractedFileNameInfoImage>{{sceneId, stampUs}};
    } catch (const std::exception &e) {
      std::cout << "Unable to parse " << fname << std::endl;
    }
  }

  return std::nullopt;
}

void ImageDirectoryConverter::processInternal() {
  std::vector<std::pair<fs::path, ExtractedFileNameInfoImage>> imageFiles;

  for (const auto &entry : fs::directory_iterator(directoryPath)) {
    if (!isRunning()) {
      break;
    }
    if (entry.is_regular_file()) {
      const auto &fname = entry.path().filename();
      auto fileInfoOpt = getInfoFromFilename(fname);
      if (fileInfoOpt.has_value()) {
        auto fileInfo = fileInfoOpt.value();
        // std::cout << "Processing image " << fileInfo.sceneId << "  " <<
        // fileInfo.stampUs << std::endl;
        imageFiles.emplace_back(entry.path(), fileInfo);
      } else {
        std::cout << "Skipping " << fname << std::endl;
      }
    }
  }

  std::sort(imageFiles.begin(), imageFiles.end(),
            [](const auto &l, const auto &r) {
              return l.second.stampUs < r.second.stampUs;
            });

  for (const auto &imageFile : imageFiles) {
    if (!isRunning()) {
      break;
    }
    cv::Mat image;
    image = imread(imageFile.first.string().c_str(), cv::IMREAD_COLOR);
    sensor_msgs::ImagePtr msg =
        cv_bridge::CvImage(std_msgs::Header(), "bgr8", image).toImageMsg();
    msg->header.stamp = stampUs2RosTime(imageFile.second.stampUs);
    sensor_msgs::Image msgCopy = *msg;

    while (true) {
      if (queue.canPush()) {
        queue.push(std::move(msgCopy));
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