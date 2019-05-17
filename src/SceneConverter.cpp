#include "nuscenes2bag/SceneConverter.hpp"
#include "nuscenes2bag/DatasetTypes.hpp"
#include "nuscenes2bag/utils.hpp"

#include "nuscenes2bag/EgoPoseConverter.hpp"
#include "nuscenes2bag/ImageDirectoryConverter.hpp"
#include "nuscenes2bag/LidarDirectoryConverter.hpp"
#include "nuscenes2bag/RadarDirectoryConverter.hpp"

#include <array>
#include <iostream>
#include <regex>
#include <string>

using namespace std;

SceneConverter::SceneConverter(const MetaDataProvider &metaDataProvider)
    : metaDataProvider(metaDataProvider) {}

std::optional<SampleType> getSampleType(const std::string_view filename) {
  std::array<std::pair<const char *, SampleType>, 3> pairs = {
      {{"CAM", SampleType::CAMERA},
       {"RADAR", SampleType::RADAR},
       {"LIDAR", SampleType::LIDAR}}};
  for (const auto &[str, SampleType] : pairs) {
    if (filename.find(str) != string::npos) {
      return std::optional(SampleType);
    }
  }
  cout << "Unknown file " << filename << endl;
  return std::nullopt;
}

template <typename T>
void writeMsg(const std::string_view topicName, const TimeStamp timeStamp,
              rosbag::Bag &outBag, std::optional<T> msgOpt) {
  if (msgOpt.has_value()) {
    auto &msg = msgOpt.value();
    msg.header.stamp = stampUs2RosTime(timeStamp);
    outBag.write(std::string(topicName).c_str(), msg.header.stamp, msg);
  }
}

static const std::regex TOPIC_REGEX = std::regex(".*__([A-Z_]+)__.*");

void SceneConverter::submit(const Token &sceneToken,
                            FileProgress &fileProgress) {
  std::optional sceneInfoOpt = metaDataProvider.getSceneInfo(sceneToken);
  // if(!sceneInfoOpt.has_value()) {
  //     // cout << "SceneInfo for " << sceneToken << " not found!" << endl;
  //     return;
  // }
  assert(sceneInfoOpt.has_value());
  SceneInfo &sceneInfo = sceneInfoOpt.value();

  sceneId = sceneInfo.sceneId;
  sampleDatas = metaDataProvider.getSceneSampleData(sceneInfo.token);
  egoPoseInfos = metaDataProvider.getEgoPoseInfo(sceneInfo.token);

  for (const auto &sampleData : sampleDatas) {
    // cout << to_debug_string(sampleData) << endl;
  }
  fileProgress.addToProcess(sampleDatas.size());
}

void SceneConverter::run(const std::filesystem::path &inPath,
                         const std::filesystem::path &outDirectoryPath,
                         FileProgress &fileProgress) {

  std::string bagName =
      outDirectoryPath.string() + "/" + std::to_string(sceneId) + ".bag";

  rosbag::Bag outBag;
  outBag.open(bagName, rosbag::bagmode::Write);

  convertEgoPoseInfos(outBag);
  convertSampleDatas(outBag, inPath, fileProgress);

  outBag.close();
}

void SceneConverter::convertSampleDatas(
    rosbag::Bag &outBag, const std::filesystem::path &inPath, FileProgress& fileProgress) {
  for (const auto &sampleData : sampleDatas) {
    std::filesystem::path sampleFilePath = inPath / sampleData.fileName;
    auto sampleTypeOpt = getSampleType(sampleFilePath.string());
    if (!sampleTypeOpt.has_value()) {
      continue;
    }
    auto sampleType = sampleTypeOpt.value();

    std::smatch m;
    auto filenameString = sampleFilePath.filename().string();
    auto matched = std::regex_search(filenameString, m, TOPIC_REGEX);
    std::string topicName = toLower(m.str(1));
    assert(!topicName.empty());

    if (sampleType == SampleType::CAMERA) {
      writeMsg(topicName, sampleData.timeStamp, outBag,
               readImageFile(sampleFilePath));
    } else if (sampleType == SampleType::LIDAR) {
      writeMsg(topicName, sampleData.timeStamp, outBag,
               readLidarFile(sampleFilePath));
    } else if (sampleType == SampleType::RADAR) {
      writeMsg(topicName, sampleData.timeStamp, outBag,
               readRadarFile(sampleFilePath));
    } else {
      cout << "Unknown sample type" << endl;
    }

    fileProgress.addToProcessed(1);
  }
}

geometry_msgs::TransformStamped makeIdentityTransform(ros::Time stamp, const char* frame_id, const char* child_frame_id) {
  geometry_msgs::TransformStamped msg;
  msg.header.frame_id = std::string(frame_id);
  msg.header.stamp = stamp;
  msg.child_frame_id = std::string(child_frame_id);
  msg.transform.rotation.w = 1;
  return msg;
}

void SceneConverter::convertEgoPoseInfos(rosbag::Bag &outBag) {
  const std::string odomTopic = "/odom";
  for (const auto &egoPose : egoPoseInfos) {
    nav_msgs::Odometry odomMsg = egoPoseInfo2OdometryMsg(egoPose);
    outBag.write(odomTopic.c_str(), odomMsg.header.stamp, odomMsg);
    

    geometry_msgs::TransformStamped tfMap2Odom = makeIdentityTransform(odomMsg.header.stamp, "map", "odom");
    geometry_msgs::TransformStamped tfOdom2Base = egoPoseInfo2TransformStamped(egoPose);
    tf::tfMessage tfMsg;
    tfMsg.transforms.push_back(tfMap2Odom);
    tfMsg.transforms.push_back(tfOdom2Base);
    outBag.write("/tf", odomMsg.header.stamp, tfMsg);
  }
}