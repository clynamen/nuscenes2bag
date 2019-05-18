#include "nuscenes2bag/utils.hpp"
#include <nuscenes2bag/MetaDataReader.hpp>

#include <algorithm>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <map>
#include <regex>

using namespace std;
namespace fs = std::filesystem;
namespace json = nlohmann;

void MetaDataReader::loadFromDirectory(const fs::path &directoryPath) {
  const fs::path sceneFile = directoryPath / "scene.json";
  const fs::path sampleFile = directoryPath / "sample.json";
  const fs::path sampleDataFile = directoryPath / "sample_data.json";
  const fs::path egoPoseFile = directoryPath / "ego_pose.json";
  const fs::path calibratedSensorFile =
      directoryPath / "calibrated_sensor.json";
  const fs::path sensorFile =
      directoryPath / "sensor.json";

  scenes = loadScenesFromFile(sceneFile);
  scene2Samples = loadSampleInfos(sampleFile);
  sample2SampleData = loadSampleDataInfos(sampleDataFile);
  calibratedSensorToken2CalibratedSensorInfo =
      loadCalibratedSensorInfo(calibratedSensorFile);
  sensorToken2CalibratedSensorName = loadCalibratedSensorNames(sensorFile);

  // build inverse (EgoPose.token -> Scene.token) map
  // and (scene.token -> calibratedSensor[]) map
  std::map<Token, Token> egoPoseToken2sceneToken;

  for (const auto &[sceneToken, sampleInfos] : scene2Samples) {
    for (const auto &sampleInfo : sampleInfos) {
      for (const auto &sampleData : sample2SampleData[sampleInfo.token]) {
        // add egoPoseInfo
        egoPoseToken2sceneToken.emplace(sampleData.egoPoseToken, sceneToken);

        auto &calibratedSensorInfoSet =
            getExistingOrDefault(scene2CalibratedSensorInfo, sceneToken);
        const auto &calibratedSensorInfo =
            calibratedSensorToken2CalibratedSensorInfo
                .find(sampleData.calibratedSensorToken)
                ->second;
        const auto &calibratedSensorName =
            sensorToken2CalibratedSensorName
                .find(calibratedSensorInfo.sensorToken)
                ->second;
        calibratedSensorInfoSet.insert(CalibratedSensorInfoAndName{
          calibratedSensorInfo,
          calibratedSensorName
        });
      }
    }
  }

  scene2EgoPose = loadEgoPoseInfos(egoPoseFile, egoPoseToken2sceneToken);

  loadFromDirectoryCalled = true;
}

json::json
MetaDataReader::slurpJsonFile(const std::filesystem::path &filePath) {
  std::ifstream file(filePath.string());
  if (!file.is_open()) {
    std::string errMsg = string("Unable to open ") + filePath.string();
    throw std::runtime_error(errMsg);
  }
  json::json newJson;
  file >> newJson;
  return newJson;
}

std::vector<SceneInfo>
MetaDataReader::loadScenesFromFile(const fs::path &filePath) {
  auto sceneJsons = slurpJsonFile(filePath);
  std::vector<SceneInfo> sceneInfos;

  std::regex sceneIdRegex("scene-(\\d+)");

  for (const auto &sceneJson : sceneJsons) {
    std::string sceneIdStr = sceneJson["name"];
    std::smatch match;
    std::regex_search(sceneIdStr, match, sceneIdRegex);
    SceneId sceneId = std::stoi(match.str(1));
    sceneInfos.push_back(SceneInfo{
        sceneJson["token"],
        sceneJson["nbr_samples"],
        sceneId,
        sceneJson["name"],
        sceneJson["description"],
        sceneJson["first_sample_token"],
    });
  }

  return sceneInfos;
}

std::map<Token, std::vector<SampleInfo>>
MetaDataReader::loadSampleInfos(const std::filesystem::path &filePath) {
  auto sampleInfos = slurpJsonFile(filePath);
  std::map<Token, std::vector<SampleInfo>> token2Samples;

  for (const auto &sampleInfo : sampleInfos) {
    Token sampleToken = sampleInfo["token"];
    Token sceneToken = sampleInfo["scene_token"];
    std::vector<SampleInfo> &samples =
        getExistingOrDefault(token2Samples, sceneToken);
    samples.push_back(
        SampleInfo{sceneToken, sampleToken, sampleInfo["timestamp"]});
  }

  return token2Samples;
}

std::map<Token, std::vector<SampleDataInfo>>
MetaDataReader::loadSampleDataInfos(const std::filesystem::path &filePath) {
  auto sampleDataJsons = slurpJsonFile(filePath);
  std::map<Token, std::vector<SampleDataInfo>> sample2SampleData;

  for (const auto &sampleDataJson : sampleDataJsons) {
    Token sampleToken = sampleDataJson["sample_token"];
    Token sampleDataToken = sampleDataJson["token"];
    std::vector<SampleDataInfo> &sampleDatas =
        getExistingOrDefault(sample2SampleData, sampleToken);
    sampleDatas.push_back(SampleDataInfo{
        sampleDataToken,
        sampleDataJson["timestamp"],
        sampleDataJson["ego_pose_token"],
        sampleDataJson["calibrated_sensor_token"],
        sampleDataJson["fileformat"],
        sampleDataJson["is_key_frame"],
        sampleDataJson["filename"],
    });
  }

  return sample2SampleData;
}

EgoPoseInfo egoPoseJson2EgoPoseInfo(const json::json &egoPoseJson) {
  EgoPoseInfo egoPoseInfo;

  egoPoseInfo.translation[0] = egoPoseJson["translation"][0];
  egoPoseInfo.translation[1] = egoPoseJson["translation"][1];
  egoPoseInfo.translation[2] = egoPoseJson["translation"][2];

  egoPoseInfo.rotation[0] = egoPoseJson["rotation"][0];
  egoPoseInfo.rotation[1] = egoPoseJson["rotation"][1];
  egoPoseInfo.rotation[2] = egoPoseJson["rotation"][2];
  egoPoseInfo.rotation[3] = egoPoseJson["rotation"][3];

  egoPoseInfo.timeStamp = egoPoseJson["timestamp"];

  return egoPoseInfo;
}

std::map<Token, std::vector<EgoPoseInfo>> MetaDataReader::loadEgoPoseInfos(
    const std::filesystem::path &filePath,
    std::map<Token, Token> sampleDataToken2SceneToken) {

  auto egoPoseJsons = slurpJsonFile(filePath);
  std::map<Token, std::vector<EgoPoseInfo>> sceneToken2EgoPoseInfos;

  for (const auto &egoPoseJson : egoPoseJsons) {
    Token sampleDataToken = egoPoseJson["token"];
    const auto &sceneTokenIt = sampleDataToken2SceneToken.find(sampleDataToken);
    if (sceneTokenIt == sampleDataToken2SceneToken.end()) {
      cout << "unable to find " << sampleDataToken << " sample token" << endl;
    } else {
      const Token sceneToken = sceneTokenIt->second;
      std::vector<EgoPoseInfo> &egoPoses =
          getExistingOrDefault(sceneToken2EgoPoseInfos, sceneToken);

      EgoPoseInfo egoPoseInfo = egoPoseJson2EgoPoseInfo(egoPoseJson);
      egoPoses.push_back(egoPoseInfo);
    }
  }

  return sceneToken2EgoPoseInfos;
}

std::map<Token, CalibratedSensorInfo> MetaDataReader::loadCalibratedSensorInfo(
    const std::filesystem::path &filePath) {
  auto calibratedSensorJsons = slurpJsonFile(filePath);
  std::map<Token, CalibratedSensorInfo>
      calibratedSensorToken2CalibratedSensorInfo;

  for (const auto &calibratedSensorJson : calibratedSensorJsons) {
    Token token = calibratedSensorJson["token"];
    auto translation = calibratedSensorJson["translation"];
    auto rotation = calibratedSensorJson["rotation"];
    CalibratedSensorInfo calibratedSensorInfo{
        token,
        calibratedSensorJson["sensor_token"],
        {translation[0], translation[1], translation[2]},
        {rotation[0], rotation[1], rotation[2], rotation[3]},
    };
    std::optional<json::json> sensor_intrinsics =
        calibratedSensorJson["rotation"];

    calibratedSensorToken2CalibratedSensorInfo.emplace(token,
                                                       calibratedSensorInfo);
  }

  return calibratedSensorToken2CalibratedSensorInfo;
}

std::map<Token, CalibratedSensorName> MetaDataReader::loadCalibratedSensorNames(
    const std::filesystem::path &filePath) {
  auto calibratedSensorNameJsons = slurpJsonFile(filePath);
  std::map<Token, CalibratedSensorName> sensorToken2CalibratedSensorName;

  for (const auto &calibratedSensorNameJson : calibratedSensorNameJsons) {
    sensorToken2CalibratedSensorName.emplace(calibratedSensorNameJson["token"], 
      CalibratedSensorName{
      calibratedSensorNameJson["token"], calibratedSensorNameJson["channel"],
          calibratedSensorNameJson["modality"]});
  };

  return sensorToken2CalibratedSensorName;
}

std::vector<Token> MetaDataReader::getAllSceneTokens() const {
  assert(loadFromDirectoryCalled);
  std::vector<Token> tokens;
  std::transform(scenes.begin(), scenes.end(), std::back_inserter(tokens),
                 [](const SceneInfo &sceneInfo) { return sceneInfo.token; });
  return tokens;
}

std::optional<SceneInfo>
MetaDataReader::getSceneInfo(const Token &sceneToken) const {
  assert(loadFromDirectoryCalled);
  auto it = std::find_if(scenes.begin(), scenes.end(),
                         [&sceneToken](const SceneInfo &sceneInfo) {
                           return sceneInfo.token == sceneToken;
                         });
  if (it == scenes.end()) {
    return std::nullopt;
  }

  return std::optional(*it);
}

std::vector<SampleDataInfo>
MetaDataReader::getSceneSampleData(const Token &sceneToken) const {
  std::vector<SampleDataInfo> sampleDataInfos;

  const auto sceneSamplesIt = scene2Samples.find(sceneToken);
  if (sceneSamplesIt == scene2Samples.end()) {
    assert(false);
    return sampleDataInfos;
  }
  const auto &sceneSamples = sceneSamplesIt->second;
  for (const auto &sceneSample : sceneSamples) {
    const Token &sceneSampleToken = sceneSample.token;
    const auto sceneSampleDatasIt = sample2SampleData.find(sceneSampleToken);
    if (sceneSampleDatasIt == sample2SampleData.end()) {
      return sampleDataInfos;
      assert(false);
    }

    const auto &sceneSampleDatas = sceneSampleDatasIt->second;
    for (const SampleDataInfo &sampleData : sceneSampleDatas) {
      sampleDataInfos.push_back(sampleData);
    }
  }

  return sampleDataInfos;
};

std::vector<EgoPoseInfo>
MetaDataReader::getEgoPoseInfo(const Token &sceneToken) const {
  return scene2EgoPose.find(sceneToken)->second;
}

CalibratedSensorInfo MetaDataReader::getCalibratedSensorInfo(
    const Token &calibratedSensorToken) const {
  return calibratedSensorToken2CalibratedSensorInfo.find(calibratedSensorToken)
      ->second;
}

CalibratedSensorName
MetaDataReader::getSensorName(const Token &sensorToken) const {
  return sensorToken2CalibratedSensorName.find(sensorToken)->second;
}

std::vector<CalibratedSensorInfoAndName>
MetaDataReader::getSceneCalibratedSensorInfo(const Token &sceneToken) const {
  std::vector<CalibratedSensorInfoAndName> sceneCalibratedSensorInfo;
  const auto &sceneCalibratedSensorInfoSet =
      scene2CalibratedSensorInfo.find(sceneToken)->second;
  std::copy(sceneCalibratedSensorInfoSet.begin(),
            sceneCalibratedSensorInfoSet.end(),
            std::back_inserter(sceneCalibratedSensorInfo));
  return sceneCalibratedSensorInfo;
}