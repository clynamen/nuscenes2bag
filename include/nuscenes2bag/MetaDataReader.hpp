#include <iostream>
#include <map>
#include <set>

#include <nlohmann/json.hpp>

#include "nuscenes2bag/MetaDataTypes.hpp"
#include "nuscenes2bag/Filesystem.hpp"
#include "nuscenes2bag/MetaDataProvider.hpp"
#include "nuscenes2bag/ToDebugString.hpp"

#include <boost/optional.hpp>

namespace nuscenes2bag {

class InvalidMetaDataException : public std::exception
{
private:
  std::string msg;

public:
  InvalidMetaDataException(const std::string& msg)
    : msg(msg)
  {}
  ~InvalidMetaDataException() throw(){}
  const char* what() const throw() { return this->msg.c_str(); }
};

class MetaDataReader : public MetaDataProvider {
public:
  void loadFromDirectory(const fs::path &directoryPath);


  std::vector<Token> getAllSceneTokens() const override;

  boost::optional<SceneInfo> getSceneInfo(const Token &sceneToken) const override;

  std::vector<SampleDataInfo>
  getSceneSampleData(const Token &sceneToken) const override;
  std::vector<EgoPoseInfo>
  getEgoPoseInfo(const Token &sceneToken) const override;
  CalibratedSensorInfo
  getCalibratedSensorInfo(const Token &calibratedSensorToken) const override;
  std::vector<CalibratedSensorInfoAndName>
  getSceneCalibratedSensorInfo(const Token &sceneToken) const override;
  CalibratedSensorName
  getSensorName(const Token &sensorToken) const override;

  boost::optional<SceneInfo>
  getSceneInfoByNumber(const uint32_t sceneNumber) const override;

private:
  static nlohmann::json slurpJsonFile(const fs::path &filePath);
  static std::vector<SceneInfo>
  loadScenesFromFile(const fs::path &filePath);
  static std::map<Token, std::vector<SampleInfo>>
  loadSampleInfos(const fs::path &filePath);
  static std::map<Token, std::vector<SampleDataInfo>>
  loadSampleDataInfos(const fs::path &filePath);
  static std::map<Token, std::vector<EgoPoseInfo>> loadEgoPoseInfos(
      const fs::path &filePath,
      std::map<Token, Token> sample2SampleData);
  static std::map<Token, CalibratedSensorInfo>
  loadCalibratedSensorInfo(const fs::path &filePath);
  static std::map<Token, CalibratedSensorName>
  loadCalibratedSensorNames(const fs::path &filePath);

  std::vector<SceneInfo> scenes;
  std::map<Token, std::vector<SampleInfo>> scene2Samples;
  std::map<Token, std::vector<SampleDataInfo>> sample2SampleData;
  std::map<Token, std::vector<EgoPoseInfo>> scene2EgoPose;
  std::map<Token, CalibratedSensorInfo> calibratedSensorToken2CalibratedSensorInfo;
  std::map<Token, std::set<CalibratedSensorInfoAndName>> scene2CalibratedSensorInfo;
  std::map<Token, CalibratedSensorName> sensorToken2CalibratedSensorName;
  bool loadFromDirectoryCalled = false;
};

}