#include <filesystem>
#include <iostream>
#include <map>

#include <nlohmann/json.hpp>

#include "nuscenes2bag/MetaData.hpp"
#include "nuscenes2bag/MetaDataProvider.hpp"
#include "nuscenes2bag/ToDebugString.hpp"

class MetaDataReader : public MetaDataProvider {
public:
  void loadFromDirectory(const std::filesystem::path &directoryPath);

  static nlohmann::json slurpJsonFile(const std::filesystem::path &filePath);
  static std::vector<SceneInfo>
  loadScenesFromFile(const std::filesystem::path &filePath);
  static std::map<Token, std::vector<SampleInfo>>
  loadSampleInfos(const std::filesystem::path &filePath);
  static std::map<Token, std::vector<SampleDataInfo>>
  loadSampleDataInfos(const std::filesystem::path &filePath);
  static std::map<Token, std::vector<EgoPoseInfo>> loadEgoPoseInfos(
      const std::filesystem::path &filePath,
      std::map<Token, Token> sample2SampleData);

  std::vector<Token> getAllSceneTokens() const override;
  std::optional<SceneInfo> getSceneInfo(const Token &sceneToken) const override;
  std::vector<SampleDataInfo>
  getSceneSampleData(const Token &sceneToken) const override;
  std::vector<EgoPoseInfo>
  getEgoPoseInfo(const Token &sceneToken) const override;

private:
  std::vector<SceneInfo> scenes;
  std::map<Token, std::vector<SampleInfo>> scene2Samples;
  std::map<Token, std::vector<SampleDataInfo>> sample2SampleData;
  std::map<Token, std::vector<EgoPoseInfo>> scene2EgoPose;
  bool loadFromDirectoryCalled = false;
};
