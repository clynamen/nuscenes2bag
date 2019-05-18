#include "nuscenes2bag/MetaData.hpp"

#include <optional>
#include <vector>

class MetaDataProvider {
public:
  virtual ~MetaDataProvider() = default;

  virtual std::vector<Token> getAllSceneTokens() const = 0;
  virtual std::optional<SceneInfo>
  getSceneInfo(const Token &sceneToken) const = 0;
  virtual std::vector<SampleDataInfo>
  getSceneSampleData(const Token &sceneSampleData) const = 0;
  virtual std::vector<EgoPoseInfo>
  getEgoPoseInfo(const Token &sceneToken) const = 0;
  virtual CalibratedSensorInfo
  getCalibratedSensorInfo(const Token &calibratedSensorToken) const = 0;
  virtual std::vector<CalibratedSensorInfoAndName>
  getSceneCalibratedSensorInfo(const Token &sceneToken) const = 0;
  virtual CalibratedSensorName
  getSensorName(const Token &sensorToken) const = 0;


};
