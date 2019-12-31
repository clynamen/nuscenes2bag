#include "nuscenes2bag/MetaDataTypes.hpp"

#include <exception>
#include <vector>

#include <boost/optional.hpp>

namespace nuscenes2bag {

class MetaDataProvider
{
public:
  virtual ~MetaDataProvider() = default;

  virtual std::vector<Token> getAllSceneTokens() const = 0;

  virtual boost::optional<SceneInfo> getSceneInfo(const Token& sceneToken) const = 0;
  virtual boost::optional<SceneInfo> getSceneInfoByNumber(const uint32_t sceneNumber) const = 0;

  virtual std::vector<SampleDataInfo> getSceneSampleData(
    const Token& sceneSampleData) const = 0;
  virtual std::vector<EgoPoseInfo> getEgoPoseInfo(
    const Token& sceneToken) const = 0;
  virtual CalibratedSensorInfo getCalibratedSensorInfo(
    const Token& calibratedSensorToken) const = 0;
  virtual std::vector<CalibratedSensorInfoAndName> getSceneCalibratedSensorInfo(
    const Token& sceneToken) const = 0;
  virtual CalibratedSensorName getSensorName(
    const Token& sensorToken) const = 0;
};

}