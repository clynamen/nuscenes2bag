#include "nuscenes2bag/MetaDataTypes.hpp"

#include <exception>
#include <vector>

#if CMAKE_CXX_STANDARD >= 17
#include <optional>
#endif

namespace nuscenes2bag {

class MetaDataProvider
{
public:
  virtual ~MetaDataProvider() = default;

  virtual std::vector<Token> getAllSceneTokens() const = 0;

#if CMAKE_CXX_STANDARD >= 17
  virtual std::optional<SceneInfo> getSceneInfo(const Token& sceneToken) const = 0;
#else
  virtual boost::shared_ptr<SceneInfo> getSceneInfo(const Token& sceneToken) const = 0;
#endif

#if CMAKE_CXX_STANDARD >= 17
  virtual std::optional<SceneInfo> getSceneInfoByNumber(const uint32_t sceneNumber) const = 0;
#else
  virtual boost::shared_ptr<SceneInfo> getSceneInfoByNumber(const uint32_t sceneNumber) const = 0;
#endif

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