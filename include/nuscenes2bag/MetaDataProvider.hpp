#include "nuscenes2bag/MetaData.hpp"

#include <vector>
#include <optional>

class MetaDataProvider {
    public:
    virtual ~MetaDataProvider() = default;

    virtual std::vector<Token> getAllSceneTokens() const = 0;
    virtual std::optional<SceneInfo> getSceneInfo(const Token& sceneToken) const = 0;
    virtual std::vector<SampleDataInfo> getSceneSampleData(const Token& sceneSampleData) const = 0;
};