#include "nuscenes2bag/LidarDirectoryConverterXYZIR.hpp"
#include "nuscenes2bag/utils.hpp"
#include <exception>

using namespace sensor_msgs;
using namespace std;

namespace nuscenes2bag {

inline void
fillFieldsForPointcloudXYZIR(std::vector<PointField>& fields)
{
  PointField field;
  field.datatype = sensor_msgs::PointField::FLOAT32;
  field.offset = 0;
  field.count = 1;
  field.name = std::string("x");
  fields.push_back(field);

  field.datatype = sensor_msgs::PointField::FLOAT32;
  field.offset = 4;
  field.count = 1;
  field.name = std::string("y");
  fields.push_back(field);

  field.datatype = sensor_msgs::PointField::FLOAT32;
  field.offset = 8;
  field.count = 1;
  field.name = std::string("z");
  fields.push_back(field);

  // This field only contains positive integers but it's encoded as a float (4
  // bytes)
  field.datatype = sensor_msgs::PointField::FLOAT32;
  field.offset = 12;
  field.count = 1;
  field.name = std::string("intensity");
  fields.push_back(field);

  // This field only contains positive integers but it's encoded as a float (4
  // bytes)
  field.datatype = sensor_msgs::PointField::FLOAT32;
  field.offset = 16;
  field.count = 1;
  field.name = std::string("ring");
  fields.push_back(field);
}

// Convert float32 to 4 bytes
union
{
  float value;
  uint8_t byte[4];
} floatToBytes;

inline void
push_back_float32_XYZIR(std::vector<uint8_t>& data, float float_data)
{
  floatToBytes.value = float_data;
  data.push_back(floatToBytes.byte[0]);
  data.push_back(floatToBytes.byte[1]);
  data.push_back(floatToBytes.byte[2]);
  data.push_back(floatToBytes.byte[3]);
}

inline std::vector<float>
readBinaryPcdFileXYZIR(std::ifstream& fin)
{
  std::vector<float> fileValues;
  float f;
  while (fin.read(reinterpret_cast<char*>(&f), sizeof(float))) {
    fileValues.push_back(f);
  }

  return fileValues;
}

boost::optional<sensor_msgs::PointCloud2>
readLidarFileXYZIR(const fs::path& filePath)
{

  PointCloud2 cloud;
  cloud.header.frame_id = std::string("lidar");
  cloud.is_bigendian = false;
  cloud.point_step = sizeof(float) * 5; // Length of each point in bytes
  cloud.height = 1;

  try {
    std::ifstream fin(filePath.string(), std::ios::binary);
    const std::vector<float> fileValues = readBinaryPcdFileXYZIR(fin);

    if (fileValues.size() % 5 != 0) {
      throw UnableToParseFileException(filePath.string());
    }
    const size_t pointsNumber = fileValues.size() / 5;
    cloud.width = pointsNumber;

    std::vector<uint8_t> data;
    for (auto float_data : fileValues) {
      push_back_float32_XYZIR(data, float_data);
    }

    fillFieldsForPointcloudXYZIR(cloud.fields);
    cloud.data = data;
    cloud.row_step = data.size(); // Length of row in bytes

  } catch (const std::exception& e) {
    PRINT_EXCEPTION(e);

    return boost::none;
  }

  return boost::optional<sensor_msgs::PointCloud2>(cloud);
}

}