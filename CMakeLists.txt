
cmake_minimum_required(VERSION 3.8) # C++17 support was added in CMake 3.8

project(nuscenes2bag)

#set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD 14)

set(CMAKE_BUILD_TYPE Release) # Debug, Release, RelWithDebInfo

#find_package(Boost 1.58.0 COMPONENTS filesystem thread REQUIRED) # Ubuntu 16.04
find_package(Boost 1.65.0 COMPONENTS filesystem thread REQUIRED) # Ubuntu 18.04

find_package(catkin
             REQUIRED
             rosbag
             sensor_msgs
             cv_bridge
             pcl_conversions
             pcl_msgs
             message_generation
             geometry_msgs
             std_msgs
             nav_msgs
             tf)

find_package(OpenCV REQUIRED core)

add_message_files(FILES RadarObjects.msg RadarObject.msg)

generate_messages(DEPENDENCIES
                  std_msgs
                  geometry_msgs
                  tf
                  nav_msgs)

catkin_package(INCLUDE_DIRS
               include
               thirdparty/json/
               DEPENDS
               OpenCV)

include_directories(SYSTEM
                    thirdparty/json/
                    ${catkin_INCLUDE_DIRS}
                    ${Boost_INCLUDE_DIRS}
                    ${OpenCV_INCLUDE_DIRS})
include_directories(include)

find_package(PCL REQUIRED COMPONENTS common io)

set(SRCS
    src/EgoPoseConverter.cpp
    src/ImageDirectoryConverter.cpp
    src/LidarDirectoryConverter.cpp
    src/LidarDirectoryConverterXYZIR.cpp
    src/RadarDirectoryConverter.cpp
    src/NuScenes2Bag.cpp
    src/FileProgress.cpp
    src/MetaDataReader.cpp
    src/MetaData.cpp
    src/SceneConverter.cpp
    src/utils.cpp)

add_executable(${PROJECT_NAME} ${SRCS} src/main.cpp)

target_compile_options(${PROJECT_NAME}
                       PRIVATE
                               -Wall
                               -Wextra
                               -pedantic
                               )

target_compile_definitions(${PROJECT_NAME} PRIVATE "-DCMAKE_CXX_STANDARD=${CMAKE_CXX_STANDARD}")

add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS}
                 ${catkin_EXPORTED_TARGETS})

target_link_libraries(${PROJECT_NAME}
                      ${OpenCV_LIBRARIES}
                      ${PCL_COMMON_LIBRARY}
                      ${PCL_IO_LIBRARY}
                      ${catkin_LIBRARIES})

install(DIRECTORY include/${PROJECT_NAME}/ thirdparty/json/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
        FILES_MATCHING
        PATTERN "*.h"
        PATTERN ".svn" EXCLUDE)
