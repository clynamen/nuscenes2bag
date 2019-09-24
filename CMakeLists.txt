cmake_minimum_required(VERSION 2.8.3)
project(nuscenes2bag)

find_package(Boost 1.69.0 REQUIRED)

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

set(SRCS
    src/EgoPoseConverter.cpp
    src/ImageDirectoryConverter.cpp
    src/LidarDirectoryConverter.cpp
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

add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS}
                 ${catkin_EXPORTED_TARGETS})

target_link_libraries(${PROJECT_NAME}
                      ${OpenCV_LIBRARIES}
                      ${catkin_LIBRARIES})

install(DIRECTORY include/${PROJECT_NAME}/ thirdparty/json/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
        FILES_MATCHING
        PATTERN "*.h"
        PATTERN ".svn" EXCLUDE)
