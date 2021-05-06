# nuScenes2Bag

 - Ubuntu 18.04 Melodic: ![](https://github.com/clynamen/nuscenes2bag/workflows/ubuntu_1804_melodic/badge.svg)
 - Ubuntu 16.04 Kinetic: ![](https://github.com/clynamen/nuscenes2bag/workflows/ubuntu_1604_kinetic/badge.svg)

Simple C++ tool for converting the [nuScenes](https://www.nuscenes.org/) dataset from [Aptiv](https://www.aptiv.com).

The tool loads the json metadata and then the sample files for each scene. The sample are converted in a suitable ROS msg and written to a bag. TF tree is also written.

Probably the original dataset is also collected by Aptiv using ROS, so most data has the same format.

![](images/ros_preview.png)

## Install

The `master` branch targets Ubuntu 18.04 and newer.
The `ubuntu_1604` branch uses C++11 and has been tested on Ubuntu 16.04.

The tool is a normal ROS package. Place it under a workspace and build it with catkin.

## Usage

**Command-line arguments:**
`--dataroot,-s`: The path to the directory that contains the 'maps', 'samples' and 'sweeps'.
`--version`: (optional) The sub-directory that contains the metadata .json files. Default = "v1.0-mini"
`--scene_number,-n`: (optional) Only convert a given scene
`--compress,-c`: (optional) whether to use compressed images to reduce file size



**Converting the 'mini' dataset:**

Convert one scene to a bag file, saved in a new directory:
Scene '0061' will be saved to 'nuscenes_bags/61.bag'
```
rosrun nuscenes2bag nuscenes2bag --scene_number 0061 --dataroot /path/to/nuscenes_mini_meta_v1.0/ --out nuscenes_bags/
```


Convert the entire dataset to bag files:
This processes 4 scenes simultaneously, however the scene numbers are not processed in numerical order.
```
rosrun nuscenes2bag nuscenes2bag --dataroot /path/to/nuscenes_mini_meta_v1.0/ --out nuscenes_bags/ --jobs 4
```

Convert one scene to a bag file with compressed images:
rosrun nuscenes2bag nuscenes2bag -c --scene_number 0061 --dataroot /path/to/nuscenes_mini_meta_v1.0/ --out nuscenes_bags/

**Converting other datasets:**

Convert a dataset with the metadata in a sub-directory called 'v2.0':
```
rosrun nuscenes2bag nuscenes2bag --dataroot /path/to/nuscenes_data_v2.0/ --version v2.0 --out nuscenes_bags/ --jobs 4
```


## Status

Currently work in progress

- [x] Image support
- [x] Pointcloud support
- [x] Radar support
- [x] EgoPose and TF support
- [x] Show progress
- [ ] Better usability

Create an Github issue for suggestion, bug and requests.

## Thirdparty

Built using:

 - https://github.com/nlohmann/json
 - http://github.com/en4bz/ThreadPool

## Authors

 - [clynamen](https://github.com/clynamen/)
 - [ChernoA](https://github.com/ChernoA)
