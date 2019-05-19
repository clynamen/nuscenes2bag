#pragma once

#include <cstdint>
#define PCL_NO_PRECOMPILE
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace nuscenes2bag {

struct PclRadarObject {
  //   FIELDS x y z dyn_prop id
  // SIZE 4 4 4 1 2
  // rcs vx vy vx_comp vy_comp
  // 4 4 4 4 4
  // is_quality_valid ambig_state x_rms y_rms invalid_state pdh0 vx_rms vy_rms
  // 1 1 1 1 1 1 1 1

  float x;
  float y;
  float z;
//   PCL_ADD_POINT4D

  int8_t dyn_prop;
  int16_t id;
  float rcs;
  float vx;
  float vy;
  float vx_comp;
  float vy_comp;
  int8_t is_quality_valid;
  int8_t ambig_state;
  int8_t x_rms;
  int8_t y_rms;
  int8_t invalid_state;
  int8_t pdh0;
  int8_t vx_rms;
  int8_t vy_rms;
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;

}

POINT_CLOUD_REGISTER_POINT_STRUCT(
    nuscenes2bag::PclRadarObject, 
    (float, x, x)
    (float, y, y)
    (float, z, z)
    (int8_t, dyn_prop, dyn_prop)
    (int16_t, id, id)
    (float, rcs, rcs)
    (float, vx, vx)
    (float, vy, vy)
    (float, vx_comp, vx_comp)
    (float, vy_comp, vy_comp)
    (int8_t, is_quality_valid, is_quality_valid)
    (int8_t, ambig_state, ambig_state)
    (int8_t, x_rms, x_rms)
    (int8_t, y_rms, y_rms)
    (int8_t, invalid_state, invalid_state)
    (int8_t, pdh0, pdh0)
    (int8_t, vx_rms, vx_rms)
    (int8_t, vy_rms, vy_rms)
    )
