#ifndef PCL__POINT_TYPES_HPP_
#define PCL__POINT_TYPES_HPP_

#include <pcl/point_types.h>

struct PointXYZIR
{
  PCL_ADD_POINT4D;
  float intensity;
  uint16_t ring;
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;

POINT_CLOUD_REGISTER_POINT_STRUCT(
  PointXYZIR,
  (float, x, x)(float, y, y)(float, z, z)(float, intensity, intensity)(uint16_t, ring, ring))

#endif  // PCL__POINT_TYPES_HPP_
