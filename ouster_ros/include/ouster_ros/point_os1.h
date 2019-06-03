/**
 * @file
 * @brief PCL point datatype for use with the OS-1
 */

#pragma once
#define PCL_NO_PRECOMPILE
#include <pcl/point_types.h>

namespace ouster_ros {
namespace OS1 {

struct EIGEN_ALIGN16 PointOS1 {
    PCL_ADD_POINT4D;
//<<<<<<< HEAD
//    int32_t time_offset_us;
//=======
    float intensity;
    uint32_t t;
//>>>>>>> v1.12.0
    uint16_t reflectivity;
    uint8_t ring;
    uint16_t noise;
    uint32_t range;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    static inline PointOS1 make(float x, float y, float z, float intensity,
                                uint32_t t, uint16_t reflectivity, uint8_t ring,
                                uint16_t noise, uint32_t range) {
        return {x, y, z, 0.0, intensity, t, reflectivity, ring, noise, range};
    }
};
}
}

// clang-format off
POINT_CLOUD_REGISTER_POINT_STRUCT(ouster_ros::OS1::PointOS1,
    (float, x, x)
    (float, y, y)
    (float, z, z)
//<<<<<<< HEAD
//    (int32_t, time_offset_us, time_offset_us)
//=======
    (float, intensity, intensity)
    (uint32_t, t, t)
//>>>>>>> v1.12.0
    (uint16_t, reflectivity, reflectivity)
    (uint8_t, ring, ring)
    (uint16_t, noise, noise)
    (uint32_t, range, range)
)
// clang-format on
