#ifndef TRANSFORM_H
#define TRANSFORM_H

#include <dji_sdk/GlobalPosition.h>
#include <dji_sdk/LocalPosition.h>
#include <eigen3/Eigen/Dense>

#define C_EARTH (double) 6378137.0
#define C_PI (double) 3.141592653589793
#define DEG2RAD(DEG) ((DEG)*((C_PI)/(180.0)))


Eigen::Quaterniond quaternion_from_rpy(const Eigen::Vector3d &rpy);
inline Eigen::Quaterniond quaternion_from_rpy(const double roll,
    const double pitch, const double yaw);

Eigen::Vector3d ned2enu(const Eigen::Vector3d &vec);
Eigen::Quaterniond ned2enu(const Eigen::Quaterniond &q);

/**
 * @brief gps_convert_ned
 * @param ned_x [out] x in NED
 * @param ned_y [out] y in NED
 * @param gps_t_lon [in] measured longitude
 * @param gps_t_lat [in] measured latitude
 * @param gps_r_lon [in] reference longitude (origin)
 * @param gps_r_lat [in] reference latitude (origin)
 */
void gps_convert_ned(float &ned_x, float &ned_y,
        double gps_t_lon, double gps_t_lat,
        double gps_r_lon, double gps_r_lat);

/**
 * @brief gps_convert_ned
 * @param loc Global position
 * @return
 */
dji_sdk::LocalPosition gps_convert_ned(dji_sdk::GlobalPosition loc,
                                       const dji_sdk::GlobalPosition& ref);

#endif // TRANSFORM_H
