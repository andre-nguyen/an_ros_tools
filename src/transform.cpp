#include "transform.h"

// Static quaternion needed for rotating between ENU and NED frames
// +PI rotation around X (North) axis follwed by +PI/2 rotation about Z (Down)
// gives the ENU frame.  Similarly, a +PI rotation about X (East) followed by
// a +PI/2 roation about Z (Up) gives the NED frame.
static const Eigen::Quaterniond NED_ENU_Q = quaternion_from_rpy(M_PI, 0.0, M_PI_2);
static const Eigen::Affine3d NED_ENU_AFFINE(NED_ENU_Q);

Eigen::Quaterniond quaternion_from_rpy(const Eigen::Vector3d &rpy)
{
    // YPR - ZYX
    return Eigen::Quaterniond(
            Eigen::AngleAxisd(rpy.z(), Eigen::Vector3d::UnitZ()) *
            Eigen::AngleAxisd(rpy.y(), Eigen::Vector3d::UnitY()) *
            Eigen::AngleAxisd(rpy.x(), Eigen::Vector3d::UnitX())
            );
}

inline Eigen::Quaterniond quaternion_from_rpy(const double roll,
                                              const double pitch,
                                              const double yaw)
{
    return quaternion_from_rpy(Eigen::Vector3d(roll, pitch, yaw));
}

Eigen::Quaterniond ned2enu(const Eigen::Quaterniond &q)
{
    return NED_ENU_Q * q;
}

Eigen::Vector3d ned2enu(const Eigen::Vector3d &vec)
{
    return NED_ENU_AFFINE * vec;
}

void gps_convert_ned(float &ned_x, float &ned_y,
        double gps_t_lon, double gps_t_lat,
        double gps_r_lon, double gps_r_lat)
{
    double d_lon = gps_t_lon - gps_r_lon;
    double d_lat = gps_t_lat - gps_r_lat;
    ned_x = DEG2RAD(d_lat) * C_EARTH;
    ned_y = DEG2RAD(d_lon) * C_EARTH * cos(DEG2RAD(gps_t_lat));
}

dji_sdk::LocalPosition gps_convert_ned(dji_sdk::GlobalPosition loc,
                                       const dji_sdk::GlobalPosition& ref)
{
    dji_sdk::LocalPosition local;
    gps_convert_ned(local.x, local.y,
        loc.longitude, loc.latitude,
        ref.longitude, ref.latitude
    );
    local.z = loc.height;
    return local;
}
