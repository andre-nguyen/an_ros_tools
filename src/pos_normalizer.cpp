#include <ros/ros.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <dji_sdk/GlobalPosition.h>
#include <dji_sdk/LocalPosition.h>
#include <nav_msgs/Odometry.h>

#define C_EARTH (double) 6378137.0
#define C_PI (double) 3.141592653589793
#define DEG2RAD(DEG) ((DEG)*((C_PI)/(180.0)))

dji_sdk::GlobalPosition global_pos_ref;
dji_sdk::LocalPosition local_pos_ref;
bool first_pos_rcvd = false;
ros::Publisher odom_norm_pub, odom_norm_ENU_pub;

void gps_convert_ned(float &ned_x, float &ned_y,
        double gps_t_lon, double gps_t_lat,
        double gps_r_lon, double gps_r_lat)
{
    double d_lon = gps_t_lon - gps_r_lon;
    double d_lat = gps_t_lat - gps_r_lat;
    ned_x = DEG2RAD(d_lat) * C_EARTH;
    ned_y = DEG2RAD(d_lon) * C_EARTH * cos(DEG2RAD(gps_t_lat));
}

dji_sdk::LocalPosition gps_convert_ned(dji_sdk::GlobalPosition loc)
{
    dji_sdk::LocalPosition local;
    gps_convert_ned(local.x, local.y,
        loc.longitude, loc.latitude,
        global_pos_ref.longitude, global_pos_ref.latitude
    );
    local.z = loc.height;
    return local;
}

void callback(const dji_sdk::GlobalPositionConstPtr& global_pos,
              const nav_msgs::OdometryConstPtr& odometry)
{
    if(!first_pos_rcvd) {
        global_pos_ref = *global_pos;
        first_pos_rcvd = true;
    }

    nav_msgs::Odometry odom_norm = *odometry;
    local_pos_ref = gps_convert_ned(*global_pos);
    odom_norm.pose.pose.position.x = local_pos_ref.x;
    odom_norm.pose.pose.position.y = local_pos_ref.y;
    odom_norm.pose.pose.position.z = -local_pos_ref.z;
    odom_norm_pub.publish(odom_norm);

    // god dammit dji
    nav_msgs::Odometry odom_norm_ENU = *odometry;
    // the following 3 lines should give an actual NED
    odom_norm_ENU.pose.pose.position.x = local_pos_ref.y;
    odom_norm_ENU.pose.pose.position.y = local_pos_ref.z;
    odom_norm_ENU.pose.pose.position.z = local_pos_ref.z;
    odom_norm_ENU.twist.twist.linear.x = odom_norm.twist.twist.linear.y;
    odom_norm_ENU.twist.twist.linear.y = odom_norm.twist.twist.linear.x;
    // looks like vz is already up
    odom_norm_ENU_pub.publish(odom_norm_ENU);
}


int main(int argc, char** argv)
{
    ros::init(argc, argv, "pos_normalizer");
    ros::NodeHandle nh;
    odom_norm_pub = nh.advertise<nav_msgs::Odometry>
            ("/dji_sdk/odometry_normalized", 10);
    odom_norm_ENU_pub = nh.advertise<nav_msgs::Odometry>
            ("/dji_sdk/odometry_normalized_ENU", 10);
    message_filters::Subscriber<dji_sdk::GlobalPosition>
            global_sub(nh, "/dji_sdk/global_position", 10);
    message_filters::Subscriber<nav_msgs::Odometry> odom_sub
            (nh, "/dji_sdk/odometry", 10);
    typedef message_filters::sync_policies::ApproximateTime
            <dji_sdk::GlobalPosition, nav_msgs::Odometry> MySyncPolicy;
    message_filters::Synchronizer<MySyncPolicy>
            sync(MySyncPolicy(10), global_sub, odom_sub);
    sync.registerCallback(boost::bind(&callback, _1, _2));

    ros::spin();


    return 0;
}
