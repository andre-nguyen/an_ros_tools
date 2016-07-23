#include <ros/ros.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <dji_sdk/GlobalPosition.h>
#include <dji_sdk/LocalPosition.h>
#include <nav_msgs/Odometry.h>
#include <eigen3/Eigen/Dense>

#include "transform.h"

dji_sdk::GlobalPosition global_pos_ref;
dji_sdk::LocalPosition local_pos_ref;
bool first_pos_rcvd = false;
ros::Publisher odom_norm_pub, odom_norm_ENU_pub;

void callback(const dji_sdk::GlobalPositionConstPtr& global_pos,
              const nav_msgs::OdometryConstPtr& odometry)
{
    if(!first_pos_rcvd) {
        global_pos_ref = *global_pos;
        first_pos_rcvd = true;
    }

    nav_msgs::Odometry odom_norm = *odometry;
    local_pos_ref = gps_convert_ned(*global_pos, global_pos_ref);
    odom_norm.pose.pose.position.x = local_pos_ref.x;
    odom_norm.pose.pose.position.y = local_pos_ref.y;
    odom_norm.pose.pose.position.z = -local_pos_ref.z;
    odom_norm.twist.twist.linear.z *= -1;
    odom_norm_pub.publish(odom_norm);

    // god dammit dji
    nav_msgs::Odometry odom_norm_ENU = *odometry;
    // The code above was for NED now convert to ENU
    Eigen::Vector3d position_ENU = ned2enu(Eigen::Vector3d(local_pos_ref.x,
                                                           local_pos_ref.y,
                                                           -local_pos_ref.z));
    Eigen::Quaterniond orientation_ENU = ned2enu(Eigen::Quaterniond(
        odom_norm.pose.pose.orientation.w, odom_norm.pose.pose.orientation.x,
        odom_norm.pose.pose.orientation.y, odom_norm.pose.pose.orientation.z));
    // wtf is this NEU? NED?
    Eigen::Vector3d velocity_ENU = ned2enu(Eigen::Vector3d(
        odom_norm.twist.twist.linear.x, odom_norm.twist.twist.linear.y,
        -odom_norm.twist.twist.linear.z));

    odom_norm_ENU.pose.pose.position.x = position_ENU(0);
    odom_norm_ENU.pose.pose.position.y = position_ENU(1);
    odom_norm_ENU.pose.pose.position.z = position_ENU(2);
    odom_norm_ENU.pose.pose.orientation.w = orientation_ENU.w();
    odom_norm_ENU.pose.pose.orientation.x = orientation_ENU.x();
    odom_norm_ENU.pose.pose.orientation.y = orientation_ENU.y();
    odom_norm_ENU.pose.pose.orientation.z = orientation_ENU.z();
    odom_norm_ENU.twist.twist.linear.x = velocity_ENU(0);
    odom_norm_ENU.twist.twist.linear.y = velocity_ENU(1);
    odom_norm_ENU.twist.twist.linear.z = velocity_ENU(2);

    odom_norm_ENU_pub.publish(odom_norm_ENU);
}


int main(int argc, char** argv)
{
    ros::init(argc, argv, "pos_normalizer");
    ros::NodeHandle nh;
    odom_norm_pub = nh.advertise<nav_msgs::Odometry>
            ("/dji_sdk/odometry_normalized_NED", 10);
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
