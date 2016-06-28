#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>

tf::Transform laser2m100, cam2m100;
void odom_cb(const nav_msgs::OdometryConstPtr& msg) {
  static tf::TransformBroadcaster br;
  tf::Transform transform;
  tf::poseMsgToTF(msg->pose.pose, transform);
  br.sendTransform(tf::StampedTransform(transform, msg->header.stamp,
    "world", "dji_m100"));
  br.sendTransform(tf::StampedTransform(laser2m100, msg->header.stamp,
    "dji_m100", "laser"));
  br.sendTransform(tf::StampedTransform(cam2m100, msg->header.stamp,
    "dji_m100", "cam1_left"));
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "odom2tf");
  ros::NodeHandle nh;
  ros::Subscriber odom_sub = nh.subscribe("odom", 10, &odom_cb);

  laser2m100.setIdentity();
  laser2m100.setOrigin(tf::Vector3(0.1, 0, 0.04));
  cam2m100.setOrigin(tf::Vector3(0.1, 0.07, 0));
  tf::Quaternion q;
  q.setRPY(-M_PI/2, 0, -M_PI/2);
  cam2m100.setRotation(q);

  ros::spin();
  return 0;
}
