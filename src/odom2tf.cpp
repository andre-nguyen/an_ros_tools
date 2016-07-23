#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>

tf::Transform laser2m100;
void odom_cb(const nav_msgs::OdometryConstPtr& msg) {
  static tf::TransformBroadcaster br;
  tf::Transform transform;
  tf::poseMsgToTF(msg->pose.pose, transform);
  br.sendTransform(tf::StampedTransform(transform, ros::Time::now(),
    "world", "m100"));
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "odom2tf");
  ros::NodeHandle nh;
  ros::Subscriber odom_sub = nh.subscribe("odom", 10, &odom_cb);

  ros::spin();

  return 0;
}
