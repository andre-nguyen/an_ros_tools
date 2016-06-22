#include <ros/ros.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <tf/tf.h>

ros::Publisher transform_pub;
ros::Publisher odom_pub;

void callback(const geometry_msgs::PoseStampedConstPtr& pose1,
              const geometry_msgs::PoseStampedConstPtr& pose2)
{
  geometry_msgs::TransformStamped t;
  t.header = pose1->header;
  t.transform.translation.x = pose1->pose.position.x - pose2->pose.position.x;
  t.transform.translation.y = pose1->pose.position.y - pose2->pose.position.y;
  t.transform.translation.z = pose1->pose.position.z - pose2->pose.position.z;
  tf::Quaternion qt, q1, q2;
  tf::quaternionMsgToTF(pose1->pose.orientation, q1);
  tf::quaternionMsgToTF(pose2->pose.orientation, q2);
  qt = q2 * q1.inverse();
  tf::quaternionTFToMsg(qt, t.transform.rotation);
  transform_pub.publish(t);
}

void callbackOdom(const nav_msgs::OdometryConstPtr& odom1,
                  const nav_msgs::OdometryConstPtr& odom2)
{
  nav_msgs::Odometry o;
  o.header = odom1->header;
  o.pose.pose.position.x = odom1->pose.pose.position.x - odom2->pose.pose.position.x;
  o.pose.pose.position.y = odom1->pose.pose.position.y - odom2->pose.pose.position.y;
  o.pose.pose.position.z = odom1->pose.pose.position.z - odom2->pose.pose.position.z;
  tf::Quaternion qt, q1, q2;
  tf::quaternionMsgToTF(odom1->pose.pose.orientation, q1);
  tf::quaternionMsgToTF(odom2->pose.pose.orientation, q2);
  qt = q2 * q1.inverse();
  tf::quaternionTFToMsg(qt, o.pose.pose.orientation);

  o.twist.twist.angular.x = odom1->twist.twist.angular.x - odom2->twist.twist.angular.x;
  o.twist.twist.angular.y = odom1->twist.twist.angular.y - odom2->twist.twist.angular.y;
  o.twist.twist.angular.z = odom1->twist.twist.angular.z - odom2->twist.twist.angular.z;

  o.twist.twist.linear.x = odom1->twist.twist.linear.x - odom2->twist.twist.linear.x;
  o.twist.twist.linear.y = odom1->twist.twist.linear.y - odom2->twist.twist.linear.y;
  o.twist.twist.linear.z = odom1->twist.twist.linear.z - odom2->twist.twist.linear.z;
  odom_pub.publish(o);
}

/**
 * @brief This node subscribes to two pose stamped messages and returns the
 * transform between the two poses. It assumes both messages are in the same
 * reference frame.
 */
int main(int argc, char** argv) {
  ros::init(argc, argv, "rel_transform");
  ros::NodeHandle nh;
  message_filters::Subscriber<geometry_msgs::PoseStamped>
      pose_sub1(nh, "pose1", 1);
  message_filters::Subscriber<geometry_msgs::PoseStamped>
      pose_sub2(nh, "pose2", 1);
  message_filters::TimeSynchronizer<geometry_msgs::PoseStamped, geometry_msgs::PoseStamped>
      sync(pose_sub1, pose_sub2, 10);
  sync.registerCallback(boost::bind(&callback, _1, _2));

  message_filters::Subscriber<nav_msgs::Odometry>
      odom_sub1(nh, "odom1", 1);
  message_filters::Subscriber<nav_msgs::Odometry>
      odom_sub2(nh, "odom2", 1);
  message_filters::TimeSynchronizer<nav_msgs::Odometry, nav_msgs::Odometry>
      sync2(odom_sub1, odom_sub2, 10);
  sync2.registerCallback(boost::bind(&callbackOdom, _1, _2));

  transform_pub = nh.advertise<geometry_msgs::TransformStamped>
      ("~/transform", 10);
  odom_pub = nh.advertise<nav_msgs::Odometry>
      ("~/odom_diff", 10);

  ros::spin();

  return 0;
}
