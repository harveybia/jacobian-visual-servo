/*
 * Robot end effector pose publisher using april tag readings
 * Henry Zhang
 */

#include <geometry_msgs/Pose.h>
#include <geometry_msgs/TransformStamped.h>
#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>
#include "ros/init.h"

// query gst tf
void get_tag_tf(geometry_msgs::TransformStamped &msg_tf) {}

int main(int argc, char **argv) {
  ros::init(argc, argv, "ee_pose_publisher");

  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");

  // setup tf
  tf2_ros::Buffer tf_buffer;
  tf2_ros::TransformListener transform_listener(tf_buffer);

  // setup publisher

  // update and publish pose
  ros::Rate rate(10.0);
  while (nh.ok()) {
    rate.sleep();
  }
  return 0;
}
