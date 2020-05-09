/*
 * Robot end effector pose publisher using april tag readings
 * Henry Zhang
 */

#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <ros/ros.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_listener.h>
#include "tf2/convert.h"

int main(int argc, char **argv) {
  ros::init(argc, argv, "ee_pose_publisher");

  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");

  // setup tf
  tf2_ros::Buffer tf_buffer;
  tf2_ros::TransformListener tf_listener(tf_buffer);

  // setup publisher
  ros::Publisher pub = nh.advertise<geometry_msgs::TransformStamped>(
      "/snake_arm/estimated_pose", 1);

  // update and publish pose
  ros::Rate rate(10.0);
  while (nh.ok()) {
    // get tf transform
    geometry_msgs::TransformStamped gst, gsw, gwc, gct;
    try {
      const auto query_time = ros::Time::now() - ros::Duration(0.1);
      gsw = tf_buffer.lookupTransform("tag_0", "base_link", query_time);
      gwc = tf_buffer.lookupTransform("camera", "tag_0", query_time);
      gct = tf_buffer.lookupTransform("ee_link", "camera", query_time);
    } catch (tf2::TransformException &ex) {
      ROS_WARN("%s", ex.what());
      ros::Duration(1.0).sleep();
      continue;
    }

    // calculate gst
    tf2::doTransform(gsw, gst, gwc);
    tf2::doTransform(gst, gst, gct);

    pub.publish(gsw);

    rate.sleep();
  }
  return 0;
}
