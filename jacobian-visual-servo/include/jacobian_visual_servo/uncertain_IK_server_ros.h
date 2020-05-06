//
// Created by dcheng on 5/6/20.
//

#ifndef JACOBIAN_VISUAL_SERVO_UNCERTAIN_IK_SERVER_ROS_H
#define JACOBIAN_VISUAL_SERVO_UNCERTAIN_IK_SERVER_ROS_H

#include <jacobian_visual_servo/uncertain_IK_server.hpp>
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>

class UncertainIKServerROS : public UncertainIKServer
{
public:
  explicit UncertainIKServerROS();

  bool sendJointAngles(const VectorXd &theta_cmd);

  bool gst_cb(geometry_msgs::PoseStampedConstPtr msg);

  bool theta_cb(); // todo: what's the data format?

private:
  ros::NodeHandle n_;
  ros::Subscriber theta_sub_;
  ros::Subscriber gst_sub_;
  ros::Publisher theta_cmd_pub_;
};

#endif //JACOBIAN_VISUAL_SERVO_UNCERTAIN_IK_SERVER_ROS_H
