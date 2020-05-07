//
// Created by dcheng on 5/6/20.
//

#ifndef JACOBIAN_VISUAL_SERVO_UNCERTAIN_IK_SERVER_ROS_H
#define JACOBIAN_VISUAL_SERVO_UNCERTAIN_IK_SERVER_ROS_H

#include <jacobian_visual_servo/uncertain_IK_server.hpp>
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/JointState.h>

class UncertainIKServerROS : public UncertainIKServer
{
public:
  UncertainIKServerROS(ros::NodeHandle *node_handle,
                    ros::NodeHandle *private_node_handle);
  ~UncertainIKServerROS() = default;

  void timer_cb(const ros::TimerEvent &event);
  void im_cb(const sensor_msgs::ImageConstPtr &im);

  void gst_gt_cb(geometry_msgs::PoseConstPtr msg);

  void joint_angles_cb(sensor_msgs::JointStateConstPtr msg);

  void gd_cb(geometry_msgs::PoseConstPtr msg);

  bool sendJointAngles(const VectorXd &theta_cmd);

  bool recvRobotStates();

private:
  void initialize_sub();
  void initialize_pub();
  void setup_ros();

  ros::NodeHandle nh;
  ros::NodeHandle pnh;

  // node info
  std::string joint_states_topic;
  std::string cam_im_topic;

  // publishers and subscribers
  ros::Publisher joint_state_pub;
  image_transport::ImageTransport itt;
  image_transport::Subscriber cam_im_sub;

  // timer triggers
  ros::Timer timer;

  // control parameters
  int dof;

  // control states
  sensor_msgs::JointState joint_state;

}; // class UncertainIKServerROS

#endif //JACOBIAN_VISUAL_SERVO_UNCERTAIN_IK_SERVER_ROS_H
