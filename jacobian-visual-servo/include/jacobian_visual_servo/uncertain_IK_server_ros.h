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
#include <std_msgs/Bool.h>

class UncertainIKServerROS : public UncertainIKServer
{
public:
  UncertainIKServerROS(ros::NodeHandle *node_handle,
                    ros::NodeHandle *private_node_handle);
  ~UncertainIKServerROS() = default;

  void gst_gt_cb(geometry_msgs::PoseConstPtr msg);

  void joint_angles_cb(sensor_msgs::JointStateConstPtr msg);

  void gd_cb(geometry_msgs::PoseConstPtr msg);

  bool sendJointAngles(const VectorXd &theta_cmd);

  bool recvRobotStates();

  bool lockRobotFrictionJoints(bool lock);

private:
  void initialize_sub();
  void initialize_pub();
  void setup_ros();

  ros::NodeHandle nh;
  ros::NodeHandle pnh;

  // node info
  std::string joint_states_topic;
  std::string friction_joint_lock_topic;
  std::string cam_im_topic;
  std::string joint_angles_topic;

  // publishers and subscribers
  ros::Publisher joint_state_pub;
  ros::Publisher friction_joint_lock_pub;
  image_transport::ImageTransport itt;
  ros::Subscriber joint_angles_sub_, gd_sub_, gst_gt_sub_;

  // control parameters
  int dof;

  // control states
  sensor_msgs::JointState joint_state;

}; // class UncertainIKServerROS

#endif //JACOBIAN_VISUAL_SERVO_UNCERTAIN_IK_SERVER_ROS_H
