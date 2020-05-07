//
// Created by dcheng on 5/6/20.
//

#include <jacobian_visual_servo/uncertain_IK_server_ros.h>

#include "jacobian_visual_servo/servo_node.hpp"

#include <math.h>
#include <sstream>
#include <boost/bind.hpp>
#include <signal.h>

#define likely(x) __builtin_expect(!!(x), 1)
#define unlikely(x) __builtin_expect(!!(x), 0)

void Pose2Trans(const geometry_msgs::Pose& pose, Matrix4d T)
{
  T = Matrix4d::Zero();

  Quaterniond qst(pose.orientation.w, pose.orientation.x, pose.orientation.y,
                  pose.orientation.z);
  T.topLeftCorner<3,3>() = qst.toRotationMatrix();
  T.topRightCorner<3,1>() =
      Vector3d(pose.position.x, pose.position.y, pose.position.z);
  T(3,3) = 1.0;
}

UncertainIKServerROS::UncertainIKServerROS(
    ros::NodeHandle *node_handle, ros::NodeHandle *private_node_handle)
: UncertainIKServer()
, nh(*node_handle)
, pnh(*private_node_handle)
, itt(*node_handle)
{
  setup_ros();
  initialize_sub();
  initialize_pub();


  recvRobotStates();
}

void UncertainIKServerROS::setup_ros()
{
  ROS_INFO("Setting up ROS stuff");
  pnh.param<std::string>(
      "joint_states_topic", joint_states_topic, "/joint_states"
  );
  pnh.param<int>(
      "joint_states_dof", dof, 9
  );
  pnh.param<std::string>(
      "ee_cam_topic", cam_im_topic, "/snake_cam/image_color"
  );

  // Periodic timer to trigger control loop
  timer = nh.createTimer(
      ros::Duration(1),&UncertainIKServerROS::timer_cb, this);
}

void UncertainIKServerROS::initialize_sub()
{
  cam_im_sub = itt.subscribe(
      cam_im_topic.c_str(), 1,
      boost::bind(&UncertainIKServerROS::im_cb, this, _1));
}

void UncertainIKServerROS::initialize_pub()
{
  // joint state publisher
  joint_state_pub = nh.advertise<sensor_msgs::JointState>(
      joint_states_topic, 1);
  joint_state.name.resize(dof);
  for (int i = 0; i < dof; i++)
  {
    std::stringstream ss;
    // Unknown robot configuration, use generic joint names
    ss << "joint_" << i;
    joint_state.name[i] = ss.str();
  }
  joint_state.position.resize(dof);
}

void UncertainIKServerROS::timer_cb(const ros::TimerEvent &event)
{
  joint_state.header.stamp = ros::Time::now();
  std::stringstream ss;
  ss << "[ ";
  for (int i = 0; i < dof; i++)
  {
    // XXX: Change this to actual control output
    joint_state.position[i] = sin(event.current_real.nsec) / 2;
    ss << joint_state.position[i] << " ";
  }
  ss << "]";
  ROS_INFO("New target joint state: %s", ss.str().c_str());
  joint_state_pub.publish(joint_state);
}

void UncertainIKServerROS::im_cb(const sensor_msgs::ImageConstPtr &im)
{
  // ROS_INFO("Received camera frame"); // Confirmed working.
  // TODO: use visual feedback to servo robot arm
}

bool UncertainIKServerROS::sendJointAngles(const VectorXd &theta_cmd)
{
  // send command
  assert(dof == theta_cmd.rows());

  for (int i = 0; i < dof; i++)
  {
    joint_state.position[i] = theta_cmd(i);
  }
  joint_state_pub.publish(joint_state);

  // wait until robot is in position.
  int itn_cnt = 0;
  while ((theta_cmd - theta_).norm() > 0.01 && (++itn_cnt) < 10)
  {
    recvRobotStates();
  }

  return itn_cnt < 10;
}

bool UncertainIKServerROS::recvRobotStates()
{
  usleep(1e5); // sleep 100 ms
  ros::spinOnce();
  return true;
}

void UncertainIKServerROS::gst_gt_cb(geometry_msgs::PoseConstPtr msg)
{
  Pose2Trans(*msg, gst_gt_);
  if (gst_init_ == Matrix4d::Zero())
    gst_init_ = gst_gt_;
}

void UncertainIKServerROS::joint_angles_cb(sensor_msgs::JointStateConstPtr msg)
{
  assert(theta_.size() == msg->position.size());
  for (int i = 0; i < msg->position.size(); i++)
    theta_(i) = msg->position[i];
}

void UncertainIKServerROS::gd_cb(geometry_msgs::PoseConstPtr msg)
{
  Pose2Trans(*msg, gd_);
}

void sigint_handler(int sig)
{
  ROS_INFO("Cleaning up renderer resources");
  ros::shutdown();
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "jacobian_servo_node");
  ros::NodeHandle node_handle;
  ros::NodeHandle private_node_handle("~");

  ROS_INFO("Instantiating UncertainIKServerROS");
  UncertainIKServerROS servo_node(&node_handle, &private_node_handle);

  signal(SIGINT, sigint_handler);

  ros::spin();
  return 0;
}
