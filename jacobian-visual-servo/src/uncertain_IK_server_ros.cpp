//
// Created by dcheng on 5/6/20.
//

#include <jacobian_visual_servo/uncertain_IK_server_ros.h>

#include "jacobian_visual_servo/servo_node.hpp"

#include <math.h>
#include <sstream>
#include <boost/bind.hpp>
#include <signal.h>

using std::cout;
using std::endl;

#define likely(x) __builtin_expect(!!(x), 1)
#define unlikely(x) __builtin_expect(!!(x), 0)

static bool are_poses_equal(
  geometry_msgs::Pose p1,
  geometry_msgs::Pose p2)
{
  double eps = 0.001;
  return (
    (p1.position.x - p2.position.x < eps) &&
    (p1.position.y - p2.position.y < eps) &&
    (p1.position.z - p2.position.z < eps) &&
    (p1.orientation.w - p2.orientation.w < eps) &&
    (p1.orientation.x - p2.orientation.x < eps) &&
    (p1.orientation.y - p2.orientation.y < eps) &&
    (p1.orientation.z - p2.orientation.z < eps)
  );
}

void Pose2Trans(geometry_msgs::PoseConstPtr pose, Matrix4d& T)
{
  T = Matrix4d::Zero();

  Quaterniond qst(pose->orientation.w, pose->orientation.x, pose->orientation.y,
                  pose->orientation.z);
  T.topLeftCorner<3,3>() = qst.toRotationMatrix();
  T.topRightCorner<3,1>() =
      Vector3d(pose->position.x, pose->position.y, pose->position.z);
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

  theta_ = VectorXd(dof);
  theta_.fill(0);
  theta_init_.resize(dof);
  theta_init_.fill(0);
  gst_init_ = Matrix4d::Zero();
  while (ros::ok() && (theta_(0) == 0 || gst_init_ == Matrix4d::Zero()))
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
  pnh.param<std::string>(
      "joint_angles_topic", joint_angles_topic, "/snake_arm/joint_states"
  );
  cout << "*** ROS params ***:" << endl
       << "  Joint cmd topic    : " << joint_states_topic << endl
       << "  Joint state dof    : " << dof << endl
       << "  Joint angles topic : " << joint_angles_topic << endl
       << "*******************\n";
}

void UncertainIKServerROS::initialize_sub()
{
  joint_angles_sub_ = nh.subscribe(joint_angles_topic, 10,
      &UncertainIKServerROS::joint_angles_cb, this);
  gd_sub_ = nh.subscribe("/gst_desired", 10,
      &UncertainIKServerROS::gd_cb, this);
  gst_gt_sub_ = nh.subscribe("/snake_arm/pose", 10,
      &UncertainIKServerROS::gst_gt_cb, this);
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


bool UncertainIKServerROS::sendJointAngles(const VectorXd &theta_cmd)
{
  // send command
  assert(dof == theta_cmd.rows());

  for (int i = 0; i < dof; i++)
  {
    joint_state.position[i] = theta_cmd(i) + theta_init_(i);
  }
  joint_state_pub.publish(joint_state);

  // wait until robot is in position.
  int itn_cnt = 0;
  VectorXd theta_prev = theta_;
  while (((theta_cmd - theta_).norm() > 0.00005
       || (theta_prev - theta_).norm() > 0.00001)
       && ((++itn_cnt) < 10)
       && ros::ok())
  {
    theta_prev = theta_;
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
  Pose2Trans(msg, gst_gt_);
  if (gst_init_ == Matrix4d::Zero())
    gst_init_ = gst_gt_;
}

void UncertainIKServerROS::joint_angles_cb(sensor_msgs::JointStateConstPtr msg)
{
  assert(theta_.rows() == msg->position.size());

  for (int i = 0; i < msg->position.size(); i++)
    theta_(i) = msg->position[i] - theta_init_(i);
}

void UncertainIKServerROS::gd_cb(geometry_msgs::PoseConstPtr msg)
{
  static geometry_msgs::Pose prev_pose;

  if (!are_poses_equal(*msg, prev_pose))
  {
    Pose2Trans(msg, gd_);
    prev_pose = *msg;
  }
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

  while (ros::ok())
  {
    ros::spinOnce();
    servo_node.process();
  }
  return 0;
}
