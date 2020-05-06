#include "jacobian_visual_servo/servo_node.hpp"

#include <math.h>
#include <sstream>
#include <boost/bind.hpp>
#include <signal.h>

#define likely(x) __builtin_expect(!!(x), 1)
#define unlikely(x) __builtin_expect(!!(x), 0)

JacobianServoNode::JacobianServoNode(
    ros::NodeHandle *node_handle, ros::NodeHandle *private_node_handle)
    : nh(*node_handle),
      pnh(*private_node_handle)
{
    setup_ros();
    initialize_sub();
    initialize_pub();
}

void JacobianServoNode::setup_ros()
{
    ROS_INFO("Setting up ROS stuff");
    pnh.param<std::string>(
        "joint_states_topic", joint_states_topic, "/joint_states");
    pnh.param<int>(
        "joint_states_dof", dof, 9);

    // Periodic timer to trigger control loop
    timer = nh.createTimer(
        ros::Duration(1),&JacobianServoNode::timer_cb, this);
}

void JacobianServoNode::initialize_sub()
{
    return;
}

void JacobianServoNode::initialize_pub()
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

void JacobianServoNode::timer_cb(const ros::TimerEvent &event)
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

    ROS_INFO("Instantiating JacobianServoNode");
    JacobianServoNode servo_node(&node_handle, &private_node_handle);

    signal(SIGINT, sigint_handler);

    ros::spin();
    return 0;
}
