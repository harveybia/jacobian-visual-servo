#ifndef __SERVO_NODE_HPP__
#define __SERVO_NODE_HPP__

/*
 * ROS node for jacobian visual servo
 */

#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/JointState.h>

class JacobianServoNode
{
public:
    JacobianServoNode(ros::NodeHandle *node_handle,
                      ros::NodeHandle *private_node_handle);
    ~JacobianServoNode() = default;

    void timer_cb(const ros::TimerEvent &event);

private:
    void initialize_sub();
    void initialize_pub();
    void setup_ros();

    ros::NodeHandle nh;
    ros::NodeHandle pnh;

    // node info
    std::string joint_states_topic;

    // publishers and subscribers
    ros::Publisher joint_state_pub;

    // timer triggers
    ros::Timer timer;

}; // class JacobianServoNode

#endif /* __SERVO_NODE_HPP__ */
