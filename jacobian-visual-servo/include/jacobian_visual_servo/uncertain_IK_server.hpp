//
// Created by dcheng on 5/6/20.
//

#ifndef JACOBIAN_VISUAL_SERVO_UNCERTAIN_IK_SERVER_HPP
#define JACOBIAN_VISUAL_SERVO_UNCERTAIN_IK_SERVER_HPP

#include <iostream>
#include <Eigen/Dense>
#include <vector>

using namespace Eigen;

/**
 * IK server with uncertain manipulator. Independent of ROS.
 */
class UncertainIKServer
{
public:
  /**
   * Constructor todo what should constructor do?
   */
  explicit UncertainIKServer(const Matrix4d& _gst_init,
      const Matrix4d& _gd);

  void setGd(const Matrix4d& gd);
  /**
   * interface for sending joint angle command, block until theta = theta_cmd,
   * also update gst_gt_.
   * @param theta_cmd input command
   * @return
   */
  virtual bool sendJointAngles(const VectorXd &theta_cmd) = 0;

  /**
   * interface for receiving joint angle command
   * @return
   */
  virtual bool recvJointAngles() = 0;

  /**
   * interface for receiving tool pose ground truth.
   * @return
   */
  virtual bool recvGstGT() = 0;

  /**
   * Process: perform IK step, check if reached goal, check FK/GroundTruth;
   *   if check failed, perform finite motion J and update twists.
   * @return true if goal is reached
   */
  bool process();

private:
  /**
   * calculate an IK step, and send new joint angle commands to environment
   * @return
   */
  bool IKStep();

  /**
   * Use finite motion to compute Jacobian matrix
   * @param J output Jacobian matrix
   * @return false if the process failed.
   */
  bool finiteMotionJ(MatrixXd& J);

  /**
   * recover and update twists from new Jacobian matrix
   * @param J
   * @return
   */
  bool calcTwistFromJ(const MatrixXd& J);

  /**
   * check if FK adds up to gst ground truth.
   * @return true if FK yields same (or similar) pose to ground truth value
   */
  bool checkFK();

  VectorXd theta_; // joint angles
  std::vector<VectorXd> twists_; // twists of joints
  Matrix4d gst_gt_; // ground truth of tool pose
  const Matrix4d gst_init_;
  Matrix4d gd_; // desired gst

  constexpr static const double FINITE_THETA_STEP = 0.01;
  constexpr static const double KD = 10;
  constexpr static const double KO = 10;
};

#endif //JACOBIAN_VISUAL_SERVO_UNCERTAIN_IK_SERVER_HPP
