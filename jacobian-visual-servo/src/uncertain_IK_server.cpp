//
// Created by dcheng on 5/6/20.
//

#include <jacobian_visual_servo/uncertain_IK_server.hpp>
#include <chrono>
#include <unistd.h>
#include <jacobian_visual_servo/kdc_utils.hpp>


UncertainIKServer::UncertainIKServer()
: gst_init_(Matrix4d::Zero())
, gd_(Matrix4d::Zero())
{
}

void UncertainIKServer::setGd(const Matrix4d &gd)
{
  gd_ = gd;
}


bool UncertainIKServer::process()
{
  if (twists_.empty() || !checkFK())
  {
    MatrixXd J;
    finiteMotionJ(J);
    calcTwistFromJ(J);
  }
  if (gd_ == Matrix4d::Zero())
    return false;
  IKStep();
  return diffG(gd_, gst_gt_, true) < 0.01;
}

bool UncertainIKServer::IKStep()
{
  recvRobotStates();

  Quaterniond q_cur(gst_gt_.topLeftCorner<3,3>());
  Quaterniond qd(gd_.topLeftCorner<3,3>());
  Quaterniond q_err = qd * q_cur.inverse();
  Vector3d p_err = gd_.topRightCorner<3,1>() - gst_gt_.topRightCorner<3,1>();

  VectorXd V(6);
  V.head<3>() = KD * p_err;
  Vector3d q_v(q_err.x(), q_err.y(), q_err.z());
  V.tail<3>() = q_v * q_err.w() * KO;

  MatrixXd J = calcJ(twists_, theta_);
  VectorXd dtheta = J.completeOrthogonalDecomposition().pseudoInverse() * V;

  VectorXd theta_cmd = theta_ + dtheta * 0.002;
  sendJointAngles(theta_cmd);

  return true;
}

bool UncertainIKServer::finiteMotionJ(MatrixXd &J)
{
  J.resize(6,6);
  VectorXd theta_prev = theta_;
  Matrix4d gst_prev = gst_gt_;
  for (int i = 0; i < theta_.rows(); i++)
  {
    // set finite motion command
    VectorXd theta_cmd(theta_prev);
    theta_cmd(i) += FINITE_THETA_STEP;
    sendJointAngles(theta_cmd);

    // load up Jacobian
    Matrix4d gdot = gst_gt_ - gst_prev;
    VectorXd Vdt = calcV(gdot, gst_prev);
    J.col(i) = Vdt / FINITE_THETA_STEP;
  }
  sendJointAngles(theta_prev); // bring joints back

  return true;
}

bool UncertainIKServer::calcTwistFromJ(const MatrixXd &J)
{
  if (twists_.size() != theta_.rows())
    twists_.resize(theta_.rows());
  Matrix4d g = Matrix4d::Identity();

  twists_[0] = J.col(0);
  for (int i = 1; i < twists_.size(); i++)
  {
    g *= exp_twist(twists_[i - 1], theta_(i - 1));
    twists_[i] = calcAdj(g).colPivHouseholderQr().solve(J.col(i));
  }
  return true;
}

bool UncertainIKServer::checkFK()
{
  recvRobotStates();

  Matrix4d gst_fk = Matrix4d::Identity();
  for (int i = 0; i < twists_.size(); i++)
  {
    gst_fk *= exp_twist(twists_[i], theta_(i));
  }
  gst_fk *= gst_init_;

  return diffG(gst_fk, gst_gt_) < 0.01;
}

