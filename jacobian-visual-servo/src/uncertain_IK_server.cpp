//
// Created by dcheng on 5/6/20.
//

#include <jacobian_visual_servo/uncertain_IK_server.hpp>

bool UncertainIKServer::recvJointAngles(const VectorXd &theta)
{
  theta_ = theta;
  return true;
}

bool UncertainIKServer::recvGstGT(const Matrix4d& gst)
{
  gst_gt_ = gst;
  return false;
}

bool UncertainIKServer::process()
{
  return false;
}

bool UncertainIKServer::IKStep(VectorXd &theta_des)
{
  return false;
}

bool UncertainIKServer::finiteMotionJ(MatrixXd &J)
{
  return false;
}

bool UncertainIKServer::calcTwistFromJ(const MatrixXd &J)
{
  return false;
}

bool UncertainIKServer::checkFK()
{
  return false;
}
