//
// Created by dcheng on 5/6/20.
//

#include <jacobian_visual_servo/kdc_utils.hpp>

Matrix4d& Rt2T(const Matrix3d& R, const Vector3d& t)
{
  Matrix4d T = Matrix4d::Zero();
  T.block<3,3>(0,0) = R;
  T.block<1,3>(0,3) = t;
  T(3,3) = 1.0;

  return T;
}

Matrix4d& exp_twist(const VectorXd& twist, double theta)
{
  Vector3d v = twist.head<3>();
  Vector3d w = twist.tail<3>();

  if (w == Vector3d::Zero())
    return Rt2T(Matrix3d::Identity(), v * theta);
  else
  {
    AngleAxisd exp_coord(theta, w);
    Matrix3d R = exp_coord.toRotationMatrix();
    return Rt2T(R, (Matrix3d::Identity() - R) * w.cross(v) + w * w.dot(v) * theta);
  }
}

MatrixXd& calcAdj(const Matrix4d& g)
{
  MatrixXd adg(6);
  Matrix3d R = g.block<3,3>(0,0);
  Vector3d p = g.block<1,3>(0,3);
  adg.topLeftCorner<3,3>() = R;
  adg.topRightCorner<3,3>() = uphat(p) * R;
  adg.bottomLeftCorner<3,3>().fill(0.);
  adg.bottomRightCorner<3,3>() = R;
  return adg;
}

Matrix3d& uphat(const Vector3d& v)
{
  Matrix3d vh;
  vh << 0    , -v(2), v(1),
        v(2) , 0    , -v(0),
        -v(1), v(0) , 0;
  return vh;
}

MatrixXd& calcJ(std::vector<VectorXd>& twists, VectorXd& theta)
{
  int n = twists.size();
  Matrix4d g = Matrix4d::Identity();
  MatrixXd J(6);

  for (int i = 0; i < n; i++)
  {
    J.col(i) = calcAdj(g) * twists[i];
    g = g * exp_twist(twists[i], theta(i));
  }

  return J;
}

Vector3d& downhat(const Matrix3d& vh)
{
  Vector3d v((vh(2,1) - vh(1,2)) / 2,
             (vh(0,2) - vh(2,0)) / 2,
             (vh(1,0) - vh(0,1)) / 2);
  return v;
}

VectorXd& calcV(const Matrix4d& gdot, const Matrix4d& g)
{
  VectorXd V(6);

  Matrix3d R = g.topLeftCorner<3,3>();
  Matrix3d Rdot = gdot.topLeftCorner<3,3>();
  Vector3d p = g.topRightCorner<3,1>();
  Vector3d pdot = gdot.topRightCorner<3,1>();

  V.head<3>() = -Rdot * R.transpose() * p + pdot;
  V.tail<3>() = downhat(Rdot * R.transpose());

  return V;
}

double diffG(const Matrix4d& g1, const Matrix4d& g2, bool verbose)
{
  double dp_norm = (g1.topRightCorner<3,1>() - g2.topRightCorner<3,1>()).norm();
  Matrix3d dR = g1.topLeftCorner<3,3>() * g2.topLeftCorner<3,3>().transpose();
  double d_angle = acos((dR.trace() - 1) / 2);

  if (verbose)
  {
    std::cout << "Trans err: rot " << d_angle / M_PI * 180 << " degrees, "
              << "t " << (g1.topRightCorner<3,1>() - g2.topRightCorner<3,1>()).transpose()
              << std::endl;
  }

  return d_angle + dp_norm;
}