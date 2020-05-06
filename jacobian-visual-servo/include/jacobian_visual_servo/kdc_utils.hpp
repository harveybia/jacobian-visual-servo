//
// Created by dcheng on 5/6/20.
//

#ifndef JACOBIAN_VISUAL_SERVO_KDC_UTILS_HPP
#define JACOBIAN_VISUAL_SERVO_KDC_UTILS_HPP

#include <iostream>
#include <Eigen/Dense>
#include <vector>

using namespace Eigen;

Matrix4d& exp_twist(const VectorXd& twist, double theta);

MatrixXd& calcAdj(const Matrix4d& g);

Matrix4d& Rt2T(const Matrix3d& R, const Vector3d& t);

Matrix3d& uphat(const Vector3d& v);

Vector3d& downhat(const Matrix3d& vh);

VectorXd& calcV(const Matrix4d& gdot, const Matrix4d& g);

MatrixXd& calcJ(std::vector<VectorXd>& twists, VectorXd& theta);

double diffG(const Matrix4d& g1, const Matrix4d& g2, bool verbose = false);

#endif //JACOBIAN_VISUAL_SERVO_KDC_UTILS_HPP
