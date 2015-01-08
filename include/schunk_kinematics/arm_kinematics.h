#ifndef ARM_KINEMATICS_H
#define ARM_KINEMATICS_H

#include <iostream>
#include <cmath>
#include <ctime>
#include <cstdlib>
#include <cstddef>
#include <cassert>
#include <vector>
#include <Eigen/Dense>

Eigen::MatrixXd forward_kinematics(const Eigen::VectorXd& seta, const Eigen::VectorXd& Linklen);

Eigen::VectorXd& inverse_kinematics(Eigen::VectorXd& seta, const Eigen::VectorXd& Linklen,
                                         const Eigen::MatrixXd& Goal, bool random = true, const double user_theta = 0);

inline double triangle(const double a, const double b, const double c)
{
    double angle;
    angle = acos((b*b+a*a-c*c)/(2*a*b));
    return angle;
}

void minimum_energy(Eigen::VectorXd& seta, const Eigen::VectorXd& beta);

double angleRotation(const Eigen::VectorXd& seta, const Eigen::VectorXd& Linklen);

Eigen::VectorXd zyzAngle(const Eigen::MatrixXd& init_matrix, const Eigen::MatrixXd& goal_matrix);

Eigen::MatrixXd zyzRotation(const Eigen::VectorXd& angle);

Eigen::MatrixXd lineInterplot(const Eigen::MatrixXd& init_matrix, const Eigen::MatrixXd& goal_matrix, double run_time, double& duration_time);

Eigen::MatrixXd circleInterplot(const Eigen::MatrixXd& init_matrix, const Eigen::MatrixXd& goal_matrix,
                        const Eigen::VectorXd& center, double run_time, double& duration_time, bool clockwise = true);

double selfMotionTime(const double run_time, double& duration_time, const double init_angle, const double angle, bool clockwise);

Eigen::VectorXd selfMotionInterplot(const Eigen::MatrixXd& init_matrix, const Eigen::VectorXd& last, const Eigen::VectorXd& Linklen, const double current_angle);

Eigen::MatrixXd homoMatrix(const double theta, const double d, const double alpha, const double a);

void getJacobi(Eigen::MatrixXd& J, const Eigen::VectorXd& seta);

#endif
