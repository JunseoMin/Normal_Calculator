#ifndef MATHTOOLS_HPP
#define MATHTOOLS_HPP

#include <vector>

#include <Eigen/Dense>
#include <Eigen/Core>

Eigen::MatrixXd vec2mat(const std::vector<Eigen::Vector3d>& points);
Eigen::MatrixXd calcCovMat(const Eigen::MatrixXd& data);

#endif // MATHTOOLS_HPP