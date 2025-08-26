#include "tools/mathtools.hpp"

/**
 * @brief Convert vector<Eigen::vector> to Eigen::MatrixXd type
 */
Eigen::MatrixXd vec2mat(const std::vector<Eigen::Vector3d>& points){
  if(points.empty()){
    throw std::invalid_argument("[Error] vec2mat: Invalid points");
  }
  assert(!points.empty());

  Eigen::MatrixXd mat( points.size(), 3);
  for (size_t i = 0; i < points.size(); i++)
  {
    mat.row(i) = points[i].transpose();
  }
  
  return mat;
}

/**
 * @brief Calc Covariance matrix form data
 */
Eigen::MatrixXd calcCovMat(const Eigen::MatrixXd& data){
  if (data.rows() == 0)
  {
    throw std::invalid_argument("[Error] calcCovMat: Invalid matrix data");
  }

  Eigen::RowVectorXd mean = data.colwise().mean();

  Eigen::MatrixXd centered = data.rowwise() - mean;

  Eigen::MatrixXd cov = (centered.adjoint()*centered) / static_cast<double>(data.rows() - 1);

  return cov;
}