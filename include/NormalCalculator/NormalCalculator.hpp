#ifndef NORMALCALCULATOR_HPP
#define NORMALCALCULATOR_HPP

#include <math.h>
#include <string.h>
#include <vector>

#include <Eigen/Dense>
#include <Eigen/Core>

#include <pcl/conversions.h>
#include <pcl/io/file_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <opencv2/opencv.hpp>

#include "tools/DataHandler.hpp"

class NormalCalculator
{
public:
  NormalCalculator( std::string &registeredDepth,
                    std::string &bboxPath,
                    std::string &transformPath,
                    std::string &rgbIntrinsicPath);
  void calc(void);
  
private:
  pcl::PointCloud<pcl::PointXYZ>::Ptr _perspectivePcd(pcl::PointCloud<pcl::PointXYZ>::Ptr pcd,
                                                      Eigen::Matrix4d Transform);
  std::vector<Eigen::Vector3d>        _getPointRoi(std::pair<Eigen::Vector2d, Eigen::Vector2d> roi,
                                                   pcl::PointCloud<pcl::PointXYZ>::Ptr rgbDepthCloud);
  Eigen::Vector3d                     _calcNormal(std::vector<Eigen::Vector3d>& roiPcd);
  

  DataHandler _dh;  // Data loader&saver class
  
  // Params for normal calculation
  std::vector<std::pair<Eigen::Vector2d, Eigen::Vector2d>> _bboxs;
  std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr>         _depthPcds;
    
  Eigen::Matrix3d _Irgb;  // rgb intrinsic
  Eigen::Matrix3d _Id;    // depth intrinsic
  Eigen::Matrix4d _Trd;   // depth camera's position reference to rgb camera

  std::vector<Eigen::Vector3d>    _normals;
};

#endif // NORMALCALCULATOR_HPP