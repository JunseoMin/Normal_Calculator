#ifndef DATAHANDLER_HPP
#define DATAHANDLER_HPP

#include <vector>
#include <string>
#include <iostream>

#include <Eigen/Core>

#include <pcl/conversions.h>
#include <pcl/io/file_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <opencv2/opencv.hpp>


class DataHandler
{
public:
  DataHandler() = default;
  
  std::vector<std::pair<Eigen::Vector2d, Eigen::Vector2d>> loadBBOX(std::string& filePath);
  std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr>         loadPcd(std::string& filePath);
  Eigen::Matrix4d                                          loadTransform(std::string& filePath);
  Eigen::Matrix3d                                          loadIntrinsic(std::string& filePath);

  void saveCSV();

private:
};


#endif //DATALOADER