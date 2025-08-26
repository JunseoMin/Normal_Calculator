#ifndef DATAHANDLER_HPP
#define DATAHANDLER_HPP

#include <fstream>
#include <sstream>
#include <iostream>
#include <vector>
#include <string>
#include <filesystem>

#include <Eigen/Core>
#include <Eigen/Dense>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

#include <opencv2/opencv.hpp>

namespace fs = std::filesystem;

class DataHandler
{
public:
  DataHandler() = default;
  
  std::vector<std::pair<Eigen::Vector2d, Eigen::Vector2d>> loadBBOX(std::string& filePath);
  std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr>         loadPcd(std::string& filePath);
  Eigen::Matrix4d                                          loadTransform(std::string& filePath);
  Eigen::Matrix3d                                          loadIntrinsic(std::string& filePath);


private:
};

void saveCSV(const std::vector<Eigen::Vector3d>& normals,const std::string& filePath);

#endif //DATALOADER