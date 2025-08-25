#include "NormalCalculator/NormalCalculator.hpp"

/**
 * @brief Load pcd and depth datas from filepath
 * @param depthPath path/to/depth.pcd
 * @param bboxPath path/to/boundingbox.csv
 * @param transformPath path/to/Transform.csv (in SO(3) reference to camera frame)
 * @param rgbIntrinsicPath path/to/rgbIntrinsic.csv
 */
NormalCalculator::NormalCalculator(std::string &depthPath,
                                   std::string &bboxPath,
                                   std::string &transformPath,
                                   std::string &rgbIntrinsicPath)
    : _dh()
{
  std::cout << "[INFO] Calculator init -----\n";

  this->_bboxs     = _dh.loadBBOX(bboxPath);
  this->_depthPcds = _dh.loadPcd(depthPath);
  this->_Trd       = _dh.loadTransform(transformPath);
  this->_Irgb      = _dh.loadIntrinsic(rgbIntrinsicPath);

  std::cout << "[INFO] Loaded datas --------\n";
  std::cout << "[INFO] TOTAL: " << _bboxs.size() << "frames----\n";
}

void NormalCalculator::calc(void)
{
  assert(_bboxs.size() == _depthPcds.size());
  std::cout << "[INFO] Calculation start ---";

  Eigen::Matrix4d Tdr = _Trd.inverse();

  for (int i = 0; i < _depthPcds.size(); i++)
  {
    Eigen::Vector3d normal;
    std::pair<Eigen::Vector2d, Eigen::Vector2d> roi;
    roi = _bboxs[i];

    pcl::PointCloud<pcl::PointXYZ>::Ptr depthCloud = _depthPcds[i];
    pcl::PointCloud<pcl::PointXYZ>::Ptr rgbDepthCloud;

    rgbDepthCloud = _perspectivePcd(depthCloud, _Trd); // Convert depth cloud reference to camera frame

    std::vector<Eigen::Vector3d> roiPcd;

    roiPcd = _getPointRoi(roi, rgbDepthCloud);
    normal = _calcNormal(roiPcd);
    _normals.emplace_back(normal);
  }
}


/**
 * @brief Get points from the ROI
 */
std::vector<Eigen::Vector3d> NormalCalculator::_getPointRoi(std::pair<Eigen::Vector2d, Eigen::Vector2d> roi,
                                                            pcl::PointCloud<pcl::PointXYZ>::Ptr rgbDepthCloud)
{
  std::vector<Eigen::Vector3d> res;

  double fx = _Irgb(0, 0);
  double fy = _Irgb(1, 1);
  double u0 = _Irgb(0, 2);
  double v0 = _Irgb(1, 2);

  for (const auto &point : *rgbDepthCloud)
  {
    double x1 = roi.first.x();
    double x2 = roi.second.x();
    double y1 = roi.first.y();
    double y2 = roi.second.y();

    double px = point.x;
    double py = point.y;
    double pz = point.z;

    double pu = (fx * px) / pz + u0;
    double pv = (fy * py) / pz + v0;

    if (x1 <= pu && pu <= x2 && y1 <= pv && pv <= y2) // If transformed pcd is in the ROI from YOLO
    {
      Eigen::Vector3d tmp;
      tmp << px, py, pz;
      res.emplace_back(tmp);
    }
  }

  return res;
}
