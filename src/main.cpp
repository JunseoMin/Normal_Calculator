#include "NormalCalculator/NormalCalculator.hpp"
#include "tools/mathtools.hpp"
#include "tools/DataHandler.hpp"

#include <iostream>
#include <vector>
#include <string>
#include <Eigen/Core>

int main(int argc, char *argv[])
{
  if (argc < 5)
  {
    std::cerr << "Useage: " << argv[0] << " <path/to/pcd> <path/to/bbox> <path/to/extrinsic> <path/to/intrinsic> <path/to/output.csv>\n";
    return 1;
  }
  std::vector<std::string> paths;
  for (int i = 1; i < argc; ++i)
  {
    paths.push_back(argv[i]);
  }

  NormalCalculator calc(paths[0], paths[1], paths[2], paths[3]);
  std::vector<Eigen::Vector3d> normals;
  normals = calc.calc();
  saveCSV(normals,paths[4]);
  
  return 0;
}


