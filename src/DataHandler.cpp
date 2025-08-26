#include "tools/DataHandler.hpp"

static long long numeric_stem_or_neg1(const fs::path &p)
{
  std::string stem = p.stem().string();
  if (stem.empty())
    return -1;
  if (!std::all_of(stem.begin(), stem.end(), ::isdigit))
    return -1;
  try
  {
    return std::stoll(stem);
  }
  catch (...)
  {
    return -1;
  }
}

/**
 * @brief Load bounding box from .csv file
 * Format: x1,y1,x2,y2 (per line, one bbox per frame)
 */
std::vector<std::pair<Eigen::Vector2d, Eigen::Vector2d>> DataHandler::loadBBOX(std::string &filePath)
{
  std::vector<std::pair<Eigen::Vector2d, Eigen::Vector2d>> res;
  std::ifstream file(filePath);
  if (!file.is_open())
  {
    std::cerr << "[ERROR] Cannot open bbox file: " << filePath << std::endl;
    return res;
  }

  std::string line;
  while (std::getline(file, line))
  {
    if (line.empty())
      continue;
    std::stringstream ss(line);
    std::string val;
    std::vector<double> coords;

    while (std::getline(ss, val, ','))
      coords.push_back(std::stod(val));

    if (coords.size() == 4)
    {
      Eigen::Vector2d p1(coords[0], coords[1]);
      Eigen::Vector2d p2(coords[2], coords[3]);
      res.emplace_back(p1, p2);
    }
  }
  return res;
}

/**
 * @brief Load point clouds from .pcd file
 * If multiple frames -> filenames like depth_0.pcd, depth_1.pcd ...
 */
std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> DataHandler::loadPcd(std::string &filePath)
{
  std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> res;

  fs::path path(filePath);

  // 디렉터리인 경우: 내부 .pcd 전부 수집 후 숫자 파일명 기준 정렬
  if (fs::is_directory(path))
  {
    std::vector<fs::path> files;
    for (auto &entry : fs::directory_iterator(path))
    {
      if (entry.is_regular_file())
      {
        auto p = entry.path();
        if (p.extension() == ".pcd")
          files.push_back(p);
      }
    }

    if (files.empty())
    {
      std::cerr << "[WARN] No .pcd files found in directory: " << filePath << std::endl;
      return res;
    }

    std::sort(files.begin(), files.end(), [](const fs::path &a, const fs::path &b)
              {
      long long na = numeric_stem_or_neg1(a);
      long long nb = numeric_stem_or_neg1(b);
      if (na >= 0 && nb >= 0)
        return na < nb;                                     
      return a.filename().string() < b.filename().string(); });

    for (const auto &p : files)
    {
      pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
      if (pcl::io::loadPCDFile<pcl::PointXYZ>(p.string(), *cloud) == -1)
      {
        std::cerr << "[ERROR] Could not read PCD file: " << p << std::endl;
        continue;
      }
      res.push_back(cloud);
    }
    std::cout << "[INFO] Loaded " << res.size() << " PCD frames from: " << filePath << std::endl;
    return res;
  }

  if (fs::is_regular_file(path))
  {
    if (path.extension() != ".pcd")
    {
      std::cerr << "[ERROR] Not a .pcd file: " << filePath << std::endl;
      return res;
    }
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    if (pcl::io::loadPCDFile<pcl::PointXYZ>(filePath, *cloud) == -1)
    {
      std::cerr << "[ERROR] Could not read PCD file: " << filePath << std::endl;
    }
    else
    {
      res.push_back(cloud);
      std::cout << "[INFO] Loaded 1 PCD frame: " << filePath << std::endl;
    }
    return res;
  }

  std::cerr << "[ERROR] Path not found: " << filePath << std::endl;
  return res;
}

/**
 * @brief Load 4x4 transform matrix from .csv file
 * Format: 4 lines, each line has 4 comma-separated values
 */
Eigen::Matrix4d DataHandler::loadTransform(std::string &filePath)
{
  Eigen::Matrix4d mat;
  mat.setIdentity();

  std::ifstream file(filePath);
  if (!file.is_open())
  {
    std::cerr << "[ERROR] Cannot open transform file: " << filePath << std::endl;
    return mat;
  }

  std::string line;
  int row = 0;
  while (std::getline(file, line) && row < 4)
  {
    std::stringstream ss(line);
    std::string val;
    int col = 0;
    while (std::getline(ss, val, ',') && col < 4)
    {
      mat(row, col) = std::stod(val);
      col++;
    }
    row++;
  }

  return mat;
}

/**
 * @brief Load 3x3 intrinsic matrix from .csv file
 * Format: 3 lines, each line has 3 comma-separated values
 */
Eigen::Matrix3d DataHandler::loadIntrinsic(std::string &filePath)
{
  Eigen::Matrix3d mat;
  mat.setIdentity();

  std::ifstream file(filePath);
  if (!file.is_open())
  {
    std::cerr << "[ERROR] Cannot open intrinsic file: " << filePath << std::endl;
    return mat;
  }

  std::string line;
  int row = 0;
  while (std::getline(file, line) && row < 3)
  {
    std::stringstream ss(line);
    std::string val;
    int col = 0;
    while (std::getline(ss, val, ',') && col < 3)
    {
      mat(row, col) = std::stod(val);
      col++;
    }
    row++;
  }

  return mat;
}

void saveCSV(const std::vector<Eigen::Vector3d> &normals, const std::string &filePath)
{
  std::ofstream out(filePath);
  if (!out.is_open())
  {
    std::cerr << "[ERROR] Failed to open file: " << filePath << std::endl;
    return;
  }

  out << "nx,ny,nz\n";

  for (const auto &n : normals)
  {
    out << n.x() << "," << n.y() << "," << n.z() << "\n";
  }

  out.close();
  std::cout << "[INFO] Saved " << normals.size() << " normals to " << filePath << std::endl;
}