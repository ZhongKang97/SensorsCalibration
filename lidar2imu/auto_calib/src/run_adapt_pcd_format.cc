#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/src/Geometry/Quaternion.h>
#include <Eigen/src/Geometry/Transform.h>
#include <boost/utility.hpp>
#include <cmath>
#include <cstddef>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <json/json.h>
#include <math.h>
#include <ostream>
#include <pcl/common/transforms.h>
#include <pcl/conversions.h>
#include <pcl/impl/point_types.hpp>
#include <pcl/io/pcd_io.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <set>
#include <string>
#include <utility>
#include <vector>
using namespace std;

#define PI 3.1415926
const float FOV_UP = 15.0;
const float FOV_DOWN = -16.0;
const int LINES = 32;

struct LidarPointXYZIRT {
  PCL_ADD_POINT4D;
  float intensity;
  uint16_t ring;
  double timestamp;
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;

POINT_CLOUD_REGISTER_POINT_STRUCT(
    LidarPointXYZIRT,
    (float, x, x)(float, y, y)(float, z, z)(float, intensity, intensity)(
        uint16_t, ring, ring)(double, timestamp, timestamp))

int main(int argc, char **argv) {
  if (argc != 4) {
    cout << "Usage: ./run_adapt_pcd_format <pcd_file_dir> "
            "<pcd_filename_text_file>"
            "<output_formated_file_dir>"
            "\nexample:\n\t"
            "./bin/run_adapt_pcd_format mydata/hesai_lidar_file/ "
            "mydata/lidar_timestamp_fullname.txt "
            "mydata/hesai_lidar_file_with_ring/"
         << endl;
    return 0;
  }
  std::string pcd_file_dir_ = argv[1];
  std::string pcd_name_file_ = argv[2];
  std::string output_dir = argv[3];
  if (pcd_file_dir_.rfind('/') != pcd_file_dir_.size() - 1)
    pcd_file_dir_ = pcd_file_dir_ + "/";
  if (output_dir.rfind('/') != output_dir.size() - 1)
    output_dir = output_dir + "/";
  std::ifstream file(pcd_name_file_);
  if (!file.is_open()) {
    std::cout << "ERROR--->>> cannot open: " << pcd_name_file_ << std::endl;
    exit(1);
  }
  std::string line;
  std::deque<string> lidarTimeFullNameSet;
  while (getline(file, line)) {
    std::stringstream ss(line);
    std::string lidarTime_STR;
    ss >> lidarTime_STR;
    lidarTimeFullNameSet.emplace_back(lidarTime_STR);
  }
  file.close();
  int num_total_file = lidarTimeFullNameSet.size();
  int now_convert = 0;
  cout << "lidarTimeFullNameSet size:" << lidarTimeFullNameSet.size()
       << std::endl;
  cout << setiosflags(ios::fixed) << setprecision(6);
  cout << "\n\n\n";
  bool demoOutput = true;
  for (auto filename : lidarTimeFullNameSet) {
    string full_path_filename = pcd_file_dir_ + filename + ".pcd";
    string output_filename = output_dir + filename + ".pcd";
    pcl::PointCloud<pcl::PointXYZ>::Ptr inputCloud(
        new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<LidarPointXYZIRT>::Ptr outputCloud(
        new pcl::PointCloud<LidarPointXYZIRT>);
    if (pcl::io::loadPCDFile(full_path_filename, *inputCloud) < 0) {
      std::cout << "cannot open pcd_file: " << full_path_filename << "\n";
      exit(1);
    }
    outputCloud->width = inputCloud->height;
    outputCloud->height = inputCloud->width;
    outputCloud->points.resize(outputCloud->width * outputCloud->height);
    cout << "\33[3A";
    cout << "Input PointCloud:\t" << full_path_filename << std::endl;
    size_t index = 0;
    for (auto point : inputCloud->points) {
      float now_x = point.x;
      float now_y = point.y;
      float now_z = point.z;
      outputCloud->points[index].x = now_x;
      outputCloud->points[index].y = now_y;
      outputCloud->points[index].z = now_z;
      float distance = sqrt(now_x * now_x + now_y * now_y + now_z * now_z);
      float cosValue = now_z / distance;
      float pitch = asin(cosValue);
      float fov_down = (FOV_DOWN / 180.0) * PI;
      float fov = ((abs(FOV_DOWN) + abs(FOV_UP)) / 180.0) * PI;
      float proj_y = (pitch + abs(fov_down)) / fov;
      proj_y *= LINES;
      proj_y = floor(proj_y);
      proj_y = min<int>(LINES - 1, proj_y);
      proj_y = max<int>(0, proj_y);
      int ring = int(proj_y);
      outputCloud->points[index].ring = ring;
      index++;
    }
    cout << "Output PointCloud:\t" << output_filename << std::endl;
    now_convert++;
    float progress = float(now_convert) / float(num_total_file);
    if (progress > 1) {
      progress = 1;
    }
    int pa = progress * 50;
    std::cout << "[" + std::string(pa, '=') + ">" + std::string(50 - pa, ' ')
              << "]  " << progress * 100 << "%\n";
    fflush(stdout);
    if (demoOutput) {
      demoOutput = false;
      pcl::io::savePCDFileASCII<LidarPointXYZIRT>("demo_ring_pcd.pcd",
                                                  *outputCloud);
    }
    pcl::io::savePCDFileBinaryCompressed<LidarPointXYZIRT>(output_filename,
                                                           *outputCloud);
  }
  // cout << pcd_file_dir_ << " " << pcd_name_file_ << endl;
  return 1;
}