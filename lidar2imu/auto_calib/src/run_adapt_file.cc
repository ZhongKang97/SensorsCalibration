#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/src/Geometry/Quaternion.h>
#include <Eigen/src/Geometry/Transform.h>
#include <boost/utility.hpp>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <json/json.h>
#include <math.h>
#include <ostream>
#include <string>
#include <utility>
#include <vector>
using namespace std;

int main(int argc, char **argv) {
  if (argc != 5) {
    cout << "Usage: ./run_adapt_file <input_file_> <lidar_timestamp_file> <lidar_timestamp_file_fullname> "
            "<output_json_file_path> "
            "\nexample:\n\t"
            "./bin/run_adapt_file mydata/gps_pose_linzk.txt "
            "mydata/lidar_timestamp.txt "
            "mydata/lidar_timestamp_fullname.txt "
            "mydata/gnss-pose-lidar-time.txt"
         << endl;
    return 0;
  }
  const std::string input_file_ = argv[1];
  string lidarTimeFile = argv[2];
  string lidarTimeFile_fullname = argv[3];
  string output_file_ = argv[4];
  std::ifstream file(input_file_);
  std::ifstream file_lidarTime(lidarTimeFile);
  std::ifstream file_lidarTime_fullname(lidarTimeFile_fullname);
  if (!file.is_open() && !file_lidarTime.is_open() &&
      !file_lidarTime_fullname.is_open()) {
    std::cout << "ERROR--->>> cannot open: " << input_file_ << " "
              << lidarTimeFile << "" << lidarTimeFile_fullname << std::endl;
    exit(1);
  }
  std::string line;
  std::string line_lidarTime;
  std::string line_lidarTime_fullname;
  std::deque<string> lidarTimeFullNameSet;
  std::deque<double> lidarTimeSet;
  std::deque<double> gnssTimeSet;
  std::deque<double> synced_time;
  std::deque<string> synced_timeSTR;
  std::deque<std::pair<Eigen::Quaterniond, Eigen::Vector3d>> transfromSet;
  std::deque<std::pair<Eigen::Quaterniond, Eigen::Vector3d>>
      synced_transfromSet;
  while (getline(file_lidarTime, line_lidarTime)) {
    std::stringstream ss(line_lidarTime);
    double lidarTime;
    std::string lidarTime_STR;
    ss >> lidarTime_STR;
    lidarTime = stod(lidarTime_STR.substr(7));
    lidarTimeSet.emplace_back(lidarTime);
  }
  while (getline(file_lidarTime_fullname, line_lidarTime_fullname)) {
    std::stringstream ss(line_lidarTime_fullname);
    std::string lidarTime_STR;
    ss >> lidarTime_STR;
    lidarTimeFullNameSet.emplace_back(lidarTime_STR);
  }
  while (getline(file, line)) {
    std::stringstream ss(line);
    double gnssTime;
    std::string gnssTime_STR;
    ss >> gnssTime_STR;
    gnssTime = stod(gnssTime_STR.substr(7));
    gnssTimeSet.emplace_back(gnssTime);
    double q_x, q_y, q_z, q_w;
    double p_x, p_y, p_z;
    ss >> q_x >> q_y >> q_z >> q_w;
    ss >> p_x >> p_y >> p_z;
    Eigen::Quaterniond quatern(q_w, q_x, q_y, q_z);
    quatern.normalize();
    Eigen::Vector3d pose(p_x, p_y, p_z);
    transfromSet.emplace_back(quatern, pose);
  }
  file_lidarTime.close();
  file.close();
  cout << "lidarTimeSet size:" << lidarTimeSet.size() << std::endl;
  cout << "gnssTimeSet size:" << gnssTimeSet.size() << std::endl;
  while (gnssTimeSet.size() >= 2) {
    if (gnssTimeSet.front() > lidarTimeSet.front()) {
      if (lidarTimeSet.size() >= 2) {
        if (lidarTimeSet.at(1) - lidarTimeSet.front() < 0.2 ||
            gnssTimeSet.at(1) - gnssTimeSet.front() > 0.2) {
          lidarTimeSet.pop_front();
          lidarTimeFullNameSet.pop_front();
        }
      } else {
        gnssTimeSet.pop_front();
        transfromSet.pop_front();
      }
      continue;
    }
    if (gnssTimeSet.at(1) < lidarTimeSet.front()) {
      gnssTimeSet.pop_front();
      transfromSet.pop_front();
      continue;
    }
    // lidar time at the middle
    double lidarTime = lidarTimeSet.front();
    string lidarTimeSTR = lidarTimeFullNameSet.front();
    double gnssTime_before = gnssTimeSet.front();
    double gnssTime_after = gnssTimeSet.at(1);
    cout << "syncing time:[ " << gnssTime_before << "] "
         << "[" << lidarTime << "] "
         << "[" << gnssTime_after << "]" << std::endl;
    double fontScale =
        (gnssTime_after - lidarTime) / (gnssTime_after - gnssTime_before);
    double backScale =
        (lidarTime - gnssTime_before) / (gnssTime_after - gnssTime_before);
    Eigen::Quaterniond quatern_before = transfromSet.front().first;
    Eigen::Vector3d pose_before = transfromSet.front().second;
    Eigen::Quaterniond quatern_after = transfromSet.at(1).first;
    Eigen::Vector3d pose_after = transfromSet.at(1).second;
    Eigen::Quaterniond quatern_lidar_time =
        quatern_before.slerp(backScale, quatern_after);
    quatern_lidar_time.normalize();
    Eigen::Vector3d pose_lidar_time =
        pose_before * fontScale + pose_after * backScale;
    synced_time.emplace_back(lidarTime);
    synced_timeSTR.emplace_back(lidarTimeSTR);
    synced_transfromSet.emplace_back(quatern_lidar_time, pose_lidar_time);
    gnssTimeSet.pop_front();
    lidarTimeSet.pop_front();
    lidarTimeFullNameSet.pop_front();
    transfromSet.pop_front();
  }
  cout << "synced_transfromSet size:" << synced_transfromSet.size()
       << std::endl;
  for (auto time_now : synced_time)
    cout << "synced time: " << fixed << setprecision(5) << time_now
         << std::endl;

  std::ofstream fCalib(output_file_);
  for (auto t : synced_timeSTR) {
    Eigen::Matrix4d TWi = Eigen::Matrix4d::Identity();
    Eigen::Quaterniond quatern_t = synced_transfromSet.front().first;
    Eigen::Vector3d pose_t = synced_transfromSet.front().second;
    TWi.block(0, 0, 3, 3) = quatern_t.matrix();
    TWi.block(0, 3, 3, 1) = pose_t;

    fCalib << t << " " << TWi(0, 0) << " " << TWi(0, 1) << " " << TWi(0, 2)
           << " " << TWi(0, 3) << " " << TWi(1, 0) << " " << TWi(1, 1) << " "
           << TWi(1, 2) << " " << TWi(1, 3) << " " << TWi(2, 0) << " "
           << TWi(2, 1) << " " << TWi(2, 2) << " " << TWi(2, 3) << std::endl;
    synced_transfromSet.pop_front();
  }
  fCalib.close();
  cout << "synced_gnss pose lidar time file:" << output_file_ << std::endl;
  return 1;
}