#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/src/Geometry/Quaternion.h>
#include <Eigen/src/Geometry/Transform.h>
#include <GeographicLib/LocalCartesian.hpp>
#include <boost/utility.hpp>
#include <cstddef>
#include <cstdio>
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

#define PI 3.14159265358979

int main(int argc, char **argv) {
  if (argc != 7) {
    cout << "Usage: ./run_adapt_file <input_file_> <lidar_timestamp_file> "
            "<lidar_timestamp_file_fullname> <skip_data_num>"
            "<output_raw2ENU_file_path> <output_synced_file_path> "
            "\nexample:\n\t"
            "./bin/run_adapt_file mydata/gps_pose_rawdata.txt "
            "mydata/lidar_timestamp.txt "
            "mydata/lidar_timestamp_fullname.txt "
            "mydata/gps_pose_raw2ENU.txt "
            "mydata/gnss-pose-lidar-time-new.txt 5"
         << endl;
    return 0;
  }
  const std::string input_file_ = argv[1];
  string lidarTimeFile = argv[2];
  string lidarTimeFile_fullname = argv[3];
  string output_file_raw2ENU = argv[4];
  string output_file_ = argv[5];
  const int skip_data = std::stoi(argv[6]);
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
  std::deque<string> gnssTimeSTRContainer;
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
  bool startPoint = true;
  GeographicLib::LocalCartesian geo_converter;
  while (getline(file, line)) {
    std::stringstream ss(line);
    double gnssTime;
    std::string gnssTime_STR;
    ss >> gnssTime_STR;
    gnssTimeSTRContainer.emplace_back(gnssTime_STR);
    gnssTime = stod(gnssTime_STR.substr(7));
    gnssTimeSet.emplace_back(gnssTime);
    double latitude, longitude, locatheight;
    double yaw, pitch, roll;
    ss >> latitude >> longitude >> locatheight;
    ss >> yaw >> pitch >> roll;
    yaw = yaw * PI / 180.0;
    pitch = pitch * PI / 180.0;
    roll = roll * PI / 180.0;
    double p_x, p_y, p_z;
    if (startPoint) {
      geo_converter.Reset(latitude, longitude, locatheight);
      p_x = 0;
      p_y = 0;
      p_z = 0;
      startPoint = false;
    } else {
      geo_converter.Forward(latitude, longitude, locatheight, p_x, p_y, p_z);
    }
    printf("Converting WGS:\t[%.6f,%.6f,%.6f] to ENU:\t[%.6f,%.6f,%.6f]\n",
           latitude, longitude, locatheight, p_x, p_y, p_z);
    Eigen::AngleAxisd rollAngle(
        Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitX()));
    Eigen::AngleAxisd pitchAngle(
        Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitY()));
    Eigen::AngleAxisd yawAngle(
        Eigen::AngleAxisd(-yaw, Eigen::Vector3d::UnitZ()));
         Eigen::Matrix3d rotation_matrix;
    rotation_matrix = yawAngle * pitchAngle * rollAngle;
    Eigen::Quaterniond quatern(rotation_matrix);
    quatern.normalize();
    Eigen::Vector3d pose(p_x, p_y, p_z); // twi
    transfromSet.emplace_back(quatern, pose);
  }
  file_lidarTime.close();
  file.close();
  cout << "lidarTimeSet size:" << lidarTimeSet.size() << std::endl;
  cout << "gnssTimeSet size:" << gnssTimeSet.size() << std::endl;

  std::ofstream raw2ENU(output_file_raw2ENU);
  size_t index = 0;
  for (auto t : gnssTimeSTRContainer) {
    Eigen::Quaterniond quatern_now = transfromSet.at(index).first;
    Eigen::Vector3d pose_now = transfromSet.at(index).second;
    raw2ENU << t << " " << quatern_now.x() << " " << quatern_now.y() << " "
            << quatern_now.z() << " " << quatern_now.w() << " " << pose_now.x()
            << " " << pose_now.y() << " " << pose_now.z() << std::endl;
    index++;
  }

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
  size_t now_index = 0;
  size_t save_count = 0;
  for (auto t : synced_timeSTR) {
    if (now_index % skip_data == 0) {
      Eigen::Matrix4d TWi = Eigen::Matrix4d::Identity();
      Eigen::Quaterniond quatern_t = synced_transfromSet.front().first;
      Eigen::Vector3d pose_t = synced_transfromSet.front().second;
      TWi.block(0, 0, 3, 3) = quatern_t.matrix();
      TWi.block(0, 3, 3, 1) = pose_t;

      fCalib << t << " " << TWi(0, 0) << " " << TWi(0, 1) << " " << TWi(0, 2)
             << " " << TWi(0, 3) << " " << TWi(1, 0) << " " << TWi(1, 1) << " "
             << TWi(1, 2) << " " << TWi(1, 3) << " " << TWi(2, 0) << " "
             << TWi(2, 1) << " " << TWi(2, 2) << " " << TWi(2, 3) << std::endl;
             save_count++;
    }
    now_index++;
    synced_transfromSet.pop_front();
  }
  fCalib.close();
  cout << "synced_gnss pose saved_count:" << save_count << std::endl;
  cout << "synced_gnss pose lidar time file:" << output_file_ << std::endl;
  return 1;
}