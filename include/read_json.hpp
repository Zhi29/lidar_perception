#ifndef READ_JSON_HPP_
#define READ_JSON_HPP_

#include <iostream>
#include "json.hpp"
#include <vector>
#include <unordered_map>
#include <fstream>
#include <perception_structure.hpp>

namespace tools {
inline std::string ReadFileToString(const std::string &file_path) {
  std::ifstream fin(file_path);
  return std::string((std::istreambuf_iterator<char>(fin)),
                     std::istreambuf_iterator<char>());
}

inline nlohmann::json ReadFileToJson(const std::string &file_path) {
  std::string str = ReadFileToString(file_path);
  return nlohmann::json::parse(str);
}

template <typename Derived>
inline void JsonArrayToEigenMatrix(const nlohmann::json &json_array,
                                   Eigen::MatrixBase<Derived> *eigen_matrix_ptr) {
  //CHECK(static_cast<int>(json_array.size()) == eigen_matrix_ptr->size())
  //  << json_array.size() << " " << eigen_matrix_ptr->size();
  // TODO: change to more effective implementation
  size_t rows = eigen_matrix_ptr->rows();
  size_t cols = eigen_matrix_ptr->cols();
  for (size_t r = 0; r < rows; ++r) {
    for (size_t c = 0; c < cols; ++c) {
      (*eigen_matrix_ptr)(r, c) = json_array[r * cols + c];
    }
  }
  return;
}

} // namespace tools

struct CameraParameters {
  CameraParameters(std::string const &cam_para_path, SensorType camera_type)
    : cam_para_path(cam_para_path), camera_type(camera_type) {
      std::cout << cam_para_path << std::endl;
    nlohmann::json j = tools::ReadFileToJson(cam_para_path);
    cam_to_front = j["cam_to_front"];
    cam_to_left = j["cam_to_left"];
    cam_to_right = j["cam_to_right"];
    cam_to_back = j["cam_to_back"];
    camera_height = j["camera_height"]; 

    translation = {-cam_to_front, (cam_to_right-cam_to_left)/2, camera_height};
    tools::JsonArrayToEigenMatrix(j["camera_matrix"], &camera_intrinsics);
    tools::JsonArrayToEigenMatrix(j["rotation_matrix"], &rotation_matrix);
    distortions.resize(4,1);
    tools::JsonArrayToEigenMatrix(j["distortion_coefficients"], &distortions);
  }

  std::string cam_para_path;
  double cam_to_back, cam_to_front, cam_to_left, cam_to_right, camera_height;
  Eigen::Matrix3d rotation_matrix, camera_intrinsics;
  Eigen::Vector3d translation;
  Eigen::MatrixXd distortions;
  SensorType camera_type;
};

struct LidarPerception {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  LidarPerception(std::string const &perception_path)
      : LidarPerception(tools::ReadFileToJson(perception_path)) {}

  LidarPerception(nlohmann::json const &j){
      nlohmann::json children = j["children"];
      //std::cout << "Is children empty? " << children.empty() << std::endl;
      if(children.empty()){
          Lidar lidar;
          lidar.height = -1;
          lidar.width = -1;
          lidar.length = -1;
          lidar.x = -1;
          lidar.y = -1;
          lidar.z = -1;
          lidar.pitch = -1;
          lidar.roll = -1;
          lidar.yaw = -1;            
          lidar.trackid = -1;
          perception_.insert({lidar.trackid, lidar});
      }
      else{
        for(int i = 0; i < children.size(); i++){
            Lidar lidar;
            lidar.height = children[i]["height"];
            lidar.width = children[i]["width"];
            lidar.length = children[i]["length"];
            lidar.x = children[i]["x"];
            lidar.y = children[i]["y"];
            lidar.z = children[i]["z"];
            lidar.pitch = children[i]["pitch"];
            lidar.roll = children[i]["roll"];
            lidar.yaw = children[i]["yaw"];
            lidar.tag = children[i]["tag"];
            lidar.trackid = children[i]["uuid"];
            lidar.score = children[i]["score"];

            //lidar.isKeyCube = children[i]["data"]["isKeyCube"][0];
            //lidar.isKeyPropertyCube = children[i]["data"]["isKeyPropertyCube"][0];
            lidar.type = children[i]["data"]["type"][0];

            //std::cout << lidar.type <<" "<< lidar.trackid << std::endl;

            perception_.insert({lidar.trackid, lidar});
        }
      }
  }
  std::unordered_map<long int, Lidar> perception_;
  double timestamp = 0;
};


struct EgoMotionReader{
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    EgoMotionReader(std::string const &egomotion_path)
        : EgoMotionReader(tools::ReadFileToJson(egomotion_path)) {}
    EgoMotionReader(nlohmann::json const &j){
        acc(0) = j["acceleration"]["x"];
        acc(1) = j["acceleration"]["y"];
        acc(2) = j["acceleration"]["z"];
        angular_vel(0) = j["angular_velocity"]["x"];
        angular_vel(1) = j["angular_velocity"]["y"];
        angular_vel(2) = j["angular_velocity"]["z"];
        pos(0) = j["position"]["x"];
        pos(1) = j["position"]["y"];
        pos(2) = j["position"]["z"];
        q.w() = j["quaternion"]["w"];
        q.x() = j["quaternion"]["x"];
        q.y() = j["quaternion"]["y"];
        q.z() = j["quaternion"]["z"];
        vel(0) = j["velocity"]["x"];
        vel(1) = j["velocity"]["y"];
        vel(2) = j["velocity"]["z"];
        timestamp = j["timestamp"];
    }

    Eigen::Vector3d acc;
    Eigen::Vector3d angular_vel;
    Eigen::Vector3d pos;
    Eigen::Quaterniond q;
    Eigen::Vector3d vel;
    double timestamp;
};

#endif