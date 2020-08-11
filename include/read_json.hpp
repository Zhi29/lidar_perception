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
  CHECK(static_cast<int>(json_array.size()) == eigen_matrix_ptr->size())
    << json_array.size() << " " << eigen_matrix_ptr->size();
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

struct LidarPerception {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    LidarPerception(std::string const &perception_path)
        : LidarPerception(tools::ReadFileToJson(perception_path)) {}
    LidarPerception(nlohmann::json const &j){
        nlohmann::json children = j["children"];
        std::cout << children[0]["data"]["isKeyCube"][0] << std::endl;
        
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
            //lidar.type = children[i]["data"]["type"][0];

            std::cout << lidar.type <<" "<< lidar.trackid<<std::endl;

            perception_.insert({lidar.trackid, lidar});
        }
        
    }
    std::unordered_map<std::string, Lidar> perception_;
    bool isKeyCube;
    bool isKeyPropertyCube;
    std::string type;
    double height, length, width;
    double pitch, roll, yaw;
    double x, y, z;
    std::string tag, uuid;
    double score;
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