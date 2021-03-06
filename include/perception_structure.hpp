#ifndef PERCEPTION_STRUCTURE_HPP_
#define PERCEPTION_STRUCTURE_HPP_

#include <string>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Eigen>

struct Lidar{
    double height, length, width;
    double roll, pitch, yaw;
    double x, y, z;
    double score;
    long int trackid;
    std::string tag;
    bool isKeyCube;
    bool isKeyPropertyCube;
    std::string type;

    Lidar lidar(){}
};

enum class SensorType : int {
    FISH_EYE_F = 1,
    FISH_EYE_B = 2,
    FISH_EYE_L = 3,
    FISH_EYE_R = 4,
    LIDAR = 5
};

struct SensorTimes{
    double time;
    SensorType type;
    SensorTimes(double time, SensorType type) : time(time), type(type){}
};

class LESS_T0
{
public:
  bool operator()(const SensorTimes left, const SensorTimes right) const
  {
    return left.time < right.time;
  } 
}; 

#endif