#ifndef INTERPOLATION_HPP_
#define INTERPOLATION_HPP_

#include "perception_structure.hpp"
#include "read_json.hpp"

#include <set>

class Interpolation{
public:
    Interpolation(const std::string &outputPath) : outputPath(outputPath){}

    void interpolateCameras(std::set<SensorTimes, LESS_T0>& data_to_be_processed, std::unordered_map<double, LidarPerception>& perceptions);
    void linearInterpolates(Lidar& front_lidar, Lidar& back_lidar, SensorType type, double camera_time, double &front_lidar_time, double &back_lidar_time);
    void writeToJson();

public:
    std::unordered_map<double, LidarPerception> Perceptions_Cameras;
    std::unordered_map<SensorType, std::vector<Lidar>> camera_perception_result;
    std::string outputPath;
    std::unordered_map<SensorType, std::string> camera_folders = {{SensorType::FISH_EYE_F, "fisheye_F_lidar/"},
                                                                    {SensorType::FISH_EYE_B, "fisheye_B_lidar/"},
                                                                    {SensorType::FISH_EYE_L, "fisheye_L_lidar/"},
                                                                    {SensorType::FISH_EYE_R, "fisheye_R_lidar/"}};

};
#endif