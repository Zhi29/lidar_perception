#ifndef INTERPOLATION_HPP_
#define INTERPOLATION_HPP_

#include "perception_structure.hpp"
#include "read_json.hpp"
#include <iomanip>
#include <cmath>
#include <set>
#include <queue>

#include <experimental/filesystem>



class Interpolation{
public:
    Interpolation(const std::string &outputPath, const std::string &dir) : outputPath(outputPath), dir(dir){
        check_result = std::ofstream(outputPath + dir + "check_result.txt");
        check_lidar_result = std::ofstream(outputPath + dir + "check_lidar_result.txt");
    }

    void lidarCheckInterpolate(std::unordered_map<double, LidarPerception>& perceptions, double front_lidar_time, double back_lidar_time, double time_to_be_processed);

    void onlineInterpolation(LidarPerception& perception, std::vector<double>& timestamps, std::map<double, std::vector<Lidar>>& interpolation_result,
                        std::unordered_map<double, std::pair<double, double>>& cam_lidar_time);
    void offlineInterpolation(LidarPerception& perception, std::vector<double>& timestamps,
                                        std::map<double, std::vector<Lidar>>& interpolation_result,
                                        std::unordered_map<double, std::unordered_map<long int, std::pair<double, double>>>& cam_lidar_time_by_ids);

    void LinearInterpolation(std::map<double, std::vector<Lidar>>& interpolation_result, std::unordered_map<double, std::unordered_map<long int, std::pair<double, double>>>& cam_lidar_time_by_ids);
    void lidarLinearInterpolation(std::map<double, std::vector<Lidar>>& interpolation_result, std::unordered_map<double, std::unordered_map<long int, std::pair<double, double>>>& cam_lidar_time_by_ids);

    void SecondOrderInterpolation(std::map<double, std::vector<Lidar>>& interpolation_result, std::unordered_map<double, std::unordered_map<long int, std::pair<double, double>>>& cam_lidar_time_by_ids);
    void lidarSecondOrderInterpolation(std::map<double, std::vector<Lidar>>& interpolation_result, std::unordered_map<double, std::unordered_map<long int, std::pair<double, double>>>& cam_lidar_time_by_ids);
    

    void onlineLinearInterpolation(std::map<double, LidarPerception>& lidarPerceptions, std::set<double>& lidar_timestamps, 
                    std::set<double>& timestamps_to_be_processed,std::map<double, std::vector<Lidar>>& interpolation_result,
                    std::unordered_map<double, std::pair<double, double>>& cam_lidar_time);
    
    double correctObjectTimestamp(Lidar& lidar, double timestamp);
    void calculate_lidar_error();

    void onlinePolynomialInterpolation(std::map<double, LidarPerception>& lidarPerceptions, std::set<double>& lidar_timestamps, 
                    std::set<double>& timestamps_to_be_processed,std::map<double, std::vector<Lidar>>& interpolation_result);
    
    bool polynomialInterpolation(std::map<double, LidarPerception>& lidarPerceptions, 
                                            std::vector<double>& lidar_times_for_interpolation,  // size = 3
                                            double camera_time, long int trackid,std::map<double, std::vector<Lidar>>& interpolation_result);

    bool secondOrderCalculation(std::map<double, LidarPerception>& lidarPerceptions, 
                                            std::vector<double>& lidar_times_for_interpolation,  // size = 3
                                            double camera_time, long int trackid, double lidar_time, std::map<double, std::vector<Lidar>>& interpolation_result);


    void onlineWriteToJson(std::vector<double>& timestamps, std::unordered_map<double, std::vector<SensorType>>& timestamp_sensortype);
    void writeToJson(std::unordered_map<double, std::vector<SensorType>>& timestamp_sensortype);

public:
    std::map<double, LidarPerception> lidarPerceptions;
    std::map<long int, std::set<double>> object_timestamps;
    std::map<double, double> find_lidar_timestamp;
    std::set<double> timestamps_to_be_processed;
    std::unordered_map<double, bool> interpolated_timestamps;
    std::unordered_map<double, std::unordered_map<long int, bool>> interpolated_ids;
    std::set<double> lidar_timestamps;
    std::map<double, std::vector<Lidar>> camera_perception_result;
    std::vector<double> all_camera_time;
    std::string outputPath, dir;
    std::map<SensorType, std::string> camera_folders = {{SensorType::FISH_EYE_F, "fisheye_F_lidar/"},
                                                                    {SensorType::FISH_EYE_B, "fisheye_B_lidar/"},
                                                                    {SensorType::FISH_EYE_L, "fisheye_L_lidar/"},
                                                                    {SensorType::FISH_EYE_R, "fisheye_R_lidar/"}};

    std::ofstream check_result;
    std::ofstream check_lidar_result;


};
#endif