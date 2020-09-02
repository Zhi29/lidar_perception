
#include "interpolation.hpp"

std::ofstream debug = std::ofstream("/home/lizhi/sample_test_result/result/debug.txt");

std::ofstream error_x = std::ofstream("/home/lizhi/sample_test_result/result/error_x.txt");
std::ofstream error_y = std::ofstream("/home/lizhi/sample_test_result/result/error_y.txt");
std::ofstream error_z = std::ofstream("/home/lizhi/sample_test_result/result/error_z.txt");
std::ofstream error_length = std::ofstream("/home/lizhi/sample_test_result/result/error_length.txt");
std::ofstream error_width = std::ofstream("/home/lizhi/sample_test_result/result/error_width.txt");
std::ofstream error_height = std::ofstream("/home/lizhi/sample_test_result/result/error_height.txt");
std::ofstream error_yaw = std::ofstream("/home/lizhi/sample_test_result/result/error_yaw.txt");

std::ofstream error_bbox = std::ofstream("/home/lizhi/sample_test_result/result/error_bbox.txt");
std::ofstream error_norm_file = std::ofstream("/home/lizhi/sample_test_result/result/error_norm.txt");




void Interpolation::onlineInterpolation(LidarPerception& perception, std::vector<double>& timestamps, 
                                        std::map<double, std::vector<Lidar>>& interpolation_result,
                                        std::unordered_map<double, std::pair<double, double>>& cam_lidar_time){
    lidarPerceptions.insert({perception.timestamp, perception});
    //std::cout << "lidar map size " << lidarPerceptions.size() << std::endl;
    lidar_timestamps.insert(perception.timestamp);
    //std::cout << "lidar_timestamps " << lidar_timestamps.size() << std::endl; 
    //std::cout << "timestamps size " << timestamps.size() << std::endl;
    for(auto timestamp : timestamps){
        timestamps_to_be_processed.insert(timestamp);
        //std::cout << std::fixed << timestamp << std::endl;
        if(!interpolated_timestamps.count(timestamp)){
            //debug << std::fixed << timestamp << std::endl;
            interpolated_timestamps[timestamp] = false;
        }
    }
    //std::cout << "timestamps to be processed size " << timestamps_to_be_processed.size() << std::endl;
    onlineLinearInterpolation(lidarPerceptions, lidar_timestamps, timestamps_to_be_processed, interpolation_result, cam_lidar_time);
    //onlinePolynomialInterpolation(lidarPerceptions, lidar_timestamps, timestamps_to_be_processed, interpolation_result);
    //timestamps_to_be_processed = std::priority_queue<double>();
}

void Interpolation::offlineInterpolation(LidarPerception& perception, std::vector<double>& timestamps,
                                        std::map<double, std::vector<Lidar>>& interpolation_result,
                                        std::unordered_map<double, std::unordered_map<long int, std::pair<double, double>>>& cam_lidar_time_by_ids){
    lidarPerceptions.insert({perception.timestamp, perception});
    std::unordered_map<long int, Lidar>::iterator iter_ids;
    for(iter_ids = perception.perception_.begin(); iter_ids != perception.perception_.end(); iter_ids++){
        if(iter_ids->first < 0) continue;
        double obj_timestamp = correctObjectTimestamp(iter_ids->second, perception.timestamp);//一次一个object
        //std::cout << std::fixed << obj_timestamp << std::endl;
        object_timestamps[iter_ids->first].insert(obj_timestamp);
        find_lidar_timestamp[obj_timestamp] = perception.timestamp;
    }
    //std::cout << object_timestamps.size() << " " << find_lidar_timestamp.size() << std::endl;
    for(auto timestamp : timestamps){
        timestamps_to_be_processed.insert(timestamp);
        all_camera_time.push_back(timestamp);
    }
    //LinearInterpolation(interpolation_result, cam_lidar_time_by_ids);
    //lidarLinearInterpolation(interpolation_result, cam_lidar_time);
    //lidarSecondOrderInterpolation(interpolation_result, cam_lidar_time_by_ids);
    SecondOrderInterpolation(interpolation_result, cam_lidar_time_by_ids);
}

void Interpolation::lidarSecondOrderInterpolation(std::map<double, std::vector<Lidar>>& interpolation_result, 
                    std::unordered_map<double, std::unordered_map<long int, std::pair<double, double>>>& cam_lidar_time_by_ids)
{
    double time_interval = 250000;
    double min_time = 150000;
    if(lidarPerceptions.size() <= 3) return;
    std::set<double>::iterator iter_timestamps;
    for(iter_timestamps = timestamps_to_be_processed.begin(); iter_timestamps != timestamps_to_be_processed.end(); iter_timestamps++){
        std::unordered_map<long int, Lidar>::iterator iter;
        for(iter = lidarPerceptions.at(*iter_timestamps).perception_.begin(); iter != lidarPerceptions.at(*iter_timestamps).perception_.end(); iter++){
            double camera_time = correctObjectTimestamp(iter->second, *iter_timestamps);
            if(interpolated_timestamps.count(camera_time)) continue;//如果这个时间戳已经计算过，则跳过
            std::map<long int, std::set<double>>::iterator iter_ids;
            double front_obj_time = 0, back_obj_time = 0;
            std::map<long int, std::pair<double, double>> obj_front_back_timestamps;
            for(iter_ids = object_timestamps.begin(); iter_ids != object_timestamps.end(); iter_ids++){
                if(interpolated_ids.count(*iter_timestamps) && interpolated_ids[*iter_timestamps].count(iter_ids->first)) continue;
                if(camera_time <= *iter_ids->second.begin() || camera_time >= *(iter_ids->second.rbegin())) continue;
                std::set<double>::iterator iter_obj_timestamp;
                for(iter_obj_timestamp = iter_ids->second.begin(); iter_obj_timestamp != iter_ids->second.end(); iter_obj_timestamp++){
                    if(camera_time > *iter_obj_timestamp){
                        front_obj_time = *iter_obj_timestamp;
                        continue;
                    }
                    else if(camera_time < *iter_obj_timestamp){
                        back_obj_time = *iter_obj_timestamp;
                        break;
                    }
                }
                //std::cout << std::fixed<< front_obj_time << " " << camera_time << " " << back_obj_time << std::endl;
                if(fabs(back_obj_time - front_obj_time) > time_interval || fabs(back_obj_time - front_obj_time) < min_time){
                    front_obj_time = 0;
                    back_obj_time = 0;
                    continue;
                }
                else{
                    //std::cout << std::fixed << "    " << front_obj_time << " " << camera_time << " " << back_obj_time << std::endl;
                    obj_front_back_timestamps.insert({iter_ids->first, std::make_pair(front_obj_time, back_obj_time)});
                    front_obj_time = 0;
                    back_obj_time = 0;
                }
            }//已有的所有id循环结束 

            std::map<long int, std::vector<double>> lidar_times_for_interpolation;
            std::map<long int, std::pair<double, double>>::iterator iter_obj_f_b;
            for(iter_obj_f_b = obj_front_back_timestamps.begin(); iter_obj_f_b != obj_front_back_timestamps.end(); iter_obj_f_b++){
                lidar_times_for_interpolation[iter_obj_f_b->first].push_back(iter_obj_f_b->second.first);
                lidar_times_for_interpolation[iter_obj_f_b->first].push_back(iter_obj_f_b->second.second);

                auto temp_front_time = object_timestamps[iter_obj_f_b->first].find(iter_obj_f_b->second.first);
                auto temp_back_time = object_timestamps[iter_obj_f_b->first].find(iter_obj_f_b->second.second);
                if(temp_front_time != object_timestamps[iter_obj_f_b->first].end() && temp_front_time != object_timestamps[iter_obj_f_b->first].begin()){
                    temp_front_time--;
                    if(fabs(*temp_front_time - iter_obj_f_b->second.first) < 150000 && fabs(*temp_front_time - iter_obj_f_b->second.first) > 30000)
                        lidar_times_for_interpolation[iter_obj_f_b->first].push_back(*temp_front_time);
                }
                if(temp_back_time != object_timestamps[iter_obj_f_b->first].end() && temp_back_time != object_timestamps[iter_obj_f_b->first].end()--){
                    temp_back_time++;
                    if(fabs(*temp_back_time - iter_obj_f_b->second.second) < 150000 && fabs(*temp_back_time - iter_obj_f_b->second.second) > 30000)
                        lidar_times_for_interpolation[iter_obj_f_b->first].push_back(*temp_back_time);
                }
                std::sort(lidar_times_for_interpolation[iter_obj_f_b->first].begin(), lidar_times_for_interpolation[iter_obj_f_b->first].end());
            }

            std::map<long int, std::vector<double>>::iterator iter_lidar_times;
            for(iter_lidar_times = lidar_times_for_interpolation.begin(); iter_lidar_times != lidar_times_for_interpolation.end(); iter_lidar_times++){
                if(iter_lidar_times->second.size() < 3) continue;
                else if(iter_lidar_times->second.size() < 4){
                    secondOrderCalculation(lidarPerceptions, iter_lidar_times->second, camera_time, iter_lidar_times->first, *iter_timestamps, interpolation_result);
                }
                else{
                    std::vector<double> sub_lidar_times1;
                    std::vector<double> sub_lidar_times2;
                    for(int i = 0; i < iter_lidar_times->second.size()-1; i++){
                        sub_lidar_times1.push_back(iter_lidar_times->second[i]);
                    }
                    for(int i = 1; i < iter_lidar_times->second.size(); i++){
                        sub_lidar_times2.push_back(iter_lidar_times->second[i]);
                    }
                    if(secondOrderCalculation(lidarPerceptions, sub_lidar_times1, camera_time, iter_lidar_times->first, *iter_timestamps, interpolation_result)
                    && secondOrderCalculation(lidarPerceptions, sub_lidar_times2, camera_time, iter_lidar_times->first, *iter_timestamps, interpolation_result))
                    {
                        Lidar temp_lidar_result_1 = camera_perception_result[*iter_timestamps].back();
                        camera_perception_result[*iter_timestamps].pop_back();
                        Lidar temp_lidar_result_2 = camera_perception_result[*iter_timestamps].back();
                        camera_perception_result[*iter_timestamps].pop_back();

                        Lidar temp_lidar_result;
                        temp_lidar_result.x = (temp_lidar_result_1.x + temp_lidar_result_2.x) / 2.0;
                        temp_lidar_result.y = (temp_lidar_result_1.y + temp_lidar_result_2.y) / 2.0;
                        temp_lidar_result.z = (temp_lidar_result_1.z + temp_lidar_result_2.z) / 2.0;
                        temp_lidar_result.length = (temp_lidar_result_1.length + temp_lidar_result_2.length) / 2.0;
                        temp_lidar_result.width = (temp_lidar_result_1.width + temp_lidar_result_2.width) / 2.0;
                        temp_lidar_result.height = (temp_lidar_result_1.height + temp_lidar_result_2.height) / 2.0;
                        temp_lidar_result.pitch = (temp_lidar_result_1.pitch + temp_lidar_result_2.pitch) / 2.0;
                        temp_lidar_result.roll = (temp_lidar_result_1.roll + temp_lidar_result_2.roll) / 2.0;
                        temp_lidar_result.yaw = (temp_lidar_result_1.yaw + temp_lidar_result_2.yaw) / 2.0;
                        temp_lidar_result.tag = temp_lidar_result_1.tag;
                        temp_lidar_result.score = temp_lidar_result_1.score;
                        temp_lidar_result.trackid = temp_lidar_result_1.trackid;
                        temp_lidar_result.type = temp_lidar_result_1.type;

                        camera_perception_result[*iter_timestamps].push_back(temp_lidar_result);
                        interpolation_result[*iter_timestamps].push_back(temp_lidar_result);
                    }
                }
            }
        }
    }
}


void Interpolation::SecondOrderInterpolation(std::map<double, std::vector<Lidar>>& interpolation_result, 
                    std::unordered_map<double, std::unordered_map<long int, std::pair<double, double>>>& cam_lidar_time_by_ids)
{
    double time_interval = 150000;
    double min_time = 30000;
    if(lidarPerceptions.size() <= 2) return;
    std::set<double>::iterator iter_timestamps;
    for(iter_timestamps = timestamps_to_be_processed.begin(); iter_timestamps != timestamps_to_be_processed.end(); iter_timestamps++){
        double camera_time = *iter_timestamps;
        std::map<long int, std::set<double>>::iterator iter_ids;
        double front_obj_time = 0, back_obj_time = 0;
        std::map<long int, std::pair<double, double>> obj_front_back_timestamps;
        for(iter_ids = object_timestamps.begin(); iter_ids != object_timestamps.end(); iter_ids++){
            if(interpolated_ids.count(camera_time) && interpolated_ids[camera_time].count(iter_ids->first)) continue;
            if(camera_time <= *iter_ids->second.begin() || camera_time >= *(iter_ids->second.rbegin())) continue;
            std::set<double>::iterator iter_obj_timestamp;
            for(iter_obj_timestamp = iter_ids->second.begin(); iter_obj_timestamp != iter_ids->second.end(); iter_obj_timestamp++){
                if(camera_time > *iter_obj_timestamp){
                    front_obj_time = *iter_obj_timestamp;
                    continue;
                }
                else if(camera_time < *iter_obj_timestamp){
                    back_obj_time = *iter_obj_timestamp;
                    break;
                }
            }
            //std::cout << std::fixed<< front_obj_time << " " << camera_time << " " << back_obj_time << std::endl;
            if(fabs(back_obj_time - front_obj_time) > time_interval || fabs(back_obj_time - front_obj_time) < min_time){
                front_obj_time = 0;
                back_obj_time = 0;
                continue;
            }
            else{
                //std::cout << std::fixed << "    " << front_obj_time << " " << camera_time << " " << back_obj_time << std::endl;
                obj_front_back_timestamps.insert({iter_ids->first, std::make_pair(front_obj_time, back_obj_time)});
                front_obj_time = 0;
                back_obj_time = 0;
            }
        }//已有的所有id循环结束 

        std::map<long int, std::vector<double>> lidar_times_for_interpolation;
        std::map<long int, std::pair<double, double>>::iterator iter_obj_f_b;
        for(iter_obj_f_b = obj_front_back_timestamps.begin(); iter_obj_f_b != obj_front_back_timestamps.end(); iter_obj_f_b++){
            lidar_times_for_interpolation[iter_obj_f_b->first].push_back(iter_obj_f_b->second.first);
            lidar_times_for_interpolation[iter_obj_f_b->first].push_back(iter_obj_f_b->second.second);

            auto temp_front_time = object_timestamps[iter_obj_f_b->first].find(iter_obj_f_b->second.first);
            auto temp_back_time = object_timestamps[iter_obj_f_b->first].find(iter_obj_f_b->second.second);
            if(temp_front_time != object_timestamps[iter_obj_f_b->first].end() && temp_front_time != object_timestamps[iter_obj_f_b->first].begin()){
                temp_front_time--;
                if(fabs(*temp_front_time - iter_obj_f_b->second.first) < time_interval && fabs(*temp_front_time - iter_obj_f_b->second.first) > min_time)
                    lidar_times_for_interpolation[iter_obj_f_b->first].push_back(*temp_front_time);
            }
            if(temp_back_time != object_timestamps[iter_obj_f_b->first].end() && temp_back_time != object_timestamps[iter_obj_f_b->first].end()--){
                temp_back_time++;
                if(fabs(*temp_back_time - iter_obj_f_b->second.second) < time_interval && fabs(*temp_back_time - iter_obj_f_b->second.second) > min_time)
                    lidar_times_for_interpolation[iter_obj_f_b->first].push_back(*temp_back_time);
            }
            std::sort(lidar_times_for_interpolation[iter_obj_f_b->first].begin(), lidar_times_for_interpolation[iter_obj_f_b->first].end());
        }

        std::map<long int, std::vector<double>>::iterator iter_lidar_times;
        for(iter_lidar_times = lidar_times_for_interpolation.begin(); iter_lidar_times != lidar_times_for_interpolation.end(); iter_lidar_times++){
            if(iter_lidar_times->second.size() < 3) continue;
            else if(iter_lidar_times->second.size() < 4){
                secondOrderCalculation(lidarPerceptions, iter_lidar_times->second, camera_time, iter_lidar_times->first, *iter_timestamps, interpolation_result);
            }
            else{
                std::vector<double> sub_lidar_times1;
                std::vector<double> sub_lidar_times2;
                for(int i = 0; i < iter_lidar_times->second.size()-1; i++){
                    sub_lidar_times1.push_back(iter_lidar_times->second[i]);
                }
                for(int i = 1; i < iter_lidar_times->second.size(); i++){
                    sub_lidar_times2.push_back(iter_lidar_times->second[i]);
                }
                if(secondOrderCalculation(lidarPerceptions, sub_lidar_times1, camera_time, iter_lidar_times->first, *iter_timestamps, interpolation_result)
                && secondOrderCalculation(lidarPerceptions, sub_lidar_times2, camera_time, iter_lidar_times->first, *iter_timestamps, interpolation_result))
                {
                    Lidar temp_lidar_result_1 = camera_perception_result[camera_time].back();
                    camera_perception_result[camera_time].pop_back();
                    Lidar temp_lidar_result_2 = camera_perception_result[camera_time].back();
                    camera_perception_result[camera_time].pop_back();

                    Lidar temp_lidar_result;
                    temp_lidar_result.x = (temp_lidar_result_1.x + temp_lidar_result_2.x) / 2.0;
                    temp_lidar_result.y = (temp_lidar_result_1.y + temp_lidar_result_2.y) / 2.0;
                    temp_lidar_result.z = (temp_lidar_result_1.z + temp_lidar_result_2.z) / 2.0;
                    temp_lidar_result.length = (temp_lidar_result_1.length + temp_lidar_result_2.length) / 2.0;
                    temp_lidar_result.width = (temp_lidar_result_1.width + temp_lidar_result_2.width) / 2.0;
                    temp_lidar_result.height = (temp_lidar_result_1.height + temp_lidar_result_2.height) / 2.0;
                    temp_lidar_result.pitch = (temp_lidar_result_1.pitch + temp_lidar_result_2.pitch) / 2.0;
                    temp_lidar_result.roll = (temp_lidar_result_1.roll + temp_lidar_result_2.roll) / 2.0;
                    temp_lidar_result.yaw = (temp_lidar_result_1.yaw + temp_lidar_result_2.yaw) / 2.0;
                    temp_lidar_result.tag = temp_lidar_result_1.tag;
                    temp_lidar_result.score = temp_lidar_result_1.score;
                    temp_lidar_result.trackid = temp_lidar_result_1.trackid;
                    temp_lidar_result.type = temp_lidar_result_1.type;

                    camera_perception_result[camera_time].push_back(temp_lidar_result);
                    interpolation_result[camera_time].push_back(temp_lidar_result);
                }
            }
        }
    }
}

bool Interpolation::secondOrderCalculation(std::map<double, LidarPerception>& lidarPerceptions, 
                                            std::vector<double>& lidar_times_for_interpolation,  // size = 3
                                            double camera_time, long int trackid, double lidar_time,
                                            std::map<double, std::vector<Lidar>>& interpolation_result)
{
    //std::cout << "size ?=  " << lidar_times_for_interpolation.size() << std::endl;
    std::sort(lidar_times_for_interpolation.begin(), lidar_times_for_interpolation.end());
    double front_time = find_lidar_timestamp[lidar_times_for_interpolation[0]];
    double middle_time = find_lidar_timestamp[lidar_times_for_interpolation[1]];
    double back_time = find_lidar_timestamp[lidar_times_for_interpolation[2]];
    //std::cout << std::fixed << front_time << " " << middle_time << " " << back_time << std::endl;

    if(fabs(lidarPerceptions.at(front_time).perception_[trackid].yaw - lidarPerceptions.at(middle_time).perception_[trackid].yaw)  > M_PI_4 || 
    fabs(lidarPerceptions.at(middle_time).perception_[trackid].yaw - lidarPerceptions.at(back_time).perception_[trackid].yaw) > M_PI_4 || 
    fabs(lidarPerceptions.at(front_time).perception_[trackid].length - lidarPerceptions.at(middle_time).perception_[trackid].length)  > 1.0 || 
    fabs(lidarPerceptions.at(middle_time).perception_[trackid].length - lidarPerceptions.at(back_time).perception_[trackid].length) > 1.0 ||
    fabs(lidarPerceptions.at(front_time).perception_[trackid].width - lidarPerceptions.at(middle_time).perception_[trackid].width)  > 1.0 || 
    fabs(lidarPerceptions.at(middle_time).perception_[trackid].width - lidarPerceptions.at(back_time).perception_[trackid].width) > 1.0 || 
    fabs(lidarPerceptions.at(front_time).perception_[trackid].height - lidarPerceptions.at(middle_time).perception_[trackid].height)  > 1.0 || 
    fabs(lidarPerceptions.at(middle_time).perception_[trackid].height - lidarPerceptions.at(back_time).perception_[trackid].height) > 1.0 ) return false;

    //double corrected_front_time = front_time, corrected_middle_time = middle_time, corrected_back_time = back_time, corrected_camera_time = camera_time;
    double corrected_front_time = lidar_times_for_interpolation[0];
    double corrected_middle_time = lidar_times_for_interpolation[1];
    double corrected_back_time = lidar_times_for_interpolation[2];

    //if(corrected_back_time - corrected_middle_time < 30000 || corrected_middle_time - corrected_front_time < 30000) return false;
    /**验证lidar专用**/
    //if(!lidarPerceptions.at(lidar_time).perception_.count(trackid)) return false;
    //double corrected_camera_time = correctObjectTimestamp(lidarPerceptions.at(camera_time).perception_[trackid], camera_time);

    //debug << std::fixed << corrected_front_time << " " << corrected_middle_time << " " << corrected_back_time << " " << corrected_camera_time << std::endl;

    //double l0 = ((corrected_camera_time - corrected_middle_time) * (corrected_camera_time - corrected_back_time)) / ((corrected_front_time - corrected_middle_time) * (corrected_front_time - corrected_back_time));
    //double l1 = ((corrected_camera_time - corrected_front_time) * (corrected_camera_time - corrected_back_time)) / ((corrected_middle_time - corrected_front_time) * (corrected_middle_time - corrected_back_time));
    //double l2 = ((corrected_camera_time - corrected_front_time) * (corrected_camera_time - corrected_middle_time)) / ((corrected_back_time - corrected_front_time) * (corrected_back_time - corrected_middle_time));
    
    double l0 = ((camera_time - corrected_middle_time) * (camera_time - corrected_back_time)) / ((corrected_front_time - corrected_middle_time) * (corrected_front_time - corrected_back_time));
    double l1 = ((camera_time - corrected_front_time) * (camera_time - corrected_back_time)) / ((corrected_middle_time - corrected_front_time) * (corrected_middle_time - corrected_back_time));
    double l2 = ((camera_time - corrected_front_time) * (camera_time - corrected_middle_time)) / ((corrected_back_time - corrected_front_time) * (corrected_back_time - corrected_middle_time));

    Eigen::Vector3d coeff(l0, l1, l2);

    Eigen::Vector3d x_(lidarPerceptions.at(front_time).perception_[trackid].x, lidarPerceptions.at(middle_time).perception_[trackid].x, lidarPerceptions.at(back_time).perception_[trackid].x);
    Eigen::Vector3d y_(lidarPerceptions.at(front_time).perception_[trackid].y, lidarPerceptions.at(middle_time).perception_[trackid].y, lidarPerceptions.at(back_time).perception_[trackid].y);
    Eigen::Vector3d z_(lidarPerceptions.at(front_time).perception_[trackid].z, lidarPerceptions.at(middle_time).perception_[trackid].z, lidarPerceptions.at(back_time).perception_[trackid].z);
    Eigen::Vector3d length_
    (lidarPerceptions.at(front_time).perception_[trackid].length, lidarPerceptions.at(middle_time).perception_[trackid].length, lidarPerceptions.at(back_time).perception_[trackid].length);
    Eigen::Vector3d width_
    (lidarPerceptions.at(front_time).perception_[trackid].width, lidarPerceptions.at(middle_time).perception_[trackid].width, lidarPerceptions.at(back_time).perception_[trackid].width);
    Eigen::Vector3d height_
    (lidarPerceptions.at(front_time).perception_[trackid].height, lidarPerceptions.at(middle_time).perception_[trackid].height, lidarPerceptions.at(back_time).perception_[trackid].height);
    Eigen::Vector3d pitch_
    (lidarPerceptions.at(front_time).perception_[trackid].pitch, lidarPerceptions.at(middle_time).perception_[trackid].pitch, lidarPerceptions.at(back_time).perception_[trackid].pitch);
    Eigen::Vector3d roll_(lidarPerceptions.at(front_time).perception_[trackid].roll, lidarPerceptions.at(middle_time).perception_[trackid].roll, lidarPerceptions.at(back_time).perception_[trackid].roll);
    Eigen::Vector3d yaw_(lidarPerceptions.at(front_time).perception_[trackid].yaw, lidarPerceptions.at(middle_time).perception_[trackid].yaw, lidarPerceptions.at(back_time).perception_[trackid].yaw);

    if(fabs(yaw_(0)) > M_PI_2 && fabs(yaw_(1)) > M_PI_2 && fabs(yaw_(2)) > M_PI_2 
        && (yaw_(0) / yaw_(1) < 0 || yaw_(0)/yaw_(2) < 0 || yaw_(1)/yaw_(2) < 0))
        {
            int index_diff = -1;
            if(yaw_(0) / yaw_(1) < 0){
                if(yaw_(0) / yaw_(2) < 0) index_diff = 0;
                else if(yaw_(1) / yaw_(2) < 0) index_diff = 1;
            }
            else index_diff = 2;
            if(yaw_[index_diff] > 0) yaw_[index_diff] -= 2*M_PI;
            else if(yaw_[index_diff] < 0) yaw_[index_diff] += 2*M_PI;
        }
    
    Lidar perception_at_camera_time;

    debug << coeff << std::endl << std::endl << y_ << std::endl;
    perception_at_camera_time.x = coeff.dot(x_);
    perception_at_camera_time.y = coeff.dot(y_);
    perception_at_camera_time.z = coeff.dot(z_);
    perception_at_camera_time.length = coeff.dot(length_);
    perception_at_camera_time.width = coeff.dot(width_);
    perception_at_camera_time.height = coeff.dot(height_);
    perception_at_camera_time.pitch = coeff.dot(pitch_);
    perception_at_camera_time.roll = coeff.dot(roll_);
    
    double temp_yaw = coeff.dot(yaw_);
    if(temp_yaw > M_PI) temp_yaw -= 2*M_PI;
    else if(temp_yaw < -M_PI) temp_yaw += 2*M_PI;
    perception_at_camera_time.yaw = temp_yaw;

    perception_at_camera_time.trackid = trackid;
    perception_at_camera_time.type = lidarPerceptions.at(front_time).perception_[trackid].type;

    perception_at_camera_time.tag = lidarPerceptions.at(front_time).perception_[trackid].tag;
    perception_at_camera_time.score = lidarPerceptions.at(front_time).perception_[trackid].score;

    camera_perception_result[camera_time].push_back(perception_at_camera_time);
    interpolation_result[camera_time].push_back(perception_at_camera_time);
    interpolated_timestamps[camera_time] = true;
    interpolated_ids[camera_time].insert({perception_at_camera_time.trackid, true});
    

    //camera_perception_result[lidar_time].push_back(perception_at_camera_time);
    //interpolation_result[lidar_time].push_back(perception_at_camera_time);
    //interpolated_timestamps[camera_time] = true;
    //interpolated_ids[lidar_time].insert({perception_at_camera_time.trackid, true});

    check_result << std::fixed << front_time << "    " 
    << trackid << " "
    << lidarPerceptions.at(front_time).perception_[trackid].x << " " <<  lidarPerceptions.at(front_time).perception_[trackid].y << " " << lidarPerceptions.at(front_time).perception_[trackid].z << " "
    << lidarPerceptions.at(front_time).perception_[trackid].length << " " << lidarPerceptions.at(front_time).perception_[trackid].width << " " << lidarPerceptions.at(front_time).perception_[trackid].height << " "
    << lidarPerceptions.at(front_time).perception_[trackid].yaw << std::endl;


    check_result << std::fixed << middle_time << "    " 
    << trackid << " "
    << lidarPerceptions.at(middle_time).perception_[trackid].x << " " <<  lidarPerceptions.at(middle_time).perception_[trackid].y << " " << lidarPerceptions.at(middle_time).perception_[trackid].z << " "
    << lidarPerceptions.at(middle_time).perception_[trackid].length << " " << lidarPerceptions.at(middle_time).perception_[trackid].width << " " << lidarPerceptions.at(middle_time).perception_[trackid].height << " "
    << lidarPerceptions.at(middle_time).perception_[trackid].yaw << std::endl;


    check_result << std::fixed << back_time << "    " 
    << trackid << " "
    << lidarPerceptions.at(back_time).perception_[trackid].x << " " <<  lidarPerceptions.at(back_time).perception_[trackid].y << " " << lidarPerceptions.at(back_time).perception_[trackid].z << " "
    << lidarPerceptions.at(back_time).perception_[trackid].length << " " << lidarPerceptions.at(back_time).perception_[trackid].width << " " << lidarPerceptions.at(back_time).perception_[trackid].height << " "
    << lidarPerceptions.at(back_time).perception_[trackid].yaw << std::endl;
                
    
    
    check_result << "----------------------------------------------------" << std::endl;
    check_result << std::fixed << camera_time << "    " 
    << perception_at_camera_time.trackid << " "
    << perception_at_camera_time.x << " " <<  perception_at_camera_time.y << " " << perception_at_camera_time.z << " "
    << perception_at_camera_time.length << " " << perception_at_camera_time.width << " " << perception_at_camera_time.height << " "
    << perception_at_camera_time.yaw << std::endl;
    check_result << std::endl;
    
    return true;
}

void Interpolation::LinearInterpolation(std::map<double, std::vector<Lidar>>& interpolation_result,
                                    std::unordered_map<double, std::unordered_map<long int, std::pair<double, double>>>& cam_lidar_time_by_ids)
{
    double time_interval = 150000;
    double min_time_interval = 30000;
    if(lidarPerceptions.size() <= 1) return;
    std::set<double>::iterator iter_timestamps;
    for(iter_timestamps = timestamps_to_be_processed.begin(); iter_timestamps != timestamps_to_be_processed.end(); iter_timestamps++){
        double camera_time = *iter_timestamps;
        //if(interpolated_timestamps[camera_time]) continue;//如果这个时间戳已经计算过，则跳过
        std::map<long int, std::set<double>>::iterator iter_ids;
        double front_obj_time = 0, back_obj_time = 0;
        std::map<long int, std::pair<double, double>> obj_front_back_timestamps;
        for(iter_ids = object_timestamps.begin(); iter_ids != object_timestamps.end(); iter_ids++){
            if(interpolated_ids.count(camera_time) && interpolated_ids[camera_time].count(iter_ids->first)) continue;
            if(camera_time <= *iter_ids->second.begin() || camera_time >= *(iter_ids->second.rbegin())) continue;
            std::set<double>::iterator iter_obj_timestamp;
            for(iter_obj_timestamp = iter_ids->second.begin(); iter_obj_timestamp != iter_ids->second.end(); iter_obj_timestamp++){
                if(camera_time > *iter_obj_timestamp){
                    front_obj_time = *iter_obj_timestamp;
                    continue;
                }
                else if(camera_time < *iter_obj_timestamp){
                    back_obj_time = *iter_obj_timestamp;
                    break;
                }
            }
            //std::cout << std::fixed<< front_obj_time << " " << camera_time << " " << back_obj_time << std::endl;
            if(fabs(back_obj_time - front_obj_time) > time_interval || fabs(back_obj_time - front_obj_time) < min_time_interval){
                front_obj_time = 0;
                back_obj_time = 0;
                continue;
            }
            else{
                //std::cout << std::fixed << "    " << front_obj_time << " " << camera_time << " " << back_obj_time << std::endl;
                obj_front_back_timestamps.insert({iter_ids->first, std::make_pair(front_obj_time, back_obj_time)});
                front_obj_time = 0;
                back_obj_time = 0;
            }
        }//已有的所有id循环结束
        //if(obj_front_back_timestamps.size())
        //    std::cout << obj_front_back_timestamps.size() << std::endl;

        std::map<long int, std::pair<double, double>>::iterator iter_objs;
        for(iter_objs = obj_front_back_timestamps.begin(); iter_objs != obj_front_back_timestamps.end(); iter_objs++){
            double front_lidar_time = find_lidar_timestamp[iter_objs->second.first];
            double back_lidar_time = find_lidar_timestamp[iter_objs->second.second];
            //std::cout << std::fixed << "        " << front_lidar_time << "    " << camera_time << "   " << back_lidar_time << std::endl;
            Lidar front_lidar = lidarPerceptions.at(front_lidar_time).perception_[iter_objs->first];
            Lidar back_lidar = lidarPerceptions.at(back_lidar_time).perception_[iter_objs->first];

            if(fabs(front_lidar.yaw - back_lidar.yaw) > M_PI/4 ||
                fabs(front_lidar.length - back_lidar.length) > 1.0 ||
                fabs(front_lidar.width - back_lidar.width) > 1.0 ||
                fabs(front_lidar.height - back_lidar.height) > 1.0
                ) continue; //前后帧出现偏航角的大跳变则视为断帧，不予插值

            //if(!lidarPerceptions.at(camera_time).perception_.count(iter_objs->first)) continue;
            //double corrected_camera_time = correctObjectTimestamp(lidarPerceptions.at(camera_time).perception_[iter_objs->first], camera_time);

            Eigen::Vector3d front_pos = {front_lidar.x, front_lidar.y, front_lidar.z};
            Eigen::Vector3d back_pos = {back_lidar.x, back_lidar.y, back_lidar.z};
            Eigen::Vector3d front_dim = {front_lidar.length, front_lidar.width, front_lidar.height};
            Eigen::Vector3d back_dim = {back_lidar.length, back_lidar.width, back_lidar.height};
            Eigen::Vector3d front_pose = {front_lidar.pitch, front_lidar.roll, front_lidar.yaw};
            Eigen::Vector3d back_pose = {back_lidar.pitch, back_lidar.roll, back_lidar.yaw};

            Eigen::Vector3d camera_pos, camera_dim, camera_pose;
            
            double corrected_front_time = iter_objs->second.first;
            double corrected_back_time = iter_objs->second.second;
            double time_scale = (camera_time - corrected_front_time) / (corrected_back_time - corrected_front_time);
            //double time_scale = (camera_time - front_lidar_time) / (back_lidar_time - front_lidar_time);
            //double time_scale = (corrected_camera_time - corrected_front_time) / (corrected_back_time - corrected_front_time);
            camera_pos = time_scale * (back_pos - front_pos) + front_pos;
            camera_dim = time_scale * (back_dim - front_dim) + front_dim;
            if(abs(back_pose(2)) > M_PI_2 && abs(front_pose(2)) > M_PI_2 && front_pose(2)/back_pose(2) < 0){
                if(back_pose(2) > 0){
                    back_pose(2) -= 2*M_PI;
                }
                else if(back_pose(2) < 0){
                    back_pose(2) += 2*M_PI; 
                }
            }
            camera_pose = time_scale * (back_pose - front_pose) + front_pose;
            if(camera_pose(2) > M_PI) camera_pose(2) -= 2*M_PI;
            else if(camera_pose(2) < -M_PI) camera_pose(2) += 2*M_PI;

            Lidar perception_at_camera_time;
            
            perception_at_camera_time.x = camera_pos(0);
            perception_at_camera_time.y = camera_pos(1);
            perception_at_camera_time.z = camera_pos(2);
            perception_at_camera_time.length = camera_dim(0);
            perception_at_camera_time.width = camera_dim(1);
            perception_at_camera_time.height = camera_dim(2);
            perception_at_camera_time.pitch = camera_pose(0);
            perception_at_camera_time.roll = camera_pose(1);
            perception_at_camera_time.yaw = camera_pose(2);
            perception_at_camera_time.trackid = iter_objs->first;
            perception_at_camera_time.type = front_lidar.type;

            perception_at_camera_time.tag = front_lidar.tag;
            perception_at_camera_time.score = front_lidar.score;
        
            camera_perception_result[camera_time].push_back(perception_at_camera_time);
            interpolation_result[camera_time].push_back(perception_at_camera_time);
            //cam_lidar_time[camera_time] = std::make_pair(front_lidar_time, back_lidar_time);
            cam_lidar_time_by_ids[camera_time].insert({iter_objs->first, std::make_pair(front_lidar_time, back_lidar_time)});

            //interpolated_timestamps[camera_time] = true;
            interpolated_ids[camera_time].insert({perception_at_camera_time.trackid, true});
            
            check_result << std::fixed << front_lidar_time << std::endl;
            check_result << std::fixed << corrected_front_time << "    " 
            << front_lidar.trackid << " "
            << front_lidar.x << " " <<  front_lidar.y << " " << front_lidar.z << " "
            << front_lidar.length << " " << front_lidar.width << " " << front_lidar.height << " "
            << front_lidar.yaw << std::endl;

            
            check_result << std::fixed << camera_time << "    "
            << perception_at_camera_time.trackid << " "
            << perception_at_camera_time.x << " " <<  perception_at_camera_time.y << " " << perception_at_camera_time.z << " "
            << perception_at_camera_time.length << " " << perception_at_camera_time.width << " " << perception_at_camera_time.height << " "
            << perception_at_camera_time.yaw << std::endl; 

            check_result << std::fixed << corrected_back_time << "    " 
            << back_lidar.trackid << " "
            << back_lidar.x << " " <<  back_lidar.y << " " << back_lidar.z << " "
            << back_lidar.length << " " << back_lidar.width << " " << back_lidar.height << " "
            << back_lidar.yaw << std::endl;
            check_result << std::fixed << back_lidar_time << std::endl;
            check_result << std::endl;
        }
    }//待插值时间戳循环结束
}


void Interpolation::lidarLinearInterpolation(std::map<double, std::vector<Lidar>>& interpolation_result,
                                    std::unordered_map<double, std::unordered_map<long int, std::pair<double, double>>>& cam_lidar_time_by_ids)
{
    double time_interval = 250000;
    double min_time_interval = 150000;
    if(lidarPerceptions.size() <= 3) return;
    std::set<double>::iterator iter_timestamps;
    for(iter_timestamps = timestamps_to_be_processed.begin(); iter_timestamps != timestamps_to_be_processed.end(); iter_timestamps++){
        //for(std::map<double, LidarPerception>::iterator iter_print = lidarPerceptions.begin(); iter_print != lidarPerceptions.end(); iter_print++){
        //    std::cout << std::fixed << iter_print->first << " ";
        //}
        //std::cout << std::endl;
        //std::cout << std::fixed << "camera time " << *iter_timestamps << std::endl;
        std::unordered_map<long int, Lidar>::iterator iter;
        for(iter = lidarPerceptions.at(*iter_timestamps).perception_.begin(); iter != lidarPerceptions.at(*iter_timestamps).perception_.end(); iter++){
            double camera_time = correctObjectTimestamp(iter->second, *iter_timestamps);
            if(interpolated_timestamps.count(camera_time)) continue;//如果这个时间戳已经计算过，则跳过
            std::map<long int, std::set<double>>::iterator iter_ids;
            double front_obj_time = 0, back_obj_time = 0;
            std::map<long int, std::pair<double, double>> obj_front_back_timestamps;
            //std::cout << object_timestamps.size() << std::endl;
            for(iter_ids = object_timestamps.begin(); iter_ids != object_timestamps.end(); iter_ids++){
                if(interpolated_ids.count(*iter_timestamps) && interpolated_ids[*iter_timestamps].count(iter_ids->first)) continue;
                if(camera_time <= *iter_ids->second.begin() || camera_time >= *(iter_ids->second.rbegin())) continue;
                //std::cout << iter_ids->second.size() << std::endl;
                std::set<double>::iterator iter_obj_timestamp;
                for(iter_obj_timestamp = iter_ids->second.begin(); iter_obj_timestamp != iter_ids->second.end(); iter_obj_timestamp++){
                    if(camera_time > *iter_obj_timestamp){
                        front_obj_time = *iter_obj_timestamp;
                        continue;
                    }
                    else if(camera_time < *iter_obj_timestamp){
                        back_obj_time = *iter_obj_timestamp;
                        break;
                    }
                }
                //std::cout << std::fixed<< front_obj_time << " " << camera_time << " " << back_obj_time << std::endl;
                if(fabs(back_obj_time - front_obj_time) > time_interval || fabs(back_obj_time - front_obj_time) < min_time_interval){
                    front_obj_time = 0;
                    back_obj_time = 0;
                    continue;
                }
                else{
                    //std::cout << std::fixed << "    " << front_obj_time << " " << camera_time << " " << back_obj_time << std::endl;
                    obj_front_back_timestamps.insert({iter_ids->first, std::make_pair(front_obj_time, back_obj_time)});
                    front_obj_time = 0;
                    back_obj_time = 0;
                }
            }//已有的所有id循环结束
            //if(obj_front_back_timestamps.size())
            //    std::cout << obj_front_back_timestamps.size() << std::endl;

            std::map<long int, std::pair<double, double>>::iterator iter_objs;
            for(iter_objs = obj_front_back_timestamps.begin(); iter_objs != obj_front_back_timestamps.end(); iter_objs++){
                double front_lidar_time = find_lidar_timestamp[iter_objs->second.first];
                double back_lidar_time = find_lidar_timestamp[iter_objs->second.second];
                //std::cout << std::fixed << "        " << front_lidar_time << "    " << camera_time << "   " << back_lidar_time << std::endl;
                Lidar front_lidar = lidarPerceptions.at(front_lidar_time).perception_[iter_objs->first];
                Lidar back_lidar = lidarPerceptions.at(back_lidar_time).perception_[iter_objs->first];
                //std::cout << "i am here " <<std::endl;
                if(fabs(front_lidar.yaw - back_lidar.yaw) > M_PI/4 ||
                    fabs(front_lidar.length - back_lidar.length) > 2.0 ||
                    fabs(front_lidar.width - back_lidar.width) > 2.0 ||
                    fabs(front_lidar.height - back_lidar.height) > 2.0
                    ) continue; //前后帧出现偏航角的大跳变则视为断帧，不予插值

                if(!lidarPerceptions.at(*iter_timestamps).perception_.count(iter_objs->first)) continue;
                //double corrected_camera_time = correctObjectTimestamp(lidarPerceptions.at(camera_time).perception_[iter_objs->first], camera_time);

                Eigen::Vector3d front_pos = {front_lidar.x, front_lidar.y, front_lidar.z};
                Eigen::Vector3d back_pos = {back_lidar.x, back_lidar.y, back_lidar.z};
                Eigen::Vector3d front_dim = {front_lidar.length, front_lidar.width, front_lidar.height};
                Eigen::Vector3d back_dim = {back_lidar.length, back_lidar.width, back_lidar.height};
                Eigen::Vector3d front_pose = {front_lidar.pitch, front_lidar.roll, front_lidar.yaw};
                Eigen::Vector3d back_pose = {back_lidar.pitch, back_lidar.roll, back_lidar.yaw};

                Eigen::Vector3d camera_pos, camera_dim, camera_pose;
                
                double corrected_front_time = iter_objs->second.first;
                double corrected_back_time = iter_objs->second.second;
                double time_scale = (camera_time - corrected_front_time) / (corrected_back_time - corrected_front_time);
                //double time_scale = (camera_time - front_lidar_time) / (back_lidar_time - front_lidar_time);
                //double time_scale = (corrected_camera_time - corrected_front_time) / (corrected_back_time - corrected_front_time);
                camera_pos = time_scale * (back_pos - front_pos) + front_pos;
                camera_dim = time_scale * (back_dim - front_dim) + front_dim;
                if(abs(back_pose(2)) > M_PI_2 && abs(front_pose(2)) > M_PI_2 && front_pose(2)/back_pose(2) < 0){
                    if(back_pose(2) > 0){
                        back_pose(2) -= 2*M_PI;
                    }
                    else if(back_pose(2) < 0){
                        back_pose(2) += 2*M_PI; 
                    }
                }
                camera_pose = time_scale * (back_pose - front_pose) + front_pose;
                if(camera_pose(2) > M_PI) camera_pose(2) -= 2*M_PI;
                else if(camera_pose(2) < -M_PI) camera_pose(2) += 2*M_PI;

                Lidar perception_at_camera_time;
                
                perception_at_camera_time.x = camera_pos(0);
                perception_at_camera_time.y = camera_pos(1);
                perception_at_camera_time.z = camera_pos(2);
                perception_at_camera_time.length = camera_dim(0);
                perception_at_camera_time.width = camera_dim(1);
                perception_at_camera_time.height = camera_dim(2);
                perception_at_camera_time.pitch = camera_pose(0);
                perception_at_camera_time.roll = camera_pose(1);
                perception_at_camera_time.yaw = camera_pose(2);
                perception_at_camera_time.trackid = front_lidar.trackid;
                perception_at_camera_time.type = front_lidar.type;

                perception_at_camera_time.tag = front_lidar.tag;
                perception_at_camera_time.score = front_lidar.score;
            
                camera_perception_result[*iter_timestamps].push_back(perception_at_camera_time);
                interpolation_result[*iter_timestamps].push_back(perception_at_camera_time);
                //cam_lidar_time[*iter_timestamps] = std::make_pair(front_lidar_time, back_lidar_time);
                interpolated_timestamps[camera_time] = true;
                interpolated_ids[*iter_timestamps].insert({perception_at_camera_time.trackid, true});
                
                /**
                check_result << std::fixed << front_lidar_time << std::endl;
                check_result << std::fixed << corrected_front_time << "    " 
                << front_lidar.trackid << " "
                << front_lidar.x << " " <<  front_lidar.y << " " << front_lidar.z << " "
                << front_lidar.length << " " << front_lidar.width << " " << front_lidar.height << " "
                << front_lidar.yaw << std::endl;

                
                check_result << std::fixed << camera_time << "    "
                << perception_at_camera_time.trackid << " "
                << perception_at_camera_time.x << " " <<  perception_at_camera_time.y << " " << perception_at_camera_time.z << " "
                << perception_at_camera_time.length << " " << perception_at_camera_time.width << " " << perception_at_camera_time.height << " "
                << perception_at_camera_time.yaw << std::endl; 

                check_result << std::fixed << *iter_timestamps << "    "
                << lidarPerceptions.at(*iter_timestamps).perception_[perception_at_camera_time.trackid].trackid << " "
                << lidarPerceptions.at(*iter_timestamps).perception_[perception_at_camera_time.trackid].x << " " <<  lidarPerceptions.at(*iter_timestamps).perception_[perception_at_camera_time.trackid].y << " " << lidarPerceptions.at(*iter_timestamps).perception_[perception_at_camera_time.trackid].z << " "
                << lidarPerceptions.at(*iter_timestamps).perception_[perception_at_camera_time.trackid].length << " " << lidarPerceptions.at(*iter_timestamps).perception_[perception_at_camera_time.trackid].width << " " << lidarPerceptions.at(*iter_timestamps).perception_[perception_at_camera_time.trackid].height << " "
                << lidarPerceptions.at(*iter_timestamps).perception_[perception_at_camera_time.trackid].yaw << std::endl; 

                check_result << std::fixed << corrected_back_time << "    " 
                << back_lidar.trackid << " "
                << back_lidar.x << " " <<  back_lidar.y << " " << back_lidar.z << " "
                << back_lidar.length << " " << back_lidar.width << " " << back_lidar.height << " "
                << back_lidar.yaw << std::endl;
                check_result << std::fixed << back_lidar_time << std::endl;
                check_result << std::endl;**/
            }
        }
    }//待插值时间戳循环结束
    //std::cout << camera_perception_result.size() << std::endl;
}



void Interpolation::writeToJson(std::unordered_map<double, std::vector<SensorType>>& timestamp_sensortype)
{
    for(int i = 0; i < all_camera_time.size(); i++){
        if(!camera_perception_result.count(all_camera_time[i])){
            nlohmann::json jsonfiles;
            jsonfiles["children"] = nlohmann::json::array();
            jsonfiles["tag"] = "_";
            jsonfiles["version"] = "1.0";
            std::unordered_map<double, std::vector<SensorType>>::iterator iter_time_sensor;
            double timestamp = all_camera_time[i];
            for(int j = 0; j < timestamp_sensortype[timestamp].size(); j++){
                std::experimental::filesystem::path dirs(outputPath + dir + "/" + camera_folders[timestamp_sensortype[timestamp][j]]);
                if(!(std::experimental::filesystem::exists(dirs))){
                    if(std::experimental::filesystem::create_directories(dirs))
                    {
                        std::cout << "create " << outputPath + dir + "/" + camera_folders[timestamp_sensortype[timestamp][j]] << std::endl;
                    }
                }
                std::string save_json_path = outputPath + dir + "/" + camera_folders[timestamp_sensortype[timestamp][j]] + std::to_string(timestamp).substr(0,16) + ".pcd.tar.json";
                std::ofstream save_json_to_file(save_json_path);
                save_json_to_file <<std::setw(4)<< jsonfiles;
            }
        }
        else{
                nlohmann::json jsonfiles;
                for(int j = 0; j < camera_perception_result[all_camera_time[i]].size(); j++){//每个相机时间戳对应的感知结果，多个物体感知
                    nlohmann::json json_file;
                    json_file["data"] = {
                        {"isKeyCube", {false}}, 
                        {"isKeyPropertyCube", {false}}, 
                        {"type", {camera_perception_result[all_camera_time[i]][j].type}}};
                    json_file["height"] = camera_perception_result[all_camera_time[i]][j].height;
                    json_file["length"] = camera_perception_result[all_camera_time[i]][j].length;
                    json_file["pitch"] = camera_perception_result[all_camera_time[i]][j].pitch;
                    json_file["roll"] = camera_perception_result[all_camera_time[i]][j].roll;
                    json_file["score"] = camera_perception_result[all_camera_time[i]][j].score;
                    json_file["tag"] = camera_perception_result[all_camera_time[i]][j].tag;
                    json_file["uuid"] = camera_perception_result[all_camera_time[i]][j].trackid;
                    json_file["width"] = camera_perception_result[all_camera_time[i]][j].width;
                    json_file["x"] = camera_perception_result[all_camera_time[i]][j].x;
                    json_file["y"] = camera_perception_result[all_camera_time[i]][j].y;
                    json_file["yaw"] = camera_perception_result[all_camera_time[i]][j].yaw;
                    json_file["z"] = camera_perception_result[all_camera_time[i]][j].z;

                    jsonfiles["children"].push_back(json_file);
                }
                jsonfiles["tag"] = "_";
                jsonfiles["version"] = "1.0";
                
                for(int k = 0; k < timestamp_sensortype[all_camera_time[i]].size(); k++){
                    std::experimental::filesystem::path dirs(outputPath + dir + "/" + camera_folders[timestamp_sensortype[all_camera_time[i]][k]]);
                    if(!(std::experimental::filesystem::exists(dirs))){
                        if(std::experimental::filesystem::create_directories(dirs))
                        {
                            std::cout << "create " << outputPath + dir + "/" + camera_folders[timestamp_sensortype[all_camera_time[i]][k]] << std::endl;
                        }
                    }
                    std::string save_json_path = outputPath + dir + "/" + camera_folders[timestamp_sensortype[all_camera_time[i]][k]] + std::to_string(all_camera_time[i]).substr(0,16) + ".pcd.tar.json";
                    std::ofstream save_json_to_file(save_json_path);
                    save_json_to_file <<std::setw(4)<< jsonfiles;
                }
        }
    }
    camera_perception_result.clear();
    //all_camera_time.clear();
}

void Interpolation::onlinePolynomialInterpolation(std::map<double, LidarPerception>& lidarPerceptions,
                                        std::set<double>& lidar_timestamps, std::set<double>& timestamps_to_be_processed,
                                        std::map<double, std::vector<Lidar>>& interpolation_result
                                    )
{
    double time_interval = 150000;
    if(lidarPerceptions.size() <= 2) return;
    std::set<double>::iterator iter_timestamps;
    for(iter_timestamps = timestamps_to_be_processed.begin(); iter_timestamps != timestamps_to_be_processed.end(); iter_timestamps++){
        double camera_time = *iter_timestamps;
        if(interpolated_timestamps[camera_time]) continue;//如果这个时间戳已经计算过，则跳过

        std::deque<double> lidar_times_for_interpolation;
        //找到lidar时间戳中，在要插值的相机时间戳的前后两帧
        std::set<double>::iterator iter_lidar_timestamps;
        double front_lidar_time = 0, back_lidar_time = 0;
        for(iter_lidar_timestamps = lidar_timestamps.begin(); iter_lidar_timestamps != lidar_timestamps.end(); iter_lidar_timestamps++){
            if(camera_time > *iter_lidar_timestamps){
                front_lidar_time = *iter_lidar_timestamps;
                continue;
            }
            else if(camera_time < *iter_lidar_timestamps){
                back_lidar_time = *iter_lidar_timestamps;
                break;
            }
        }
        //如果前后两帧lidar相差大于0.2s则不进行插值，这一条视精度要求可更改
        if(front_lidar_time == 0 || back_lidar_time == 0) continue;
        if(fabs(back_lidar_time - front_lidar_time) > time_interval) continue;

        lidar_times_for_interpolation.push_back(front_lidar_time);
        lidar_times_for_interpolation.push_back(back_lidar_time);

        auto temp_front_time = lidar_timestamps.find(front_lidar_time);
        auto temp_back_time = lidar_timestamps.find(back_lidar_time);
        if(temp_front_time != lidar_timestamps.end() && temp_front_time != lidar_timestamps.begin()){
            temp_front_time--;
            if(fabs(*temp_front_time - front_lidar_time) < 150000) lidar_times_for_interpolation.push_back(*temp_front_time);
        }
        if(temp_back_time != lidar_timestamps.end() && temp_back_time != lidar_timestamps.end()--){
            temp_back_time++;
            if(fabs(*temp_back_time - back_lidar_time) < 150000) lidar_times_for_interpolation.push_back(*temp_back_time);
        }

        /**检查跟踪id情况**/
        std::unordered_map<long int, std::vector<double>> timestamps_by_ids;
        std::deque<double>::iterator iter_lidar_times;
        for(iter_lidar_times = lidar_times_for_interpolation.begin(); iter_lidar_times != lidar_times_for_interpolation.end(); iter_lidar_times++){
            std::unordered_map<long int, Lidar>::iterator iter_ids;
            for(iter_ids = lidarPerceptions.at(*iter_lidar_times).perception_.begin(); iter_ids != lidarPerceptions.at(*iter_lidar_times).perception_.end(); iter_ids++){
                if(iter_ids->first < 0) continue;
                timestamps_by_ids[iter_ids->first].push_back(*iter_lidar_times);
            }
        }

        std::unordered_map<long int, std::vector<double>>::iterator iter_t_by_id;
        for(iter_t_by_id = timestamps_by_ids.begin(); iter_t_by_id != timestamps_by_ids.end(); iter_t_by_id++){
            
            if(iter_t_by_id->second.size() < 3) continue;
            else if(iter_t_by_id->second.size() < 4){
                std::sort(iter_t_by_id->second.begin(), iter_t_by_id->second.end());
                if(iter_t_by_id->second[1] - iter_t_by_id->second[0] > 150000 || iter_t_by_id->second[2] - iter_t_by_id->second[1] > 150000) continue;
                else{
                    //std::cout << "size 3" << std::endl;
                    //std::cout << std::fixed << iter_t_by_id->second[0] << " " << iter_t_by_id->second[1] << " " << iter_t_by_id->second[2] << std::endl;
                    polynomialInterpolation(lidarPerceptions, iter_t_by_id->second, camera_time, iter_t_by_id->first, interpolation_result);
                }
            }
            else{
                std::sort(iter_t_by_id->second.begin(), iter_t_by_id->second.end());

                std::vector<double> sub_lidar_times1;
                std::vector<double> sub_lidar_times2;
                for(int i = 0; i < iter_t_by_id->second.size()-1; i++){
                    sub_lidar_times1.push_back(iter_t_by_id->second[i]);
                }
                for(int i = 1; i < iter_t_by_id->second.size(); i++){
                    sub_lidar_times2.push_back(iter_t_by_id->second[i]);
                }

                bool flag_1 = polynomialInterpolation(lidarPerceptions, sub_lidar_times1, camera_time, iter_t_by_id->first, interpolation_result);
                bool flag_2 = polynomialInterpolation(lidarPerceptions, sub_lidar_times2, camera_time, iter_t_by_id->first, interpolation_result);
                if(flag_1 && flag_2){
                    Lidar temp_lidar_result_1 = camera_perception_result[camera_time].back();
                    debug << std::fixed << camera_time << " " << iter_t_by_id->first << std::endl;
                    debug << temp_lidar_result_1.x << " " << temp_lidar_result_1.y << " " << temp_lidar_result_1.z << " " << temp_lidar_result_1.length
                    << " " << temp_lidar_result_1.width << " " << temp_lidar_result_1.height << " " << temp_lidar_result_1.yaw << std::endl;
                    camera_perception_result[camera_time].pop_back();
                    Lidar temp_lidar_result_2 = camera_perception_result[camera_time].back();
                    debug << temp_lidar_result_2.x << " " << temp_lidar_result_2.y << " " << temp_lidar_result_2.z << " " << temp_lidar_result_2.length
                    << " " << temp_lidar_result_2.width << " " << temp_lidar_result_2.height << " " << temp_lidar_result_2.yaw << std::endl;
                    camera_perception_result[camera_time].pop_back();
                    Lidar temp_lidar_result;
                    temp_lidar_result.x = (temp_lidar_result_1.x + temp_lidar_result_2.x) / 2.0;
                    temp_lidar_result.y = (temp_lidar_result_1.y + temp_lidar_result_2.y) / 2.0;
                    temp_lidar_result.z = (temp_lidar_result_1.z + temp_lidar_result_2.z) / 2.0;
                    temp_lidar_result.length = (temp_lidar_result_1.length + temp_lidar_result_2.length) / 2.0;
                    temp_lidar_result.width = (temp_lidar_result_1.width + temp_lidar_result_2.width) / 2.0;
                    temp_lidar_result.height = (temp_lidar_result_1.height + temp_lidar_result_2.height) / 2.0;
                    temp_lidar_result.pitch = (temp_lidar_result_1.pitch + temp_lidar_result_2.pitch) / 2.0;
                    temp_lidar_result.roll = (temp_lidar_result_1.roll + temp_lidar_result_2.roll) / 2.0;
                    temp_lidar_result.yaw = (temp_lidar_result_1.yaw + temp_lidar_result_2.yaw) / 2.0;
                    temp_lidar_result.tag = temp_lidar_result_1.tag;
                    temp_lidar_result.score = temp_lidar_result_1.score;
                    temp_lidar_result.trackid = temp_lidar_result_1.trackid;
                    temp_lidar_result.type = temp_lidar_result_1.type;

                    debug << temp_lidar_result.x << " " << temp_lidar_result.y << " " << temp_lidar_result.z << " " << temp_lidar_result.length
                    << " " << temp_lidar_result.width << " " << temp_lidar_result.height << " " << temp_lidar_result.yaw << std::endl;
                    debug << "------" << std::endl;

                    camera_perception_result[camera_time].push_back(temp_lidar_result);
                    interpolation_result[camera_time].push_back(temp_lidar_result);
                }
            }
        }
    }
    calculate_lidar_error();
}

bool Interpolation::polynomialInterpolation(std::map<double, LidarPerception>& lidarPerceptions, 
                                            std::vector<double>& lidar_times_for_interpolation,  // size = 3
                                            double camera_time, long int trackid, 
                                            std::map<double, std::vector<Lidar>>& interpolation_result)
{
    std::sort(lidar_times_for_interpolation.begin(), lidar_times_for_interpolation.end());
    double front_time = lidar_times_for_interpolation[0];
    double middle_time = lidar_times_for_interpolation[1];
    double back_time = lidar_times_for_interpolation[2];
    std::cout << std::fixed << front_time << " " << middle_time << " " << back_time << std::endl;

    if(fabs(lidarPerceptions.at(front_time).perception_[trackid].yaw - lidarPerceptions.at(middle_time).perception_[trackid].yaw)  > M_PI_4 || 
    fabs(lidarPerceptions.at(middle_time).perception_[trackid].yaw - lidarPerceptions.at(back_time).perception_[trackid].yaw) > M_PI_4) return false;

    //double corrected_front_time = front_time, corrected_middle_time = middle_time, corrected_back_time = back_time, corrected_camera_time = camera_time;
    double corrected_front_time = correctObjectTimestamp(lidarPerceptions.at(front_time).perception_[trackid], front_time);
    double corrected_middle_time = correctObjectTimestamp(lidarPerceptions.at(middle_time).perception_[trackid], middle_time);
    double corrected_back_time = correctObjectTimestamp(lidarPerceptions.at(back_time).perception_[trackid], back_time);

    //if(corrected_back_time - corrected_middle_time < 30000 || corrected_middle_time - corrected_front_time < 30000) return false;
    /**验证lidar专用**/
    //if(!lidarPerceptions.at(camera_time).perception_.count(trackid)) return false;
    //double corrected_camera_time = correctObjectTimestamp(lidarPerceptions.at(camera_time).perception_[trackid], camera_time);

    //debug << std::fixed << corrected_front_time << " " << corrected_middle_time << " " << corrected_back_time << " " << corrected_camera_time << std::endl;

    //double l0 = ((corrected_camera_time - corrected_middle_time) * (corrected_camera_time - corrected_back_time)) / ((corrected_front_time - corrected_middle_time) * (corrected_front_time - corrected_back_time));
    //double l1 = ((corrected_camera_time - corrected_front_time) * (corrected_camera_time - corrected_back_time)) / ((corrected_middle_time - corrected_front_time) * (corrected_middle_time - corrected_back_time));
    //double l2 = ((corrected_camera_time - corrected_front_time) * (corrected_camera_time - corrected_middle_time)) / ((corrected_back_time - corrected_front_time) * (corrected_back_time - corrected_middle_time));
    
    double l0 = ((camera_time - corrected_middle_time) * (camera_time - corrected_back_time)) / ((corrected_front_time - corrected_middle_time) * (corrected_front_time - corrected_back_time));
    double l1 = ((camera_time - corrected_front_time) * (camera_time - corrected_back_time)) / ((corrected_middle_time - corrected_front_time) * (corrected_middle_time - corrected_back_time));
    double l2 = ((camera_time - corrected_front_time) * (camera_time - corrected_middle_time)) / ((corrected_back_time - corrected_front_time) * (corrected_back_time - corrected_middle_time));

    Eigen::Vector3d coeff(l0, l1, l2);

    Eigen::Vector3d x_(lidarPerceptions.at(front_time).perception_[trackid].x, lidarPerceptions.at(middle_time).perception_[trackid].x, lidarPerceptions.at(back_time).perception_[trackid].x);
    Eigen::Vector3d y_(lidarPerceptions.at(front_time).perception_[trackid].y, lidarPerceptions.at(middle_time).perception_[trackid].y, lidarPerceptions.at(back_time).perception_[trackid].y);
    Eigen::Vector3d z_(lidarPerceptions.at(front_time).perception_[trackid].z, lidarPerceptions.at(middle_time).perception_[trackid].z, lidarPerceptions.at(back_time).perception_[trackid].z);
    Eigen::Vector3d length_
    (lidarPerceptions.at(front_time).perception_[trackid].length, lidarPerceptions.at(middle_time).perception_[trackid].length, lidarPerceptions.at(back_time).perception_[trackid].length);
    Eigen::Vector3d width_
    (lidarPerceptions.at(front_time).perception_[trackid].width, lidarPerceptions.at(middle_time).perception_[trackid].width, lidarPerceptions.at(back_time).perception_[trackid].width);
    Eigen::Vector3d height_
    (lidarPerceptions.at(front_time).perception_[trackid].height, lidarPerceptions.at(middle_time).perception_[trackid].height, lidarPerceptions.at(back_time).perception_[trackid].height);
    Eigen::Vector3d pitch_
    (lidarPerceptions.at(front_time).perception_[trackid].pitch, lidarPerceptions.at(middle_time).perception_[trackid].pitch, lidarPerceptions.at(back_time).perception_[trackid].pitch);
    Eigen::Vector3d roll_(lidarPerceptions.at(front_time).perception_[trackid].roll, lidarPerceptions.at(middle_time).perception_[trackid].roll, lidarPerceptions.at(back_time).perception_[trackid].roll);
    Eigen::Vector3d yaw_(lidarPerceptions.at(front_time).perception_[trackid].yaw, lidarPerceptions.at(middle_time).perception_[trackid].yaw, lidarPerceptions.at(back_time).perception_[trackid].yaw);

    if(fabs(yaw_(0)) > M_PI_2 && fabs(yaw_(1)) > M_PI_2 && fabs(yaw_(2)) > M_PI_2 
        && (yaw_(0) / yaw_(1) < 0 || yaw_(0)/yaw_(2) < 0 || yaw_(1)/yaw_(2) < 0))
        {
            int index_diff = -1;
            if(yaw_(0) / yaw_(1) < 0){
                if(yaw_(0) / yaw_(2) < 0) index_diff = 0;
                else if(yaw_(1) / yaw_(2) < 0) index_diff = 1;
            }
            else index_diff = 2;
            if(yaw_[index_diff] > 0) yaw_[index_diff] -= 2*M_PI;
            else if(yaw_[index_diff] < 0) yaw_[index_diff] += 2*M_PI;
        }
    
    Lidar perception_at_camera_time;

    debug << coeff << std::endl << std::endl << y_ << std::endl;
    perception_at_camera_time.x = coeff.dot(x_);
    perception_at_camera_time.y = coeff.dot(y_);
    perception_at_camera_time.z = coeff.dot(z_);
    perception_at_camera_time.length = coeff.dot(length_);
    perception_at_camera_time.width = coeff.dot(width_);
    perception_at_camera_time.height = coeff.dot(height_);
    perception_at_camera_time.pitch = coeff.dot(pitch_);
    perception_at_camera_time.roll = coeff.dot(roll_);
    
    double temp_yaw = coeff.dot(yaw_);
    if(temp_yaw > M_PI) temp_yaw -= 2*M_PI;
    else if(temp_yaw < -M_PI) temp_yaw += 2*M_PI;
    perception_at_camera_time.yaw = temp_yaw;

    perception_at_camera_time.trackid = trackid;
    perception_at_camera_time.type = lidarPerceptions.at(front_time).perception_[trackid].type;

    perception_at_camera_time.tag = lidarPerceptions.at(front_time).perception_[trackid].tag;
    perception_at_camera_time.score = lidarPerceptions.at(front_time).perception_[trackid].score;

    camera_perception_result[camera_time].push_back(perception_at_camera_time);
    interpolation_result[camera_time].push_back(perception_at_camera_time);
    interpolated_timestamps[camera_time] = true;
    interpolated_ids[camera_time].insert({perception_at_camera_time.trackid, true});


    check_result << std::fixed << front_time << "    " 
    << trackid << " "
    << lidarPerceptions.at(front_time).perception_[trackid].x << " " <<  lidarPerceptions.at(front_time).perception_[trackid].y << " " << lidarPerceptions.at(front_time).perception_[trackid].z << " "
    << lidarPerceptions.at(front_time).perception_[trackid].length << " " << lidarPerceptions.at(front_time).perception_[trackid].width << " " << lidarPerceptions.at(front_time).perception_[trackid].height << " "
    << lidarPerceptions.at(front_time).perception_[trackid].yaw << std::endl;


    check_result << std::fixed << middle_time << "    " 
    << trackid << " "
    << lidarPerceptions.at(middle_time).perception_[trackid].x << " " <<  lidarPerceptions.at(middle_time).perception_[trackid].y << " " << lidarPerceptions.at(middle_time).perception_[trackid].z << " "
    << lidarPerceptions.at(middle_time).perception_[trackid].length << " " << lidarPerceptions.at(middle_time).perception_[trackid].width << " " << lidarPerceptions.at(middle_time).perception_[trackid].height << " "
    << lidarPerceptions.at(middle_time).perception_[trackid].yaw << std::endl;


    check_result << std::fixed << back_time << "    " 
    << trackid << " "
    << lidarPerceptions.at(back_time).perception_[trackid].x << " " <<  lidarPerceptions.at(back_time).perception_[trackid].y << " " << lidarPerceptions.at(back_time).perception_[trackid].z << " "
    << lidarPerceptions.at(back_time).perception_[trackid].length << " " << lidarPerceptions.at(back_time).perception_[trackid].width << " " << lidarPerceptions.at(back_time).perception_[trackid].height << " "
    << lidarPerceptions.at(back_time).perception_[trackid].yaw << std::endl;
                
    
    
    check_result << "----------------------------------------------------" << std::endl;
    check_result << std::fixed << camera_time << "    " 
    << perception_at_camera_time.trackid << " "
    << perception_at_camera_time.x << " " <<  perception_at_camera_time.y << " " << perception_at_camera_time.z << " "
    << perception_at_camera_time.length << " " << perception_at_camera_time.width << " " << perception_at_camera_time.height << " "
    << perception_at_camera_time.yaw << std::endl;
    check_result << std::endl;
    
    return true;
}


void Interpolation::onlineLinearInterpolation(std::map<double, LidarPerception>& lidarPerceptions,
                                        std::set<double>& lidar_timestamps, std::set<double>& timestamps_to_be_processed,
                                        std::map<double, std::vector<Lidar>>& interpolation_result,
                                        std::unordered_map<double, std::pair<double, double>>& cam_lidar_time)
{
    if(lidarPerceptions.size() <= 1) return;
    std::set<double>::iterator iter_timestamps;
    for(iter_timestamps = timestamps_to_be_processed.begin(); iter_timestamps != timestamps_to_be_processed.end(); iter_timestamps++){
        double camera_time = *iter_timestamps;
        double cam_time = camera_time + 50000;
        if(interpolated_timestamps[camera_time]) continue;//如果这个时间戳已经计算过，则跳过
        //找到lidar时间戳中，在要插值的相机时间戳的前后两帧
        std::set<double>::iterator iter_lidar_timestamps;
        double front_lidar_time = 0, back_lidar_time = 0;
        for(iter_lidar_timestamps = lidar_timestamps.begin(); iter_lidar_timestamps != lidar_timestamps.end(); iter_lidar_timestamps++){
            if(cam_time > *iter_lidar_timestamps){
                front_lidar_time = *iter_lidar_timestamps;
                continue;
            }
            else if(cam_time < *iter_lidar_timestamps){
                back_lidar_time = *iter_lidar_timestamps;
                break;
            }
        }

        //std::cout << std::fixed << "front " << front_lidar_time << " back " << back_lidar_time << std::endl;
        //std::cout << "back - front " << back_lidar_time - front_lidar_time << std::endl;
        //如果前后两帧lidar相差大于0.2s则不进行插值，这一条视精度要求可更改
        if(front_lidar_time == 0 || back_lidar_time == 0) continue;
        if(fabs(back_lidar_time - front_lidar_time) > 150000) continue;
        
        
        std::unordered_map<long int, Lidar>::iterator iter_lidar_front;
        for(iter_lidar_front = lidarPerceptions.at(front_lidar_time).perception_.begin();
        iter_lidar_front != lidarPerceptions.at(front_lidar_time).perception_.end(); iter_lidar_front++)//一个感知文件内多个被感知到的物体，按trackid遍历
        {
            if(lidarPerceptions.at(back_lidar_time).perception_.find(iter_lidar_front->second.trackid) != lidarPerceptions.at(back_lidar_time).perception_.end()
                    && iter_lidar_front->second.trackid >= 0)//前一帧lidar某个物体的感知 能否 在后一帧中能够找到，如果找到，则对这一物体的感知结果进行插值
            {
                //std::cout << iter_lidar_front->second.trackid << " " << iter_lidar_front->first << std::endl;
                Lidar front_lidar = lidarPerceptions.at(front_lidar_time).perception_[iter_lidar_front->second.trackid];
                Lidar back_lidar = lidarPerceptions.at(back_lidar_time).perception_[iter_lidar_front->second.trackid];
                std::set<SensorTimes, LESS_T0>::iterator iter;
                //std::unordered_map<double, SensorType> timestamp_sensortype; 
                if(fabs(front_lidar.yaw - back_lidar.yaw) > M_PI/4) continue; //前后帧出现偏航角的大跳变则视为断帧，不予插值
                //if(fabs(front_lidar.length - back_lidar.length) > 1.0) continue;
                
                double corrected_front_time = correctObjectTimestamp(front_lidar, front_lidar_time);
                double corrected_back_time = correctObjectTimestamp(back_lidar, back_lidar_time);

                //if(!lidarPerceptions.at(camera_time).perception_.count(iter_lidar_front->second.trackid)) continue;
                //double corrected_camera_time = correctObjectTimestamp(lidarPerceptions.at(camera_time).perception_[iter_lidar_front->second.trackid], camera_time);
                if(corrected_back_time - corrected_front_time < 30000) continue;

                Eigen::Vector3d front_pos = {front_lidar.x, front_lidar.y, front_lidar.z};
                Eigen::Vector3d back_pos = {back_lidar.x, back_lidar.y, back_lidar.z};
                Eigen::Vector3d front_dim = {front_lidar.length, front_lidar.width, front_lidar.height};
                Eigen::Vector3d back_dim = {back_lidar.length, back_lidar.width, back_lidar.height};
                Eigen::Vector3d front_pose = {front_lidar.pitch, front_lidar.roll, front_lidar.yaw};
                Eigen::Vector3d back_pose = {back_lidar.pitch, back_lidar.roll, back_lidar.yaw};

                Eigen::Vector3d camera_pos, camera_dim, camera_pose;
                
                double time_scale = (cam_time - corrected_front_time) / (corrected_back_time - corrected_front_time);
                //double time_scale = (camera_time - front_lidar_time) / (back_lidar_time - front_lidar_time);
                //double time_scale = (corrected_camera_time - corrected_front_time) / (corrected_back_time - corrected_front_time);
                camera_pos = time_scale * (back_pos - front_pos) + front_pos;
                camera_dim = time_scale * (back_dim - front_dim) + front_dim;
                if(abs(back_pose(2)) > M_PI_2 && abs(front_pose(2)) > M_PI_2 && front_pose(2)/back_pose(2) < 0){
                    if(back_pose(2) > 0){
                        back_pose(2) -= 2*M_PI;
                    }
                    else if(back_pose(2) < 0){
                        back_pose(2) += 2*M_PI; 
                    }
                }
                camera_pose = time_scale * (back_pose - front_pose) + front_pose;
                if(camera_pose(2) > M_PI) camera_pose(2) -= 2*M_PI;
                else if(camera_pose(2) < -M_PI) camera_pose(2) += 2*M_PI;

                Lidar perception_at_camera_time;
                
                perception_at_camera_time.x = camera_pos(0);
                perception_at_camera_time.y = camera_pos(1);
                perception_at_camera_time.z = camera_pos(2);
                perception_at_camera_time.length = camera_dim(0);
                perception_at_camera_time.width = camera_dim(1);
                perception_at_camera_time.height = camera_dim(2);
                perception_at_camera_time.pitch = camera_pose(0);
                perception_at_camera_time.roll = camera_pose(1);
                perception_at_camera_time.yaw = camera_pose(2);
                perception_at_camera_time.trackid = front_lidar.trackid;
                perception_at_camera_time.type = front_lidar.type;

                perception_at_camera_time.tag = front_lidar.tag;
                perception_at_camera_time.score = front_lidar.score;
            
                camera_perception_result[camera_time].push_back(perception_at_camera_time);
                interpolation_result[camera_time].push_back(perception_at_camera_time);
                cam_lidar_time[camera_time] = std::make_pair(front_lidar_time, back_lidar_time);
                debug << std::fixed << camera_time << std::endl;
                interpolated_timestamps[camera_time] = true;

                check_result << std::fixed << front_lidar_time << std::endl;
                check_result << std::fixed << corrected_front_time << "    " 
                << iter_lidar_front->second.trackid << " "
                << front_lidar.x << " " <<  front_lidar.y << " " << front_lidar.z << " "
                << front_lidar.length << " " << front_lidar.width << " " << front_lidar.height << " "
                << front_lidar.yaw << std::endl;

                
                check_result << std::fixed << camera_time << "    "
                << perception_at_camera_time.trackid << " "
                << perception_at_camera_time.x << " " <<  perception_at_camera_time.y << " " << perception_at_camera_time.z << " "
                << perception_at_camera_time.length << " " << perception_at_camera_time.width << " " << perception_at_camera_time.height << " "
                << perception_at_camera_time.yaw << std::endl; 

                check_result << std::fixed << corrected_back_time << "    " 
                << iter_lidar_front->second.trackid << " "
                << back_lidar.x << " " <<  back_lidar.y << " " << back_lidar.z << " "
                << back_lidar.length << " " << back_lidar.width << " " << back_lidar.height << " "
                << back_lidar.yaw << std::endl;
                check_result << std::fixed << back_lidar_time << std::endl;
                check_result << std::endl;
            }
        }
    }
    calculate_lidar_error();
}

double Interpolation::correctObjectTimestamp(Lidar& lidar, double timestamp)
{
    /*
    double s_per_rad = 100000 / (2*M_PI);
    double delta_angle = std::atan2(lidar.y, lidar.x);
    return timestamp - delta_angle * s_per_rad; //lidar顺时针转
    */
    double s_per_rad = 100000 / (2*M_PI);
    double delta_angle = std::atan2(lidar.y, lidar.x);
    if(delta_angle <= 0){
        return timestamp + (-M_PI - delta_angle) * s_per_rad;
    }
    else{
        return timestamp - (M_PI + delta_angle) * s_per_rad;
    }
}

void Interpolation::calculate_lidar_error(){
    std::map<double, std::vector<Lidar>>::iterator iter_print;
    for(iter_print = camera_perception_result.begin(); iter_print != camera_perception_result.end(); iter_print++){
        for(int i = 0; i < iter_print->second.size(); i++){
            if(lidarPerceptions.at(iter_print->first).perception_.count(iter_print->second[i].trackid)){
                /**check_lidar_result << std::fixed << iter_print->first << "    " 
                << iter_print->second[i].trackid << " "
                << iter_print->second[i].x << " " <<  iter_print->second[i].y << " " << iter_print->second[i].z << " "
                << iter_print->second[i].length << " " << iter_print->second[i].width << " " << iter_print->second[i].height << " "
                << iter_print->second[i].yaw << std::endl;
                check_lidar_result << std::fixed << iter_print->first << "    " 
                << lidarPerceptions.at(iter_print->first).perception_[iter_print->second[i].trackid].trackid << " "
                << lidarPerceptions.at(iter_print->first).perception_[iter_print->second[i].trackid].x << " " 
                << lidarPerceptions.at(iter_print->first).perception_[iter_print->second[i].trackid].y << " " 
                << lidarPerceptions.at(iter_print->first).perception_[iter_print->second[i].trackid].z << " "
                << lidarPerceptions.at(iter_print->first).perception_[iter_print->second[i].trackid].length << " " 
                << lidarPerceptions.at(iter_print->first).perception_[iter_print->second[i].trackid].width << " " 
                << lidarPerceptions.at(iter_print->first).perception_[iter_print->second[i].trackid].height << " "
                << lidarPerceptions.at(iter_print->first).perception_[iter_print->second[i].trackid].yaw << std::endl;**/

                double x = iter_print->second[i].x;
                double y = iter_print->second[i].y;
                double z = iter_print->second[i].z;
                double l = iter_print->second[i].length;
                double w = iter_print->second[i].width;
                double h = iter_print->second[i].height;
                double yaw = iter_print->second[i].yaw;

                double x_truth = lidarPerceptions.at(iter_print->first).perception_[iter_print->second[i].trackid].x;
                double y_truth = lidarPerceptions.at(iter_print->first).perception_[iter_print->second[i].trackid].y;
                double z_truth = lidarPerceptions.at(iter_print->first).perception_[iter_print->second[i].trackid].z;
                double l_truth = lidarPerceptions.at(iter_print->first).perception_[iter_print->second[i].trackid].length;
                double w_truth = lidarPerceptions.at(iter_print->first).perception_[iter_print->second[i].trackid].width;
                double h_truth = lidarPerceptions.at(iter_print->first).perception_[iter_print->second[i].trackid].height;
                double yaw_truth = lidarPerceptions.at(iter_print->first).perception_[iter_print->second[i].trackid].yaw;

                std::cout << " x " << x << " x truth " << x_truth << std::endl;

                if(fabs(yaw - yaw_truth) >= 3 && fabs(yaw - yaw_truth) <=6 ){
                    if(yaw_truth > 0) yaw_truth -= M_PI;
                    else yaw_truth += M_PI;
                }


                Eigen::Vector3d center(x,y,z);

                Eigen::Matrix3d transform;
                
                transform << cos(yaw), -sin(yaw), 0,
                            sin(yaw), cos(yaw), 0,
                            0,          0,      1;

                std::vector<Eigen::Vector3d> bbox;
                Eigen::Vector3d p1(x - 0.5 * l, y + 0.5 * w, z - 0.5 * h); 
                Eigen::Vector3d p2(x - 0.5 * l, y - 0.5 * w, z - 0.5 * h);
                Eigen::Vector3d p3(x + 0.5 * l, y - 0.5 * w, z - 0.5 * h);
                Eigen::Vector3d p4(x + 0.5 * l, y + 0.5 * w, z - 0.5 * h);
                Eigen::Vector3d p5(x - 0.5 * l, y + 0.5 * w, z + 0.5 * h);
                Eigen::Vector3d p6(x - 0.5 * l, y - 0.5 * w, z + 0.5 * h);  
                Eigen::Vector3d p7(x + 0.5 * l, y - 0.5 * w, z + 0.5 * h);               
                Eigen::Vector3d p8(x + 0.5 * l, y + 0.5 * w, z + 0.5 * h);
                
                bbox.push_back(p1);
                bbox.push_back(p2);
                bbox.push_back(p3);
                bbox.push_back(p4);
                bbox.push_back(p5);
                bbox.push_back(p6);
                bbox.push_back(p7);
                bbox.push_back(p8);

                for(int j = 0; j < bbox.size(); j++){
                    bbox[j] = center + transform*(bbox[j] - center);
                }

                Eigen::Vector3d center_truth(x_truth, y_truth, z_truth);

                Eigen::Matrix3d transform_truth;
                transform_truth << cos(yaw_truth), -sin(yaw_truth), 0,
                                sin(yaw_truth), cos(yaw_truth), 0,
                                    0,          0,      1;
                std::vector<Eigen::Vector3d> bbox_truth;

                Eigen::Vector3d p1_truth(x_truth - 0.5 * l_truth, y_truth + 0.5 * w_truth, z_truth - 0.5 * h_truth);                
                Eigen::Vector3d p2_truth(x_truth - 0.5 * l_truth, y_truth - 0.5 * w_truth, z_truth - 0.5 * h_truth);               
                Eigen::Vector3d p3_truth(x_truth + 0.5 * l_truth, y_truth - 0.5 * w_truth, z_truth - 0.5 * h_truth);                
                Eigen::Vector3d p4_truth(x_truth + 0.5 * l_truth, y_truth + 0.5 * w_truth, z_truth - 0.5 * h_truth);                
                Eigen::Vector3d p5_truth(x_truth - 0.5 * l_truth, y_truth + 0.5 * w_truth, z_truth + 0.5 * h_truth);
                Eigen::Vector3d p6_truth(x_truth - 0.5 * l_truth, y_truth - 0.5 * w_truth, z_truth + 0.5 * h_truth);                
                Eigen::Vector3d p7_truth(x_truth + 0.5 * l_truth, y_truth - 0.5 * w_truth, z_truth + 0.5 * h_truth);               
                Eigen::Vector3d p8_truth(x_truth + 0.5 * l_truth, y_truth + 0.5 * w_truth, z_truth + 0.5 * h_truth);

                bbox_truth.push_back(p1_truth);
                bbox_truth.push_back(p2_truth);
                bbox_truth.push_back(p3_truth);
                bbox_truth.push_back(p4_truth);
                bbox_truth.push_back(p5_truth);
                bbox_truth.push_back(p6_truth);
                bbox_truth.push_back(p7_truth);
                bbox_truth.push_back(p8_truth);
            
                for(int j = 0; j < bbox_truth.size(); j++){
                    bbox_truth[j] = center_truth + transform_truth * (bbox_truth[j] - center_truth);
                }


                error_bbox << std::fixed << iter_print->first <<" " << iter_print->second[i].trackid << std::endl;                
                error_bbox <<"--------------------------------------------"<< std::endl;
                error_bbox << "yaw: " << yaw << " yaw_truth " << yaw_truth << std::endl;
                error_bbox << transform << std::endl;
                error_bbox << std::endl;
                                
                error_bbox << transform_truth << std::endl;
                for(int j = 0; j < bbox_truth.size(); j++){
                    error_bbox << "预测： "<< bbox[j](0) << " " << bbox[j](1) << " " << bbox[j](2) << std::endl ;
                    error_bbox << "真值： "<< bbox_truth[j](0) << " " << bbox_truth[j](1) << " " << bbox_truth[j](2) << std::endl ;
                }
                error_bbox << std::endl;

                
                double error_norm = 0;
                for(int j = 0; j < bbox.size(); j++){
                    Eigen::Vector3d error_p = bbox[j] - bbox_truth[j];
                    error_norm += error_p.norm();
                    //error_bbox << fabs(error_p(0)) << " " << fabs(error_p(1)) << " " << fabs(error_p(2)) << std::endl;
                    //if(fabs(error_p(0)) > 10)
                    //error_x << std::fixed << iter_print->first << " " << iter_print->second[i].trackid << " " << fabs(error_p(0)) << " " << yaw << " " << yaw_truth << std::endl;
                    error_x << error_p(0) << std::endl;
                    //if(fabs(error_p(1)) > 5)
                    //error_y  << fabs(error_p(1)) << std::endl;//<< std::fixed << iter_print->first << " " << iter_print->second[i].trackid << " " << fabs(error_p(1)) << " " << yaw << " " << yaw_truth << std::endl;
                    error_y << error_p(1) << std::endl;
                    //error_z << fabs(error_p(2)) << std::endl;
                    error_z << error_p(2) << std::endl;
                }
                error_bbox << std::endl;
                error_norm /= bbox.size();
                error_norm_file << error_norm << std::endl;
                //error_x << fabs(iter_print->second[i].x - lidarPerceptions.at(iter_print->first).perception_[iter_print->second[i].trackid].x) << std::endl;
                //error_y << fabs(iter_print->second[i].y - lidarPerceptions.at(iter_print->first).perception_[iter_print->second[i].trackid].y) << std::endl;
                //error_z << fabs(iter_print->second[i].z - lidarPerceptions.at(iter_print->first).perception_[iter_print->second[i].trackid].z) << std::endl;
                error_length << fabs(iter_print->second[i].length - lidarPerceptions.at(iter_print->first).perception_[iter_print->second[i].trackid].length) << std::endl;
                error_width << fabs(iter_print->second[i].width - lidarPerceptions.at(iter_print->first).perception_[iter_print->second[i].trackid].width) << std::endl;
                error_height << fabs(iter_print->second[i].height - lidarPerceptions.at(iter_print->first).perception_[iter_print->second[i].trackid].height) << std::endl;
                double err_yaw = fabs(iter_print->second[i].yaw - lidarPerceptions.at(iter_print->first).perception_[iter_print->second[i].trackid].yaw);
                if(err_yaw > 2 * M_PI / 3) err_yaw = fabs(err_yaw - 2 * M_PI);
                error_yaw <<  err_yaw << std::endl; //<< std::fixed << iter_print->first << " " << iter_print->second[i].trackid << " " 
            }
        }
    }
}

void Interpolation::onlineWriteToJson(std::vector<double>& timestamps, std::unordered_map<double, std::vector<SensorType>>& timestamp_sensortype){
    if(camera_perception_result.empty()){
        nlohmann::json jsonfiles;
        jsonfiles["children"] = nlohmann::json::array();
        jsonfiles["tag"] = "_";
        jsonfiles["version"] = "1.0";
        std::unordered_map<double, std::vector<SensorType>>::iterator iter_time_sensor;
        for(auto timestamp : timestamps){
            for(int i = 0; i < timestamp_sensortype[timestamp].size(); i++){
                    std::experimental::filesystem::path dirs(outputPath + dir + "/" + camera_folders[timestamp_sensortype[timestamp][i]]);
                    if(!(std::experimental::filesystem::exists(dirs))){
                        if(std::experimental::filesystem::create_directories(dirs))
                        {
                            std::cout << "create " << outputPath + dir + "/" + camera_folders[timestamp_sensortype[timestamp][i]] << std::endl;
                        }
                    }
                    std::string save_json_path = outputPath + dir + "/" + camera_folders[timestamp_sensortype[timestamp][i]] + std::to_string(timestamp).substr(0,16) + ".pcd.tar.json";
                    std::ofstream save_json_to_file(save_json_path);
                    save_json_to_file <<std::setw(4)<< jsonfiles;
            }
        }
    }
    else{
        std::map<double, std::vector<Lidar>>::iterator iter;
        for(iter = camera_perception_result.begin(); iter != camera_perception_result.end(); iter++){//按照lidar两帧之间所有相机时间戳进行循环
            nlohmann::json jsonfiles;
            for(int i = 0; i < iter->second.size(); i++){//每个相机时间戳对应的感知结果，多个物体感知
                nlohmann::json json_file;
                json_file["data"] = {
                    {"isKeyCube", {false}}, 
                    {"isKeyPropertyCube", {false}}, 
                    {"type", {iter->second[i].type}}};
                json_file["height"] = iter->second[i].height;
                json_file["length"] = iter->second[i].length;
                json_file["pitch"] = iter->second[i].pitch;
                json_file["roll"] = iter->second[i].roll;
                json_file["score"] = iter->second[i].score;
                json_file["tag"] = iter->second[i].tag;
                json_file["uuid"] = iter->second[i].trackid;
                json_file["width"] = iter->second[i].width;
                json_file["x"] = iter->second[i].x;
                json_file["y"] = iter->second[i].y;
                json_file["yaw"] = iter->second[i].yaw;
                json_file["z"] = iter->second[i].z;

                jsonfiles["children"].push_back(json_file);
            }
            jsonfiles["tag"] = "_";
            jsonfiles["version"] = "1.0";
            
            for(int i = 0; i < timestamp_sensortype[iter->first].size(); i++){
                std::experimental::filesystem::path dirs(outputPath + dir + "/" + camera_folders[timestamp_sensortype[iter->first][i]]);
                if(!(std::experimental::filesystem::exists(dirs))){
                    if(std::experimental::filesystem::create_directories(dirs))
                    {
                        std::cout << "create " << outputPath + dir + "/" + camera_folders[timestamp_sensortype[iter->first][i]] << std::endl;
                    }
                }
                std::string save_json_path = outputPath + dir + "/" + camera_folders[timestamp_sensortype[iter->first][i]] + std::to_string(iter->first).substr(0,16) + ".pcd.tar.json";
                std::ofstream save_json_to_file;
                save_json_to_file.open(save_json_path, std::ofstream::app);
                save_json_to_file <<std::setw(4)<< jsonfiles;
            }
        }
    }
    camera_perception_result.clear();
}
