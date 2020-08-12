
#include "interpolation.hpp"


void Interpolation::interpolateCameras(std::set<SensorTimes, LESS_T0>& data_to_be_processed, std::unordered_map<double, LidarPerception>& perceptions)
{
    std::set<SensorTimes, LESS_T0>::iterator iter;
    std::unordered_map<double, SensorType> timestamp_sensortype; 
    for(iter = data_to_be_processed.begin(); iter != data_to_be_processed.end(); iter++){ // 按不同相机循环，每次处理一个时间对应的相机
        if(iter->type == SensorType::LIDAR) continue;
        else{
            std::unordered_map<long int, Lidar>::iterator iter_lidar_front;
            double front_lidar_time = data_to_be_processed.begin()->time;
            double back_lidar_time = data_to_be_processed.rbegin()->time;
            if(data_to_be_processed.begin()->type != SensorType::LIDAR) std::cout << "front not lidar" << std::endl;
            if(data_to_be_processed.rbegin()->type != SensorType::LIDAR) std::cout << "back not lidar" << std::endl;
            for(iter_lidar_front = perceptions.at(front_lidar_time).perception_.begin();
                iter_lidar_front != perceptions.at(front_lidar_time).perception_.end(); iter_lidar_front++)//一个感知文件内多个被感知到的物体，按trackid遍历
            {
                if(perceptions.at(back_lidar_time).perception_.find(iter_lidar_front->second.trackid) != perceptions.at(back_lidar_time).perception_.end()){
                    //前一帧lidar某个物体的感知 能否 在后一帧中能够找到，如果找到，则对这一物体的感知结果进行插值
                    Lidar front_lidar = perceptions.at(front_lidar_time).perception_[iter_lidar_front->second.trackid];
                    Lidar back_lidar = perceptions.at(back_lidar_time).perception_[iter_lidar_front->second.trackid];

                    linearInterpolates(front_lidar, back_lidar, iter->type, iter->time, front_lidar_time, back_lidar_time);
                }
            }
            timestamp_sensortype[iter->time] = iter->type;
        }
    }
    writeToJson(timestamp_sensortype);
}

void Interpolation::linearInterpolates(Lidar& front_lidar, Lidar& back_lidar, SensorType sensortype, double camera_time, double &front_lidar_time, double &back_lidar_time){
    Eigen::Vector3d front_pos = {front_lidar.x, front_lidar.y, front_lidar.z};
    Eigen::Vector3d back_pos = {back_lidar.x, back_lidar.y, back_lidar.z};
    Eigen::Vector3d front_dim = {front_lidar.length, front_lidar.width, front_lidar.height};
    Eigen::Vector3d back_dim = {back_lidar.length, back_lidar.width, back_lidar.height};
    Eigen::Vector3d front_pose = {front_lidar.pitch, front_lidar.roll, front_lidar.yaw};
    Eigen::Vector3d back_pose = {back_lidar.pitch, back_lidar.roll, back_lidar.yaw};

    Eigen::Vector3d camera_pos, camera_dim, camera_pose;
    double time_scale = (camera_time - front_lidar_time) / (back_lidar_time - front_lidar_time);
    camera_pos = time_scale * (back_pos - front_pos) + front_pos;
    camera_dim = time_scale * (back_dim - front_dim) + front_dim;
    camera_pose = time_scale * (back_pose - front_pose) + front_pose;

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

    camera_perception_result[camera_time].push_back(perception_at_camera_time);//这里有问题，应该是按时间搜索，不应该按type，因为一个type对应多个时间,但是也需要type
}

void Interpolation::writeToJson(std::unordered_map<double, SensorType>& timestamp_sensortype){
    std::unordered_map<double, std::vector<Lidar>>::iterator iter;
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
        

        std::string save_json_path = outputPath + camera_folders[timestamp_sensortype[iter->first]] + std::to_string(iter->first) + ".pcd.tar.json";
        //std::cout << save_json_path << std::endl;
        std::ofstream save_json_to_file(save_json_path);
        jsonfiles.dump(4);
        save_json_to_file <<std::setw(4)<< jsonfiles;
        //save_json_to_file.write(jsonfiles, save_json_to_file, pretty_print);
    }
    

}