#include <iostream>
#include <unordered_map>
#include "read_json.hpp"
#include <dirent.h>
#include <regex>
#include <sys/types.h>
#include <set>
#include "interpolation.hpp"

void search(std::string curr_directory, std::string extension, std::vector<std::string>& lidar_files){
    DIR* dir_point = opendir(curr_directory.c_str());
	dirent* entry = readdir(dir_point);
	while (entry){									// if !entry then end of directory
		if (entry->d_type == DT_DIR){				// if entry is a directory
			std::string fname = entry->d_name;
			if (fname != "." && fname != "..")
				search(entry->d_name, extension, lidar_files);	// search through it
		}
		else if (entry->d_type == DT_REG){		// if entry is a regular file
			std::string fname = entry->d_name;	// filename
												// if filename's last characters are extension
            if(fname.substr(0,1).compare(".") != 0){
                if (fname.find(extension, (fname.length() - extension.length())) != std::string::npos)
                    lidar_files.push_back(fname);		// add filename to results vector
            }
		}
		entry = readdir(dir_point);
	}
	return;
}

void search_directories(std::string curr_directory, std::vector<std::string>& sub_directories){
    DIR* dir_point = opendir(curr_directory.c_str());
	dirent* entry = readdir(dir_point);
	while (entry){									// if !entry then end of directory
		if (entry->d_type == DT_DIR){				// if entry is a directory
			std::string fname = entry->d_name;
            //std::cout << fname << std::endl;
			if (fname != "." && fname != "..")
			{
                //if(fname.find(extension, (fname.length() - extension.length())) != std::string::npos)
                sub_directories.push_back(fname);
            }
		}
		entry = readdir(dir_point);
	}
	return;
}

int main(int argc, char *argv[])
{
    if(argc < 3){
        std::cout << "Not enough argument" << std::endl;
        return 0;
    }
    //std::string perception_path = argv[1];
    //std::string egomotion_path = argv[2];
    std::string project_path = argv[1];
    std::string outputPath = argv[2];

//获得所有数据包的文件夹名称
    std::vector<std::string> project_dirs;
    search_directories(project_path, project_dirs);
    for(int i = 0; i < project_dirs.size(); i++){
        std::cout << project_dirs[i] << std::endl;
    }

    for(auto dir : project_dirs){


    // 读取lidar 感知结果
        std::unordered_map<double, LidarPerception> perceptions;    
        std::vector<std::string> lidar_files;
        //std::set<SensorTimes, LESS_T0> lidar_times;
        std::priority_queue<double, std::vector<double>, std::greater<double>> lidar_times;
        search(project_path + dir + "/msd_score_lidar_ap/", "new", lidar_files); // read all lidar json file name

        std::vector<double> timestamps_to_be_processed;
        
        Interpolation interpolation(outputPath, dir);

        for(int i = 0; i < lidar_files.size(); i++){
            LidarPerception lidar_perception(project_path + dir + "/msd_score_lidar_ap/" + lidar_files[i]);
            double lidar_timestamp = std::stod(lidar_files[i].substr(0, lidar_files[i].find(".")));
            lidar_perception.timestamp = lidar_timestamp;
            
            lidar_times.push(lidar_timestamp);
            //timestamps_to_be_processed.push_back(lidar_timestamp);
            //SensorTimes lidar_time(lidar_timestamp, SensorType::LIDAR);
            //lidar_times.insert(lidar_time);
            perceptions.insert({lidar_timestamp, lidar_perception});
        }

        std::priority_queue<double, std::vector<double>, std::greater<double>> timestamps_copy = lidar_times;
        timestamps_copy.pop();
        std::map<double, std::vector<Lidar>> interpolation_results;   
        std::unordered_map<double, std::pair<double, double>> cam_lidar_time;
        std::unordered_map<double, std::unordered_map<long int, std::pair<double, double>>> cam_lidar_time_by_ids;
        while(!lidar_times.empty()){
            //std::cout << "priority top " << timestamps_copy.top() << std::endl;
            if(interpolation.lidarPerceptions.size() >= 3)
                timestamps_to_be_processed.push_back(timestamps_copy.top());
            //interpolation.onlineInterpolation(perceptions.at(lidar_times.top()), timestamps_to_be_processed, interpolation_results, cam_lidar_time);
            //std::cout << timestamps_to_be_processed.size() << std::endl;
            interpolation.offlineInterpolation(perceptions.at(lidar_times.top()), timestamps_to_be_processed, interpolation_results, cam_lidar_time_by_ids);
            lidar_times.pop();
            if(interpolation.lidarPerceptions.size() >= 3)
                timestamps_copy.pop();
            timestamps_to_be_processed.clear();
        }
        interpolation.calculate_lidar_error();
        interpolation.camera_perception_result.clear();
        interpolation.object_timestamps.clear();
        interpolation.find_lidar_timestamp.clear();
        interpolation.all_camera_time.clear();
        interpolation.timestamps_to_be_processed.clear();
        std::cout << interpolation_results.size() << std::endl;
    }//外层文件夹的循环结束

}

