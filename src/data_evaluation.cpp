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
			if (fname.find(extension, (fname.length() - extension.length())) != std::string::npos)
				lidar_files.push_back(fname);		// add filename to results vector
		}
		entry = readdir(dir_point);
	}
	return;
}


int main(int argc, char *argv[])
{
    if(argc < 2) {
        std::cout << "Not enough arguments" << std::endl;
        return 0;
    }
    std::string perception_path = argv[1];

// 读取lidar 感知结果
    //std::vector<LidarPerception> perceptions; 
    std::unordered_map<std::string, std::vector<Lidar>> perceptions_by_ids;
 
    std::vector<std::string> lidar_files;
    std::set<SensorTimes, LESS_T0> lidar_times;
    search(argv[1], "json", lidar_files); // read all lidar json file name

    for(int i = 0; i < lidar_files.size(); i++){
        LidarPerception lidar_perception(perception_path + lidar_files[i]);
        double lidar_timestamp = std::stod(lidar_files[i].substr(0, lidar_files[i].find(".")));
        //perceptions.push_back(lidar_perception);
        std::unordered_map<std::string, Lidar>::iterator iter;
        for(iter = lidar_perception.perception_.begin(); iter != lidar_perception.perception_.end(); iter++){
            perceptions_by_ids[iter->first].push_back(iter->second);
        }
    }

}