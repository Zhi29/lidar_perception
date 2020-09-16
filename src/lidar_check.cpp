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
        std::set<SensorTimes, LESS_T0> lidar_times;
        search(project_path + dir + "/msd_score_lidar_ap/", "new", lidar_files); // read all lidar json file name

        for(int i = 0; i < lidar_files.size(); i++){
            LidarPerception lidar_perception(project_path + dir + "/msd_score_lidar_ap/" + lidar_files[i]);
            double lidar_timestamp = std::stod(lidar_files[i].substr(0, lidar_files[i].find(".")));
            SensorTimes lidar_time(lidar_timestamp, SensorType::LIDAR);
            lidar_times.insert(lidar_time);
            perceptions.insert({lidar_timestamp, lidar_perception});
        }


        Interpolation interpolation(outputPath, dir);

        double front_lidar_time = lidar_times.begin()->time;
        lidar_times.erase(lidar_times.begin());
        double time_to_be_processed = lidar_times.begin()->time;
        lidar_times.erase(lidar_times.begin());
        double back_lidar_time = lidar_times.begin()->time;
        lidar_times.erase(lidar_times.begin());

        while(!lidar_times.empty()){
            interpolation.lidarCheckInterpolate(perceptions, front_lidar_time, back_lidar_time, time_to_be_processed);
            front_lidar_time = time_to_be_processed;
            time_to_be_processed = back_lidar_time;
            back_lidar_time = lidar_times.begin()->time;
            lidar_times.erase(lidar_times.begin());
        }
        
 
    }//外层文件夹的循环结束

}

