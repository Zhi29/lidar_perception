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
    if(argc < 2){
        std::cout << "Not enough argument" << std::endl;
        return 0;
    }
    //std::string perception_path = argv[1];
    //std::string egomotion_path = argv[2];
    std::string project_path = argv[1];
    std::string outputPath = "/home/lizhi/HDmap/test_lidar_interpolation/";

//获得所有数据包的文件夹名称
    std::vector<std::string> project_dirs;
    search_directories(project_path, project_dirs);
    //for(int i = 0; i < project_dirs.size(); i++){
    //    std::cout << project_dirs[i] << std::endl;
    //}

    for(auto dir : project_dirs){


    // 读取lidar 感知结果
        std::unordered_map<double, LidarPerception> perceptions;    
        std::vector<std::string> lidar_files;
        std::set<SensorTimes, LESS_T0> lidar_times;
        search(project_path + dir + "/msd_score_lidar_ap/", "new", lidar_files); // read all lidar json file name
        //for(int i = 0; i < lidar_files.size(); i++){
        //    std::cout << lidar_files[i] << std::endl;
        //}

        for(int i = 0; i < lidar_files.size(); i++){
            LidarPerception lidar_perception(project_path + dir + "/msd_score_lidar_ap/" + lidar_files[i]);
            double lidar_timestamp = std::stod(lidar_files[i].substr(0, lidar_files[i].find(".")));
            SensorTimes lidar_time(lidar_timestamp, SensorType::LIDAR);
            lidar_times.insert(lidar_time);
            //lidar_files[i] = lidar_files[i].substr(0, lidar_files[i].find("."));
            //std::cout << std::fixed << lidar_timestamp << std::endl;
            perceptions.insert({lidar_timestamp, lidar_perception});
            //std::cout << lidar_files[i] << std::endl;
        }

    //读取ego motion 结果
    /**
        std::vector<std::string> egomotion_files;
        std::unordered_map<double, EgoMotionReader> egoMotions;
        search(project_path + dir + "/navi_fusion/", "json", egomotion_files);
        for(int i = 0; i < egomotion_files.size(); i++){
            EgoMotionReader ego_reader(project_path + dir + "/navi_fusion/" + egomotion_files[i]);
            egoMotions.insert({ego_reader.timestamp, ego_reader});
            std::cout << ego_reader.timestamp << std::endl;
            std::cout << egomotion_files[i] << std::endl;
        }
        **/

    //读取FISH_EYE_F的时间戳
        std::vector<std::string> fish_eye_f_files;
        std::set<SensorTimes, LESS_T0> fish_eye_f_;
        search(project_path + dir + "/fisheye_F/", "jpg", fish_eye_f_files);
        for(int i = 0; i < fish_eye_f_files.size(); i++){
            SensorTimes fish_eye_f(stod(fish_eye_f_files[i].substr(0, fish_eye_f_files[i].find("."))), SensorType::FISH_EYE_F);
            fish_eye_f_.insert(fish_eye_f);
        }
        fish_eye_f_files.clear();

        //std::set<SensorTimes, LESS_T0>::iterator iter;
        //for(iter = fish_eye_f_.begin(); iter != fish_eye_f_.end(); iter++) std::cout << iter->time << std::endl;

    //读取FISH_EYE_B的时间戳
        std::vector<std::string> fish_eye_b_files;
        std::set<SensorTimes, LESS_T0> fish_eye_b_;
        search(project_path + dir + "/fisheye_B/", "jpg", fish_eye_b_files);
        for(int i = 0; i < fish_eye_b_files.size(); i++){
            SensorTimes fish_eye_b(stod(fish_eye_b_files[i].substr(0, fish_eye_b_files[i].find("."))), SensorType::FISH_EYE_B);
            fish_eye_b_.insert(fish_eye_b);
        }
        fish_eye_b_files.clear();

    //读取FISH_EYE_L的时间戳
        std::vector<std::string> fish_eye_l_files;
        std::set<SensorTimes, LESS_T0> fish_eye_l_;
        search(project_path + dir + "/fisheye_L/", "jpg", fish_eye_l_files);
        for(int i = 0; i < fish_eye_l_files.size(); i++){
            SensorTimes fish_eye_l(stod(fish_eye_l_files[i].substr(0, fish_eye_l_files[i].find("."))), SensorType::FISH_EYE_L);
            fish_eye_l_.insert(fish_eye_l);
        }
        fish_eye_l_files.clear();

    //读取FISH_EYE_R的时间戳
        std::vector<std::string> fish_eye_r_files;
        std::set<SensorTimes, LESS_T0> fish_eye_r_;
        search(project_path + dir + "/fisheye_R/", "jpg", fish_eye_r_files);
        for(int i = 0; i < fish_eye_r_files.size(); i++){
            SensorTimes fish_eye_r(stod(fish_eye_r_files[i].substr(0, fish_eye_r_files[i].find("."))), SensorType::FISH_EYE_R);
            fish_eye_r_.insert(fish_eye_r);
        }
        fish_eye_r_files.clear();

        std::set<SensorTimes, LESS_T0> data_to_be_processed;
        data_to_be_processed.insert(*lidar_times.begin());
        std::cout << "lidar size before: " << lidar_times.size() << std::endl;
        lidar_times.erase(lidar_times.begin());
        data_to_be_processed.insert(*lidar_times.begin());
        lidar_times.erase(lidar_times.begin());
        std::cout << "lidar size after: " << lidar_times.size() << std::endl;

        
        std::vector<SensorType> sensortypes = {SensorType::FISH_EYE_F, SensorType::FISH_EYE_B, 
                                                SensorType::FISH_EYE_L, SensorType::FISH_EYE_R
                                                };

        Interpolation interpolation(outputPath);

        while(!lidar_times.empty())
        {
            for(int i = 0; i < sensortypes.size(); i++){
                switch(sensortypes[i])
                {
                    case SensorType::FISH_EYE_F:{
                        //std::cout << "fish time ; begin ; rbegin: " << std::fixed << fish_eye_f_.begin()->time 
                        //<<  data_to_be_processed.begin()->time << data_to_be_processed.rbegin()->time << std::endl;
                        while(fish_eye_f_.begin()->time < data_to_be_processed.rbegin()->time){
                            if(fish_eye_f_.begin()->time < data_to_be_processed.begin()->time){
                                fish_eye_f_.erase(fish_eye_f_.begin());
                            }
                            else{
                                data_to_be_processed.insert(*fish_eye_f_.begin());
                                fish_eye_f_.erase(fish_eye_f_.begin());
                            }
                            //std::cout << "fish_eye_f_ " << fish_eye_f_.size() << std::endl;
                        }
                        break;
                    }
                    case SensorType::FISH_EYE_B:{
                        while(fish_eye_b_.begin()->time < data_to_be_processed.rbegin()->time){
                            if(fish_eye_b_.begin()->time < data_to_be_processed.begin()->time){
                                fish_eye_b_.erase(fish_eye_b_.begin());
                            }
                            else{
                                data_to_be_processed.insert(*fish_eye_b_.begin());
                                fish_eye_b_.erase(fish_eye_b_.begin());
                            }
                        }
                        break;
                    }
                    case SensorType::FISH_EYE_L:{
                        while(fish_eye_l_.begin()->time < data_to_be_processed.rbegin()->time){
                            if(fish_eye_l_.begin()->time < data_to_be_processed.begin()->time){
                                fish_eye_l_.erase(fish_eye_l_.begin());
                            }
                            else{
                                data_to_be_processed.insert(*fish_eye_l_.begin());
                                fish_eye_l_.erase(fish_eye_l_.begin());
                            }
                        }
                        break;
                    }
                    case SensorType::FISH_EYE_R:{
                        while(fish_eye_r_.begin()->time < data_to_be_processed.rbegin()->time){
                            if(fish_eye_r_.begin()->time < data_to_be_processed.begin()->time){
                                fish_eye_r_.erase(fish_eye_r_.begin());
                            }
                            else{
                                data_to_be_processed.insert(*fish_eye_r_.begin());
                                fish_eye_r_.erase(fish_eye_r_.begin());
                            }
                        }
                        break;
                    }
                    default: 
                        break;
                }
            }

            std::set<SensorTimes, LESS_T0>::iterator iter_data_tbp;
            for(iter_data_tbp = data_to_be_processed.begin(); iter_data_tbp != data_to_be_processed.end(); iter_data_tbp++)
                std::cout << std::fixed << iter_data_tbp->time << std::endl;
            interpolation.interpolateCameras(data_to_be_processed, perceptions);
            data_to_be_processed.erase(data_to_be_processed.begin(), --data_to_be_processed.end());
            //std::cout << "data tbp size: " << data_to_be_processed.size() << std::endl;
            data_to_be_processed.insert(*lidar_times.begin());
            lidar_times.erase(lidar_times.begin());
            
        }

    }//外层文件的循环结束

}

