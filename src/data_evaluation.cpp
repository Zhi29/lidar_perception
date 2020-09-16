#include <iostream>
#include <unordered_map>
#include <utility>      // std::pair, std::make_pair
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
    if(argc < 2) {
        std::cout << "Not enough arguments" << std::endl;
        return 0;
    }
    std::string project_path = argv[1];
    std::string outputPath = argv[2];

//获得所有数据包的文件夹名称
    std::vector<std::string> project_dirs;
    search_directories(project_path, project_dirs);
    for(int i = 0; i < project_dirs.size(); i++){
        std::cout << project_dirs[i] << std::endl;
    }
    //std::map<long int, std::vector<Lidar>> perceptions_by_ids;

    std::map<long int, std::map<double, std::vector<Lidar>>> perceptions_by_ids;

    double delta_t = 0;
    for(auto dir : project_dirs){
    // 读取lidar 感知结果
        //std::vector<LidarPerception> perceptions; 
        
    
        std::vector<std::string> lidar_files;
        std::set<SensorTimes, LESS_T0> lidar_times;
        search(project_path + dir + "/msd_score_lidar_ap/", "new", lidar_files); // read all lidar json file name
        for(int i = 0; i < lidar_files.size(); i++){
            std::cout << lidar_files[i] << std::endl;
        }


        for(int i = 0; i < lidar_files.size(); i++){
            LidarPerception lidar_perception(project_path + dir + "/msd_score_lidar_ap/" + lidar_files[i]);
            double lidar_timestamp = std::stod(lidar_files[i].substr(0, lidar_files[i].find(".")));
            SensorTimes lidar_time(lidar_timestamp, SensorType::LIDAR);
            lidar_times.insert(lidar_time);
            //perceptions.push_back(lidar_perception);
            std::unordered_map<long int, Lidar>::iterator iter;
            for(iter = lidar_perception.perception_.begin(); iter != lidar_perception.perception_.end(); iter++){
                if(iter->first >= 0){
                    perceptions_by_ids[iter->first][lidar_timestamp].push_back(iter->second);
                }
            }
        }

        double front_t = lidar_times.begin()->time;
        lidar_times.erase(lidar_times.begin());
        double back_t = lidar_times.begin()->time;
        delta_t = back_t - front_t;
        std::cout << delta_t << std::endl;
    }
    
    std::map<long int, std::map<double, std::vector<Lidar>>>::iterator iter_print;
    for(iter_print = perceptions_by_ids.begin(); iter_print != perceptions_by_ids.end(); iter_print++){
        std::cout << "trackid: " << iter_print->first << " is " << iter_print->second.size() << std::endl;
    }

    std::map<long int, std::unordered_map<std::string, std::vector<double>>> mean_by_id;
    std::unordered_map<long int, std::unordered_map<std::string, std::vector<double>>> std_by_id;

    std::map<long int, std::unordered_map<std::string, std::pair<double, double>>> max_min_by_ids;

    std::ofstream std_file(outputPath + "std_by_ids.txt");
    std::ofstream max_min_file(outputPath + "max_min.txt");
    //std::ofstream data_analysis(outputPath + "data_analysis");

    std::ofstream max_min_length(outputPath + "max_min_length.txt");
    std::ofstream max_min_width(outputPath + "max_min_width.txt");
    std::ofstream max_min_height(outputPath + "max_min_height.txt");
    std::ofstream max_min_yaw(outputPath + "max_min_yaw.txt");

    std::map<long int, std::vector<double>> lengths, widths, heights, yaws, xs, ys, zs;

    long int new_index = 0;

    std::ofstream data_analysis(outputPath + "data_analysis/" + std::to_string(new_index) + ".txt");
    data_analysis << "id: " << new_index << std::endl;
    
    for(iter_print = perceptions_by_ids.begin(); iter_print != perceptions_by_ids.end(); iter_print++){
        //计算均值 和 最大最小值 
        double sum_length = 0, sum_width = 0, sum_height = 0, sum_pitch = 0, sum_roll = 0, sum_yaw = 0;
        double max_length = -10000, min_length = 100000;
        double max_width = -10000, min_width = 100000;
        double max_height = -10000, min_height = 100000;
        double max_pitch = -10000, min_pitch = 100000;
        double max_roll = -10000, min_roll = 100000;
        double max_yaw = -10000, min_yaw = 100000;
        int size_by_id = 0;

        std::map<double, std::vector<Lidar>>::iterator iter_time;


        for(iter_time = iter_print->second.begin(); iter_time != iter_print->second.end(); iter_time++){
            std::map<double, std::vector<Lidar>>::iterator iter_prev;
            if(iter_time != iter_print->second.begin()){
                iter_prev = iter_time;
                iter_prev--;
            }
            
            //data_analysis << "id: " << new_index << std::endl;
            if(iter_time == iter_print->second.begin() || (iter_time->first - iter_prev->first) <= 1.1*delta_t){
                for(int i = 0; i < iter_time->second.size(); i++){
                    sum_length += iter_time->second[i].length;
                    sum_width += iter_time->second[i].width;
                    sum_height += iter_time->second[i].height;
                    sum_pitch += iter_time->second[i].pitch;
                    sum_roll += iter_time->second[i].roll;
                    sum_yaw += iter_time->second[i].yaw;

                    lengths[new_index].push_back(iter_time->second[i].length);
                    widths[new_index].push_back(iter_time->second[i].width);
                    heights[new_index].push_back(iter_time->second[i].height);
                    yaws[new_index].push_back(iter_time->second[i].yaw);
                    xs[new_index].push_back(iter_time->second[i].x);
                    ys[new_index].push_back(iter_time->second[i].y);
                    zs[new_index].push_back(iter_time->second[i].z);


                    max_length = std::max(max_length, iter_time->second[i].length);
                    min_length = std::min(min_length, iter_time->second[i].length);
                    max_width = std::max(max_width, iter_time->second[i].width);
                    min_width = std::min(min_width, iter_time->second[i].width);
                    max_height = std::max(max_height, iter_time->second[i].height);
                    min_height = std::min(min_height, iter_time->second[i].height);
                    max_pitch= std::max(max_pitch, iter_time->second[i].pitch);
                    min_pitch = std::min(min_pitch, iter_time->second[i].pitch);
                    max_roll = std::max(max_roll, iter_time->second[i].roll);
                    min_roll = std::min(min_roll, iter_time->second[i].roll);


                    max_yaw = std::max(max_yaw, iter_time->second[i].yaw);
                    min_yaw = std::min(min_yaw, iter_time->second[i].yaw);
                    
                    
                    
                    data_analysis << "  time: " <<  std::fixed << std::to_string(iter_time->first).substr(0,16) << std::endl;
                        
                    data_analysis << "      length " << iter_time->second[i].length << std::endl;
                    data_analysis << "      width " << iter_time->second[i].width << std::endl;
                    data_analysis << "      height " << iter_time->second[i].height << std::endl;
                    data_analysis << "      pitch " << iter_time->second[i].pitch << std::endl;
                    data_analysis << "      roll " << iter_time->second[i].roll << std::endl;
                    data_analysis << "      yaw " << iter_time->second[i].yaw << std::endl;
                    data_analysis << "          x: " << iter_time->second[i].x << std::endl;
                    data_analysis << "          y: " << iter_time->second[i].y << std::endl;
                    data_analysis << "          z: " << iter_time->second[i].z << std::endl;
                    size_by_id++;
                }
                
            }
            else{
                if(size_by_id != 0){
                    double mean_length = sum_length / size_by_id;
                    double mean_width = sum_width / size_by_id;
                    double mean_height = sum_height / size_by_id;
                    double mean_pitch = sum_pitch / size_by_id;
                    double mean_roll = sum_roll / size_by_id;
                    double mean_yaw = sum_yaw / size_by_id;
                    size_by_id = 0;
                    mean_by_id[new_index]["length"].push_back(mean_length);
                    mean_by_id[new_index]["width"].push_back(mean_width);
                    mean_by_id[new_index]["height"].push_back(mean_height);
                    mean_by_id[new_index]["pitch"].push_back(mean_pitch);
                    mean_by_id[new_index]["roll"].push_back(mean_roll);
                    mean_by_id[new_index]["yaw"].push_back(mean_yaw);

                    max_min_by_ids[new_index]["length"] = std::make_pair(max_length, min_length);
                    max_min_by_ids[new_index]["width"] = std::make_pair(max_width, min_width);
                    max_min_by_ids[new_index]["height"] = std::make_pair(max_height, min_height);
                    max_min_by_ids[new_index]["pitch"] = std::make_pair(max_pitch, min_pitch);
                    max_min_by_ids[new_index]["roll"] = std::make_pair(max_roll, min_roll);
                    max_min_by_ids[new_index]["yaw"] = std::make_pair(max_yaw, min_yaw);

                    max_length = -100000; min_length = 100000;
                    max_width = -100000; min_width = 100000;
                    max_height = -1000000; min_height = 100000;
                    max_pitch = -1000000; min_pitch = 100000;
                    max_roll = -10000; min_roll = 100000;
                    max_yaw = -100000; min_yaw = 100000;

                    new_index++;
                    data_analysis << std::endl;
                    data_analysis << "id: " << new_index << std::endl;
                    sum_length = 0;
                    sum_width = 0;
                    sum_height = 0;
                    sum_pitch = 0;
                    sum_roll = 0;
                    sum_yaw = 0;
                }


            }

        } 





        //计算标准差
        /**
        size_by_id = 0;
        int index = 0;
        double sum_square_length = 0, sum_square_width = 0, sum_square_height = 0, sum_square_pitch = 0, sum_square_roll = 0, sum_square_yaw = 0;
        for(iter_time = iter_print->second.begin(); iter_time != iter_print->second.end(); iter_time++){
            std::map<double, std::vector<Lidar>>::iterator iter_prev;
            if(iter_time != iter_print->second.begin()){
                iter_prev = iter_time;
                iter_prev--;
            }
            
            if(iter_time == iter_print->second.begin() || (iter_time->first - iter_prev->first) <= 1.1*delta_t){
                for(int i = 0; i < iter_time->second.size(); i++){
                    sum_square_length += pow((iter_time->second[i].length - mean_by_id[iter_print->first]["length"][index]),2);
                    sum_square_width += pow((iter_time->second[i].width - mean_by_id[iter_print->first]["width"][index]),2);
                    sum_square_height += pow((iter_time->second[i].height - mean_by_id[iter_print->first]["height"][index]),2);
                    sum_square_pitch += pow((iter_time->second[i].pitch - mean_by_id[iter_print->first]["pitch"][index]),2);
                    sum_square_roll += pow((iter_time->second[i].roll - mean_by_id[iter_print->first]["roll"][index]),2);
                    sum_square_yaw += pow((iter_time->second[i].yaw - mean_by_id[iter_print->first]["yaw"][index]),2);
                }
                size_by_id++;
            }
            else{
                if(size_by_id != 0){
                    double std_length = sqrt(sum_square_length / size_by_id);
                    double std_width = sqrt(sum_square_width / size_by_id);
                    double std_height = sqrt(sum_square_height / size_by_id);
                    double std_pitch = sqrt(sum_square_pitch / size_by_id);
                    double std_roll = sqrt(sum_square_roll / size_by_id);
                    double std_yaw = sqrt(sum_square_yaw / size_by_id);
                    size_by_id = 0;
                    index++;
                    std_by_id[iter_print->first]["length"].push_back(std_length);
                    std_by_id[iter_print->first]["width"].push_back(std_width);
                    std_by_id[iter_print->first]["height"].push_back(std_height);
                    std_by_id[iter_print->first]["pitch"].push_back(std_pitch);
                    std_by_id[iter_print->first]["roll"].push_back(std_roll);
                    std_by_id[iter_print->first]["yaw"].push_back(std_yaw);
                    sum_square_length = 0;
                    sum_square_width = 0;
                    sum_square_height = 0;
                    sum_square_pitch = 0;
                    sum_square_roll = 0;
                    sum_square_yaw = 0;
                }

            }
        }
        **/


        /**for(int j = 0; j < mean_by_id[iter_print->first]["length"].size(); j++){
            std_file << "id: " << iter_print->first << std::endl;
            std_file << "  mean length: " <<  mean_by_id[iter_print->first]["length"][j] << " std length: " << std_by_id[iter_print->first]["length"][j] << std::endl;
            std_file << "  mean width: " <<  mean_by_id[iter_print->first]["width"][j] << " std width: " << std_by_id[iter_print->first]["width"][j] << std::endl;
            std_file << "  mean height: " <<  mean_by_id[iter_print->first]["height"][j] << " std height: " << std_by_id[iter_print->first]["height"][j] << std::endl;
            std_file << "  mean pitch: " <<  mean_by_id[iter_print->first]["pitch"][j] << " std pitch: " << std_by_id[iter_print->first]["pitch"][j] << std::endl;
            std_file << "  mean roll: " <<  mean_by_id[iter_print->first]["roll"][j] << " std roll: " << std_by_id[iter_print->first]["roll"][j] << std::endl;
            std_file << "  mean yaw: " <<  mean_by_id[iter_print->first]["yaw"][j] << " std yaw: " << std_by_id[iter_print->first]["yaw"][j] << std::endl;

            std::cout << "id: " << iter_print->first << std::endl;
            std::cout << "  mean length: " <<  mean_by_id[iter_print->first]["length"][j] << " std length: " << std_by_id[iter_print->first]["length"][j] << std::endl;
            std::cout << "  mean width: " <<  mean_by_id[iter_print->first]["width"][j] << " std width: " << std_by_id[iter_print->first]["width"][j] << std::endl;
            std::cout << "  mean height: " <<  mean_by_id[iter_print->first]["height"][j] << " std height: " << std_by_id[iter_print->first]["height"][j] << std::endl;
            std::cout << "  mean pitch: " <<  mean_by_id[iter_print->first]["pitch"][j] << " std pitch: " << std_by_id[iter_print->first]["pitch"][j] << std::endl;
            std::cout << "  mean roll: " <<  mean_by_id[iter_print->first]["roll"][j] << " std roll: " << std_by_id[iter_print->first]["roll"][j] << std::endl;
            std::cout << "  mean yaw: " <<  mean_by_id[iter_print->first]["yaw"][j] << " std yaw: " << std_by_id[iter_print->first]["yaw"][j] << std::endl;
        }**/
    }
    
        std::map<long int, std::unordered_map<std::string, std::vector<double>>>::iterator iter_mean;
        int j = 0;
        for(iter_mean = mean_by_id.begin(); iter_mean != mean_by_id.end(); iter_mean++){
            std_file << "id: " << iter_mean->first << std::endl;
            std_file << "  mean length: " <<  iter_mean->second["length"][j] << std::endl;//<< " std length: " << std_by_id[iter_print->first]["length"][j] << std::endl;
            std_file << "  mean width: " <<  iter_mean->second["width"][j] << std::endl;//<< " std width: " << std_by_id[iter_print->first]["width"][j] << std::endl;
            std_file << "  mean height: " <<  iter_mean->second["height"][j] << std::endl;//<< " std height: " << std_by_id[iter_print->first]["height"][j] << std::endl;
            //std_file << "  mean pitch: " <<  iter_mean->second["pitch"][j] << std::endl;//<< " std pitch: " << std_by_id[iter_print->first]["pitch"][j] << std::endl;
            //std_file << "  mean roll: " <<  iter_mean->second["roll"][j] << std::endl;//<< " std roll: " << std_by_id[iter_print->first]["roll"][j] << std::endl;
            std_file << "  mean yaw: " <<  iter_mean->second["yaw"][j] << std::endl;//<< " std yaw: " << std_by_id[iter_print->first]["yaw"][j] << std::endl;
            std::cout << "id: " << iter_mean->first << std::endl;
            std::cout << "  mean length: " <<  iter_mean->second["length"][j] << std::endl;//<< " std length: " << std_by_id[iter_print->first]["length"][j] << std::endl;
            std::cout << "  mean width: " <<  iter_mean->second["width"][j] << std::endl;//<< " std width: " << std_by_id[iter_print->first]["width"][j] << std::endl;
            std::cout << "  mean height: " <<  iter_mean->second["height"][j] << std::endl;//<< " std height: " << std_by_id[iter_print->first]["height"][j] << std::endl;
            //std::cout << "  mean pitch: " <<  iter_mean->second["pitch"][j] << std::endl;//<< " std pitch: " << std_by_id[iter_print->first]["pitch"][j] << std::endl;
            //std::cout << "  mean roll: " <<  iter_mean->second["roll"][j] << std::endl;//<< " std roll: " << std_by_id[iter_print->first]["roll"][j] << std::endl;
            std::cout << "  mean yaw: " <<  iter_mean->second["yaw"][j] << std::endl;//<< " std yaw: " << std_by_id[iter_print->first]["yaw"][j] << std::endl;

            
            std::ofstream length_fluc(outputPath + "/length/" + std::to_string(iter_mean->first) + ".txt");
            std::ofstream width_fluc(outputPath + "/width/" + std::to_string(iter_mean->first) + ".txt");
            std::ofstream height_fluc(outputPath + "/height/" + std::to_string(iter_mean->first) + ".txt");
            std::ofstream yaw_fluc(outputPath + "/yaw/" + std::to_string(iter_mean->first) + ".txt");

            for(int k = 0; k < lengths[iter_mean->first].size(); k++){
                length_fluc << lengths[iter_mean->first][k] -  iter_mean->second["length"][0] << std::endl;//<< " " << lengths[iter_mean->first][k] - iter_mean->second["length"][0] << std::endl;
                width_fluc << widths[iter_mean->first][k] -  iter_mean->second["width"][0] << std::endl;
                height_fluc << heights[iter_mean->first][k] -  iter_mean->second["height"][0] << std::endl;
                yaw_fluc << yaws[iter_mean->first][k] -  iter_mean->second["yaw"][0] << std::endl;
            }

        }

        std::map<long int, std::unordered_map<std::string, std::pair<double, double>>>::iterator iter_max_min;
        for(iter_max_min = max_min_by_ids.begin(); iter_max_min != max_min_by_ids.end(); iter_max_min++){
            max_min_file << "id: " << iter_max_min->first << "  max - min " << std::endl;
            max_min_file  << "  length: " << iter_max_min->second["length"].first - iter_max_min->second["length"].second << std::endl;
            //if(iter_max_min->second["length"].first - iter_max_min->second["length"].second > 1.0){
            //    for(int i = 0; i < xs[iter_max_min->first].size(); i++){
            //        max_min_file << "       x: " << xs[iter_max_min->first][i] << std::endl;
            //        max_min_file << "           y: " << ys[iter_max_min->first][i] << std::endl;
            //        max_min_file << "               z: " << zs[iter_max_min->first][i] << std::endl;
            //    }
                
            //}
            max_min_file  << "  width: " << iter_max_min->second["width"].first - iter_max_min->second["width"].second << std::endl;
            max_min_file  << "  height: " << iter_max_min->second["height"].first - iter_max_min->second["height"].second << std::endl;
            max_min_file  << "  yaw: " << abs(abs(iter_max_min->second["yaw"].first) - abs(iter_max_min->second["yaw"].second)) << std::endl;

            max_min_length << iter_max_min->second["length"].first -  iter_max_min->second["length"].second << std::endl;
            max_min_width << iter_max_min->second["width"].first -  iter_max_min->second["width"].second << std::endl;
            max_min_height << iter_max_min->second["height"].first -  iter_max_min->second["height"].second << std::endl;
            //max_min_yaw << abs(abs(iter_max_min->second["yaw"].first) - abs(iter_max_min->second["yaw"].second)) << std::endl;
        }

        for(iter_max_min = max_min_by_ids.begin(); iter_max_min != max_min_by_ids.end(); iter_max_min++){
            max_min_file << "id: " << iter_max_min->first << "  max - min " << std::endl;
            max_min_file  << "  length: " << iter_max_min->second["length"].first - iter_max_min->second["length"].second << std::endl;
            if(iter_max_min->second["length"].first - iter_max_min->second["length"].second > 1.0){
                for(int i = 0; i < xs[iter_max_min->first].size(); i++){
                    max_min_file << "       x: " << xs[iter_max_min->first][i] << std::endl;
                    max_min_file << "           y: " << ys[iter_max_min->first][i] << std::endl;
                    max_min_file << "               z: " << zs[iter_max_min->first][i] << std::endl;
                }
                
            }
            //max_min_file  << "  width: " << iter_max_min->second["width"].first - iter_max_min->second["width"].second << std::endl;
            //max_min_file  << "  height: " << iter_max_min->second["height"].first - iter_max_min->second["height"].second << std::endl;
            ///max_min_file  << "  yaw: " << abs(abs(iter_max_min->second["yaw"].first) - abs(iter_max_min->second["yaw"].second)) << std::endl;

            //max_min_length << iter_max_min->second["length"].first -  iter_max_min->second["length"].second << std::endl;
            //max_min_width << iter_max_min->second["width"].first -  iter_max_min->second["width"].second << std::endl;
            //max_min_height << iter_max_min->second["height"].first -  iter_max_min->second["height"].second << std::endl;
            //max_min_yaw << abs(abs(iter_max_min->second["yaw"].first) - abs(iter_max_min->second["yaw"].second)) << std::endl;
        }

        std::map<long int, std::vector<double>>::iterator iter_yaws;
        //yaws[new_index].push_back(iter_time->second[i].yaw);
        for(iter_yaws = yaws.begin(); iter_yaws != yaws.end(); iter_yaws++){
            double max_delta_yaw = -100000;
            for(int i = 0; i < iter_yaws->second.size(); i++){
                for(int j = 0; j < iter_yaws->second.size(); j++){
                    double delta_yaw = fabs(iter_yaws->second[i] - iter_yaws->second[j]);
                    if(delta_yaw > M_PI){
                        delta_yaw = 2*M_PI - delta_yaw;
                    }
                    //delta_yaw = abs(delta_yaw);
                    max_delta_yaw = std::max(max_delta_yaw, delta_yaw);
                }
            }
            std::cout << "yaw max - min: " << max_delta_yaw << std::endl;
            max_min_yaw << max_delta_yaw << std::endl;
        }


    
}