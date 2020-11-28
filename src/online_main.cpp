#include <iostream>
#include <unordered_map>
#include "read_json.hpp"
#include <dirent.h>
#include <regex>
#include <sys/types.h>
#include <set>

#include <opencv2/core/core.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>

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

std::vector<Eigen::Vector3d> calculatebbox(Lidar& lidar){
                double x = lidar.x;
                double y = lidar.y;
                double z = lidar.z;
                double l = lidar.length;
                double w = lidar.width;
                double h = lidar.height;
                double yaw = lidar.yaw;

                Eigen::Vector3d center(x, y, z);

                Eigen::Matrix3d transform;
                
                transform << cos(yaw), -sin(yaw), 0,
                            sin(yaw), cos(yaw), 0,
                            0,          0,      1;


                std::vector<Eigen::Vector3d> bbox;
                Eigen::Vector3d p1(x - 0.5 * l,
                                    y + 0.5 * w,
                                    z - 0.5 * h);
                
                Eigen::Vector3d p2(x - 0.5 * l,
                                    y - 0.5 * w,
                                    z - 0.5 * h);
                
                Eigen::Vector3d p3(x + 0.5 * l,
                                    y - 0.5 * w,
                                    z - 0.5 * h);
                
                Eigen::Vector3d p4(x + 0.5 * l,
                                    y + 0.5 * w,
                                    z - 0.5 * h);
                
                Eigen::Vector3d p5(x - 0.5 * l,
                                    y + 0.5 * w,
                                    z + 0.5 * h);

                Eigen::Vector3d p6(x - 0.5 * l,
                                    y - 0.5 * w,
                                    z + 0.5 * h);
                
                Eigen::Vector3d p7(x + 0.5 * l,
                                    y - 0.5 * w,
                                    z + 0.5 * h);
                
                Eigen::Vector3d p8(x + 0.5 * l,
                                    y + 0.5 * w,
                                    z + 0.5 * h);
                
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
                return bbox;
}

std::vector<Eigen::Vector3d> projection(std::vector<Eigen::Vector3d>& bbox, std::map<SensorType, CameraParameters>& cams, SensorType type){
    std::vector<Eigen::Vector3d> bbox_in_camera;
    for(int k = 0; k < bbox.size(); k++){
        //std::cout << "rotation matrix \n" << cams.at(type).rotation_matrix << std::endl;
        //std::cout << "translation \n" << cams.at(type).translation << std::endl;

        Eigen::Vector3d pts_in_cam = cams.at(type).rotation_matrix * (bbox[k] - cams.at(type).translation);
        if(pts_in_cam(2) < 0) break;
        //std::cout << k+1 << "th pts in camera frame \n " << pts_in_cam << std::endl; 
        pts_in_cam = pts_in_cam / pts_in_cam(2);
        //std::cout << k+1 << "th pts in camera frame normalized \n " << pts_in_cam << std::endl; 


        double r = std::sqrt(std::pow(pts_in_cam(0),2) + std::pow(pts_in_cam(1),2));
        double theta = std::atan(r);
        double theta_d = theta * (1.0 + cams.at(type).distortions(0) * std::pow(theta,2) + cams.at(type).distortions(1)*std::pow(theta, 4)
                                + cams.at(type).distortions(2) * std::pow(theta , 6) + cams.at(type).distortions(3) * std::pow(theta, 8));
        //std::cout << "distortion \n" << cams.at(type).distortions << std::endl;
        pts_in_cam(0) = theta_d / r * pts_in_cam(0);
        pts_in_cam(1) = theta_d / r * pts_in_cam(1);
        //std::cout << k+1 << "th pts in camera frame after distortion \n" << pts_in_cam << std::endl;


        Eigen::Vector3d pts_in_pixel = cams.at(type).camera_intrinsics * pts_in_cam;
        bbox_in_camera.push_back(pts_in_pixel);
    }
    return bbox_in_camera;
}

int main(int argc, char *argv[])
{
    if(argc < 4){
        std::cout << "Not enough argument" << std::endl;
        return 0;
    }
    //std::string perception_path = argv[1];
    //std::string egomotion_path = argv[2];
    std::string project_path = argv[1];
    std::string outputPath = argv[2];
    std::string img_output = argv[3];

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
        std::priority_queue<double, std::vector<double>, std::greater<double>> lidar_times;
        search(project_path + dir + "/msd_score_lidar_ap/", "new", lidar_files); // read all lidar json file name

        for(int i = 0; i < lidar_files.size(); i++){
            LidarPerception lidar_perception(project_path + dir + "/msd_score_lidar_ap/" + lidar_files[i]);
            double lidar_timestamp = std::stod(lidar_files[i].substr(0, lidar_files[i].find(".")));
            lidar_perception.timestamp = lidar_timestamp;
            //SensorTimes lidar_time(lidar_timestamp, SensorType::LIDAR);
            lidar_times.push(lidar_timestamp);
            perceptions.insert({lidar_timestamp, lidar_perception});
        }


    std::unordered_map<double, std::vector<SensorType>> timestamp_sensortype;

    std::unordered_map<double, std::map<SensorType, std::string>> img_paths;

    //读取FISH_EYE_F的时间戳
        std::vector<std::string> fish_eye_f_files;
        std::priority_queue<double, std::vector<double>, std::greater<double>> fish_eye_f_;
        search(project_path + dir + "/fisheye_F/", "jpg", fish_eye_f_files);
        for(int i = 0; i < fish_eye_f_files.size(); i++){
            double fish_eye_f = stod(fish_eye_f_files[i].substr(0, fish_eye_f_files[i].find(".")));//截取出图片文件名中时间戳的部分
            fish_eye_f_.push(fish_eye_f);//按时间顺序存入
            timestamp_sensortype[fish_eye_f].push_back(SensorType::FISH_EYE_F);
            img_paths[fish_eye_f][SensorType::FISH_EYE_F] = project_path+dir+"/fisheye_F/" + fish_eye_f_files[i];
        }
        fish_eye_f_files.clear();


    //读取FISH_EYE_B的时间戳
        std::vector<std::string> fish_eye_b_files;
        std::priority_queue<double, std::vector<double>, std::greater<double>> fish_eye_b_;
        search(project_path + dir + "/fisheye_B/", "jpg", fish_eye_b_files);
        for(int i = 0; i < fish_eye_b_files.size(); i++){
            double fish_eye_b = stod(fish_eye_b_files[i].substr(0, fish_eye_b_files[i].find(".")));
            fish_eye_b_.push(fish_eye_b);
            timestamp_sensortype[fish_eye_b].push_back(SensorType::FISH_EYE_B);
            img_paths[fish_eye_b][SensorType::FISH_EYE_B] = project_path+dir+"/fisheye_B/" + fish_eye_b_files[i];
        }
        fish_eye_b_files.clear();

    //读取FISH_EYE_L的时间戳
        std::vector<std::string> fish_eye_l_files;
        std::priority_queue<double, std::vector<double>, std::greater<double>> fish_eye_l_;
        search(project_path + dir + "/fisheye_L/", "jpg", fish_eye_l_files);
        for(int i = 0; i < fish_eye_l_files.size(); i++){
            double fish_eye_l = stod(fish_eye_l_files[i].substr(0, fish_eye_l_files[i].find(".")));
            fish_eye_l_.push(fish_eye_l);
            timestamp_sensortype[fish_eye_l].push_back(SensorType::FISH_EYE_L);
            img_paths[fish_eye_l][SensorType::FISH_EYE_L] = project_path+dir+"/fisheye_L/" + fish_eye_l_files[i];
        }
        fish_eye_l_files.clear();

    //读取FISH_EYE_R的时间戳
        std::vector<std::string> fish_eye_r_files;
        std::priority_queue<double, std::vector<double>, std::greater<double>> fish_eye_r_;
        search(project_path + dir + "/fisheye_R/", "jpg", fish_eye_r_files);
        for(int i = 0; i < fish_eye_r_files.size(); i++){
            double fish_eye_r = stod(fish_eye_r_files[i].substr(0, fish_eye_r_files[i].find(".")));
            fish_eye_r_.push(fish_eye_r);
            timestamp_sensortype[fish_eye_r].push_back(SensorType::FISH_EYE_R);
            img_paths[fish_eye_r][SensorType::FISH_EYE_R] = project_path+dir+"/fisheye_R/" + fish_eye_r_files[i];
        }
        fish_eye_r_files.clear();

    //读取camera parameters
        CameraParameters cam_front(project_path+dir+"/calib_source/cameraFront195.json", SensorType::FISH_EYE_F);
        CameraParameters cam_left(project_path+dir+"/calib_source/cameraLeft195.json", SensorType::FISH_EYE_L);
        CameraParameters cam_right(project_path+dir+"/calib_source/cameraRight195.json", SensorType::FISH_EYE_R);
        CameraParameters cam_back(project_path+dir+"/calib_source/cameraRear195.json", SensorType::FISH_EYE_B);
        std::map<SensorType, CameraParameters> cams = {{SensorType::FISH_EYE_F, cam_front}, {SensorType::FISH_EYE_L, cam_left},
                                                    {SensorType::FISH_EYE_R, cam_right}, {SensorType::FISH_EYE_B, cam_back}};

        
        Interpolation interpolation(outputPath, dir);
        std::vector<double> timestamps_to_be_processed;
        std::map<double, std::vector<Lidar>> interpolation_results;

        std::unordered_map<double, std::pair<double, double>> cam_lidar_time;
        std::unordered_map<double, std::unordered_map<long int, std::pair<double, double>>> cam_lidar_time_by_ids;
        std::map<SensorType, std::string> camera_folders = {{SensorType::FISH_EYE_F, "fisheye_F_lidar/"},
                                                                    {SensorType::FISH_EYE_B, "fisheye_B_lidar/"},
                                                                    {SensorType::FISH_EYE_L, "fisheye_L_lidar/"},
                                                                    {SensorType::FISH_EYE_R, "fisheye_R_lidar/"}};

        double lidar_time;

        while(!fish_eye_f_.empty())
        {   
            if(!lidar_times.empty())
                lidar_time = lidar_times.top();
            timestamps_to_be_processed.push_back(fish_eye_f_.top());
            timestamps_to_be_processed.push_back(fish_eye_b_.top());
            timestamps_to_be_processed.push_back(fish_eye_l_.top());
            timestamps_to_be_processed.push_back(fish_eye_r_.top());
            //interpolation.onlineInterpolation(perceptions.at(lidar_time), timestamps_to_be_processed, interpolation_results, cam_lidar_time);
            interpolation.offlineInterpolation(perceptions.at(lidar_time), timestamps_to_be_processed, interpolation_results, cam_lidar_time_by_ids);
            //interpolation.onlineWriteToJson(timestamps_to_be_processed, timestamp_sensortype);
            
            //interpolation.camera_perception_result.clear();
            
            //interpolation.camera_perception_result.clear();


            lidar_times.pop();
            fish_eye_f_.pop();
            fish_eye_b_.pop();
            fish_eye_l_.pop();
            fish_eye_r_.pop();
            timestamps_to_be_processed.clear();
        }
        interpolation.writeToJson(timestamp_sensortype);
        
        std::unordered_map<double, bool>::iterator iter_ts;
        for(iter_ts = interpolation.interpolated_timestamps.begin(); iter_ts != interpolation.interpolated_timestamps.end(); iter_ts++){
            if(!iter_ts->second){
                std::vector<Lidar> v;
                v.clear();
                interpolation_results.insert({iter_ts->first, v});
            }
        }
        
       
        std::cout << interpolation.all_camera_time.size() << std::endl;
        for(int i = 0; i < interpolation.all_camera_time.size(); i++){
            if(!interpolation_results.count(interpolation.all_camera_time[i])){
                std::vector<Lidar> v;
                v.clear();
                interpolation_results.insert({interpolation.all_camera_time[i], v});
            }
        }
        std::cout << interpolation_results.size() << std::endl;


    
        std::map<double, std::map<SensorType, std::vector<std::vector<Eigen::Vector3d>>>> bbox_in_cam_results;
        std::map<double, std::map<SensorType, std::vector<std::vector<Eigen::Vector3d>>>> bbox_in_lidar1_results;
        std::map<double, std::map<SensorType, std::vector<std::vector<Eigen::Vector3d>>>> bbox_in_lidar2_results;

        std::cout << cam_lidar_time_by_ids.size() << std::endl;
        std::map<double, std::vector<Lidar>>::iterator iter_results;
        for(iter_results = interpolation_results.begin(); iter_results != interpolation_results.end(); iter_results++){
            for(int i = 0; i < iter_results->second.size(); i++){
                std::vector<Eigen::Vector3d> bbox = calculatebbox(iter_results->second[i]);

               // std::cout << std::fixed << cam_lidar_time_by_ids.at(iter_results->first).at(iter_results->second[i].trackid).first
                //<< " id " << iter_results->second[i].trackid << std::endl;
                double camera_time = iter_results->first;
                long int obj_id = iter_results->second[i].trackid;
                double lidar_1_time = cam_lidar_time_by_ids.at(camera_time).at(obj_id).first;
                double lidar_2_time = cam_lidar_time_by_ids.at(camera_time).at(obj_id).second;
                //std::cout << lidar_1_time << std::endl;
                std::vector<Eigen::Vector3d> bbox_lidar_1 = calculatebbox(perceptions.at(lidar_1_time).perception_.at(obj_id));

                std::vector<Eigen::Vector3d> bbox_lidar_2 = calculatebbox(perceptions.at(lidar_2_time).perception_.at(obj_id));
        
                for(int j = 0; j < timestamp_sensortype[iter_results->first].size(); j++){
                    SensorType type = timestamp_sensortype[iter_results->first][j];

                    std::vector<Eigen::Vector3d> bbox_in_camera = projection(bbox, cams, type);
                    std::vector<Eigen::Vector3d> bbox_in_lidar_1 = projection(bbox_lidar_1, cams, type);
                    std::vector<Eigen::Vector3d> bbox_in_lidar_2 = projection(bbox_lidar_2, cams, type);
                    if(bbox_in_camera.size() == 8)
                        bbox_in_cam_results[iter_results->first][type].push_back(bbox_in_camera);
                    if(bbox_in_lidar_1.size() == 8)
                        bbox_in_lidar1_results[iter_results->first][type].push_back(bbox_in_lidar_1);
                    if(bbox_in_lidar_2.size() == 8)
                        bbox_in_lidar2_results[iter_results->first][type].push_back(bbox_in_lidar_2);
                }
            }

            std::map<SensorType, std::string>::iterator iter_paths;
            for(iter_paths = img_paths[iter_results->first].begin(); iter_paths != img_paths[iter_results->first].end(); iter_paths++){
                cv::Mat img = cv::imread(iter_paths->second);
                std::pair<cv::Mat, SensorType> image = std::make_pair(img,iter_paths->first);

                for(int i = 0; i < bbox_in_cam_results[iter_results->first][iter_paths->first].size(); i++){
                    std::vector<Eigen::Vector3d> draw_bbox = bbox_in_cam_results[iter_results->first][iter_paths->first][i];
                    cv::Point2d point_1(draw_bbox[0](0), draw_bbox[0](1)), point_2(draw_bbox[1](0), draw_bbox[1](1)), 
                    point_3(draw_bbox[2](0), draw_bbox[2](1)), point_4(draw_bbox[3](0), draw_bbox[3](1)),
                    point_5(draw_bbox[4](0), draw_bbox[4](1)), point_6(draw_bbox[5](0), draw_bbox[5](1)), 
                    point_7(draw_bbox[6](0), draw_bbox[6](1)), point_8(draw_bbox[7](0), draw_bbox[7](1));

                    cv::line(img, point_1, point_2, cv::Scalar( 0, 255, 0), 1);
                    cv::line(img, point_1, point_4, cv::Scalar( 0, 255, 0), 1);
                    cv::line(img, point_1, point_5, cv::Scalar( 0, 255, 0), 1);
                    cv::line(img, point_2, point_3, cv::Scalar( 0, 255, 0), 1);
                    cv::line(img, point_2, point_6, cv::Scalar( 0, 255, 0), 1);
                    cv::line(img, point_3, point_4, cv::Scalar( 0, 255, 0), 1);
                    cv::line(img, point_3, point_7, cv::Scalar( 0, 255, 0), 1);
                    cv::line(img, point_4, point_8, cv::Scalar( 0, 255, 0), 1);
                    cv::line(img, point_5, point_6, cv::Scalar( 0, 255, 0), 1);
                    cv::line(img, point_5, point_8, cv::Scalar( 0, 255, 0), 1);
                    cv::line(img, point_6, point_7, cv::Scalar( 0, 255, 0), 1);
                    cv::line(img, point_7, point_8, cv::Scalar( 0, 255, 0), 1);
                }//一张图片多个object
                for(int i = 0; i < bbox_in_lidar1_results[iter_results->first][iter_paths->first].size(); i++){
                    std::vector<Eigen::Vector3d> draw_bbox = bbox_in_lidar1_results[iter_results->first][iter_paths->first][i];
                    cv::Point2d point_1(draw_bbox[0](0), draw_bbox[0](1)), point_2(draw_bbox[1](0), draw_bbox[1](1)), 
                    point_3(draw_bbox[2](0), draw_bbox[2](1)), point_4(draw_bbox[3](0), draw_bbox[3](1)),
                    point_5(draw_bbox[4](0), draw_bbox[4](1)), point_6(draw_bbox[5](0), draw_bbox[5](1)), 
                    point_7(draw_bbox[6](0), draw_bbox[6](1)), point_8(draw_bbox[7](0), draw_bbox[7](1));

                    cv::line(img, point_1, point_2, cv::Scalar( 0, 0, 255), 1);
                    cv::line(img, point_1, point_4, cv::Scalar( 0, 0, 255), 1);
                    cv::line(img, point_1, point_5, cv::Scalar( 0, 0, 255), 1);
                    cv::line(img, point_2, point_3, cv::Scalar( 0, 0, 255), 1);
                    cv::line(img, point_2, point_6, cv::Scalar( 0, 0, 255), 1);
                    cv::line(img, point_3, point_4, cv::Scalar( 0, 0, 255), 1);
                    cv::line(img, point_3, point_7, cv::Scalar( 0, 0, 255), 1);
                    cv::line(img, point_4, point_8, cv::Scalar( 0, 0, 255), 1);
                    cv::line(img, point_5, point_6, cv::Scalar( 0, 0, 255), 1);
                    cv::line(img, point_5, point_8, cv::Scalar( 0, 0, 255), 1);
                    cv::line(img, point_6, point_7, cv::Scalar( 0, 0, 255), 1);
                    cv::line(img, point_7, point_8, cv::Scalar( 0, 0, 255), 1);
                }//一张图片多个object
                for(int i = 0; i < bbox_in_lidar2_results[iter_results->first][iter_paths->first].size(); i++){
                    std::vector<Eigen::Vector3d> draw_bbox = bbox_in_lidar2_results[iter_results->first][iter_paths->first][i];
                    cv::Point2d point_1(draw_bbox[0](0), draw_bbox[0](1)), point_2(draw_bbox[1](0), draw_bbox[1](1)), 
                    point_3(draw_bbox[2](0), draw_bbox[2](1)), point_4(draw_bbox[3](0), draw_bbox[3](1)),
                    point_5(draw_bbox[4](0), draw_bbox[4](1)), point_6(draw_bbox[5](0), draw_bbox[5](1)), 
                    point_7(draw_bbox[6](0), draw_bbox[6](1)), point_8(draw_bbox[7](0), draw_bbox[7](1));

                    cv::line(img, point_1, point_2, cv::Scalar( 0, 0, 255), 1);
                    cv::line(img, point_1, point_4, cv::Scalar( 0, 0, 255), 1);
                    cv::line(img, point_1, point_5, cv::Scalar( 0, 0, 255), 1);
                    cv::line(img, point_2, point_3, cv::Scalar( 0, 0, 255), 1);
                    cv::line(img, point_2, point_6, cv::Scalar( 0, 0, 255), 1);
                    cv::line(img, point_3, point_4, cv::Scalar( 0, 0, 255), 1);
                    cv::line(img, point_3, point_7, cv::Scalar( 0, 0, 255), 1);
                    cv::line(img, point_4, point_8, cv::Scalar( 0, 0, 255), 1);
                    cv::line(img, point_5, point_6, cv::Scalar( 0, 0, 255), 1);
                    cv::line(img, point_5, point_8, cv::Scalar( 0, 0, 255), 1);
                    cv::line(img, point_6, point_7, cv::Scalar( 0, 0, 255), 1);
                    cv::line(img, point_7, point_8, cv::Scalar( 0, 0, 255), 1);
                }//一张图片多个object
                
                //for(int i = 0; i < timestamp_sensortype[iter_results->first].size(); i++)
                {
                    std::experimental::filesystem::path dirs(img_output + dir + "/" + camera_folders[image.second]);
                    if(!(std::experimental::filesystem::exists(dirs))){
                        if(std::experimental::filesystem::create_directories(dirs))
                        {
                            std::cout << "create " << img_output + dir + "/" + camera_folders[image.second] << std::endl;
                        }
                    }
                    std::string save_img_path = img_output + dir + "/" + 
                        camera_folders[image.second] + std::to_string(iter_results->first).substr(0,16) + ".jpg";
                    cv::imwrite(save_img_path, img);
                }


                //if(bbox_in_cam_results[iter_results->first][iter_paths->first].size() > 0){//if there are boxes in image then draw
                //    std::cout << std::fixed << iter_results->first << std::endl;
                //    std::cout << " 2 lidar times: \n" << cam_lidar_time[iter_results->first].first << std::endl << cam_lidar_time[iter_results->first].second << std::endl;
                //   cv::imshow("image", img);
                //    cv::waitKey(0);
                //}

            }
            
        }
        
    }//外层文件夹的循环结束

}

