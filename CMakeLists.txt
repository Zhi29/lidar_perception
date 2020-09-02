cmake_minimum_required(VERSION 3.5)
project(inter_lidar)

set(CMAKE_PREFIX_PATH /home/lizhi/thirdparty/install)
include_directories(/home/lizhi/thirdparty/install/include)
#set(CMAKE_CXX_STANDARD 17)
#set(CMAKE_CXX_STANDARD_REQUIRED ON)
#set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_BUILD_TYPE "RELEASE")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 ")
#set(BOOST_ROOT "${CMAKE_PREFIX_PATH}/include/boost_1_73_0")

#set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED)
#find_package(Boost REQUIRED)

include_directories(
    ${EIGEN3_INCLUDE_DIRS}
 #   "${CMAKE_PREFIX_PATH}/include/boost_1_73_0"
    ${PROJECT_SOURCE_DIR}/include
)

add_library(interpolation SHARED 
            ${PROJECT_SOURCE_DIR}/src/read_json.cpp
            ${PROJECT_SOURCE_DIR}/src/interpolation.cpp
            
            )

#add_executable(lidar_interpolation ${PROJECT_SOURCE_DIR}/src/main.cpp)
#target_link_libraries(lidar_interpolation
#                        interpolation
#                        stdc++fs
#                        )

#add_executable(data_evaluation ${PROJECT_SOURCE_DIR}/src/data_evaluation.cpp)
#target_link_libraries(data_evaluation interpolation stdc++fs)

#add_executable(lidar_check ${PROJECT_SOURCE_DIR}/src/lidar_check.cpp)
#target_link_libraries(lidar_check interpolation stdc++fs)

add_executable(lidar_check2 ${PROJECT_SOURCE_DIR}/src/lidar_check2.cpp)
target_link_libraries(lidar_check2 interpolation stdc++fs)

add_executable(online_main ${PROJECT_SOURCE_DIR}/src/online_main.cpp)
target_link_libraries(online_main interpolation stdc++fs opencv_core opencv_highgui)

